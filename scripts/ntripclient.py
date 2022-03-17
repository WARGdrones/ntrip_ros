#!/usr/bin/python3

from base64 import b64encode
from threading import Thread
import socket
import re
import binascii

from datetime import *


from http.client import HTTPConnection
from http.client import IncompleteRead

from time import sleep

import rospy
# from nmea_msgs.msg import Sentence
from mavros_msgs.msg import RTCM


# This is to fix the IncompleteRead error
# http://bobrochel.blogspot.com/2010/11/bad-servers-chunked-encoding-and.html
import http.client


def patch_http_response_read(func):
    """Patching http_response_read to accept incomplete reads"""
    def inner(*args):
        try:
            return func(*args)
        except http.client.IncompleteRead as e:
            return e.partial
    return inner


http.client.HTTPResponse.read = patch_http_response_read(
    http.client.HTTPResponse.read)

ORIGINAL_HTTP_CLIENT_READ_STATUS = http.client.HTTPResponse._read_status  # Function pointer


def nice_to_icy(self):
    """https://stackoverflow.com/questions/4247248/record-streaming-and-saving-internet-radio-in-python
        Fixing that "ICY 200 OK" doesn't get recognized as ok"""
    class InterceptedHTTPResponse():
        """just passing through"""
        pass
    import io
    line = self.fp.readline().replace(
        b"ICY 200 OK\r\n", b"HTTP/1.0 200 OK\r\n")  # do we even need \r\n??
    intercepted_self = InterceptedHTTPResponse()
    intercepted_self.fp = io.BufferedReader(io.BytesIO(line))
    intercepted_self.debuglevel = self.debuglevel
    intercepted_self._close_conn = self._close_conn
    return ORIGINAL_HTTP_CLIENT_READ_STATUS(intercepted_self)


class NtripConnect(Thread):
    """Creating a ntrip connection"""

    def __init__(self, ntc):
        super(NtripConnect, self).__init__()
        self.ntc = ntc
        self.stop = False

    # Helper functions to create the GPPA string by ourselves
    def to_dec_minutes(self, degree):
        """Converts degrees to degree minutes"""
        dd = abs(degree)
        minutes, seconds = divmod(dd*3600, 60)
        degrees, minutes = divmod(minutes, 60)
        return str(int(degrees)) + "{:0>2d}".format(int(minutes)) + "{:8.7f}".format(seconds/60)

    def lat_dir(self, latitude):
        """Returns the direction of the latitude (N/S)"""
        if latitude > 0:
            return 'N'
        else:
            return 'S'

    def long_dir(self, longitude):
        """Returns the direction of the longitude (E/W)"""
        if longitude > 0:
            return 'E'
        else:
            return 'W'

    def checksum(self, sentence):
        """ Remove any newlines, calculate XOR checksum and return given sentence with checksum """
        calc_cksum = 0
        for s in sentence:
            calc_cksum ^= ord(s)
        # Return the nmeadata, the checksum from
        # sentence, and the calculated checksum
        return sentence + "*" + hex(calc_cksum)[2:]

    # Function actually doing stuff
    def run(self):
        # print(self.ntc.ntrip_user)
        # print(self.ntc.ntrip_pass)
        header = \
            F"GET /{self.ntc.ntrip_stream} HTTP/1.1\r\n" +\
            F"Host: {self.ntc.ntrip_server}\r\n" +\
            "Ntrip-Version: Ntrip/2.0\r\n" +\
            "User-Agent: NTRIP ntrip_ros\r\n" +\
            F"Authorization: Basic {b64encode((self.ntc.ntrip_user + ':' + str(self.ntc.ntrip_pass)).encode('ascii')).decode('ascii')}\r\n" +\
            "Connection: close\r\n\r\n"
        dt = datetime.utcnow()

        restart_count = 0

        while not self.stop:
            try:
                gga = "$GPGGA,{:0>2d}".format(dt.hour) + "{:0>2d}".format(dt.minute) + "{:0>2d}".format(
                    dt.second) + ".{:0>2d}".format(int(dt.microsecond / 10000))
                gga += ',' + \
                    self.to_dec_minutes(self.ntc.latitude) + \
                    ',' + self.lat_dir(self.ntc.latitude)
                gga += ',' + \
                    self.to_dec_minutes(self.ntc.longitude) + \
                    ',' + self.long_dir(self.ntc.longitude)
                gga += ',1,10,1.2,{:.4f},M,-2.860,M,,0000'.format(
                    self.ntc.altitude)
                gga = self.checksum(gga)
                # rospy.loginfo(gga)

                hostname = self.ntc.ntrip_server.split(':')[0]
                port = self.ntc.ntrip_server.split(':')[1]

                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.connect((hostname, int(port)))

                rospy.loginfo("Connection established")

                s.send(header.encode('ascii'))
                rospy.loginfo("Request sent")

                try:
                    response = s.recv(4096).decode(
                        'utf-8')  # get all the bytes, make it to string
                except Exception as exept:
                    print(F"Getresponse excepted with {exept}")
                    raise exept

                response_lines = response.split("\r\n")
                for line in response_lines:
                    if line == "":
                        pass  # end of header
                    elif line.find("SOURCETABLE") >= 0:
                        raise Exception("NTRIP mount point doesn't exist")
                    elif line.find("401 Unauthorized") >= 0:
                        raise Exception("Unauthorized request")
                    elif line.find("404 Not Found") >= 0:
                        raise Exception("Mount Point does not exist")
                    elif line.find("ICY 200 OK") >= 0:
                        pass
                    elif line.find("HTTP/1.0 200 OK") >= 0:
                        pass
                    elif line.find("HTTP/1.1 200 OK") >= 0:
                        pass
                    else:
                        raise Exception(F"Expected an OK code, got {line}")

                # send GGA
                try:
                    s.sendall(gga.encode("utf-8"))
                except Exception as exept:
                    print(F"Sending GGA data excepted with {exept}")
                    raise exept

                rospy.loginfo("GGA data sent")

                buf = bytes()
                rmsg = RTCM()

                while not self.stop:
                    # '''
                    # data = response.read(100)
                    # pos = data.find('\r\n')
                    # if pos != -1:
                    #     rmsg.message = buf + data[:pos]
                    #     rmsg.header.seq += 1
                    #     rmsg.header.stamp = rospy.get_rostime()
                    #     buf = data[pos+2:]
                    #     self.ntc.pub.publish(rmsg)
                    # else: buf += data
                    # '''

                    rospy.loginfo("Reading response now:")
                    # This now separates individual RTCM messages and publishes each one on the same topic
                    data = s.recv(1)

                    print(data)
                    if len(data) != 0:

                        if data[0] == 211:
                            buf += data
                            data = s.recv(2)
                            buf += data
                            cnt = data[0] * 256 + data[1]
                            data = s.recv(2)
                            buf += data
                            typ = (data[0] * 256 + data[1]) / 16
                            print(str(datetime.now()), cnt, typ)
                            cnt = cnt + 1
                            for x in range(cnt):
                                data = s.recv(1)
                                buf += data
                            rmsg.message = buf
                            rmsg.header.seq += 1
                            rmsg.header.stamp = rospy.get_rostime()
                            self.ntc.pub.publish(rmsg)
                            buf = bytes()
                        else:
                            print(data)
                    else:
                        # If zero length data, close connection and reopen it
                        restart_count = restart_count + 1
                        rospy.logwarn(
                            "Zero length: restart_count = %d", restart_count)
                        s.close()

                        # connection = HTTPConnection(self.ntc.ntrip_server)
                        # now = datetime.datetime.utcnow()
                        # connection.request('GET', '/'+self.ntc.ntrip_stream, self.ntc.nmea_gga %
                        #                    (now.hour, now.minute, now.second), headers)
                        # response = connection.getresponse()
                        # if response.status != 200:
                        #     raise Exception("Error: Response not HTTP200 (OK)")
                        buf = bytes()
                        raise socket.timeout()  # misuse this to get into the outer loop

                s.close()

            except socket.timeout:
                sleep(0.01)
                # we'll ignore timeout errors and reconnect
            except Exception as exept:
                print(F"Request exception `{exept}`, exiting")
                break


class NtripClient:
    """ Ntrip client ros adapter  """

    def __init__(self):
        rospy.init_node('ntripclient', anonymous=True)

        self.rtcm_topic = rospy.get_param(
            '~rtcm_topic', 'mavros/gps_rtk/send_rtcm')
        self.nmea_topic = rospy.get_param('~nmea_topic', 'nmea')

        self.ntrip_server = rospy.get_param('~ntrip_server')
        self.ntrip_user = rospy.get_param('~ntrip_user')
        self.ntrip_pass = rospy.get_param('~ntrip_pass')
        self.ntrip_stream = rospy.get_param('~ntrip_stream')

        self.latitude = rospy.get_param('~latitude')
        self.longitude = rospy.get_param('~longitude')
        self.altitude = rospy.get_param('~altitude')

        self.pub = rospy.Publisher(self.rtcm_topic, RTCM, queue_size=10)

        self.connection = None
        self.connection = NtripConnect(self)
        self.connection.start()

    def run(self):
        """ """
        rospy.spin()
        if self.connection is not None:
            self.connection.stop = True


if __name__ == '__main__':
    c = NtripClient()
    c.run()
