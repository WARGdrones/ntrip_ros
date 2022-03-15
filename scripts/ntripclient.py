#!/usr/bin/python

import rospy
from datetime import *
import socket
import re
import binascii

# from nmea_msgs.msg import Sentence
from mavros_msgs.msg import RTCM

from base64 import b64encode
from threading import Thread

from http.client import HTTPConnection
from http.client import IncompleteRead

from time import sleep

''' This is to fix the IncompleteRead error
    http://bobrochel.blogspot.com/2010/11/bad-servers-chunked-encoding-and.html'''
import http.client


def patch_http_response_read(func):
    def inner(*args):
        try:
            return func(*args)
        except http.client.IncompleteRead as e:
            return e.partial
    return inner


http.client.HTTPResponse.read = patch_http_response_read(
    http.client.HTTPResponse.read)


class ntripconnect(Thread):
    def __init__(self, ntc):
        super(ntripconnect, self).__init__()
        self.ntc = ntc
        self.stop = False

    def to_dec_minutes(self, degree):
        dd = abs(degree)
        minutes, seconds = divmod(dd*3600, 60)
        degrees, minutes = divmod(minutes, 60)
        return str(int(degrees)) + "{:0>2d}".format(int(minutes)) + "{:8.7f}".format(seconds/60)

    def lat_dir(self, latitude):
        if latitude > 0:
            return 'N'
        else:
            return 'S'

    def long_dir(self, longitude):
        if longitude > 0:
            return 'E'
        else:
            return 'W'

    def checksum(self, sentence):
        """ Remove any newlines """

        calc_cksum = 0
        for s in sentence:
            calc_cksum ^= ord(s)

        """ Return the nmeadata, the checksum from
            sentence, and the calculated checksum
        """
        return sentence + "*" + hex(calc_cksum)[2:]

    def run(self):

        headers = {
            'Ntrip-Version': 'Ntrip/2.0',
            'User-Agent': 'NTRIP ntrip_ros',
            'Connection': 'close',
            'Authorization': 'Basic ' + b64encode((self.ntc.ntrip_user + ':' + str(self.ntc.ntrip_pass)).encode("utf-8")).decode("utf-8")
        }
        dt = datetime.utcnow()
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
                rospy.loginfo(gga)

                connection = HTTPConnection(
                    self.ntc.ntrip_server)
                # now = datetime.datetime.utcnow()
                # connection.request('GET', '/'+self.ntc.ntrip_stream, self.ntc.nmea_gga %
                #                    (now.hour, now.minute, now.second), headers)
                connection.request(
                    'GET', '/'+self.ntc.ntrip_stream, gga, headers)

                response = connection.getresponse()
                if response.status != 200:
                    raise Exception("Error: Response not HTTP200 (OK)")

                buf = bytes()
                rmsg = RTCM()
                restart_count = 0
                while not self.stop:

                    '''
                    data = response.read(100)
                    pos = data.find('\r\n')
                    if pos != -1:
                        rmsg.message = buf + data[:pos]
                        rmsg.header.seq += 1
                        rmsg.header.stamp = rospy.get_rostime()
                        buf = data[pos+2:]
                        self.ntc.pub.publish(rmsg)
                    else: buf += data
                    '''

                    ''' This now separates individual RTCM messages and publishes each one on the same topic '''
                    data = response.read(1)
                    if len(data) != 0:
                        if data[0] == 211:
                            buf += data
                            data = response.read(2)
                            buf += data
                            cnt = data[0] * 256 + data[1]
                            data = response.read(2)
                            buf += data
                            typ = (data[0] * 256 + data[1]) / 16
                            print(str(datetime.now()), cnt, typ)
                            cnt = cnt + 1
                            for x in range(cnt):
                                data = response.read(1)
                                buf += data
                            rmsg.message = buf
                            rmsg.header.seq += 1
                            rmsg.header.stamp = rospy.get_rostime()
                            self.ntc.pub.publish(rmsg)
                            buf = bytes()
                        else:
                            print(data)
                    else:
                        ''' If zero length data, close connection and reopen it '''
                        restart_count = restart_count + 1
                        rospy.logwarn("Zero length ", restart_count)
                        connection.close()

                        # connection = HTTPConnection(self.ntc.ntrip_server)
                        # now = datetime.datetime.utcnow()
                        # connection.request('GET', '/'+self.ntc.ntrip_stream, self.ntc.nmea_gga %
                        #                    (now.hour, now.minute, now.second), headers)
                        # response = connection.getresponse()
                        # if response.status != 200:
                        #     raise Exception("Error: Response not HTTP200 (OK)")
                        buf = bytes()
                        raise socket.timeout()  # misuse this to get into the outer loop

                connection.close()

            except socket.timeout:
                sleep(0.01)
                pass  # we'll ignore timeout errors and reconnect
            except Exception as e:
                print("Request exception `{}`, exiting".format(e))
                break


class ntripclient:
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
        self.connection = ntripconnect(self)
        self.connection.start()

    def run(self):
        rospy.spin()
        if self.connection is not None:
            self.connection.stop = True


if __name__ == '__main__':
    c = ntripclient()
    c.run()
