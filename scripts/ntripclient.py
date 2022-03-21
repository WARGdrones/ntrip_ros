#!/usr/bin/python3

from base64 import b64encode
from threading import Thread
import socket
import datetime

from time import sleep

import rospy
# from nmea_msgs.msg import Sentence
from mavros_msgs.msg import RTCM


class NtripConnect(Thread):
    """Creating a ntrip connection"""

    def __init__(self, ntc):
        super(NtripConnect, self).__init__()
        self.ntc = ntc
        self.stop = False

    # Helper functions to create the GPPA string by ourselves
    def to_dec_minutes(self, degree, is_long=False):
        """Converts degrees to degree minutes
        isLong: Boolean. If we have a longitude, an extra digit for degrees is expected"""
        dd = abs(degree)
        minutes, seconds = divmod(dd*3600, 60)
        degrees, minutes = divmod(minutes, 60)
        if is_long:
            return F"{int(degrees):03d}" + "{:07.4f}".format(int(minutes)+seconds/60)
        else:
            return F"{int(degrees):02d}" + "{:07.4f}".format(int(minutes)+seconds/60)
        # # print(str(int(degrees)) + "{:07.4f}".format(int(minutes)+seconds/60))

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
            calc_cksum = calc_cksum ^ ord(s)
        # Return the nmeadata, the checksum from
        # sentence, and the calculated checksum
        return sentence + F"*{calc_cksum:02X}\r\n"

    def generate_gga_string(self):
        """Generates the GGA string to tell the base where we are
        fields:
        Message ID:     $GPGGA
        UTC Time:       hhmmss.sss
        Latitude:       ddmm.mmmm
        N/S Indicator:  N=North, S=South
        Longitude:      dddmm.mmmm
        E/W Indicator:  E=East, W=West
        Pos Fix Ind:    should always be 1
        Sats used:      should always be 10 (or some number)
        HDOP:           Just use 1
        MSL Altitude:   We might need to calculate that or so
        Alt Unit:       M for meters
        Geoid seperation: 0
        Geoid Unit:     M for meters
        Age of diff corr: use 0.0
        ref station id: 0
        Checksum:       XOR-Checksum
        all fields are comma-seperated, except for the checksum, that uses a '*'
        """
        dt = datetime.datetime.utcnow()

        gga = F"GPGGA,{dt.hour:0>2d}{dt.minute:0>2d}{dt.second:0>2d}.{int(dt.microsecond / 10000):0>2d}"
        gga += ',' +\
            self.to_dec_minutes(self.ntc.latitude) +\
            ',' + self.lat_dir(self.ntc.latitude)
        gga += ',' +\
            self.to_dec_minutes(degree=self.ntc.longitude, is_long=True) +\
            ',' + self.long_dir(self.ntc.longitude)
        gga += F',1,10,1.0,{self.ntc.altitude:.2f},M,0,M,0.0,0'
        gga = "$"+self.checksum(gga)
        return gga

    # Function actually doing stuff
    def run(self):
        # print(self.ntc.ntrip_user)
        # print(self.ntc.ntrip_pass)
        header = F"GET /{self.ntc.ntrip_stream} HTTP/1.1\r\n" +\
            F"Host: {self.ntc.ntrip_server}\r\n" +\
            "Ntrip-Version: Ntrip/2.0\r\n" +\
            "User-Agent: NTRIP ntrip_ros\r\n" +\
            F"Authorization: Basic {b64encode((self.ntc.ntrip_user + ':' + str(self.ntc.ntrip_pass)).encode('ascii')).decode('ascii')}\r\n" +\
            "Connection: close\r\n\r\n"

        restart_count = 0

        while not self.stop:
            try:
                gga = self.generate_gga_string()
                # rospy.loginfo(gga)

                hostname = self.ntc.ntrip_server.split(':')[0]
                port = self.ntc.ntrip_server.split(':')[1]

                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

                try:
                    s.connect((hostname, int(port)))
                except Exception as excep:
                    print(F"Connect failed with {excep}")
                    raise

                rospy.loginfo("Connection established")

                s.sendall(header.encode('ascii'))
                rospy.loginfo("Request sent")

                try:
                    # get all the bytes, make it to string
                    response = s.recv(4096)
                except Exception as excep:
                    print(F"Getresponse excepted with {excep}")
                    raise excep

                response_lines = response.decode('utf-8').split("\r\n")
                for line in response_lines:
                    print("'" + line+"'")
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
                    s.sendall(bytes(gga, "ascii"))  # ("utf-8"))
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
                            print(str(datetime.datetime.now()), cnt, typ)
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
        """ run this class """
        rospy.spin()
        if self.connection is not None:
            self.connection.stop = True


if __name__ == '__main__':
    c = NtripClient()
    c.run()
