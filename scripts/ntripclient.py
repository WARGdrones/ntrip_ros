#!/usr/bin/python3

from base64 import b64encode
from threading import Thread
import socket
import datetime
import math

from time import sleep

import rospy
# from nmea_msgs.msg import Sentence
from mavros_msgs.msg import RTCM
from hub_control_interface.msg import HubState
from ntrip_ros.srv import ProvideRTCM

# for RTCM3 infos: https://github.com/ArduPilot/MissionPlanner/blob/master/ExtLibs/Utilities/rtcm3.cs

# Currently, we are getting the following data from SAPOS:
# 1021: 622 -> Helmet/Abridged Molodenski Message (Transformation parameter information)
# 1025: 622 -> Projection Parameters Message (Transformation parameter information)
# 1023: 622 -> Representation Residual Message (Transformation parameter information)
# 1013: 623 -> System Parameters (Auxiliary Operation Information)
# 1006: 623 -> Station Coordinates (1)
# 1032: 623 -> Station Coordinates (3)
# 1033: 623 -> Receiver and Antenna Description
# 1008: 623 -> Antenna Description
# 1030: 208 -> GPS Network RTK Residual Message (Network RTK Corrections)
# 1031: 623 -> GLONASS Network RTK Residual Message (Network RTK Corrections)
# 1074: 6227 -> GPS MSM4 (Full GPS Pseudoranges and PhaseRanges plus CNR)
# 1094: 6227 -> Galileo MSM4 (Full GALILEO Pseudoranges and PhaseRanges plus CNR)
# 1124: 6227 -> BeiDou MSM4 (Full BeiDou Pseudoranges and PhaseRanges plus CNR)

# M8P Doesn't support Galileo, but GLONASS -- how do we get that?
# M8P supports and wants 1230 for GLONASS -> Do we need to do anything to get it?
# M8P supported messages:(maybe add a 1033 or 1008 -> 1007 conversion)
# 1001 -> GPS L1 observations
# 1002 -> GPS L1 observations
# 1003 -> GPS L1/L2 observations
# 1004 -> GPS L1/L2 observations
# 1005 -> Station coordinates
# 1006 -> Station coordinates
# 1007 -> Station antenna information
# 1009 -> GLONASS L1 observations
# 1010 -> GLONASS L1 observations
# 1011 -> GLONASS L1/L2 observations
# 1012 -> GLONASS L1/L2 observations
# 1074 -> MSM4 GPS observations
# 1075 -> MSM5 GPS observations
# 1077 -> MSM7 GPS observations
# 1084 -> MSM4 GLONASS observations
# 1085 -> MSM5 GLONASS observations
# 1087 -> MSM7 GLONASS observations
# 1124 -> MSM4 BeiDou observations
# 1125 -> MSM5 BeiDou observations
# 1127 -> MSM7 BeiDou observations
# 1230 -> GLONASS code-phase biases
# 4072 -> Reference station PVT (u-blox proprietary RTCM Message)

verbose = False
#verbose = True


class NtripConnect(Thread):
    """Creating a ntrip connection"""

    def __init__(self, ntc):
        super(NtripConnect, self).__init__()
        if (ntc.ntrip_user is None or ntc.ntrip_user == "<fill in>"):
            raise Exception("no user data given")
        self.ntc = ntc
        self.stop = False
        self.sock = None
        self.seenMsgsDict = {}

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

    def gga_worker(self):
        """Sending out gga strings regularly"""
        sleep(10)  # every 10 seconds send gga string
        while not self.stop:
            gga = self.generate_gga_string()
            try:
                self.sock.sendall(bytes(gga, "ascii"))  # ("utf-8"))
            except Exception as exept:
                print(F"Sending GGA data excepted with {exept}")
                raise exept

            if verbose:
                rospy.loginfo("GGA data sent")
                rospy.loginfo("seen messages thus far: ")
                for i in self.seenMsgsDict:
                    rospy.loginfo(F"{i}: {self.seenMsgsDict[i]}")

            sleep(10)  # every 10 seconds send gga string

    # Function actually doing stuff
    def run(self):
        header = F"GET /{self.ntc.ntrip_stream} HTTP/1.1\r\n" +\
            F"Host: {self.ntc.ntrip_server}\r\n" +\
            "Ntrip-Version: Ntrip/2.0\r\n" +\
            "User-Agent: NTRIP ntrip_wd\r\n" +\
            F"Authorization: Basic {b64encode((self.ntc.ntrip_user + ':' + str(self.ntc.ntrip_pass)).encode('ascii')).decode('ascii')}\r\n" +\
            "Connection: close\r\n\r\n"

        restart_count = 0

        while not self.stop:
            try:
                gga = self.generate_gga_string()
                # rospy.loginfo(gga)

                hostname = self.ntc.ntrip_server.split(':')[0]
                port = self.ntc.ntrip_server.split(':')[1]

                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

                try:
                    self.sock.connect((hostname, int(port)))
                except Exception as excep:
                    print(F"Connect failed with {excep}")
                    raise

                if verbose:
                    rospy.loginfo("Connection established")

                self.sock.sendall(header.encode('ascii'))
                if verbose:
                    rospy.loginfo("Request sent")

                try:
                    # get all the bytes, make it to string
                    response = self.sock.recv(4096)
                except Exception as excep:
                    print(F"Getresponse excepted with {excep}")
                    raise excep

                response_lines = response.decode('utf-8').split("\r\n")
                if verbose:
                    print("Initial response:")
                for line in response_lines:
                    if verbose:
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
                    self.sock.sendall(bytes(gga, "ascii"))  # ("utf-8"))
                except Exception as exept:
                    print(F"Sending GGA data excepted with {exept}")
                    raise exept

                if verbose:
                    rospy.loginfo("GGA data sent")

                    # continue sending GGA data
                gga_sender_thread = Thread(target=self.gga_worker)
                gga_sender_thread.start()

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

                    # rospy.loginfo("Reading response now:")
                    # This now separates individual RTCM messages and publishes each one on the same topic
                    data = self.sock.recv(1)

                    if len(data) != 0:
                        # 211 (==0xD3==0b11010011) is the RTCM3 preamble byte
                        # *256 is left-shift by 8
                        # /16 is right-shift by 4
                        if data[0] == 211:
                            buf += data
                            # it is followed by 6 bits of reserved space and
                            # 10 bits of length information (thus 2 bytes):
                            data = self.sock.recv(2)
                            buf += data
                            cnt = data[0] * 256 + data[1]
                            # then follows the actual packet, first with the message number as a uint12:
                            data = self.sock.recv(2)
                            buf += data
                            msg_type = math.floor(
                                (data[0] * 256 + data[1]) / 16)
                            # rospy.loginfo(F"{cnt}, {int(typ)}")
                            # rospy.loginfo(F"Message type: {int(msg_type)}")
                            if msg_type in self.seenMsgsDict:
                                self.seenMsgsDict[msg_type] += 1
                            else:
                                self.seenMsgsDict[msg_type] = 1
                            cnt = cnt + 1
                            for x in range(cnt):
                                data = self.sock.recv(1)
                                buf += data
                            rmsg.data = buf
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
                        self.sock.close()
                        buf = bytes()
                        raise socket.timeout()  # misuse this to get into the outer loop

                self.sock.close()
                self.sock = None

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

        self.ntrip_server = rospy.get_param('~ntrip_server', None)
        self.ntrip_user = rospy.get_param('~ntrip_user', None)
        self.ntrip_pass = rospy.get_param('~ntrip_pass', None)
        self.ntrip_stream = rospy.get_param('~ntrip_stream', None)

        if verbose:
            rospy.loginfo("Waiting for hub/state to know current position")
        hub_state = rospy.wait_for_message(
            topic="hub/state", topic_type=HubState)

        self.latitude = hub_state.position.latitude
        self.longitude = hub_state.position.longitude
        self.altitude = hub_state.position.altitude
        # self.latitude = 53.156192
        # self.longitude = 8.164861
        # self.altitude = 0
        if verbose:
            rospy.loginfo("Got position")

        self.pub = rospy.Publisher(self.rtcm_topic, RTCM, queue_size=10)

        self.provide_service = rospy.Service(
            'provide_rtcm', ProvideRTCM, self.handle_provide_rtcm)

        self.connection = None

    def handle_provide_rtcm(self, req):
        """Handles starting and stopping of RTCM sending """
        if req.on:
            if self.connection is None or self.connection.stop:
                self.connection = NtripConnect(self)
                self.connection.start()
                rospy.loginfo("Providing rtcm started")
                return {"error_code": 0, "error_message": "ok"}
            else:
                return {"error_code": 1, "error_message": "RTCM is already running"}
        else:
            if self.connection is not None and not self.connection.stop:
                self.connection.stop = True
                rospy.loginfo("Providing rtcm stopped")
                return {"error_code": 0, "error_message": "ok"}
            else:
                return {"error_code": 1, "error_message": "RTCM is not running, can't stop"}

    def run(self):
        """ run this class """
        rospy.spin()
        if self.connection is not None:
            self.connection.stop = True


if __name__ == '__main__':
    c = NtripClient()
    c.run()
