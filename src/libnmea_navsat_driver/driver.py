# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Eric Perko
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import math

import rclpy

from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference
from geometry_msgs.msg import TwistStamped, QuaternionStamped, PointStamped
from sensor_msgs.msg import Imu
from tf_transformations import quaternion_from_euler 
from libnmea_navsat_driver.checksum_utils import check_nmea_checksum, check_bynav_checksum
from libnmea_navsat_driver import parser
from rtcm_msgs.msg import Message
from threading import Thread
# import socket

NMEA_NO_FIX = 0           # Fix not valid
NMEA_GPS_FIX = 1          # GPS fix
NMEA_DGPS_FIX = 2         # Differential GPS fix (DGNSS), SBAS, OmniSTAR VBS, Beacon, RTX in GVBS mode
NMEA_NOT_APPLICABLE = 3   # Not applicable
NMEA_RTK_FIXED = 4        # RTK Fixed, xFill
NMEA_RTK_FLOAT = 5        # RTK Float, OmniSTAR XP/HP, Location RTK, RTX
NMEA_INS_DR = 6           # INS Dead reckoning

class Ros2NMEADriver(Node):
    def __init__(self):
        super().__init__('nmea_navsat_driver')

        self.fix_pub = self.create_publisher(NavSatFix, 'fix', 10)
        self.vel_pub = self.create_publisher(TwistStamped, 'vel', 10)
        self.heading_pub = self.create_publisher(PointStamped, 'heading', 10)
        self.imu_pub = self.create_publisher(Imu, 'heading_imu', 10)
        self.time_ref_pub = self.create_publisher(TimeReference, 'time_reference', 10)

        self.time_ref_source = self.declare_parameter('time_ref_source', 'gps').value
        self.use_RMC = self.declare_parameter('useRMC', False).value
        self.valid_fix = False

        # Parameter for RTCM serial port and socket settings
        # self.socket_host = self.declare_parameter('socket_host', 'localhost').value  # RTCM stream host
        # self.socket_port = self.declare_parameter('socket_port', 2789).value  # RTCM stream port

        # Start the socket reader in a separate thread
        # self.socket_thread = Thread(target=self.read_rtcm_socket)
        # self.socket_thread.daemon = True
        # self.socket_thread.start()

        # Subscribe to RTCM topic
        # self.rtcm_sub = self.create_subscription(Message, 'rtcm', self.rtcm_callback, 10)
        # self.rtcm_sub
        
        # epe = estimated position error
        self.default_epe_quality0 = self.declare_parameter('epe_quality0', 1000000).value
        self.default_epe_quality1 = self.declare_parameter('epe_quality1', 4.0).value
        self.default_epe_quality2 = self.declare_parameter('epe_quality2', 0.1).value
        self.default_epe_quality4 = self.declare_parameter('epe_quality4', 0.02).value
        self.default_epe_quality5 = self.declare_parameter('epe_quality5', 4.0).value
        self.default_epe_quality9 = self.declare_parameter('epe_quality9', 3.0).value

        self.using_receiver_epe = False

        self.lon_std_dev = float("nan")
        self.lat_std_dev = float("nan")
        self.alt_std_dev = float("nan")

        """Format for this dictionary is the fix type from a GGA message as the key, with
        each entry containing a tuple consisting of a default estimated
        position error, a NavSatStatus value, and a NavSatFix covariance value."""
        self.gps_qualities = {
            # Unknown
            -1: [
                self.default_epe_quality0,
                NavSatStatus.STATUS_NO_FIX,
                NavSatFix.COVARIANCE_TYPE_UNKNOWN
            ],
            # Invalid
            0: [
                self.default_epe_quality0,
                NavSatStatus.STATUS_NO_FIX,
                NavSatFix.COVARIANCE_TYPE_UNKNOWN
            ],
            # SPS
            1: [
                self.default_epe_quality1,
                NavSatStatus.STATUS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # DGPS
            2: [
                self.default_epe_quality2,
                NavSatStatus.STATUS_SBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # RTK Fix
            4: [
                self.default_epe_quality4,
                NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # RTK Float
            5: [
                self.default_epe_quality5,
                NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # WAAS
            9: [
                self.default_epe_quality9,
                NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ]
        }

    # def rtcm_callback(self, msg):
    #     # self.get_logger().info('I heard: "%s"' % msg.data)
    #     self.get_logger().info(f"Written RTCM data to serial port {msg.message}.")

    # def read_rtcm_socket(self):
    #     # Set up a TCP/UDP socket to read RTCM data
    #     try:
    #         with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    #             s.connect((self.socket_host, self.socket_port))
    #             self.get_logger().info(f"Connected to RTCM stream at {self.socket_host}:{self.socket_port}")

    #             while rclpy.ok():
    #                 data = s.recv(1024)  # Adjust buffer size as needed
    #                 if data:
    #                     self.handle_rtcm_data(data)
    #     except Exception as e:
    #         self.get_logger().error(f"Error reading from RTCM socket: {e}")
    
    # def handle_rtcm_data(self, data):
    #     # Convert the raw data into an RTCM message
    #     try:
    #         rtcm_msg = Message()
    #         rtcm_msg.message = data  # Assuming 'data' is in the correct format (binary)
            
    #         # Write the RTCM message data to the GPS serial port
    #         # self.GPS.write(rtcm_msg.message)
    #         self.get_logger().info("Written RTCM data to serial port.")
    #     except Exception as e:
    #         self.get_logger().error(f"Error handling RTCM data: {e}\n {data}")

    # def rtcm_callback(self, msg):
    #     # self.get_logger().info(f"Written RTCM data to serial port {msg}.")
    #     try:
    #         # Ensure the message contains RTCM data and write to serial
    #         if isinstance(msg, Message):
    #             # Convert RTCM message to byte string
    #             rtcm_data = msg.message
    #             # self.GPS.write(rtcm_data)
    #             self.get_logger().info("Written RTCM data to serial port.")
    #         else:
    #             self.get_logger().info("Ding ding")
    #     except Exception as e:
    #         self.get_logger().error(f"Failed to write RTCM data to serial: {e}")

    
    # Returns True if we successfully did something with the passed in
    # nmea_string
    def add_sentence(self, nmea_string, frame_id, timestamp=None):
        parsed_sentence = ''

        if nmea_string[0] == '$':
            self.get_logger().debug(f"NMEA Sentence: {nmea_string}")
            
            if not check_nmea_checksum(nmea_string):
                self.get_logger().warn("Received a sentence with an invalid checksum. " +
                                    "Sentence was: %s" % nmea_string)
                return False

            parsed_sentence = parser.parse_nmea_sentence(nmea_string)
            if not parsed_sentence:
                self.get_logger().debug("Failed to parse NMEA sentence. Sentence was: %s" % nmea_string)
                return False
            
            # self.get_logger().info(f"Parsing NMEA {parsed_sentence}")

        elif nmea_string[0] == '#':
            # self.get_logger().info(f"Parsing BYNAV")
            parsed_sentence = parser.parse_bynav_sentence(nmea_string)
            
        if timestamp:
            current_time = timestamp
        else:
            current_time = self.get_clock().now().to_msg()

        current_fix = NavSatFix()
        current_fix.header.stamp = current_time
        current_fix.header.frame_id = frame_id

        current_time_ref = TimeReference()
        current_time_ref.header.stamp = current_time
        current_time_ref.header.frame_id = frame_id
        if self.time_ref_source:
            current_time_ref.source = self.time_ref_source
        else:
            current_time_ref.source = frame_id

        if not self.use_RMC and 'GGA' in parsed_sentence:
            current_fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

            data = parsed_sentence['GGA']
            fix_type = data['fix_type']
            if not (fix_type in self.gps_qualities):
                fix_type = -1
            gps_qual = self.gps_qualities[fix_type]
            default_epe = gps_qual[0]
            current_fix.status.status = gps_qual[1]
            current_fix.position_covariance_type = gps_qual[2]
            if current_fix.status.status > 0:
                self.valid_fix = True
            else:
                self.valid_fix = False

            current_fix.status.service = NavSatStatus.SERVICE_GPS
            latitude = data['latitude']
            if data['latitude_direction'] == 'S':
                latitude = -latitude
            current_fix.latitude = latitude

            longitude = data['longitude']
            if data['longitude_direction'] == 'W':
                longitude = -longitude
            current_fix.longitude = longitude

            # Altitude is above ellipsoid, so adjust for mean-sea-level
            altitude = data['altitude'] + data['mean_sea_level']
            current_fix.altitude = altitude

            # use default epe std_dev unless we've received a GST sentence with epes
            if not self.using_receiver_epe or math.isnan(self.lon_std_dev):
                self.lon_std_dev = default_epe
            if not self.using_receiver_epe or math.isnan(self.lat_std_dev):
                self.lat_std_dev = default_epe
            if not self.using_receiver_epe or math.isnan(self.alt_std_dev):
                self.alt_std_dev = default_epe * 2

            hdop = data['hdop']
            current_fix.position_covariance[0] = (hdop * self.lon_std_dev) ** 2
            current_fix.position_covariance[4] = (hdop * self.lat_std_dev) ** 2
            current_fix.position_covariance[8] = (hdop * self.alt_std_dev) ** 2  # FIXME

            self.fix_pub.publish(current_fix)

            if not math.isnan(data['utc_time']):
                current_time_ref.time_ref = rclpy.time.Time(seconds=data['utc_time']).to_msg()
                self.last_valid_fix_time = current_time_ref
                self.time_ref_pub.publish(current_time_ref)

        elif not self.use_RMC and 'VTG' in parsed_sentence:
            data = parsed_sentence['VTG']

            # Only report VTG data when you've received a valid GGA fix as well.
            if self.valid_fix:
                current_vel = TwistStamped()
                current_vel.header.stamp = current_time
                current_vel.header.frame_id = frame_id
                current_vel.twist.linear.x = data['speed'] * math.sin(data['true_course'])
                current_vel.twist.linear.y = data['speed'] * math.cos(data['true_course'])
                self.vel_pub.publish(current_vel)

        elif 'RMC' in parsed_sentence:
            data = parsed_sentence['RMC']

            # Only publish a fix from RMC if the use_RMC flag is set.
            if self.use_RMC:
                if data['fix_valid']:
                    current_fix.status.status = NavSatStatus.STATUS_FIX
                else:
                    current_fix.status.status = NavSatStatus.STATUS_NO_FIX

                current_fix.status.service = NavSatStatus.SERVICE_GPS

                latitude = data['latitude']
                if data['latitude_direction'] == 'S':
                    latitude = -latitude
                current_fix.latitude = latitude

                longitude = data['longitude']
                if data['longitude_direction'] == 'W':
                    longitude = -longitude
                current_fix.longitude = longitude

                current_fix.altitude = float('NaN')
                current_fix.position_covariance_type = \
                    NavSatFix.COVARIANCE_TYPE_UNKNOWN

                self.fix_pub.publish(current_fix)

                if not math.isnan(data['utc_time']):
                    current_time_ref.time_ref = rclpy.time.Time(seconds=data['utc_time']).to_msg()
                    self.time_ref_pub.publish(current_time_ref)

            # Publish velocity from RMC regardless, since GGA doesn't provide it.
            if data['fix_valid']:
                current_vel = TwistStamped()
                current_vel.header.stamp = current_time
                current_vel.header.frame_id = frame_id
                current_vel.twist.linear.x = data['speed'] * math.sin(data['true_course'])
                current_vel.twist.linear.y = data['speed'] * math.cos(data['true_course'])
                self.vel_pub.publish(current_vel)
        elif 'GST' in parsed_sentence:
            data = parsed_sentence['GST']

            # Use receiver-provided error estimate if available
            self.using_receiver_epe = True
            self.lon_std_dev = data['lon_std_dev']
            self.lat_std_dev = data['lat_std_dev']
            self.alt_std_dev = data['alt_std_dev']

        elif 'BESTNAVA' in parsed_sentence:
            try:
                
                current_fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

                data = parsed_sentence['BESTNAVA']
                # self.get_logger().info(f"Pos data: {data}")

                if data['solution_status'] == 'SOL_COMPUTED':  # Only process if solution is computed

                    fix_type = data['position_type']

                    position_type_mapping = {
                        # RTK Fixed
                        'NARROW_INT':               NMEA_RTK_FIXED,
                        'WIDE_INT':                 NMEA_RTK_FIXED,
                        'INS_RTKFIXED':             NMEA_RTK_FIXED,

                        # RTK Float
                        'NARROW_FLOAT':             NMEA_RTK_FLOAT,
                        'L1_FLOAT':                 NMEA_RTK_FLOAT,
                        'IONOFREE_FLOAT':           NMEA_RTK_FLOAT,
                        'INS_RTKFLOAT':             NMEA_RTK_FLOAT,

                        # Single point / Differential GNSS
                        'SINGLE':                   NMEA_GPS_FIX,
                        'DIFF':                     NMEA_GPS_FIX,
                        'PSRDIFF':                  NMEA_GPS_FIX,
                        'WAAS':                     NMEA_GPS_FIX,
                        'INS_PSRDIFF':              NMEA_GPS_FIX,
                        'INS_SBAS':                 NMEA_GPS_FIX,
                        'INS_PSRSP':                NMEA_GPS_FIX,

                        # PPP (Precise Point Positioning)
                        'PPP_CONVERGING':           NMEA_GPS_FIX,
                        'PPP':                      NMEA_GPS_FIX,
                        'PPP_BASIC_CONVERGING':     NMEA_GPS_FIX,
                        'PPP_BASIC':                NMEA_GPS_FIX,
                        'INS_PPP_Converging':       NMEA_GPS_FIX,
                        'INS_PPP':                  NMEA_GPS_FIX,
                        'INS_PPPP_BASIC_Converging':NMEA_GPS_FIX,
                        'INS_PPPP_BASIC':           NMEA_GPS_FIX,

                        # Special States
                        'DOPPLER_VELOCITY':         NMEA_NO_FIX,  # Could be mapped to STATUS_FIX for simplicity
                        'FLOATCONV':                NMEA_NO_FIX,         # Floating carrier phase ambiguity

                        # No fix
                        'NONE':                     NMEA_NO_FIX,           # No solution
                        'FIXEDPOS':                 NMEA_NO_FIX,       # Fixed by a command, not a dynamic fix
                        'FIXEDHEIGHT':              NMEA_NO_FIX,    # Fixed height by a command
                        'PROPAGATED':               NMEA_NO_FIX,     # Propagated without new observations
                        'INS_PSRSP':                NMEA_NO_FIX,      # INS with pseudorange single point

                        # Reserved states or out-of-bound solutions are treated as NO_FIX
                        'RESERVED':                 NMEA_NO_FIX,
                        'OUT_OF_BOUNDS':            NMEA_NO_FIX,
                        'WARNING':                  NMEA_NO_FIX,
                    }

                    # Default to STATUS_NO_FIX if the type is unknown
                    try:
                        current_fix.status.status = position_type_mapping[fix_type]
                        current_fix.status.service = NavSatStatus.SERVICE_GPS
                    except Exception as e:
                        self.get_logger().error(f"Convert fix type failed: {e}")
                    
                    # self.get_logger().info(f"Position types: {position_type_mapping}")
                    # self.get_logger().info(f"BESTPOSA Position Type: {fix_type} = {current_fix.status.status}")

                    # Set latitude, longitude, and altitude
                    current_fix.latitude = data['latitude']
                    current_fix.longitude = data['longitude']
                    current_fix.altitude = data['altitude']  # Altitude above ellipsoid

                    # Handle position covariance
                    lat_std_dev = data.get('lat_std_dev', float('NaN'))
                    lon_std_dev = data.get('lon_std_dev', float('NaN'))
                    alt_std_dev = data.get('alt_std_dev', float('NaN'))

                    if not math.isnan(lat_std_dev) and not math.isnan(lon_std_dev) and not math.isnan(alt_std_dev):
                        current_fix.position_covariance[0] = lat_std_dev ** 2
                        current_fix.position_covariance[4] = lon_std_dev ** 2
                        current_fix.position_covariance[8] = alt_std_dev ** 2
                        current_fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
                    else:
                        current_fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

                    # Log position type and covariance details
                    # self.get_logger().info(f"BESTPOSA Position Type: {fix_type}")
                    # self.get_logger().info(f"Position Covariance: {current_fix.position_covariance}")

                    # Publish the fix message
                    self.fix_pub.publish(current_fix)
            except Exception as e:
                self.get_logger().error(f"BestPosA Message Error found: {e}")

        elif 'UNIHEADINGA' in parsed_sentence:
            data = parsed_sentence['UNIHEADINGA']
            # self.get_logger().info(f"Heading data: {data}")

            if data['heading']:  # Only process if solution is computed
                # if data['position_type'] == 'NARROW_INT':

                    try:
                        current_heading = PointStamped()
                        current_heading.header.stamp = current_time
                        current_heading.header.frame_id = frame_id

                        fix_type = data['position_type']

                        position_type_mapping = {
                            # RTK Fixed
                            'NARROW_INT':               NMEA_RTK_FIXED,
                            'WIDE_INT':                 NMEA_RTK_FIXED,
                            'INS_RTKFIXED':             NMEA_RTK_FIXED,

                            # RTK Float
                            'NARROW_FLOAT':             NMEA_RTK_FLOAT,
                            'L1_FLOAT':                 NMEA_RTK_FLOAT,
                            'IONOFREE_FLOAT':           NMEA_RTK_FLOAT,
                            'INS_RTKFLOAT':             NMEA_RTK_FLOAT,

                            # Single point / Differential GNSS
                            'SINGLE':                   NMEA_GPS_FIX,
                            'DIFF':                     NMEA_GPS_FIX,
                            'PSRDIFF':                  NMEA_GPS_FIX,
                            'WAAS':                     NMEA_GPS_FIX,
                            'INS_PSRDIFF':              NMEA_GPS_FIX,
                            'INS_SBAS':                 NMEA_GPS_FIX,
                            'INS_PSRSP':                NMEA_GPS_FIX,

                            # PPP (Precise Point Positioning)
                            'PPP_CONVERGING':           NMEA_GPS_FIX,
                            'PPP':                      NMEA_GPS_FIX,
                            'PPP_BASIC_CONVERGING':     NMEA_GPS_FIX,
                            'PPP_BASIC':                NMEA_GPS_FIX,
                            'INS_PPP_Converging':       NMEA_GPS_FIX,
                            'INS_PPP':                  NMEA_GPS_FIX,
                            'INS_PPPP_BASIC_Converging':NMEA_GPS_FIX,
                            'INS_PPPP_BASIC':           NMEA_GPS_FIX,

                            # Special States
                            'DOPPLER_VELOCITY':         NMEA_NO_FIX,  # Could be mapped to STATUS_FIX for simplicity
                            'FLOATCONV':                NMEA_NO_FIX,         # Floating carrier phase ambiguity

                            # No fix
                            'NONE':                     NMEA_NO_FIX,           # No solution
                            'FIXEDPOS':                 NMEA_NO_FIX,       # Fixed by a command, not a dynamic fix
                            'FIXEDHEIGHT':              NMEA_NO_FIX,    # Fixed height by a command
                            'PROPAGATED':               NMEA_NO_FIX,     # Propagated without new observations
                            'INS_PSRSP':                NMEA_NO_FIX,      # INS with pseudorange single point

                            # Reserved states or out-of-bound solutions are treated as NO_FIX
                            'RESERVED':                 NMEA_NO_FIX,
                            'OUT_OF_BOUNDS':            NMEA_NO_FIX,
                            'WARNING':                  NMEA_NO_FIX,
                        }

                        # Default to STATUS_NO_FIX if the type is unknown
                        try:
                            fix_status = position_type_mapping[fix_type]
                        except Exception as e:
                            self.get_logger().error(f"Convert fix type failed: {e}")

                        # Set heading and pitch
                        current_heading.point.z = data['heading']  # Heading in degrees
                        current_heading.point.y = data['pitch']  # Pitch in degrees (optional usage)
                        current_heading.point.x = float(fix_status)  # Pitch in degrees (optional usage)

                        # Publish the heading message
                        self.heading_pub.publish(current_heading)

                        # Log standard deviations for debugging or optional use
                        heading_std_dev = data.get('heading_std_dev', None)
                        pitch_std_dev = data.get('pitch_std_dev', None)
                        # if heading_std_dev or pitch_std_dev:
                            # self.get_logger().info(f"HEADINGA Std Dev: Heading={heading_std_dev}, Pitch={pitch_std_dev}")

                        # Publish IMU message
                        imu_message = Imu()
                        imu_message.header.stamp = current_time
                        imu_message.header.frame_id = frame_id

                        # Convert heading (yaw) and pitch to a quaternion
                        heading_radians = math.radians(data['heading'])
                        pitch_radians = math.radians(data['pitch'])
                        roll_radians = 0.0  # Assuming no roll data available

                        # Use tf transformations to convert to quaternion
                        q = quaternion_from_euler(roll_radians, pitch_radians, heading_radians)
                        imu_message.orientation.x = q[0]
                        imu_message.orientation.y = q[1]
                        imu_message.orientation.z = q[2]
                        imu_message.orientation.w = q[3]

                        # Populate orientation covariance if available
                        if heading_std_dev and pitch_std_dev:
                            imu_message.orientation_covariance = [
                                pitch_std_dev ** 2, 0.0, 0.0,
                                0.0, heading_std_dev ** 2, 0.0,
                                0.0, 0.0, float('NaN')  # No roll covariance available
                            ]
                        else:
                            imu_message.orientation_covariance = [
                                float('NaN'), 0.0, 0.0,
                                0.0, float('NaN'), 0.0,
                                0.0, 0.0, float('NaN')
                            ]
                        # Publish the IMU message
                        self.imu_pub.publish(imu_message)
                        # self.get_logger().info(f"Published IMU: Orientation={q}, Covariance={imu_message.orientation_covariance}")
                    except Exception as e:
                        self.get_logger().error(f"Heading Message Error found: {e}")

        elif 'HDT' in parsed_sentence:
            data = parsed_sentence['HDT']
            if data['heading']:
                if self.status_type == 4:
                    current_heading = PointStamped()
                    current_heading.header.stamp = current_time
                    current_heading.header.frame_id = frame_id
                    current_heading.point.z = data['heading']
                    self.heading_pub.publish(current_heading)
                # # old
                # current_heading = QuaternionStamped()
                # current_heading.header.stamp = current_time
                # current_heading.header.frame_id = frame_id
                # q = quaternion_from_euler(0, 0, math.radians(data['heading']))
                # current_heading.quaternion.x = q[0]
                # current_heading.quaternion.y = q[1]
                # current_heading.quaternion.z = q[2]
                # current_heading.quaternion.w = q[3]
                # self.heading_pub.publish(current_heading)
        # elif 'HEADINGA' in parsed_sentence:
        #     print("DING DING")
        else:
            return False
        return True

    """Helper method for getting the frame_id with the correct TF prefix"""
    def get_frame_id(self):
        frame_id = self.declare_parameter('frame_id', 'gps_link').value
        prefix = self.declare_parameter('tf_prefix', '').value
        if len(prefix):
            return '%s/%s' % (prefix, frame_id)
        return frame_id
