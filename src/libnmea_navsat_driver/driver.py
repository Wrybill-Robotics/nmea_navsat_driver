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


class Ros2NMEADriver(Node):
    def __init__(self):
        super().__init__('nmea_navsat_driver')

        self.fix_pub = self.create_publisher(NavSatFix, 'fix', 10)
        self.vel_pub = self.create_publisher(TwistStamped, 'vel', 10)
        self.heading_pub = self.create_publisher(PointStamped, 'heading', 10)
        self.time_ref_pub = self.create_publisher(TimeReference, 'time_reference', 10)

        self.time_ref_source = self.declare_parameter('time_ref_source', 'gps').value
        self.use_RMC = self.declare_parameter('useRMC', False).value
        self.valid_fix = False

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

        elif nmea_string[0] == '#':
            # self.get_logger().info(f"ByNAV Sentence: {nmea_string}")

            # if not check_bynav_checksum(nmea_string):
            #     self.get_logger().warn("Received a sentence with an invalid checksum. " +
            #                         "Sentence was: %s" % nmea_string)
            #     return False

            parsed_sentence = parser.parse_bynav_sentence(nmea_string)
            self.get_logger().info(f"Parsed Sentence: {parsed_sentence}")
            return True
            # if not parsed_sentence:
            #     self.get_logger().debug("Failed to parse ByNAV sentence. Sentence was: %s" % nmea_string)
            #     return False

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
            
            # store fixed type for HEADING
            self.status_type = fix_type

            if self.status_type==4:
            # self.get_logger().info(f"{self.status_type}")
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
                current_fix.position_covariance[0] = (hdop/100 * self.lon_std_dev) ** 2
                current_fix.position_covariance[4] = (hdop/100 * self.lat_std_dev) ** 2
                current_fix.position_covariance[8] = (hdop/100 * self.alt_std_dev) ** 2  # FIXME

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
        elif 'BESTPOSA' in parsed_sentence:
            data = parsed_sentence['BESTPOSA']
            if data['solution_status'] == 'SOL_COMPUTED':  # Only process if solution is computed
                current_fix.status.status = NavSatStatus.STATUS_FIX
                current_fix.status.service = NavSatStatus.SERVICE_GPS

                current_fix.latitude = data['latitude']
                current_fix.longitude = data['longitude']
                current_fix.altitude = data['altitude']  # Altitude above ellipsoid

                # Position covariance
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

                # Publish the fix message
                self.fix_pub.publish(current_fix)

        # elif 'HEADINGA' in parsed_sentence:
        #     data = parsed_sentence['HEADINGA']
        #     if data['solution_status'] == 'SOL_COMPUTED':  # Only process if solution is computed
        #         current_heading = PointStamped()
        #         current_heading.header.stamp = current_time
        #         current_heading.header.frame_id = frame_id

        #         # Set heading and pitch
        #         current_heading.point.z = data['heading']  # Heading in degrees
        #         current_heading.point.y = data['pitch']  # Pitch in degrees (optional usage)

        #         # Publish the heading message
        #         self.heading_pub.publish(current_heading)

        #         # Log standard deviations for debugging or optional use
        #         heading_std_dev = data.get('heading_std_dev', None)
        #         pitch_std_dev = data.get('pitch_std_dev', None)
        #         if heading_std_dev or pitch_std_dev:
        #             self.get_logger().debug(f"HEADINGA Std Dev: Heading={heading_std_dev}, Pitch={pitch_std_dev}")

        # elif 'HDT' in parsed_sentence:
        #     data = parsed_sentence['HDT']
        #     if data['heading']:
        #         if self.status_type == 4:
        #             current_heading = PointStamped()
        #             current_heading.header.stamp = current_time
        #             current_heading.header.frame_id = frame_id
        #             current_heading.point.z = data['heading']
        #             self.heading_pub.publish(current_heading)
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
