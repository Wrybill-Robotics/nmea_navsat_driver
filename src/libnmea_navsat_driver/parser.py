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

import re
import time
import calendar
import math
import rclpy

logger = rclpy.logging.get_logger('nmea_navsat_driver')

def safe_float(field):
    try:
        return float(field)
    except ValueError:
        return float('NaN')

def safe_int(field):
    try:
        return int(field)
    except ValueError:
        return 0

def convert_latitude(field):
    return safe_float(field[0:2]) + safe_float(field[2:]) / 60.0


def convert_longitude(field):
    return safe_float(field[0:3]) + safe_float(field[3:]) / 60.0

def convert_time(nmea_utc):
    # Get current time in UTC for date information
    utc_struct = time.gmtime()  # immutable, so cannot modify this one
    utc_list = list(utc_struct)
    # If one of the time fields is empty, return NaN seconds
    if not nmea_utc[0:2] or not nmea_utc[2:4] or not nmea_utc[4:6]:
        return float('NaN')
    else:
        hours = int(nmea_utc[0:2])
        minutes = int(nmea_utc[2:4])
        seconds = int(nmea_utc[4:6])
        utc_list[3] = hours
        utc_list[4] = minutes
        utc_list[5] = seconds
        unix_time = calendar.timegm(tuple(utc_list))
        return unix_time

def convert_status_flag(status_flag):
    if status_flag == "A":
        return True
    elif status_flag == "V":
        return False
    else:
        return False

def convert_knots_to_mps(knots):
    return safe_float(knots) * 0.514444444444

# Need this wrapper because math.radians doesn't auto convert inputs
def convert_deg_to_rads(degs):
    return math.radians(safe_float(degs))

def convert_heading_to_enu(field):
    """Convert the incoming data from the device to ENU format.
    Wrapper is needed because heading data from HDT is in NED format.
    Ros requires ENU compliance of data
    Args:
        field: The field (usually a str) to convert to float.
    Returns:
        The float value of VTG in ENU format.
    """
    if len(field) != 0:
        heading_yaw = float(field)
        # heading_yaw = -heading_yaw
        if heading_yaw > 180.0:
            heading_yaw = heading_yaw - 360

        if heading_yaw < -180:
            heading_yaw = heading_yaw + 360

        return safe_float(heading_yaw)

"""Format for this dictionary is a sentence identifier (e.g. "GGA") as the key, with a
list of tuples where each tuple is a field name, conversion function and index
into the split sentence"""
parse_maps = {
    "GGA": [
        ("fix_type", int, 6),
        ("latitude", convert_latitude, 2),
        ("latitude_direction", str, 3),
        ("longitude", convert_longitude, 4),
        ("longitude_direction", str, 5),
        ("altitude", safe_float, 9),
        ("mean_sea_level", safe_float, 11),
        ("hdop", safe_float, 8),
        ("num_satellites", safe_int, 7),
        ("utc_time", convert_time, 1),
    ],
    "RMC": [
        ("utc_time", convert_time, 1),
        ("fix_valid", convert_status_flag, 2),
        ("latitude", convert_latitude, 3),
        ("latitude_direction", str, 4),
        ("longitude", convert_longitude, 5),
        ("longitude_direction", str, 6),
        ("speed", convert_knots_to_mps, 7),
        ("true_course", convert_deg_to_rads, 8),
    ],
    "GST": [
        ("utc_time", convert_time, 1),
        ("ranges_std_dev", safe_float, 2),
        ("semi_major_ellipse_std_dev", safe_float, 3),
        ("semi_minor_ellipse_std_dev", safe_float, 4),
        ("semi_major_orientation", safe_float, 5),
        ("lat_std_dev", safe_float, 6),
        ("lon_std_dev", safe_float, 7),
        ("alt_std_dev", safe_float, 8),
    ],
    "HDT": [
        ("heading", safe_float, 1),
        # ("heading", convert_heading_to_enu, 1)
    ],
    "BESTPOSA": [
        ("solution_status", str, 1),  # Solution status (e.g., SOL_COMPUTED)
        ("position_type", str, 2),  # Position type (e.g., NARROW_INT)
        ("latitude", safe_float, 3),  # Latitude in degrees
        ("longitude", safe_float, 4),  # Longitude in degrees
        ("altitude", safe_float, 5),  # Altitude above the ellipsoid
        ("undulation", safe_float, 6),  # Geoid separation
        ("datum", str, 6),  # Datum (e.g., WGS84)
        ("lat_std_dev", safe_float, 7),  # Latitude standard deviation
        ("lon_std_dev", safe_float, 8),  # Longitude standard deviation
        ("alt_std_dev", safe_float, 9),  # Altitude standard deviation
        ("covariance", safe_float, 10),  # Position covariance
    ],
    "HEADINGA": [
        ("solution_status", str, 1),  # Solution status (see Table 4-2)
        ("position_type", str, 2),  # Heading status (see Table 4-3)
        ("baseline_length", safe_float, 3),  # Heading baseline length (m)
        ("heading", safe_float, 4),  # Heading (0~360°)
        ("pitch", safe_float, 5),  # Pitch (-90~+90°)
        ("reserved", safe_float, 6),  # Reserved
        ("heading_std_dev", safe_float, 7),  # Heading standard deviation (°)
        ("pitch_std_dev", safe_float, 8),  # Pitch standard deviation (°)
        ("rover_station_id", str, 9),  # Rover station ID
        ("master_station_id", str, 10),  # Master station ID
        ("num_satellites_tracked", int, 11),  # Number of satellites tracked
        ("num_satellites_used_in_heading", int, 12),  # Number of satellites used in heading
        ("num_observations", int, 13),  # Number of satellites above elevation of heading antenna
        ("num_l2_satellites", int, 14),  # Number of L2 satellites above elevation of heading antenna
        ("solution_source", str, 15),  # Solution source (see Table 4-8)
        ("extended_solution_status", int, 16),  # Extended solution status (see Table 4-4)
        ("galileo_beidou_signal_mask", str, 17),  # Galileo and BeiDou signal mask (see Table 4-5)
        # ("gps_glonass_signal_mask", str, 18),  # GPS and GLONASS signal mask (see Table 4-5)
        # ("gps_glonass_mask", str, 19),  # GPS-GLONASS mask
        # ("crc", str, 21),  # 32-bit CRC check code
        # ("terminator", str, 22),  # Message terminator (ASCII only)
    ],
    "VTG": [
        ("true_course", convert_deg_to_rads, 1),
        ("speed", convert_knots_to_mps, 5)
    ]
}

def parse_nmea_sentence(nmea_sentence):
    # Check for a valid nmea sentence

    if not re.match(r'(^\$GP|^\$GN|^\$GL|^\$IN).*\*[0-9A-Fa-f]{2}$', nmea_sentence):
        logger.debug("Regex didn't match, sentence not valid NMEA? Sentence was: %s"
                     % repr(nmea_sentence))
        return False
    
    fields = [field.strip(',') for field in nmea_sentence.split(',')]

    # Ignore the $ and talker ID portions (e.g. GP)
    sentence_type = fields[0][3:]

    if sentence_type not in parse_maps:
        logger.debug("Sentence type %s not in parse map, ignoring."
                     % repr(sentence_type))
        return False

    parse_map = parse_maps[sentence_type]

    parsed_sentence = {}
    for entry in parse_map:
        parsed_sentence[entry[0]] = entry[1](fields[entry[2]])

    # logger.info(f"Sentence: {parsed_sentence}")
    return {sentence_type: parsed_sentence}

def parse_bynav_sentence(bynav_sentence):
    # Validate message format (must start with '#' and contain '*XXXXXXXX')
    if not re.match(r'^\#.*\*[0-9A-Fa-f]{8}$', bynav_sentence):
        logger.debug(f"Regex didn't match, sentence not valid ByNAV? Sentence was: {repr(bynav_sentence)}")
        return False

    # Extract sentence body and type
    sentence_body = bynav_sentence[bynav_sentence.find(';') + 1 : bynav_sentence.find('*')]
    sentence_type = bynav_sentence[1 : bynav_sentence.find(',')].strip()
    logger.debug(f"Detected sentence type: {sentence_type}")
    logger.debug(f"Sentence body: {sentence_body}")

    # Split the sentence into fields
    fields = [field.strip() for field in sentence_body.split(',')]
    logger.debug("Fields with indices:")
    # logger.debug(f"Message Type")
    # for i, field in enumerate(fields, start=1):
    #     logger.info(f"  [{i}]: {field}")

    # sentence_type = fields[0]  # ByNAV sentence type follows the '#'
    # sentence_type = bynav_sentence[1:bynav_sentence.find(',')].strip()
    # logger.info(f"Detected sentence type: {sentence_type}")
    
    # logger.info(f"Fields with indices: {[f'ID {i}: {field}' for i, field in enumerate(fields, start=1)]}")


    # logger.info(f"Fields and indices from the sentence: {[(i, field) for i, field in enumerate(fields)]}")

    # Check if the sentence type is supported
    if sentence_type not in parse_maps:
        logger.warning(f"Sentence type {repr(sentence_type)} not in parse map, ignoring.")
        return False

    parse_map = parse_maps[sentence_type]
    # logger.info("Parse map:")
    # for entry in parse_map:
    #     field_name = entry[0]
    #     conversion_func = entry[1]
    #     field_index = entry[2]
    #     logger.info(f"  Field: {field_name}, Converter: {conversion_func}, Index: {field_index}")
    # logger.info(f"Parse map: {repr(parse_map)}")

    
    # Parse fields based on the map
    parsed_sentence = {}
    for entry in parse_map:
        try:
            # parsed_sentence[entry[0]] = entry[1](fields[entry[2]])
            value = fields[entry[2]-1]
            parsed_sentence[entry[0]] = entry[1](value)
            logger.debug(f"Parsing field {entry[0]} (index {entry[2]}): {value} -> {parsed_sentence[entry[0]]}")

        except (IndexError, ValueError) as e:
            logger.warning(f"Failed to parse field {entry[0]} in sentence: {bynav_sentence} - {e}")
            parsed_sentence[entry[0]] = None

    return {sentence_type: parsed_sentence}

