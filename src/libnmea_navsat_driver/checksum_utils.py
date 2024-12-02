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


# Check the NMEA sentence checksum. Return True if passes and False if failed
def check_nmea_checksum(nmea_sentence):
    split_sentence = nmea_sentence.split('*')
    if len(split_sentence) != 2:
        # No checksum bytes were found... improperly formatted/incomplete NMEA data?
        return False
    transmitted_checksum = split_sentence[1].strip()

    # Remove the $ at the front
    data_to_checksum = split_sentence[0][1:]
    checksum = 0
    for c in data_to_checksum:
        checksum ^= ord(c)

    return ("%02X" % checksum) == transmitted_checksum.upper()

def compute_crc32(data):
    crc = 0xFFFFFFFF
    for char in data:
        crc ^= ord(char)
        for _ in range(8):
            if (crc & 1):
                crc = (crc >> 1) ^ 0xEDB88320
            else:
                crc >>= 1
    crc = crc ^ 0xFFFFFFFF
    return f"{crc:08x}"

    #define CRC32_POLYNOMIAL 0xEDB88320L
    # Calculate a CRC value
    # value: Value

CRC32_POLYNOMIAL = 0xEDB88320

def calc_crc32_value(value):
    ulCRC = value
    for _ in range(8):  # Process each bit in the byte
        if ulCRC & 1:
            ulCRC = (ulCRC >> 1) ^ CRC32_POLYNOMIAL
        else:
            ulCRC >>= 1
    return ulCRC

def calc_block_crc32(data):
    if isinstance(data, str):
        data = data.encode('ascii')

    ulCRC = 0  # Initialize CRC to 0
    for byte in data:
        ulTmp1 = (ulCRC >> 8) & 0x00FFFFFF
        ulTmp2 = calc_crc32_value((ulCRC ^ byte) & 0xFF)
        ulCRC = ulTmp1 ^ ulTmp2
    return ulCRC

def check_bynav_checksum(bynav_sentence):
    """
    Check the CRC for a ByNAV proprietary sentence.
    ByNAV sentences start with '#' and end with '*XXXXXXXX', where XXXXXXXX is the 32-bit CRC.

    Args:
        bynav_sentence (str): The full ByNAV sentence to validate.

    Returns:
        bool: True if the checksum is valid, False otherwise.
    """
    # Split the sentence into data and checksum parts
    split_sentence = bynav_sentence.split('*')
    if len(split_sentence) != 2:
        # Invalid format (missing '*' or checksum)
        return False

    transmitted_crc = split_sentence[1].strip()

    # Ensure the sentence starts with '#'
    if not bynav_sentence.startswith('#'):
        return False

    # Compute CRC on everything between '#' and '*'
    data_to_crc = bynav_sentence[1:bynav_sentence.find('*')]
    computed_crc = calc_block_crc32(data_to_crc)

    # Compare the computed CRC with the transmitted one
    return computed_crc == transmitted_crc.lower()


# BESTPOSA,COM3,0,92.5,FINESTEERING,2343,94540.400,00000000,0000,781;SOL_COMPUTED,NARROW_INT,-40.37993097965,175.61293101574,28.5369,15.0493,WGS84,0.0156,0.0163,0.0301,"1195",0.400,0.017,31,22,22,22,00,00,30,37*1d21563a
