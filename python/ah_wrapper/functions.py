import struct
from typing import List

DEGREES_CONSTANT = 32767 / 150
DEGREES_CONSTANT_INV = 150 / 32767
VELOCITY_CONSTANT = 32767 / 3000
VELOCITY_CONSTANT_INV = 3000 / 32767
LIMIT = 32767
C = 620.606079
K = 1.49


def deg_to_bytes(degrees: float) -> bytes:
    """Converts a position in degrees to a two byte message"""
    return struct.pack(
        "<h", int(max(min(degrees * DEGREES_CONSTANT, LIMIT), -LIMIT))
    )


def vel_to_bytes(deg_p_s: float) -> bytes:
    """Converts a velocity in degrees per second to a two byte message"""
    return struct.pack(
        "<h", int(max(min(deg_p_s * VELOCITY_CONSTANT, LIMIT), -LIMIT))
    )


def cur_to_bytes(current: float) -> bytes:
    """Converts a current in amps to a two byte message"""
    return struct.pack("<h", int(current * C))


def compute_chksum(array: bytearray | List[bytes | int] | bytes) -> int:
    """Compute checksum on list or bytearray"""
    return -sum(array) & 0xFF


def bytes_to_pos(byte_array: bytearray) -> float:
    """Converts position pair of bytes into a float"""
    return struct.unpack("<h", byte_array)[0] * DEGREES_CONSTANT_INV


def bytes_to_vel(byte_array: bytearray) -> float:
    """Converts a vel pair of bytes into a float"""
    return struct.unpack("<h", byte_array)[0] * VELOCITY_CONSTANT_INV


def bytes_to_cur(byte_array: bytearray) -> float:
    """Converts a current pair of bytes into a float"""
    return struct.unpack("<h", byte_array)[0] / C


def map_value(x, in_min, in_max, out_min, out_max):
    """Converts value in one range to another, enforces out range even if x is not in in range"""
    mapped = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    return max(out_min, min(out_max, mapped))  # Clamp within range
