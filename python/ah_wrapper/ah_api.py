import struct
import logging
from typing import List

import config
from ah_wrapper.functions import compute_chksum, map_value
from ah_wrapper.ppp_stuffing import ppp_stuff

DEGREES_CONSTANT = 32767 / 150
DEGREES_CONSTANT_INV = 150 / 32767
VELOCITY_CONSTANT = 32767 / 3000
VELOCITY_CONSTANT_INV = 3000 / 32767
LIMIT = 32767
VOLTAGE_LIMIT = 3546
C = 620.606079

"""All messages created using this API assume the byte stuffing is enabled.  To 
enable issue We46 and We47 commands to the hand"""


def create_misc_msg(cmd, addr: int = 0x50):
    """Create misc message and stuff it"""
    msg = list(struct.pack("<BB", addr, cmd))
    msg.append(compute_chksum(msg))
    msg = ppp_stuff(msg)
    return msg


def create_grip_msg(cmd: int, speed: int = 0xFF, addr: int = 0x50):
    msg = list(struct.pack("<BBBB", addr, 0x1D, cmd, speed))
    msg.append(compute_chksum(msg))
    msg = ppp_stuff(msg)
    return msg


def create_pos_msg(
    reply_mode: int, positions: int | float | List[float], addr: int = 0x50
):
    if type(positions) in (float, int):
        # Allows users to pass a single value rather than a list
        positions = [positions] * 6
        positions[-1] *= -1
    elif type(positions) == list:
        if len(positions) != 6:
            if config.write_log:
                logging.warning("Invalid Position Message")
            return None
    else:
        if config.write_log:
            logging.warning("Invalid Position Message")
        return None
    msg = list(struct.pack("<BB", addr, 0x10 + reply_mode))
    for p in positions:
        p_scaled = max(min(p * DEGREES_CONSTANT, LIMIT), -LIMIT)
        p_bytes = struct.pack("<h", int(p_scaled))
        for b in p_bytes:
            msg.append(b)
    msg.append(compute_chksum(msg))
    msg = ppp_stuff(msg)
    return msg


def create_torque_msg(
    reply_mode: int, currents: int | float | List[float], addr: int = 0x50
):
    if type(currents) in (int, float):
        currents = [currents] * 6
        currents[-1] *= -1
    elif type(currents) == list:
        if len(currents) != 6:
            if config.write_log:
                logging.warning("Invalid Torque Message")
            return None
    else:
        if config.write_log:
            logging.warning("Invalid Torque Message")
        return None
    msg = list(struct.pack("<BB", addr, 0x30 + reply_mode))
    for a in currents:
        a_bytes = struct.pack("<h", int(a * C))
        for b in a_bytes:
            msg.append(b)
    msg.append(compute_chksum(msg))
    msg = ppp_stuff(msg)
    return msg


def create_vel_msg(
    reply_mode: int, velocities: int | float | List[float], addr: int = 0x50
):
    if config.velocity_warning:
        print(
            "WARNING VELOCITY TARGETS UNSTABLE AT THE MOMENT AND MAY CAUSE THUMB OSCILLATIONS, SUGGEST USING POSITION INSTEAD"
        )
        config.velocity_warning = False
    if type(velocities) in (int, float):
        velocities = [velocities] * 6
        velocities[-1] *= -1
    elif type(velocities) == list:
        if len(velocities) != 6:
            if config.write_log:
                logging.warning("Invalid Velocity Command")
            return None
    else:
        if config.write_log:
            logging.warning("Invalid Velocity Command")
        return None
    msg = list(struct.pack("<BB", addr, 0x20 + reply_mode))
    for v in velocities:
        v_bytes = struct.pack("<h", int(v * VELOCITY_CONSTANT))
        for b in v_bytes:
            msg.append(b)
    msg.append(compute_chksum(msg))
    msg = ppp_stuff(msg)
    return msg


def create_duty_msg(
    reply_mode: int, duties: int | float | List[int], addr: int = 0x50
):
    if type(duties) in (int, float):
        duties = [duties] * 6
        duties[-1] *= -1
    elif type(duties) == list:
        if len(duties) != 6:
            if config.write_log:
                logging.warning("Invalid Duty Msg")
            return None
    else:
        if config.write_log:
            logging.warning("Invalid Duty Msg")
        return None
    msg = list(struct.pack("BB", addr, 0x40 + reply_mode))
    for d in duties:
        d_mapped = map_value(d, -100, 100, -VOLTAGE_LIMIT, VOLTAGE_LIMIT)
        d_bytes = struct.pack("<h", int(d_mapped))
        for b in d_bytes:
            msg.append(b)
    msg.append(compute_chksum(msg))
    msg = ppp_stuff(msg)
    return msg


def create_write_reg_msg(val_addr: int, val: int, addr: int = 0x50):
    msg = [(struct.pack("<B", addr))[0], (struct.pack("<B", 0xDE))[0]]
    b2 = struct.pack("<i", val_addr)
    for b in b2:
        msg.append(b)
    b2 = struct.pack("<i", val)
    for b in b2:
        msg.append(b)
    msg.append(compute_chksum(msg))
    msg = ppp_stuff(msg)
    return msg


def create_read_reg_msg(val_addr: int, addr: int = 0x50):
    msg = [(struct.pack("<B", addr))[0], (struct.pack("<B", 0xDA))[0]]
    b2 = struct.pack("<i", val_addr)
    for b in b2:
        msg.append(b)
    msg.append(compute_chksum(msg))
    msg = ppp_stuff(msg)
    return msg
