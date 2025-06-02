import struct
import logging

import config
from ah_wrapper.functions import DEGREES_CONSTANT_INV, VELOCITY_CONSTANT_INV, C

V_CONSTANT = 3.3 / 4096.0
C_1 = 121591.0
C_2 = 0.878894


class AbstractPacket:
    """All lists of data used the following data structure
    [index, middle, ring, pinky, thumb flexor, thumb rotator]
    """

    def __init__(self):
        self.valid = False  # If frame was properly parsed this becomes true
        self.pos = [0] * 6
        self.vel = [0] * 6
        self.cur = [0] * 6
        self.fsr = [0] * 30
        self.hot_cold = 0
        self.size_lookup = (71, 71, 38)

    def _convert_pos(self):
        for i in range(len(self.pos)):
            self.pos[i] *= DEGREES_CONSTANT_INV

    def _convert_vel(self):
        for i in range(len(self.vel)):
            self.vel[i] *= VELOCITY_CONSTANT_INV / 4

    def _convert_cur(self):
        for i in range(len(self.cur)):
            self.cur[i] /= C

    def _convert_fsr(self):
        # List comprehension slightly faster
        self.fsr = [
            C_1 / (33000.0 / (D * V_CONSTANT) + 10000) + C_2 if D != 0 else 0
            for D in self.fsr
        ]


class Type1Packet(AbstractPacket):
    def __init__(self, buffer: bytearray):
        super().__init__()

        if len(buffer) == self.size_lookup[0]:
            (
                self.header,
                self.pos[0],
                self.cur[0],
                self.pos[1],
                self.cur[1],
                self.pos[2],
                self.cur[2],
                self.pos[3],
                self.cur[3],
                self.pos[4],
                self.cur[4],
                self.pos[5],
                self.cur[5],
            ) = struct.unpack("<bhhhhhhhhhhhh", buffer[0:25])

            idx = 0
            for i in range(25, 70, 3):
                # Process three bytes and two INT12's at a time
                self.fsr[idx] = (
                    struct.unpack("<H", buffer[i : i + 2])[0] & 0x0FFF
                )
                self.fsr[idx + 1] = (
                    struct.unpack("<H", buffer[i + 1 : i + 3])[0] & 0xFFF0
                ) >> 4
                idx += 2

            self.hot_cold = struct.unpack("<b", buffer[-1:])[0]
            if self.hot_cold:
                if config.write_log:
                    logging.warning(f"Collision Detected: {self.hot_cold}")
            self._convert_pos()
            self._convert_cur()
            self._convert_fsr()
            self.valid = True
        else:
            if config.write_log:
                logging.warning(f"Bad sized Type 1 Frame: {buffer}")


class Type2Packet(AbstractPacket):
    def __init__(self, buffer):
        super().__init__()

        if len(buffer) == self.size_lookup[1]:
            (
                self.header,
                self.pos[0],
                self.vel[0],
                self.pos[1],
                self.vel[1],
                self.pos[2],
                self.vel[2],
                self.pos[3],
                self.vel[3],
                self.pos[4],
                self.vel[4],
                self.pos[5],
                self.vel[5],
            ) = struct.unpack("<bhhhhhhhhhhhh", buffer[0:25])

            idx = 0
            for i in range(25, 70, 3):
                # Process three bytes and two INT12's at a time
                self.fsr[idx] = (
                    struct.unpack("<H", buffer[i : i + 2])[0] & 0x0FFF
                )
                self.fsr[idx + 1] = (
                    struct.unpack("<H", buffer[i + 1 : i + 3])[0] & 0xFFF0
                ) >> 4
                idx += 2

            self.hot_cold = struct.unpack("<b", buffer[-1:])[0]
            if self.hot_cold:
                if config.write_log:
                    logging.warning(f"Collision Detected: {self.hot_cold}")
            self._convert_pos()
            self._convert_vel()
            self._convert_fsr()
            self.valid = True
        else:
            if config.write_log:
                logging.warning(f"Bad sized Type 2 Frame: {buffer}")


class Type3Packet(AbstractPacket):
    def __init__(self, buffer: bytearray):
        super().__init__()
        if len(buffer) == self.size_lookup[2]:
            (
                self.header,
                self.pos[0],
                self.cur[0],
                self.pos[1],
                self.cur[1],
                self.pos[2],
                self.cur[2],
                self.pos[3],
                self.cur[3],
                self.pos[4],
                self.cur[4],
                self.pos[5],
                self.cur[5],
                self.vel[0],
                self.vel[1],
                self.vel[2],
                self.vel[3],
                self.vel[4],
                self.vel[5],
                self.hot_cold,
            ) = struct.unpack("<bhhhhhhhhhhhhhhhhhhb", buffer)
            self._convert_pos()
            self._convert_vel()
            self._convert_cur()
            self.valid = True
            if self.hot_cold:
                if config.write_log:
                    logging.warning(f"Collision Detected: {self.hot_cold}")
        else:
            if config.write_log:
                logging.warning(f"Bad size type 3 frame: {buffer}")


def parse_packet(
    byte_array: bytearray,
) -> None | Type1Packet | Type2Packet | Type3Packet:
    """Takes in un-stuffed frames, checks reply mode and checksum then converts
    to class derived from AbstractPacket"""
    if len(byte_array) == 1:
        return (
            None  # TODO 0x00 is a valid frame that has a reply mode and passes
        )
    reply_mode = byte_array[0] & 0x0F
    if reply_mode not in (0, 1, 2):
        # TODO Maybe check if byte array in one of 12 reply_mode variants
        if config.write_log:
            logging.warning("Unknown packet received" + str(byte_array))
        return None

    checksum = byte_array.pop()
    if checksum != -sum(byte_array) & 0xFF:
        if config.write_log:
            logging.warning("Checksum failed on " + str(byte_array))
            logging.warning(
                f"Anticipated {-sum(byte_array) & 0xFF} Received {checksum}"
            )
        return None

    # Parse packet - chooses class based on reply_mode idx and passes byte array into constructor
    packet = (Type1Packet, Type2Packet, Type3Packet)[reply_mode](byte_array)
    if packet:
        return packet
    else:
        return None
