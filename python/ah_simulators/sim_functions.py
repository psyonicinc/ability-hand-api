import random

from ah_wrapper.ppp_stuffing import ppp_stuff

from ah_wrapper.functions import (
    compute_chksum,
    cur_to_bytes,
    deg_to_bytes,
    vel_to_bytes,
)

MODES_AND_VAR = [
    ["POSITION", [0x10, 0x11, 0x12]],
    ["VELOCITY", [0x20, 0x21, 0x22]],
    ["VOLTAGE", [0x40, 0x41, 0x42]],
    ["READ", [0xA0, 0xA1, 0xA2]],
]

FRAME_CHAR = 0x7E
ESC_CHAR = 0x7D
ESC_MASK = 0x20


class GeneratedPacket:
    def __init__(
        self,
        pos=None,
        cur=None,
        vel=None,
        fsr=None,
        reply_mode=None,
        mode=None,
    ):
        if pos is None:
            self.pos = [random.randint(0, 90) for _ in range(6)]
            self.pos[-1] *= -1
        else:
            self.pos = pos

        if cur is None:
            self.cur = [random.random() * 2.5 for _ in range(6)]
        else:
            self.cur = cur

        if vel is None:
            self.vel = [random.randint(0, 90) for _ in range(6)]
        else:
            self.vel = vel

        if fsr is None:
            pass

        if reply_mode is None:
            self.reply_mode = random.randint(0, 2)
        else:
            self.reply_mode = reply_mode

        if mode is None:
            self.mode = random.randint(0, 3)
        else:
            self.mode = mode

        self.packet = ppp_stuff(self.generate_packet())
        self.packet_list = [int(i) for i in self.packet]

    def generate_packet(self) -> bytearray:
        msg = bytearray()
        msg.append(
            MODES_AND_VAR[self.mode][1][self.reply_mode]
        )  # Byte 0 - Format Header

        for i in range(6):  # Handles Bytes 1 - 24
            for b in deg_to_bytes(self.pos[i]):
                msg.append(b)
            if self.reply_mode == 0 or self.reply_mode == 2:
                for b in cur_to_bytes(self.cur[i]):
                    msg.append(b)
            elif self.reply_mode == 1:
                for b in vel_to_bytes(self.vel[i]):
                    msg.append(b)

        if self.reply_mode == 0 or self.reply_mode == 1:
            pass
        elif self.reply_mode == 2:
            for i in range(6):  # Handle velocity bytes 25 - 36
                for b in vel_to_bytes(self.vel[i]):
                    msg.append(b)

        msg.append(0)  # Byte 37 / 70
        msg.append(compute_chksum(msg))  # Byte 38 / 71
        return msg
