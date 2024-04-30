import numpy as np
from PPP_stuffing import *
from abh_api_core import *
import socket


class AbilityHandClient:
    def __init__(self, hand_addr):
        self.serial_address = hand_addr
        self.soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.soc.settimeout(0.0)
        
        self.dest_addr = ('127.0.0.1',5006)

        self.tPos = np.array([15,15,15,15,15,-15])
        self.tCurrent = np.array([0,0,0,0,0,0])
        self.tVelocity = np.array([0,0,0,0,0,0])
        self.tVoltageDuty = np.array([0,0,0,0,0,0])     #-1 to 1

        self.rPos = np.array([])
        self.rCurrent = np.array([])
        self.rVelocity = np.array([])
        self.rFsrs = np.array([])

        self.currentConversionRatio = 3300/(5*100*4096)

        self.reply_mode = 0

    def writePos(self):
        msg = farr_to_abh_frame(self.serial_address, self.tPos*32767/150, 0x10 + self.reply_mode)
        stuffed_payload = PPP_stuff(bytearray(msg))
        self.soc.sendto(stuffed_payload, self.dest_addr)

    def writeVelocity(self):
        msg = farr_to_abh_frame(self.serial_address, self.tVelocity*32767/3000, 0x20 + self.reply_mode)
        stuffed_payload = PPP_stuff(bytearray(msg))
        self.soc.sendto(stuffed_payload, self.dest_addr)

    def writeCurrent(self):
        msg = farr_to_abh_frame(self.serial_address, self.tCurrent / self.currentConversionRatio, 0x30 + self.reply_mode)
        stuffed_payload = PPP_stuff(bytearray(msg))
        self.soc.sendto(stuffed_payload, self.dest_addr)

    def writeVoltageDuty(self):
        msg = farr_to_abh_frame(self.serial_address, self.tVoltageDuty*3546, 0x40 + self.reply_mode)
        stuffed_payload = PPP_stuff(bytearray(msg))
        self.soc.sendto(stuffed_payload, self.dest_addr)



    def read(self):
        try:
            pkt, src = self.soc.recvfrom(512)
            self.rPos, self.rCurrent, self.rVelocity, self.rFsrs = parse_hand_data(pkt)
        except BlockingIOError:
            pass
        except ConnectionResetError:
                pass

