import numpy as np
from PPP_stuffing import *
from abh_api_core import *
import socket


class AbilityHandClient:
    def __init__(self, hand_addr=0x50):
        self.serial_address = hand_addr
        self.soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.soc.settimeout(0.0)
        
        self.dest_addr = ('127.0.0.1',5006) #default, change to target external machine running serial server node

        self.tPos = np.array([15.,15.,15.,15.,15.,-15.],dtype=np.float32)
        self.tCurrent = np.array([0,0,0,0,0,0],dtype=np.float32)
        self.tVelocity = np.array([0,0,0,0,0,0],dtype=np.float32)
        self.tVoltageDuty = np.array([0,0,0,0,0,0],dtype=np.float32)     #-1 to 1

        self.rPos = np.array([],dtype=np.float64)
        self.rCurrent = np.array([],dtype=np.float64)
        self.rVelocity = np.array([],dtype=np.float64)
        self.rFsrs = np.array([],dtype=np.float64)

        self.currentConversionRatio = 3300/(5*100*4096)

        self.reply_mode = 0

        self.read_pkt = bytearray([])

    def __read(self):
        try:
            self.read_pkt, src = self.soc.recvfrom(512)
            self.rPos, self.rCurrent, self.rVelocity, self.rFsrs = parse_hand_data(self.read_pkt)
        except BlockingIOError:
            pass
        except ConnectionResetError:
                pass

    def writePos(self):
        msg = farr_to_abh_frame(self.serial_address, self.tPos*32767/150, 0x10 + self.reply_mode)
        stuffed_payload = PPP_stuff(bytearray(msg))
        self.soc.sendto(stuffed_payload, self.dest_addr)
        self.__read()   #note: this reads the data from the PRIOR frame, not the current one

    def writeVelocity(self):
        msg = farr_to_abh_frame(self.serial_address, self.tVelocity*32767/3000, 0x20 + self.reply_mode)
        stuffed_payload = PPP_stuff(bytearray(msg))
        self.soc.sendto(stuffed_payload, self.dest_addr)
        self.__read()

    def writeCurrent(self):
        msg = farr_to_abh_frame(self.serial_address, self.tCurrent / self.currentConversionRatio, 0x30 + self.reply_mode)
        stuffed_payload = PPP_stuff(bytearray(msg))
        self.soc.sendto(stuffed_payload, self.dest_addr)
        self.__read()

    def writeVoltageDuty(self):
        msg = farr_to_abh_frame(self.serial_address, self.tVoltageDuty*3546, 0x40 + self.reply_mode)
        stuffed_payload = PPP_stuff(bytearray(msg))
        self.soc.sendto(stuffed_payload, self.dest_addr)
        self.__read()


    def __del__(self):
        self.soc.close()
    

