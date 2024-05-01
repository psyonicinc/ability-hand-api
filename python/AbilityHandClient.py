import numpy as np
from PPP_stuffing import *
from abh_api_core import *
import socket
import time


class AbilityHandClient:
    def __init__(self, hand_addr=0x50):
        self.serial_address = hand_addr
        self.soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.soc.settimeout(0.0)
        self.block_read = True
        
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

        self.startTime=time.time()
        self.num_writes = 0
        self.num_reads = 0


    def __readblocking(self):
        read_success = False
        iter = 0
        while(read_success == False):
            try:
                self.read_pkt, src = self.soc.recvfrom(512)
                self.rPos, self.rCurrent, self.rVelocity, self.rFsrs = parse_hand_data(self.read_pkt)
                read_success = True
                self.num_reads = self.num_reads + 1
            except BlockingIOError:
                pass
            except ConnectionResetError:
                pass
            iter = iter+1
            if(iter > 100000):
                break

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
        if(self.block_read == True):
            self.__readblocking()
        else:
            self.__read()

    def writeVelocity(self):
        msg = farr_to_abh_frame(self.serial_address, self.tVelocity*32767/3000, 0x20 + self.reply_mode)
        stuffed_payload = PPP_stuff(bytearray(msg))
        self.soc.sendto(stuffed_payload, self.dest_addr)
        if(self.block_read == True):
            self.__readblocking()
        else:
            self.__read()

    def writeCurrent(self):
        msg = farr_to_abh_frame(self.serial_address, self.tCurrent / self.currentConversionRatio, 0x30 + self.reply_mode)
        stuffed_payload = PPP_stuff(bytearray(msg))
        self.soc.sendto(stuffed_payload, self.dest_addr)
        if(self.block_read == True):
            self.__readblocking()
        else:
            self.__read()

    def writeVoltageDuty(self):
        msg = farr_to_abh_frame(self.serial_address, self.tVoltageDuty*3546, 0x40 + self.reply_mode)
        self.num_writes = self.num_writes + 1
        stuffed_payload = PPP_stuff(bytearray(msg))
        self.soc.sendto(stuffed_payload, self.dest_addr)
        if(self.block_read == True):
            self.__readblocking()
        else:
            self.__read()


    def __del__(self):
        runtime = time.time() - self.startTime
        ratio = ((self.num_reads+1)/self.num_writes)    #the way the software works, we'll always drop 1 read
        ctl_freq_Hz = self.num_reads/runtime
        print(ctl_freq_Hz, "Hz, ratio=", ratio)
        self.soc.close()
    

