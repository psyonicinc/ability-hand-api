import numpy as np
from PPP_stuffing import *
from abh_api_core import *
import socket
import time
import threading


class AbilityHandClient:
    def __init__(self, hand_addr=0x50, bind_port=8176):
        self.serial_address = hand_addr
        self.soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.soc.bind( ("0.0.0.0", bind_port) )
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

        self.startTime=time.time()
        self.num_writes = 0
        self.num_reads = 0


        self.continue_reading = False
        self.readlock = threading.Lock()

    def close(self):
        self.continue_reading = False

    def __readloop(self):
        print("Starting read thread")
        while(self.continue_reading == True):
            try:
                read_pkt, src = self.soc.recvfrom(512)
                rPos, rCurrent, rVelocity, rFsrs = parse_hand_data(read_pkt)
                with self.readlock:
                    self.rPos = rPos
                    self.rCurrent = rCurrent
                    self.rVelocity = rVelocity
                    self.rFsrs = rFsrs
                    self.num_reads = self.num_reads + 1
            except BlockingIOError:
                pass
            except ConnectionResetError:
                pass
        print("Ending read thread")
    # def __read(self):
    #     try:
    #         self.read_pkt, src = self.soc.recvfrom(512)
    #         self.rPos, self.rCurrent, self.rVelocity, self.rFsrs = parse_hand_data(self.read_pkt)
    #         self.num_reads = self.num_reads + 1
    #     except BlockingIOError:
    #         pass
    #     except ConnectionResetError:
    #         pass

    def create_read_thread(self):
        self.continue_reading = True
        self.readthread = threading.Thread(target=self.__readloop)
        self.readthread.start()

    def writePos(self):
        msg = farr_to_abh_frame(self.serial_address, self.tPos*32767/150, 0x10 + self.reply_mode)
        self.num_writes = self.num_writes + 1
        stuffed_payload = PPP_stuff(bytearray(msg))
        self.soc.sendto(stuffed_payload, self.dest_addr)

    def writeVelocity(self):
        msg = farr_to_abh_frame(self.serial_address, self.tVelocity*32767/3000, 0x20 + self.reply_mode)
        self.num_writes = self.num_writes + 1
        stuffed_payload = PPP_stuff(bytearray(msg))
        self.soc.sendto(stuffed_payload, self.dest_addr)

    def writeCurrent(self):
        msg = farr_to_abh_frame(self.serial_address, self.tCurrent / self.currentConversionRatio, 0x30 + self.reply_mode)
        self.num_writes = self.num_writes + 1
        stuffed_payload = PPP_stuff(bytearray(msg))
        self.soc.sendto(stuffed_payload, self.dest_addr)

    def writeVoltageDuty(self):
        msg = farr_to_abh_frame(self.serial_address, self.tVoltageDuty*3546, 0x40 + self.reply_mode)
        self.num_writes = self.num_writes + 1
        stuffed_payload = PPP_stuff(bytearray(msg))
        self.soc.sendto(stuffed_payload, self.dest_addr)


    def __del__(self):
        self.continue_reading = False
        runtime = time.time() - self.startTime
        ratio = ((self.num_reads+1)/self.num_writes)    #the way the software works, we'll always drop 1 read
        ctl_freq_Hz = self.num_reads/runtime
        print(ctl_freq_Hz, "Hz, ratio=", ratio)
        self.soc.close()

    

