import numpy as np
from PPP_stuffing import *
from abh_api_core import *
import time
import threading
import serial
from serial.tools import list_ports



class AbilityHandSerialClient:
    def __init__(self, hand_addr=0x50,baudrate=460800):
        self.serial_address = hand_addr
        self.baudrate = baudrate
        self.tPos = np.array([15.,15.,15.,15.,15.,-15.],dtype=np.float32)
        self.tCurrent = np.array([0,0,0,0,0,0],dtype=np.float32)
        self.tVelocity = np.array([0,0,0,0,0,0],dtype=np.float32)
        self.tVoltageDuty = np.array([0,0,0,0,0,0],dtype=np.float32)     #-1 to 1

        self.rPos = np.array([],dtype=np.float64)
        self.rCurrent = np.array([],dtype=np.float64)
        self.rVelocity = np.array([],dtype=np.float64)
        self.rFsrs = np.array([],dtype=np.float64)

        self.currentConversionRatio = 3300/(5*100*4096)

        self.reply_mode = 1

        self.startTime=time.time()
        self.num_writes = 0
        self.num_reads = 0


        self.continue_reading = False
        self.readlock = threading.Lock()

        com_ports_list = list(list_ports.comports())
        serialport = ""
        for p in com_ports_list:
            if(p):
                serialport = p
                print("Attempting to connect to", serialport)
                try:
                    self.ser = serial.Serial(serialport[0], self.baudrate, timeout=0, write_timeout=1)
                    print("connected!")
                    break
                except:
                    print("Connect failed.")
        if not serialport:
            print("no port found")
            raise NameError("No Serial Port Found")



    def close(self):
        self.continue_reading = False

    def __readloop(self):
        print("Starting read thread")
        stuff_buffer = np.zeros(512, dtype=np.uint8)
        bidx = 0
        while(self.continue_reading == True):
            nb = self.ser.read(512)
            if(len(nb) != 0):
                npbytes = np.frombuffer(nb, np.uint8)
                for b in npbytes:
                    pld, stuff_buffer, bidx, pld_valid = unstuff_PPP_stream_Cstyle_fast(b, stuff_buffer, bidx)
                    if(pld_valid == True and len(pld) != 0):    #valid zero length pld is possible, so need both conditions
                        rPos, rCurrent, rVelocity, rFsrs = parse_hand_data(pld)
                        with self.readlock:
                            self.rPos = rPos
                            self.rCurrent = rCurrent
                            self.rVelocity = rVelocity
                            self.rFsrs = rFsrs
                            self.num_reads = self.num_reads + 1
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
        msg = farr_to_abh_frame(self.serial_address, self.tPos*32767/150, 0x10 + self.reply_mode-1)
        self.num_writes = self.num_writes + 1
        stuffed_payload = PPP_stuff(bytearray(msg))
        self.ser.write(stuffed_payload)

    def writeVelocity(self):
        msg = farr_to_abh_frame(self.serial_address, self.tVelocity*32767/3000, 0x20 + self.reply_mode-1)
        self.num_writes = self.num_writes + 1
        stuffed_payload = PPP_stuff(bytearray(msg))
        self.ser.write(stuffed_payload)

    def writeCurrent(self):
        msg = farr_to_abh_frame(self.serial_address, self.tCurrent / self.currentConversionRatio, 0x30 + self.reply_mode-1)
        self.num_writes = self.num_writes + 1
        stuffed_payload = PPP_stuff(bytearray(msg))
        self.ser.write(stuffed_payload)

    def writeVoltageDuty(self):
        msg = farr_to_abh_frame(self.serial_address, self.tVoltageDuty*3546, 0x40 + self.reply_mode-1)
        self.num_writes = self.num_writes + 1
        stuffed_payload = PPP_stuff(bytearray(msg))
        self.ser.write(stuffed_payload)


    def __del__(self):
        self.continue_reading = False
        runtime = time.time() - self.startTime
        ratio = ((self.num_reads+1)/self.num_writes)    #the way the software works, we'll always drop 1 read
        ctl_freq_Hz = self.num_reads/runtime
        print(ctl_freq_Hz, "Hz, ratio=", ratio)
        self.ser.close()

    

