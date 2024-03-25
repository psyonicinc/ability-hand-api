import serial
from serial.tools import list_ports
from plot_floats import plot_floats
import time
import numpy as np
import struct
import sys
import platform
from PPP_stuffing import *
from abh_api_core import *
import socket
import json
import threading

class AbilityHand:
    def __init__(self, baudrate=460800, udp_ip="localhost", udp_port_receiver=5006, udp_port_sender=5005, AutoSelect = True, port=None):
        self.ser = []
        self.reset_count = 0
        self.total_count = 0
        self.tstart = 0
        self.hand_address = 0x50
        self.reply_mode = 0x10
        self.num_lines = 6
        self.plot_position = False
        self.plot_touch = False
        self.stuff_data = False
        self.isRS485 = False
        self.isFingerWave = False
        self.Baudrate = baudrate
        self.start_uart_event = threading.Event()
        self.End_Program = threading.Event()
        self.udp_ip = udp_ip
        self.udp_port_receiver = udp_port_receiver
        self.udp_port_sender = udp_port_sender
        self.auto_select = AutoSelect
        self.UARTport = port
        self.CMD = []

    def Send_UDP_Data(self,Message):
            self.sock.sendto(Message,(self.udp_ip, self.udp_port_sender))

    def setup_serial(self, auto_select=True, port=None):
        if auto_select:
            print("Searching for serial ports...")
            com_ports_list = list(list_ports.comports())
            port = ""

            for p in com_ports_list:
                if(p):
                    if platform.system() == 'Linux' or platform.system() == 'Darwin':
                        port = p
                        print("Found:", port)
                        
                    elif platform.system() == 'Windows':
                        if "COM" in p[0]:
                            port = p
                            print("Found:", port)
                    try: 
                        print("Connecting to port:", port)
                        self.ser = serial.Serial(port[0], self.Baudrate, timeout=0)
                        print("Connected!")
                        return True
                    except: 
                        print("Failed to Connect!")
                        
            if not port:
                print("No port found")
                return False
        else:
            if port is None:
                print("No port provided")
                return False
            else:
                try: 
                    print("Connecting to port:", port)
                    self.ser = serial.Serial(port[0], self.Baudrate, timeout=0)
                    print("Connected!")
                    return True
                except: 
                    print("Failed to Connect!")
                    return False
                    
    def create_misc_msg(self, cmd):
        barr = []
        barr.append((struct.pack('<B', 0x50))[0])  # device ID
        barr.append((struct.pack('<B', cmd))[0])  # command!
        sum_val = 0
        for b in barr:
            sum_val = sum_val + b
        checksum = (-sum_val) & 0xFF
        barr.append(checksum)
        return barr

    def generateTX(self):
        
        txBuf = []
        
        
        ## Address in byte 0
        # print(self.hand_address)
        try:
            txBuf.append((struct.pack('<B',self.hand_address))[0])
            
            ## Format Header in byte 1
            txBuf.append((struct.pack('<B',self.reply_mode))[0])
            reply_variant = np.bitwise_and(self.reply_mode , 0xF0)
            if(reply_variant != 0xA0):
            ## Position data for all 6 fingers, scaled to fixed point representation
                for i in range(0,6):
                    posFixed = int(self.CMD[i+2] * 32767 / 150)
                    txBuf.append((struct.pack('<B',(posFixed & 0xFF)))[0])
                    txBuf.append((struct.pack('<B',(posFixed >> 8) & 0xFF))[0])
            
            ## calculate checksum
            cksum = 0
            for b in txBuf:
                cksum = cksum + b
            cksum = (-cksum) & 0xFF
            txBuf.append((struct.pack('<B', cksum))[0])
        except:
            print("Error parcing")

        return txBuf

    # def calculate_positions(self, current_positions, last_command):
    #     moveUp = False
    #     moveDown = False
    #     if keyboard.is_pressed('esc'):
    #         self.isFingerWave = False
    #     elif keyboard.is_pressed('up') or keyboard.is_pressed('w'):
    #         self.isFingerWave = False
    #         moveUp = True
    #     elif keyboard.is_pressed('down') or keyboard.is_pressed('s'):
    #         self.isFingerWave = False
    #         moveDown = True
    #     elif keyboard.is_pressed('space'):
    #         self.isFingerWave = True
        
    #     positions = [0.0] * 6
        
    #     for i in range(0, 6):
    #         if self.isFingerWave:
    #             ft = time.time() * 3 + i
    #             positions[i] = (0.5 * math.sin(ft) + 0.5) * 45 + 15
    #         elif moveUp:
    #             positions[i] = abs(last_command[i]) - 0.5
    #             if positions[i] < 5:
    #                 positions[i] = 5
    #         elif moveDown:
    #             positions[i] = abs(last_command[i]) + 0.5
    #             if positions[i] > 90:
    #                 positions[i] = 90
    #             if i == 4 or i == 5:
    #                 if positions[i] > 50:
    #                     positions[i] = 50
    #         else:
    #             positions[i] = current_positions[i]
        
    #     positions[5] = -positions[5]  # Invert thumb rotator
    #     return positions

    def serialComm(self):

        ## Upsample the thumb rotator
        msg = create_misc_msg(0x50,0xC2)
        self.ser.write(msg)
        ## Resused arrays for position
	    ## Safe "last position" for hand to start at
        posRead = [15] * 6
        Var1 = [0] * 6
        Var2 = [0] * 6
        posRead[5] = -posRead[5]
        prev_posRead = posRead.copy()
        lastPosCmd = posRead.copy()
        rPos = 0
        rI = 0
        rV = 0
        rFSR = 0

        ## Reused Arrays for touch data
        touchRead = [0] * 30
        prev_touchRead = touchRead.copy()

        ## [0] is time, then data
        plotData = [0] * (1 + self.num_lines)

        ##Clear Buffer to start
        self.ser.reset_input_buffer()    
        stuff_buffer = np.array([])
        bytebuffer = bytes([])
        num_writes = 0
        num_reads = 0
        while 1:
            if(self.End_Program.is_set()):
                self.ser.close()
                break
            if(self.start_uart_event.is_set()):
                self.start_uart_event.clear()
                t = time.time() - self.tstart
                ## Get message with new positions
                # posCmd = self.calculate_positions(posRead, lastPosCmd)
                # # print(posCmd) 
                # lastPosCmd = posCmd
                ## Send Message
                msg = self.generateTX()
                reply_variant = np.bitwise_and(self.CMD[1] , 0x0F) + 1
                # print(msg)
                # msg = self.CMD
                
                if(self.stuff_data):
                    self.ser.write(PPP_stuff(bytearray(msg)))
                    num_writes = num_writes + 1

                    nb = bytes([])
                    while(len(nb) == 0):
                        nb = self.ser.read(512)

                    plotData[0] = t
                    
                    bytebuffer = bytebuffer + nb
                    if(len(bytebuffer) != 0):
                        npbytes = np.frombuffer(bytebuffer, np.uint8)
                        for b in npbytes:
                            payload, stuff_buffer = unstuff_PPP_stream(b,stuff_buffer)
                            if(len(payload) != 0):
                                rPos,rI0,rV0,rFSR = parse_hand_data(payload)
                                
                                if( (rPos.size + rI0.size + rV0.size + rFSR.size) != 0):
                                    posRead = rPos.copy()
                                    touchRead = rFSR.copy()
                                    rI = rI0
                                    rV = rV0
                                    bytebuffer = bytes([])
                                    stuff_buffer = np.array([])
                                    num_reads = num_reads + 1
                                    # print(rI0)
                        
                    self.reset_count = num_writes - num_reads #this global is like an error counter, so we'll do it like this for stuffing method

                    prev_posRead = posRead.copy()
                    prev_touchRead = touchRead.copy()
                    
                    self.total_count +=1
                    UDP_Data = []
                    UDP_Data.append(posRead)  
                    
                    if(reply_variant == 1):
                        UDP_Data.append(rI)
                        UDP_Data.append(touchRead)
                    if(reply_variant == 2):
                        UDP_Data.append(rV)
                        UDP_Data.append(touchRead)
                    if(reply_variant == 3):
                        UDP_Data.append(rI)
                        UDP_Data.append(rV) 
                    # print(UDP_Data)
                    UDP_Data_cleaned = [item.tolist() if isinstance(item, np.ndarray) else item for item in UDP_Data]
                    self.Send_UDP_Data(json.dumps(UDP_Data_cleaned).encode('utf-8'))            #TODO: change json.dumps to struct.pack and unpack for faster transmission
                    # yield plotData
                
                else: #old/original, unstuffed protocol. Terrible performance on traditional operating systems
                    # print(msg)
                    self.ser.write(msg)

                    if(self.isRS485 == False):
                        ## Read first response byte - format header
                        data = self.ser.read(1)
                        if len(data) == 1:
                            replyFormat = data[0]
                            ## Reply variant 3 length is 
                            if (replyFormat & 0xF) == 2:
                                replyLen = 38
                            else:
                                replyLen = 71
                            ##read the rest of the data
                            data = self.ser.read(replyLen)
                            plotData[0] = t
                            needReset = False
                            if len(data) == replyLen:
                                ## Verify Checksum	
                                sum = replyFormat
                                for byte in data: 
                                    sum = (sum + byte)%256
                                
                                if sum != 0:
                                    needReset = True
                                else:
                                    ## Extract Position Data
                                    ## Position Data is included in all formats in same way
                                    ## So we can safely do this no matter the format
                                    for i in range(0, 6):
                                        rawData = struct.unpack('<h', data[i*4:2+(i*4)])[0]
                                        posRead[i] = rawData * 150 / 32767
                                        ## Bad data, reset serial device - probably framing error
                                        if posRead[i] > 150:
                                            needReset = True
                                    for i in range(0, 6):
                                        rawData = struct.unpack('<h', data[2+(i*4):4+(i*4)])[0]
                                        if(reply_variant == 1 or reply_variant ==3):
                                            Var1[i] = rawData *3000/32767
                                        if(reply_variant == 2):
                                            Var1[i] = rawData /4
                                        ## Bad data, reset serial device - probably framing error
                                        if Var1[i] > 2050:
                                            needReset = True
                                    ## Extract Touch Data if Available
                                    if replyLen == 71:
                                        ## Extract Data two at a time
                                        for i in range(0, 15):
                                            dualData = data[(i*3)+24:((i+1)*3)+24]
                                            data1 = struct.unpack('<H', dualData[0:2])[0] & 0x0FFF
                                            data2 = (struct.unpack('<H', dualData[1:3])[0] & 0xFFF0) >> 4
                                            touchRead[i*2] = int(data1)
                                            touchRead[(i*2)+1] = int(data2)
                                            if data1 > 4096 or data2 > 4096:
                                                needReset = True
                                    else:
                                        for i in range(0, 6):
                                            rawData = struct.unpack('<h', data[24+i*4:26+i*4])[0]
                                            Var2[i] = rawData 
                                            ## Bad data, reset serial device - probably framing error
                                            if Var2[i] > 2050:
                                                needReset = True
                                                
                            else:
                                needReset = True
                        else: 
                            needReset = True
                            
                        if needReset:
                            self.ser.reset_input_buffer()    
                            self.reset_count+=1
                            needReset = False
                            posRead = prev_posRead.copy()
                            touchRead = prev_touchRead.copy()
                        
                        prev_posRead = posRead.copy()
                        prev_touchRead = touchRead.copy()
                        
                        self.total_count +=1
                        UDP_Data = []
                        UDP_Data.append(posRead) 
                        if(reply_variant == 1):
                            UDP_Data.append(Var1)
                            UDP_Data.append(touchRead)
                        if(reply_variant == 2):
                            UDP_Data.append(Var2)
                            UDP_Data.append(touchRead)
                        if(reply_variant == 3):
                            UDP_Data.append(Var1)
                            UDP_Data.append(Var2) 
                         
                        self.Send_UDP_Data(json.dumps(UDP_Data).encode('utf-8'))
                    else:
                        if 1 == 1:
                            data = self.ser.read(73)
                            if(len(data) > 1):
                                data = data[1:len(data)]
                                replyFormat = data[0]
                                data = data[1:len(data)]
                            
                            plotData[0] = t
                            replyLen = 71
                            needReset = False
                            if len(data) == 71:
                                ## Verify Checksum
                                sum = replyFormat
                                for byte in data: 
                                    sum = (sum + byte)%256
                                
                                if sum != 0:
                                    needReset = True
                                else:
                                    ## Extract Position Data
                                    ## Position Data is included in all formats in same way
                                    ## So we can safely do this no matter the format
                                    for i in range(0, 6):
                                        rawData = struct.unpack('<h', data[i*4:2+(i*4)])[0]
                                        posRead[i] = rawData * 150 / 32767
                                        ## Bad data, reset serial device - probably framing error
                                        if posRead[i] > 150:
                                            needReset = True
                                    for i in range(0, 6):
                                        rawData = struct.unpack('<h', data[2+(i*4):4+(i*4)])[0]
                                        if(reply_variant == 1 or reply_variant ==3):
                                            Var1[i] = rawData *3000/32767
                                        if(reply_variant == 2):
                                            Var1[i] = rawData /4
                                        ## Bad data, reset serial device - probably framing error
                                        if Var1[i] > 2050:
                                            needReset = True
                                    ## Extract Touch Data if Available
                                    if replyLen == 71:
                                        ## Extract Data two at a time
                                        for i in range(0, 15):
                                            dualData = data[(i*3)+24:((i+1)*3)+24]
                                            data1 = struct.unpack('<H', dualData[0:2])[0] & 0x0FFF
                                            data2 = (struct.unpack('<H', dualData[1:3])[0] & 0xFFF0) >> 4
                                            touchRead[i*2] = int(data1)
                                            touchRead[(i*2)+1] = int(data2)
                                            if data1 > 4096 or data2 > 4096:
                                                needReset = True
                                    else:
                                        for i in range(0, 6):
                                            rawData = struct.unpack('<h', data[24+i*4:26+i*4])[0]
                                            Var2[i] = rawData 
                                            ## Bad data, reset serial device - probably framing error
                                            if Var2[i] > 2050:
                                                needReset = True
                            else:
                                needReset = True
                            
                            if needReset:
                                self.ser.reset_input_buffer()    
                                self.reset_count+=1
                                needReset = False
                                posRead = prev_posRead.copy()
                                touchRead = prev_touchRead.copy()
                            
                            prev_posRead = posRead.copy()
                            prev_touchRead = touchRead.copy()

                            
                            self.total_count +=1
                            UDP_Data = []
                            UDP_Data.append(posRead) 
                            if(reply_variant == 1):
                                UDP_Data.append(Var1)
                                UDP_Data.append(touchRead)
                            if(reply_variant == 2):
                                UDP_Data.append(Var1)
                                UDP_Data.append(touchRead)
                                
                            if(reply_variant == 3):
                                UDP_Data.append(Var1)
                                UDP_Data.append(Var2)    
                            self.Send_UDP_Data(json.dumps(UDP_Data).encode('utf-8'))


    def print_instructions(self):
        print("********************************************")
        print("*     Press Space for Finger Wave          *")
        print("*     Hold UP or W to Open Hand            *")
        print("*     Hold DOWN or S to Close Hand         *")
        print("*     Press Escape to Stop Movement        *")
        print("********************************************")

    def print_stuffing_warning(self):
        print("\r\n\r\n")
        print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        print("!                 !!!!!!!!!WARNING!!!!!!!!!                   !")
        print("!         You are running without the --stuff option          !")
        print("! Stuffing is required for optimal performance of this script !")
        print("!     To enable byte stuffing, send We46 and We47             !")
        print("!     to the hand over BLE, using the developer console       !")
        print("!     in the Ability Hand App, then re-run with --stuff       !")
        print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        print("\r\n\r\n")

    def udp_data_receiver(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port_receiver))
        while True:
            data, addr = self.sock.recvfrom(1024)
            received_array = json.loads(data.decode('utf-8'))  
            checksum = sum(received_array[:-1])
            checksum = (-checksum) & 0xFF
            if checksum == received_array[-1]:
                # print(received_array) 
                if len(received_array) == 5:    #this lenght is for config                         
                    if received_array[-2] == 0:
                        self.sock.close()
                        self.End_Program.set()
                        self.start_uart_event.set()
                        break
                    else:
                        if received_array[-2] == 1: #it happen only once per execution, the GUI gets blocked and doesnt allow to send this message again
                            self.isRS485 = received_array[0]
                            self.Baudrate = int(received_array[1])
                            self.stuff_data = received_array[2]
                            self.hand_address = int(received_array[3])
                            self.start_uart_event.set()
                elif len(received_array) == 9: #this lenght is for the hand data [addr,header,index,ring,middle,pinky,thumnb_Flex,thumb_rot]
                    self.reply_mode = int(received_array[1])
                    self.hand_address = int(received_array[0])
                    self.CMD = received_array
                    self.start_uart_event.set()

    def run(self):
        
        T1  = threading.Thread(target=self.udp_data_receiver) 
        T1.setDaemon(True)
        T1.start()
        self.start_uart_event.wait()
        self.start_uart_event.clear()
        if(self.stuff_data == False):
            self.print_stuffing_warning()
        if(self.End_Program.is_set()):
            print("Exit Program")
            sys.exit()            
        elif (self.setup_serial(self.auto_select,self.UARTport)):
            self.print_instructions()
            self.serialComm()
        else:
            self.End_Program.set()
            print("Com not found, exit program")
            sys.exit()
                 

if __name__ == '__main__':
    ability_hand1 = AbilityHand(460800,"localhost",5006,5005,True)
    ability_hand1.run()
