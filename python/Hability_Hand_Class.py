import serial
from serial.tools import list_ports
from plot_floats import plot_floats
import time
import numpy as np
import struct
import sys
import platform
import math
import keyboard
import argparse
from PPP_stuffing import *
from abh_api_core import *
import json
import threading
import socket

class Hability_Hand_CLASS:
    def __init__(self, udp_ip="localhost", udp_port_receiver=5006, udp_port_sender=5005, AutoSelect = True, port=None):
        self.ser = []
        self.reset_count = 0
        self.total_count = 0
        self.tstart = 0
        # self.hand_address = 0x50
        # self.reply_mode = 0x10
        self.num_lines = 6
        self.plot_position = False
        self.plot_touch = False
        self.stuff_data = False
        self.isRS485 = False
        self.isFingerWave = False 
        self.Mode = 0
        self.start_uart_event = threading.Event()
        # self.setup_serial(AutoSelect,port)
        self.udp_ip = udp_ip
        self.udp_port_receiver = udp_port_receiver
        self.udp_port_sender = udp_port_sender
        self.udp_thread = threading.Thread(target=self.udp_data_receiver)
        self.udp_thread.setDaemon(True) #so it doesnt get stuck if the program closes abruptly
        self.udp_thread.start()

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
                        break
                    except: 
                        print("Failed to Connect!")
                        sys.exit()
            if not port:
                print("No port found")
                quit()
        else:
            if port is None:
                print("No port provided")
                quit()
            else:
                try: 
                    print("Connecting to port:", port)
                    self.ser = serial.Serial(port[0], self.Baudrate, timeout=0)
                    print("Connected!")
                except: 
                    print("Failed to Connect!")
                    sys.exit()

    def udp_data_receiver(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port_receiver))
        while True:
            data, addr = self.sock.recvfrom(1024)
            received_array = json.loads(data.decode('utf-8'))  
            checksum = sum(received_array[:-1])
            if len(received_array) == 7:
                if checksum == received_array[-1]:
                    print(received_array)    
                    if received_array[-2] == 0:
                        self.sock.close()
                        self.start_uart_event.clear()
                        break
                    else:
                        if received_array[-2] == 1:
                            self.start_uart_event.set()
                            self.isRS485 = received_array[0]
                            self.Baudrate = int(received_array[1])
                            self.stuff_data = received_array[2]
                            self.hand_address = hex(received_array[3])
                        self.Mode = received_array[4]
            elif len(received_array) == 8:
                print(received_array)

    def create_misc_msg(self, cmd):
        barr = []
        barr.append( (struct.pack('<B',0x50))[0] );  #device ID
        barr.append( (struct.pack('<B', cmd))[0] );  #command!
        sum = 0
        for b in barr:
            sum = sum + b
        chksum = (-sum) & 0xFF
        barr.append(chksum)
        return barr

    def generateTX(self,hand_address,reply_mode, positions):
        txBuf = []

        txBuf.append((struct.pack('<B',hand_address))[0])
        txBuf.append((struct.pack('<B',reply_mode))[0])

        for i in range(0,6):
            posFixed = int(positions[i] * 32767 / 150)
            txBuf.append((struct.pack('<B',(posFixed & 0xFF)))[0])
            txBuf.append((struct.pack('<B',(posFixed >> 8) & 0xFF))[0])

        cksum = 0
        for b in txBuf:
            cksum = cksum + b
        cksum = (-cksum) & 0xFF
        txBuf.append((struct.pack('<B', cksum))[0])

        return txBuf

    def calculatePositions(self, currentPositions, lastCommand):
        moveUp = False
        moveDown = False
        ## Check keyboard input to see what to do
        ## Up/Down only when pressed
        ## Finger wave continues when key released
        if keyboard.is_pressed('esc'):
            self.isFingerWave = False
        elif keyboard.is_pressed('up') or keyboard.is_pressed('w'):
            self.isFingerWave = False
            moveUp = True
        elif keyboard.is_pressed('down') or keyboard.is_pressed('s'):
            self.isFingerWave = False
            moveDown = True
        elif keyboard.is_pressed('space'):
            self.isFingerWave = True

        positions = [0.0] * 6

        for i in range(0,6):
            ## Wave fingers in sinusoudal pattern with offset from each other
            if self.isFingerWave:
                ft = time.time() * 3 + i
                positions[i] = (0.5*math.sin(ft) + 0.5) * 45 + 15
            ## Move up/open from current position
            elif moveUp:
                positions[i] = abs(lastCommand[i]) - 0.5
                ## Floor at 5
                if positions[i] < 5:
                    positions[i] = 5
            ## Move down/closed from current position        
            elif moveDown:
                positions[i] = abs(lastCommand[i]) + 0.5
                ## Cap at 90 for fingers, 50 for thumb to avoid collision
                if positions[i] > 90:
                    positions[i] = 90
                if i == 4 or i == 5:
                    if positions[i] > 50:
                        positions[i] = 50
            else:
                ## If we aren't doing anything keep it where it is
                positions[i] = currentPositions[i]
        ## Invert thumb rotator	
        positions[5] = -positions[5]
        return positions

    def serialComm(self):

        ## Upsample the thumb rotator
        msg = create_misc_msg(0xC2)
        self.ser.write(msg)
        ## Resused arrays for position
	    ## Safe "last position" for hand to start at
        posRead = [15] * 6
        posRead[5] = -posRead[5]
        prev_posRead = posRead.copy()
        lastPosCmd = posRead.copy()

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
            t = time.time() - self.tstart
            ## Get message with new positions
            posCmd = self.calculatePositions(posRead, lastPosCmd)
            lastPosCmd = posCmd
            msg = self.generateTX(posCmd)
            ## Send Message
            msg = generateTX(posCmd)
            
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
                            rPos,rI,rV,rFSR = parse_hand_data(payload)
                            if( (rPos.size + rI.size + rV.size + rFSR.size) != 0):
                                posRead = rPos.copy()
                                touchRead = rFSR.copy()
                                
                                bytebuffer = bytes([])
                                stuff_buffer = np.array([])
                                num_reads = num_reads + 1
                    
                self.reset_count = num_writes - num_reads #this global is like an error counter, so we'll do it like this for stuffing method

                prev_posRead = posRead.copy()
                prev_touchRead = touchRead.copy()
                if self.num_lines == 6:
                    for i in range(0,6):
                        plotData[i+1] = posRead[i]
                elif self.num_lines == 30:
                    for i in range(0,30):
                        plotData[i+1] = touchRead[i]
                
                self.total_count +=1
                yield plotData
            
            else: #old/original, unstuffed protocol. Terrible performance on traditional operating systems
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
                    if self.num_lines == 6:
                        for i in range(0,6):
                            plotData[i+1] = posRead[i]
                    elif self.num_lines == 30:
                        for i in range(0,30):
                            plotData[i+1] = touchRead[i]
                    
                    self.total_count +=1
                    yield plotData
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
                            needReset = True
                        
                        if needReset:
                            self.ser.reset_input_buffer()    
                            self.reset_count+=1
                            needReset = False
                            posRead = prev_posRead.copy()
                            touchRead = prev_touchRead.copy()
                        
                        prev_posRead = posRead.copy()
                        prev_touchRead = touchRead.copy()
                        if self.num_lines == 6:
                            for i in range(0,6):
                                plotData[i+1] = posRead[i]
                        elif self.num_lines == 30:
                            for i in range(0,30):
                                plotData[i+1] = touchRead[i]
                        
                        self.total_count +=1
                        yield plotData

    def printInstructions(self):
        print("********************************************")
        print("*     Press Space for Finger Wave          *")
        print("*     Hold UP or W to Open Hand            *")
        print("*     Hold DOWN or S to Close Hand         *")
        print("*     Press Escape to Stop Movement        *")
        print("********************************************")

    def printStuffingWarning(self):
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

    def start_plot(self, bufWidth, position, touch):
        self.plot_position = position
        self.plot_touch = touch

        if self.plot_position:
            self.num_lines = 6
            self.reply_mode = 0x10
            plot_floats(self.num_lines, bufWidth, self.serialComm, (0,90), (0,30), title="Ability Hand Finger Positions", xlabel="Time(s)", ylabel="Finger Angle (degrees)")
        elif self.plot_touch:
            self.num_lines = 30
            self.reply_mode = 0x10
            plot_floats(self.num_lines, bufWidth, self.serialComm, (0,4500), (0,30), title="Ability Hand Touch Sensor Data", xlabel="Time(s)", ylabel="Raw Touch Data")

    def plot_lines(self, baud, addr, bufWidth, position, touch):
        self.hand_address = addr
        self.tstart = time.time()
                    
        if self.setupSerial(baud):
            self.printInstructions()
            self.start_plot(bufWidth, position, touch)
            self.ser.close()        
            print("Completed with " + str(self.reset_count) + " serial device resets")
            print("Total Runs: " + str(self.total_count))
            print("Elapsed Time(s): " + str(time.time() - self.tstart))

    def run(self):
        parser = argparse.ArgumentParser(description='Psyonic Ability Hand API Live Plotting Demo')
        parser.add_argument('-b', '--baud', type=int, help="Serial Baud Rate", default=460800)
        parser.add_argument('-a', '--address', type=int, help='Hand Address', default=0x50)
        parser.add_argument('-w', '--width', type=int, help='X width of Plot', default=500)
        parser.add_argument('--position', help="Plot Position Data", action='store_true')
        parser.add_argument('--touch', help="Plot Touch Sensor Data", action='store_true')
        parser.add_argument('--stuff', help="Enable forward and reverse byte stuffing for much improved reliability and bandwidth. Ensure We46 and We47 on target ability hand are enabled.",action='store_true')
        parser.add_argument('--rs485',help="flag to indicate if an RS485 dongle is used",action='store_true')
        args = parser.parse_args()

        self.stuff_data = args.stuff
        if(self.stuff_data == False):
            self.printStuffingWarning()

        self.isRS485 = args.rs485
        pos = False
        touch = False

        if args.position:
            print("Plotting Position Data")
            pos = True
        elif args.touch:
            print("Plotting Touch Sensor Data")
            touch = True
        else:
            sys.exit("Error: --position or --touch argument required")
        
        print("Baud Rate: " + str(args.baud))
        print("Hand Address: " + str(args.address))
        print("Buffer Width: " + str(args.width))

        self.plot_lines(args.baud, args.address, args.width, pos, touch)

if __name__ == "__main__":
    hability_hand = Hability_Hand_CLASS()
    hability_hand.run()
