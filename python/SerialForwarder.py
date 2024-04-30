import numpy as np
from PPP_stuffing import *
from abh_api_core import *
import socket
import serial
from serial.tools import list_ports


class SerialForwarder:
    def __init__(self, receiver_port=5006, baudrate=460800, destination_addr=("127.0.0.1", 5010), hardload_destination=False):
        self.baudrate = baudrate
        self.recv_port = receiver_port
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_sock.settimeout(0.0)
        self.server_address = ('0.0.0.0',self.recv_port)
        
        self.destination_addr = destination_addr    #dummy address
        self.hardload_dest = hardload_destination                #if false, we pipe serial data to whoever targeted our IP and port most recently

        try:
            print("Binding: "+self.server_address[0]+", "+str(self.server_address[1]))
            self.server_sock.bind(self.server_address)
        except:
            print("Bind failed")
            raise

        com_ports_list = list(list_ports.comports())
        serialport = ""
        for p in com_ports_list:
            if(p):
                serialport = p
                print("Attempting to connect to", serialport)
                try:
                    self.ser = serial.Serial(serialport[0], self.baudrate, timeout=0)
                    print("connected!")
                    break
                except:
                    print("Connect failed.")
        if not serialport:
            print("no port found")
            raise NameError("No Serial Port Found")
    def __del__(self):
        print("deleting")
        self.server_sock.close()
        self.ser.close()

    
    def run(self):
        stuff_buffer = np.array([])
        while(True):
            try:
                pkt,source = self.server_sock.recvfrom(512)
                self.ser.write(pkt)
                if(self.hardload_dest == False):
                    self.destination_addr = source
            except BlockingIOError:
                pass
            except ConnectionResetError:
                pass
            
            nb = self.ser.read(512)
            if(len(nb) != 0):
                npbytes = np.frombuffer(nb, np.uint8)
                for b in npbytes:
                    payload, stuff_buffer = unstuff_PPP_stream(b, stuff_buffer)
                    if(len(payload)!=0):
                        self.server_sock.sendto(payload, self.destination_addr)
