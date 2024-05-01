import numpy as np
from PPP_stuffing import *
import socket
import serial
from serial.tools import list_ports


class SerialForwarder:
    def __init__(self, receiver_port=5006, baudrate=921600, destination_addr=("127.0.0.1", 5010), hardload_destination=False):
        self.baudrate = baudrate
        self.recv_port = receiver_port
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_sock.settimeout(0.0)
        self.server_address = ('0.0.0.0',self.recv_port)
        
        self.destination_addr = destination_addr    #dummy address
        self.hardload_dest = hardload_destination                #if false, we pipe serial data to whoever targeted our IP and port most recently

        self.num_writes = 0
        self.num_reads = 0


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
                    self.ser = serial.Serial(serialport[0], self.baudrate, timeout=0, write_timeout=0)
                    print("connected!")
                    break
                except:
                    print("Connect failed.")
        if not serialport:
            print("no port found")
            raise NameError("No Serial Port Found")
    def __del__(self):
        print("CLosing out with", self.num_writes, "writes and", self.num_reads, "reads")
        if(self.num_writes != 0):
            print("Ratio:", self.num_reads/self.num_writes)
        self.server_sock.close()
        self.ser.close()

    def run(self):
        stuff_buffer = np.zeros(512, dtype=np.uint8)
        bidx = 0
        while(True):
            try:
                pkt,source = self.server_sock.recvfrom(512)
                if(len(pkt) != 0):
                    self.ser.write(pkt)
                    self.num_writes = self.num_writes + 1
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
                    pld, stuff_buffer, bidx, pld_valid = unstuff_PPP_stream_Cstyle_fast(b, stuff_buffer, bidx)
                    if(pld_valid == True and len(pld) != 0):    #valid zero length pld is possible, so need both conditions
                        self.server_sock.sendto(pld, self.destination_addr)
                        self.num_reads = self.num_reads + 1
                    # payload, stuff_buffer = unstuff_PPP_stream(b, stuff_buffer)   #NOTE: Proper usage of this function requires NOT erasing stuff_buffer at the end
                    # if(len(payload)!=0):
                    #     self.server_sock.sendto(payload, self.destination_addr)
                    #     self.num_reads = self.num_reads + 1
