import serial
from serial.tools import list_ports
import time
import numpy as np
import struct
import sys
import platform
import math
import keyboard
import argparse


## Search for Serial Port to use
def setupSerial(baud):
	ser = []

	print("Searching for serial ports...")
	com_ports_list = list(list_ports.comports())
	port = ""

	for p in com_ports_list:
		if(p):
			if platform.system() == 'Linux' or platform.system() == 'Darwin':
				if "USB" in p[0]:
					port = p
					print("Found:", port)
					break
			elif platform.system() == 'Windows':
				if "COM" in p[0]:
					port = p
					print("Found:", port)
					break
	if not port:
		print("No port found")
		quit()
		
	try: 
		print("Connecting...")
		ser = serial.Serial(port[0], baud, timeout = 0.02)
		print("Connected!")
	except: 
		print("Failed to Connect!")
		return ser, False
		
	return ser, True
	
	

## Send Miscellanous Command to Ability Hand
def create_misc_msg(cmd):
	barr = []
	barr.append( (struct.pack('<B',0x50))[0] );	#device ID
	cmdsize = np.size(cmd)
	if(cmdsize > 1):
		for i in range(0,cmdsize):
			barr.append( (struct.pack('<B', cmd[i]))[0] );	#command!
	else:
		barr.append(cmd)
	sum = 0
	for b in barr:
		sum = sum + b
	chksum = (-sum) & 0xFF;
	barr.append(chksum)
	return barr
	