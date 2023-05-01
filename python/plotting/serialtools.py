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
def create_misc_msg(address, cmd):
	barr = []
	barr.append( (struct.pack('<B',address))[0] );	#device ID
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


"""
"""
def compute_checksum(barr):
	#prepare checksum
	sum = 0
	for b in barr:
		sum = sum + b
	chksum = (-sum) & 0xFF
	return chksum

"""
	Takes as input the payload. depending on size, checksum match, and header, will return:
		all empty: invalid format!
		reply format 1: position, current, fsr will be non-empty, rest empty
		reply format 2: position, rotor velocity, fsr, current empty
		reply format 3: position, current, velocity, fsr empty
"""
def parse_abh_response(payload):
	header_list = [
		[0x10, 72],
		[0x11, 72],
		[0x12, 72],
		[0x20, 72],
		[0x21, 72],
		[0x22, 72],
		[0x30, 72],
		[0x31, 72],
		[0x32, 72],
		[0x40, 72],
		[0x41, 72],
		[0x42, 72],
		[0xA0, 72],
		[0xA1, 72],
		[0xA2, 72]
	]
	
	pdata = []
	current = []
	velocity = []
	fsrdata = []
	chk = compute_checksum(payload[0:len(payload)-1])
	if(chk == payload[len(payload)-1]):
		# print("checksum match")
		header_idx = -1
		for hidx in range(0,len(header_list)):
			if(payload[0] == header_list[hidx][0]):
				header_idx = hidx
				break
		if (header_idx != -1):
			# print("header valid")
			pass
		
		
	return pdata, current, velocity, fsrdata
