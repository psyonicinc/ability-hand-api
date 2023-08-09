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
"""
def check_payload_valid(payload):
	header_list = [
		[0x10, 72, 1],
		[0x11, 72, 2],
		[0x12, 39, 3],
		[0x20, 72, 1],
		[0x21, 72, 2],
		[0x22, 39, 3],
		[0x30, 72, 1],
		[0x31, 72, 2],
		[0x32, 39, 3],
		[0x40, 72, 1],
		[0x41, 72, 2],
		[0x42, 39, 3],
		[0xA0, 72, 1],
		[0xA1, 72, 2],
		[0xA2, 39, 3]
	]
	payload_valid = 0
	reply_variant = -1
	chk = compute_checksum(payload[0:len(payload)-1])
	if(chk == payload[len(payload)-1]):
		# print("checksum match")
		header_idx = -1
		for hidx in range(0,len(header_list)):
			if(payload[0] == header_list[hidx][0]):
				header_idx = hidx
				break
		lenmatch = 0
		if (header_idx != -1):
			# print("header valid")
			if(len(payload) == header_list[header_idx][1]):
				lenmatch = 1
				# print("length match")
		if(lenmatch == 1):
			payload_valid = 1
			reply_variant = header_list[header_idx][2]
	return payload_valid, reply_variant

"""
	Takes as input the payload. depending on size, checksum match, and header, will return:
		all empty: invalid format!
		reply format 1: position, current, fsr will be non-empty, rest empty
		reply format 2: position, rotor velocity, fsr, current empty
		reply format 3: position, current, velocity, fsr empty
"""
def parse_abh_response(payload):
	pdata = []
	current = []
	velocity = []
	fsrdata = []
	is_valid, reply_variant = check_payload_valid(payload)
	if(is_valid):
		if(reply_variant == 1):
			bfinger = payload[1:25]
			mdat = []
			for i in range(0,12):
				barr = bfinger[2*i:2*i+2]
				i16 = struct.unpack('<h',barr)[0]
				mdat.append(i16)
			# mdat = struct.unpack('<h',bfinger)[0]
			pdata = [mdat[0],mdat[2],mdat[4],mdat[6],mdat[8],mdat[10]]
			current = [mdat[1],mdat[3],mdat[5],mdat[7],mdat[9],mdat[11]]
			
		elif(reply_variant == 2):
			bfinger = payload[1:24]
			mdat = []
			for i in range(0,12):
				barr = bfinger[2*i:2*i+2]
				i16 = struct.unpack('<h',barr)[0]
				mdat.append(i16)
			pdata = [mdat[0],mdat[2],mdat[4],mdat[6],mdat[8],mdat[10]]
			velocity = [mdat[1],mdat[3],mdat[5],mdat[7],mdat[9],mdat[11]]
			
		elif(reply_variant == 3):
			bmdat_all = payload[1:36]
			mdat = []
			for i in range(0,18):
				barr = bmdat_all[2*i:2*i+2]
				i16 = struct.unpack('<h',barr)[0]
				mdat.append(i16)
			pdata = [mdat[0],mdat[2],mdat[4],mdat[6],mdat[8],mdat[10]]
			current = [mdat[1],mdat[3],mdat[5],mdat[7],mdat[9],mdat[11]]
			velocity = [mdat[12],mdat[13],mdat[14],mdat[15],mdat[16],mdat[17]]
	
	return pdata, current, velocity, fsrdata
