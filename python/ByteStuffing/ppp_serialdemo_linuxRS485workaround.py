import serial
from serial.tools import list_ports
from abh_api_core import *
import math
import time
import numpy as np
from PPP_stuffing import *
from sys import platform

""" 
	Find a serial com port.
"""
com_ports_list = list(list_ports.comports())
port = []
slist = []
for p in com_ports_list:
	if(p):
		pstr = ""
		pstr = p
		port.append(pstr)
		print("Found:", pstr)
if not port:
	print("No port found")

for p in port:
	try:
		ser = []
		ser = (serial.Serial(p[0],'460800', timeout = 0, write_timeout = 0))
		slist.append(ser)
		print ("connected!", p)
		# print ("found: ", p)
	except:
		print("failded.")
		pass
print( "found ", len(slist), "ports.")
		
		
def m_print_nparr(arr):
	print('[',end='')
	for i, val in enumerate( arr ):
		if(i < len(arr) -1):
			strv = "{:3.2f}".format(val)
			print(str(val) + str(','), end='')
		else:
			print(str(val), end='')
	print(']',end='')

		

# for s in slist:
	# buf = create_misc_msg(0x50, 0xC2) # cmd to enable upsampling of the thumb rotator
	# print ("writing thumb filter message on com port: ", s)
	# s.write(buf)

fpos = [15., 15., 15., 15., 15., -15.]																									
try:
	rPos = np.array([])
	rI = np.array([])
	rV = np.array([])
	rFSR = np.array([])
	
	stuff_buffer = np.array([])
	bytebuffer = bytes([])
	num_writes = 0
	num_reads = 0
	while 1:
		
		for i in range(0, len(fpos)):
			ft = time.time()*3 + i*(2*np.pi)/12
			fpos[i] = (.5*math.sin(ft)+.5)*45+15
		fpos[5] = -fpos[5]
		
		msg = farr_to_dposition(0x50, fpos, 1)
		slist[0].write(PPP_stuff(bytearray(msg)))
		num_writes = num_writes + 1
		# slist[0].write(msg)
		
		nb = bytes([])
		while(len(nb) == 0):
			nb = slist[0].read(512)	#gigantic read size with nonblocking

		bytebuffer = bytebuffer + nb
		if(len(bytebuffer) != 0): #redundant, but fine to keep
			npbytes = np.frombuffer(bytebuffer, np.uint8)
			for b in npbytes:
				payload, stuff_buffer = unstuff_PPP_stream(b,stuff_buffer)
				if(len(payload) != 0):
					rPos,rI,rV,rFSR = parse_hand_data(payload)
					if( (rPos.size + rI.size + rV.size + rFSR.size) != 0):
						"""If the parser got something, print it out. This is blocking, time consuming, and execution time is not guaranteed, but it is guaranteed to reduce average bandwidth"""
						print("deg=",end='')
						m_print_nparr(np.int16(rPos))	#cast to int to just show finger position to the 'nearest' (floored) degree
						# if(len(rI) != 0):
							# print(", amps=",end='')
							# m_print_nparr(rI)
						# if(len(rV) != 0):
							# print(", rad/sec=",end='')
							# m_print_nparr(rV)
						if(len(rFSR) != 0):
							print(", bin=",end='')
							m_print_nparr(rFSR)
						print('')

						# pstrfm =  ''.join('{:02x}'.format(x) for x in payload)
						# print("Pass: len: " + str(len(payload)) + ", " + pstrfm)

						bytebuffer = bytes([])
						stuff_buffer = np.array([])
						num_reads = num_reads + 1						

					# else:	
					# 	# print("Fail, "+str(len(payload)))
					# 	# pstrfm =  ''.join('{:02x}'.format(x) for x in payload)
					# 	pstrfm =  ''.join('{:02x}'.format(x) for x in bytebuffer)
					# 	# print("Fail: len: " + str(len(bytebuffer)) + ", " + pstrfm)


		if(platform == 'win32'):
			time.sleep(0.001)
		
except KeyboardInterrupt:
	pass
	
print(str(num_writes)+" Writes, "+str(num_reads)+" Reads, "+"Ratio = "+str(num_reads/num_writes))

for s in slist:
	s.close()
