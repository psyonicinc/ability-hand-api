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

ser = []
reset_count = 0
total_count = 0
tstart = 0
hand_address = 0x50
reply_mode = 0x10
num_lines = 6
plot_position = False
plot_touch = False


## Search for Serial Port to use
def setupSerial(baud):
	global ser

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
		return False
		
	return True


## Send Miscellanous Command to Ability Hand
def create_misc_msg(cmd):
	barr = []
	barr.append( (struct.pack('<B',0x50))[0] );	#device ID
	barr.append( (struct.pack('<B', cmd))[0] );	#command!
	sum = 0
	for b in barr:
		sum = sum + b
	chksum = (-sum) & 0xFF;
	barr.append(chksum)
	return barr
	

## Generate Message to send to hand from array of positions (floating point)
def generateTX(positions):
	global reply_mode, hand_address
	
	txBuf = []
	
	
	## Address in byte 0
	txBuf.append((struct.pack('<B',hand_address))[0])
	
	## Format Header in byte 1
	txBuf.append((struct.pack('<B',reply_mode))[0])
	
	## Position data for all 6 fingers, scaled to fixed point representation
	for i in range(0,6):
		posFixed = int(positions[i] * 32767 / 150)
		txBuf.append((struct.pack('<B',(posFixed & 0xFF)))[0])
		txBuf.append((struct.pack('<B',(posFixed >> 8) & 0xFF))[0])
	
	## calculate checksum
	cksum = 0
	for b in txBuf:
		cksum = cksum + b
	cksum = (-cksum) & 0xFF
	txBuf.append((struct.pack('<B', cksum))[0])
	
	return txBuf


## Read keyboard input and calculate positions
isFingerWave = False
def calculatePositions(currentPositions, lastCommand):
	global isFingerWave
	global tstart
	
	moveUp = False
	moveDown = False
	## Check keyboard input to see what to do
	## Up/Down only when pressed
	## Finger wave continues when key released
	if keyboard.is_pressed('esc'):
		isFingerWave = False
	elif keyboard.is_pressed('up') or keyboard.is_pressed('w'):
		isFingerWave = False
		moveUp = True
	elif keyboard.is_pressed('down') or keyboard.is_pressed('s'):
		isFingerWave = False
		moveDown = True
	elif keyboard.is_pressed('space'):
		isFingerWave = True
	
	positions = [0.0] * 6
	
	
	for i in range(0,6):
		## Wave fingers in sinusoudal pattern with offset from each other
		if isFingerWave:
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
		## If we aren't doing anything keep it where it is
		else:
			positions[i] = currentPositions[i]
	
	## Invert thumb rotator	
	positions[5] = -positions[5]
	return positions


## Communicate with the hand 
def serialComm():

	global tstart
	global reset_count
	global total_count
	global num_lines
	
	## Upsample the thumb rotator
	msg = create_misc_msg(0xC2)
	ser.write(msg)
	
	
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
	plotData = [0] * (1 + num_lines)
	
	##Clear Buffer to start
	ser.reset_input_buffer()	
	
	while 1:
		t = time.time() - tstart
		
		## Get message with new positions
		posCmd = calculatePositions(posRead, lastPosCmd)
		lastPosCmd = posCmd
		msg = generateTX(posCmd)
		
		## Send Message
		msg = generateTX(posCmd) 
		ser.write(msg)
		
		## Read first response byte - format header
		data = ser.read(1)
		if len(data) == 1:
			replyFormat = data[0]
			## Reply variant 3 length is 
			if (replyFormat & 0xF) == 2:
				replyLen = 38
			else:
				replyLen = 71
			##read the rest of the data
			data = ser.read(replyLen)
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
						posRead[i] = abs(rawData * 150 / 32767)
						
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
			ser.reset_input_buffer()	
			reset_count+=1
			needReset = False
			posRead = prev_posRead.copy()
			touchRead = prev_touchRead.copy()
		
		
		prev_posRead = posRead.copy()
		prev_touchRead = touchRead.copy()
		if num_lines == 6:
			for i in range(0,6):
				plotData[i+1] = posRead[i]
		elif num_lines == 30:
			for i in range(0,30):
				plotData[i+1] = touchRead[i]
		
		total_count +=1
		yield plotData
		


## Print instructions for how to control
def printInstructions():
	print("********************************************")
	print("*     Press Space for Finger Wave          *")
	print("*     Hold UP or W to Open Hand            *")
	print("*     Hold DOWN or S to Close Hand         *")
	print("*     Press Escape to Stop Movement        *")
	print("********************************************")


## Configure for specified plotting
def start_plot(bufWidth, position, touch):
	global plot_position
	global plot_touch
	global reply_mode
	global num_lines
	plot_position = position
	plot_touch = touch
	
	if plot_position:
		num_lines = 6
		reply_mode = 0x12
		plot_floats(num_lines, bufWidth, serialComm, (0,90), (0,30), title="Ability Hand Finger Positions", xlabel="Time(s)", ylabel="Finger Angle (degrees)")
	elif plot_touch:
		num_lines = 30
		reply_mode = 0x10
		plot_floats(num_lines, bufWidth, serialComm, (0,4500), (0,30), title="Ability Hand Touch Sensor Data", xlabel="Time(s)", ylabel="Raw Touch Data")


## Setup and Launch Line plotting
def plot_lines(baud, addr, bufWidth, position, touch):
	global tstart
	global total_count
	global f
	global hand_address
	hand_address = addr
	tstart = time.time()
			
	if setupSerial(baud):
		printInstructions()
		start_plot(bufWidth, position, touch)
		ser.close()		
		print("Completed with " + str(reset_count) + " serial device resets")
		print("Total Runs: " + str(total_count))
		print("Elapsed Time(s): " + str(time.time() - tstart))




## Check the args and run
if __name__ == "__main__":

	##Define all arguments
	
	parser = argparse.ArgumentParser(description='Psyonic Ability Hand API Live Plotting Demo')
	parser.add_argument('-b', '--baud', type=int, help="Serial Baud Rate", default=460800)
	parser.add_argument('-a', '--address', type=int, help='Hand Address', default=0x50)
	parser.add_argument('-w', '--width', type=int, help='X width of Plot', default=500)
	parser.add_argument('--position', help="Plot Position Data", action='store_true')
	parser.add_argument('--touch', help="Plot Touch Sensor Data", action='store_true')
	args=parser.parse_args()
	
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

	
	plot_lines(args.baud, args.address, args.width, pos, touch)  

