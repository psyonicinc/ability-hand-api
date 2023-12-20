import struct
import numpy as np

def udp_pkt(farr):
	barr = []
	for fp in farr:
		b4 = struct.pack('<f', fp)
		for b in b4:
			barr.append(b)
	b4 = struct.pack('<L',3000)
	for b in b4:
		barr.append(b)
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
def send_grip_cmd(addr, cmd, speed):		
	barr = []
	barr.append( (struct.pack('<B', addr))[0] )
	barr.append( (struct.pack('<B', 0x1D))[0] )
	barr.append( (struct.pack('<B', cmd))[0] )
	barr.append( (struct.pack('<B', speed))[0] )
	barr.append(compute_checksum(barr))
	
	return barr


"""
	For V10, conv ratio = 1/mdrv_iq_conv = 620.606060606
"""
def current_to_barr(addr, currents, conv_ratio):
	
	#prepare header
	barr = []
	barr.append( (struct.pack('<B', addr))[0] )
	barr.append( (struct.pack('<B', 0x30))[0] )
	
	#parse message contents
	for amps in currents:
		vf = amps*conv_ratio
		vi = int(vf)
		b2 = struct.pack('<h', vi)
		for b in b2:
			barr.append(b)
	
	#prepare checksum
	barr.append(compute_checksum(barr))	
	
	return barr



"""
	Sends the array farr (which should have only 6 elements, or the hand won't do anything)
	Byte positions:
		0th: 0x50 
		1st: AD (control mode)
		payload: farr as the payload (4 bytes per value),
		last: checksum
	Must be 27 total bytes for the hand to do anything in response.
"""
def farr_to_barr(addr, farr):
	barr = []
	barr.append( (struct.pack('<B', addr))[0] )	#device ID
	barr.append( (struct.pack('<B',0xAD))[0] )	#control mode
	#following block of code converts fpos into a floating point byte array and 
	#loads it into barr bytewise
	for fp in farr:
		b4 = struct.pack('<f',fp)
		for b in b4:
			barr.append(b)
	
	# last step: calculate the checksum and load it into the final byte
	barr.append(compute_checksum(barr))
	
	return barr

"""
	Test for position control mode
"""
def farr_to_dposition(addr, farr, tx_option):
	barr = []
	barr.append( (struct.pack('<B', addr))[0] )	#device ID
	barr.append( (struct.pack('<B',0x10 + tx_option))[0] )	#control mode

	for fp in farr:
		fscaled = fp * 32767 / 150
		lim = 32767
		fscaled = max(min(fscaled,lim),-lim)
		b2 = struct.pack('<h', int(fscaled))
		for b in b2:
			barr.append(b)

	# last step: calculate the checksum and load it into the final byte
	barr.append(compute_checksum(barr))

	return barr

"""
	Test for voltage control mode
"""
def farr_to_vduty(farr):
	pass

"""
	Test for current control mode
"""
def farr_to_dcurrent(farr):
	pass




"""
	Sends a 3 byte payload.
	0th is device id
	1st is the misc. command
	2nd is the checksum!
"""
def create_misc_msg(addr, cmd):
	barr = []
	barr.append( (struct.pack('<B', addr))[0] )	#device ID
	barr.append( (struct.pack('<B', cmd))[0] )	#command!

	barr.append(compute_checksum(barr))
	
	return barr
	

"""
takes a bytearray argument of de-stuffed hand data and converts it to floating point
data in specified units
"""
def parse_hand_data(buffer):

	positions = np.array([])
	current = np.array([])
	velocity = np.array([])
	fsrs = np.array([])
		
	buf = np.frombuffer(buffer, dtype = np.uint8)
	replyFormat = buf[0]
	reply_variant = np.bitwise_and(replyFormat , 0x0F) + 1
	
	#optional: check if the format header is allowed. Not implemented here but could help
	
	#check size match based on reply variant: rejection method 1
	if reply_variant == 3:
		if(buf.size != 38):
			return positions, current, velocity, fsrs			
	else:
		if(buf.size != 72):
			return positions, current, velocity, fsrs
	
	# validate checksum
	bufsigned = np.int8(buf[0:buf.size-1])
	chk = np.uint8(-np.sum(bufsigned))
	if(chk != buf[buf.size-1]):	
		return positions, current, velocity, fsrs
			
	#checksum and size is correct, so proceed to parsing!
	if(reply_variant == 1 or reply_variant == 2):
		bidx = 1
		positions = np.zeros(6)
		for ch in range(0,6):
			val = bytes(buf[bidx:bidx+2])
			unpacked = struct.unpack('<h', val)[0]
			positions[ch] = (np.float64(unpacked)*150.0)/32767.0
			bidx = bidx + 2
			
			if(reply_variant == 1):
				val = bytes(buf[bidx:bidx+2])
				unpacked = struct.unpack('<h', val)[0]
				current = np.append(current,np.float64(unpacked))
				bidx = bidx + 2
			else:
				val = bytes(buf[bidx:bidx+2])
				unpacked = struct.unpack('<h', val)[0]
				velocity = np.append(velocity, np.float64(unpacked)/4 )
				bidx = bidx + 2
		
		fsrs = np.int16(np.zeros(30))
		## Extract Data two at a time
		for i in range(0, 15):
			dualData = buf[(i*3)+25:((i+1)*3)+25]
			data1 = struct.unpack('<H', dualData[0:2])[0] & 0x0FFF
			data2 = (struct.unpack('<H', dualData[1:3])[0] & 0xFFF0) >> 4
			fsrs[i*2] = np.uint16(data1)
			fsrs[(i*2)+1] = np.uint16(data2)

	
	return positions, current, velocity, fsrs