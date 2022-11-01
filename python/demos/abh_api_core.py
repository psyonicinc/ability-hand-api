import struct

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
def send_grip_cmd(cmd, speed):		
	barr = []
	barr.append( (struct.pack('<B', 0x50))[0] )
	barr.append( (struct.pack('<B', 0x1D))[0] )
	barr.append( (struct.pack('<B', cmd))[0] )
	barr.append( (struct.pack('<B', speed))[0] )
	barr.append(compute_checksum(barr))
	
	return barr


"""
	For V10, conv ratio = 1/mdrv_iq_conv = 620.606060606
"""
def current_to_barr(currents, conv_ratio):
	
	#prepare header
	barr = []
	barr.append( (struct.pack('<B', 0x50))[0] )
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
def farr_to_barr(farr):
	barr = []
	barr.append( (struct.pack('<B',0x50))[0] )	#device ID
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
def farr_to_dposition(farr, tx_option):
	barr = []
	barr.append( (struct.pack('<B',0x50))[0] )	#device ID
	barr.append( (struct.pack('<B',0x10 + tx_option))[0] )	#control mode

	for fp in farr:
		fscaled = fp * 32767 / 150
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
def create_misc_msg(cmd):
	barr = []
	barr.append( (struct.pack('<B',0x50))[0] )	#device ID
	barr.append( (struct.pack('<B', cmd))[0] )	#command!

	barr.append(compute_checksum(barr))
	
	return barr
