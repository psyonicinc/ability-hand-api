from AbilityHandSerialClient import *
import time

def bytearray_to_hex(byte_arr):
    """Converts a bytearray to a hexadecimal string prefixed with 0x, ensuring two characters per byte."""
    hex_str = ''.join(f'{b:02X}' for b in byte_arr)  # Ensure two characters per byte
    return "0x" + hex_str

def write_uart_register(addr, val):
	barr = write_register(0x50, int(addr), int(val))
	stuffed_payload = PPP_stuff(bytearray(barr))
	try:
		abh.ser.write(stuffed_payload)
		got = False
		st = time.time()
		rv = []
		while(time.time() - st < 0.5 and got == False):
			with abh.readlock:
				if(len(abh.unstuffed_buffer) != 0):
					# print("0x"+ hex(abh.unstuffed_buffer[3])[2:].zfill(2) + hex(abh.unstuffed_buffer[2])[2:].zfill(2) + hex(abh.unstuffed_buffer[1])[2:].zfill(2) + hex(abh.unstuffed_buffer[0])[2:].zfill(2) )
					rv = abh.unstuffed_buffer
					abh.unstuffed_buffer = bytearray([])
					got=True
	except:
		print("Failed due to something being wrong with serial")
	return rv

def read_uart_register(addr):
	barr = read_register(0x50, int(addr))
	stuffed_payload = PPP_stuff(bytearray(barr))
	try:
		abh.ser.write(stuffed_payload)
		got = False
		st = time.time()
		rv = []
		while(time.time() - st < 0.5 and got == False):
			with abh.readlock:
				if(len(abh.unstuffed_buffer) != 0):	
					rv = abh.unstuffed_buffer
					abh.unstuffed_buffer = bytearray([])
					got = True
					return rv
		abh.continue_reading = 0
	except:
		print("Failed due to something being wrong with serial")
	return rv

abh = AbilityHandSerialClient(baudrate=460800)
abh.reply_mode = 2  # 1, 2, or 3
abh.create_read_thread()

index = [3000, 14, 1, 18, 1000, 14, 1, 1773, 4, 3546]
middle = [3, 4, 1, 21, 0, 10, 1, 1773, 4, 3546]
ring = [3, 4, 1, 21, 0, 10, 1, 1773, 4, 3546]
pinky = [3000, 14, 1, 18, 1000, 14, 1, 1773, 4, 3546]
thumbflexor = [3, 4, 1, 21, 0, 10, 1, 1773, 4, 3546]
thumbrotator = [500, 0, 5, 3, 14, 6, 1, 1773, 8, 3546]
gains = [index,middle,ring,pinky,thumbflexor,thumbrotator]

#read 
size = 10   #number of elements per finger
channels = [3, 5]
for channel in channels:
	for enumval in range(0,10):
		addr = channel*size+enumval+1
		
		rv = read_uart_register(channel*size+enumval+1)
		v = struct.unpack("<i", bytearray(rv[4:8]))[0]
		print("Initial value:", v)

		rc = write_uart_register(addr, gains[channel][enumval])
		v = struct.unpack("<i", bytearray(rc[0:4]))[0]
		if(v != 0x4e11fea4):
			print("failed to write")
		# else:
		# 	print("Wrote addr", hex(addr), "success")

		rv = read_uart_register(channel*size+enumval+1)
		v = struct.unpack("<i", bytearray(rv[4:8]))[0]
		print("Final value:", v)


#read the 64 bit unique id as two 32bit words from the hand over UART
rv = read_uart_register(61)
# print(bytearray_to_hex(rv))
v = struct.unpack("<i", bytearray(rv[4:8]))[0]
print("reconstructed value:", hex(v))
rv = read_uart_register(62)
# print(bytearray_to_hex(rv))
v = struct.unpack("<i", bytearray(rv[4:8]))[0]
print("reconstructed value:", hex(v))


abh.close()
