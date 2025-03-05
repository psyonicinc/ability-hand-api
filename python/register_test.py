from AbilityHandSerialClient import *
import time

def bytearray_to_hex(byte_arr):
    """Converts a bytearray to a hexadecimal string prefixed with 0x, ensuring two characters per byte."""
    hex_str = ''.join(f'{b:02X}' for b in byte_arr)  # Ensure two characters per byte
    return "0x" + hex_str

def write_uart_register(addr, val):
	barr = write_register(0x50, int(addr), int(val))
	stuffed_payload = PPP_stuff(bytearray(barr))
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
	return rv

def read_uart_register(addr):
	barr = read_register(0x50, int(addr))
	stuffed_payload = PPP_stuff(bytearray(barr))
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

abh = AbilityHandSerialClient(baudrate=460800)
abh.reply_mode = 2  # 1, 2, or 3
abh.create_read_thread()


#read 
size = 10   #number of elements per finger
channel = 5	#thumb rotator
enumval = 0 #proportional gain
print(bytearray_to_hex(write_uart_register(channel*size+enumval+1, 1000)))
rv = read_uart_register(channel*size+enumval+1)
print(bytearray_to_hex(rv))
v = struct.unpack("<i", bytearray(rv[4:8]))[0]
print("reconstructed value:", v)


#read the 64 bit unique id as two 32bit words from the hand over UART
rv = read_uart_register(61)
print(bytearray_to_hex(rv))
v = struct.unpack("<i", bytearray(rv[4:8]))[0]
print("reconstructed value:", v)
rv = read_uart_register(62)
print(bytearray_to_hex(rv))
v = struct.unpack("<i", bytearray(rv[4:8]))[0]
print("reconstructed value:", v)


abh.close()
