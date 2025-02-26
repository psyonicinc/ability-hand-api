from AbilityHandSerialClient import *
import time

def print_32b_from_bytearray_hack(buf, idx):
	base_bidx = idx * 4
	print("0x"+ hex(buf[base_bidx + 3])[2:].zfill(2) + hex(buf[base_bidx + 2])[2:].zfill(2) + hex(buf[base_bidx + 1])[2:].zfill(2) + hex(buf[base_bidx + 0])[2:].zfill(2) )

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

size = 10
channel = 5	#thumb rotator
enumval = 0
print(write_uart_register(channel*size+enumval, 1000))
print(read_uart_register(channel*size+enumval))



abh.close()