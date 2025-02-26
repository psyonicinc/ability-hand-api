from AbilityHandSerialClient import *
import time

def print_32b_from_bytearray_hack(buf, idx):
	base_bidx = idx * 4
	print("0x"+ hex(buf[base_bidx + 3])[2:].zfill(2) + hex(buf[base_bidx + 2])[2:].zfill(2) + hex(buf[base_bidx + 1])[2:].zfill(2) + hex(buf[base_bidx + 0])[2:].zfill(2) )

abh = AbilityHandSerialClient(baudrate=460800)
abh.reply_mode = 2  # 1, 2, or 3
abh.create_read_thread()

barr = write_register(0x50, int(1+10*5+0), int(500))
stuffed_payload = PPP_stuff(bytearray(barr))
abh.ser.write(stuffed_payload)
print("write requested")

got = False
st = time.time()
while(time.time() - st < 0.5 and got == False):
	with abh.readlock:
		if(len(abh.unstuffed_buffer) != 0):
			print("0x"+ hex(abh.unstuffed_buffer[3])[2:].zfill(2) + hex(abh.unstuffed_buffer[2])[2:].zfill(2) + hex(abh.unstuffed_buffer[1])[2:].zfill(2) + hex(abh.unstuffed_buffer[0])[2:].zfill(2) )
			abh.unstuffed_buffer = bytearray([])
			got = True


barr = read_register(0x50, int(1+10*5+0))
stuffed_payload = PPP_stuff(bytearray(barr))
abh.ser.write(stuffed_payload)
print("read requested")

got = False
st = time.time()
while(time.time() - st < 0.5 and got == False):
	with abh.readlock:
		if(len(abh.unstuffed_buffer) != 0):
			
			print_32b_from_bytearray_hack(abh.unstuffed_buffer, 0)
			print_32b_from_bytearray_hack(abh.unstuffed_buffer, 1)
			abh.unstuffed_buffer = bytearray([])
			got = True




abh.close()