from serialtools import *
import time

ser, ret = setupSerial(460800)

while(1):
	print('close')
	ser.write(create_misc_msg([0x1d,0xFF,0xFF]))
	time.sleep(1.5)
	print('open')
	ser.write(create_misc_msg([0x1d,0x00,0xFF]))
	time.sleep(1.5)
