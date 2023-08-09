import time
import sys
sys.path.insert(0, '../plotting/')
from serialtools import *

ser, ret = setupSerial(460800)

while(1):
	print('close')
	ser.write(create_misc_msg(0x50, [0x1d,0x01,0xFF]))
	time.sleep(1.5)
	print('open')
	ser.write(create_misc_msg(0x50, [0x1d,0x00,0xFF]))
	time.sleep(1.5)
