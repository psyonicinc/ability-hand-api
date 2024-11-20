from AbilityHandSerialClient import *
import time

abh = AbilityHandSerialClient(baudrate=460800)
abh.reply_mode=2	#1, 2, or 3
abh.create_read_thread()

print("beginning test")
try:
	while(True):
		abh.writeGripCommand(0x01, 0xFF)
		time.sleep(1)
		abh.writeGripCommand(0x00, 0xFF)
		time.sleep(1)
		
except KeyboardInterrupt:
	abh.close()
	print("stopping")
