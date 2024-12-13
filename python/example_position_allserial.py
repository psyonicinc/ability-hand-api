from AbilityHandSerialClient import *
import time

abh = AbilityHandSerialClient(baudrate=460800)
abh.reply_mode=2	#1, 2, or 3
abh.create_read_thread()

print("beginning test")
try:
	fpos = np.array([30,30,30,30,30,-30])
	while(True):


		t = time.time()
		while(time.time() - t < 3):
			fpos = np.array([1,1,1,1,1,-1])*15
			fpos[4] = 50
			fpos[5] = -75
			abh.writePos()
			with abh.readlock:
				if(len(abh.rPos) != 0):
					abh.tPos = fpos
					print(abh.rPos)
					# time.sleep(.0001)

		t = time.time()
		while(time.time() - t < 300000000000):
			fpos = np.array([1,1,1,1,1,-1])*90
			fpos[4] = 50
			fpos[5] = -75
			abh.writePos()
			with abh.readlock:
				if(len(abh.rPos) != 0):
					abh.tPos = fpos
					# print(abh.rPos)
					print(time.time()-t)
					# time.sleep(.0001)

except KeyboardInterrupt:
	abh.close()
	print("stopping")
