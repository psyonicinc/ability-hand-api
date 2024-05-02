from AbilityHandClient import *
import time

abh = AbilityHandClient()
abh.reply_mode=2
abh.block_read=False
abh.dest_addr = ("192.168.123.180" , 5006)
abh.create_read_thread()

print("beginning test")
try:
	while(True):

		for i in range(0, len(abh.tPos)):
			ft = time.time()*3 + i*(2*np.pi)/12
			abh.tPos[i] = (.5*np.sin(ft)+.5)*45+15
		abh.tPos[5] = -abh.tPos[5]

		abh.writePos()
		with abh.readlock:
			print(abh.rVelocity)

		# time.sleep(0.01)

except KeyboardInterrupt:
	abh.close()
	print("stopping")

