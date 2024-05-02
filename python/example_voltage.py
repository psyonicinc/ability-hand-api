from AbilityHandClient import *
import time

abh = AbilityHandClient()
abh.reply_mode=2
abh.block_read=False
abh.dest_addr = ("192.168.123.180" , 5006)
abh.create_read_thread()

print("beginning test")
try:
	fpos = np.array([30,30,30,30,30,-30])
	while(True):

		for i in range(0, len(fpos)):
			ft = time.time()*3 + i*(2*np.pi)/12
			fpos[i] = (.5*np.sin(ft)+.5)*45+15
		fpos[5] = -fpos[5]


		abh.writeVoltageDuty()
		with abh.readlock:
			if(len(abh.rPos) != 0):
				abh.tVoltageDuty = (fpos-abh.rPos)*0.05 - abh.rVelocity*.0001

except KeyboardInterrupt:
	abh.close()
	print("stopping")
