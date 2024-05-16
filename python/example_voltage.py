from AbilityHandClient import *
import time

abh = AbilityHandClient()
abh.reply_mode=2	#1, 2, or 3
abh.create_read_thread()

print("beginning test")
try:
	voltage = np.zeros(6)
	while(True):

		for i in range(0, len(voltage)):
			ft = time.time()*3 + i*(2*np.pi)/12
			voltage[i] = 0.4*np.sin(ft)

		abh.writeVoltageDuty()
		with abh.readlock:
			if(len(abh.rPos) != 0):
				print(abh.rPos)
				time.sleep(.0001)
except KeyboardInterrupt:
	abh.close()
	print("stopping")
