from AbilityHandClient import *
import time

abh = AbilityHandClient()
abh.reply_mode=0

print("beginning test")
try:
	fpos = np.array([15,15,15,15,15,-15])
	while(True):

		# create position targets
		for i in range(0, len(abh.tPos)):
			ft = time.time()*3 + i*(2*np.pi)/12
			fpos[i] = (.5*np.sin(ft)+.5)*45+15
		fpos[5] = -fpos[5]


		abh.writeVoltageDuty()
		if(len(abh.rPos) != 0):
			abh.tVoltageDuty = (fpos-abh.rPos)*0.05
			# print(abh.rPos)
		# time.sleep(0.01)

except KeyboardInterrupt:
	print("stopping")
	abh.soc.close()
