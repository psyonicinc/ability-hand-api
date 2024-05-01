from AbilityHandClient import *
import time

abh = AbilityHandClient()
abh.reply_mode=0

print("beginning test")
try:
	while(True):

		for i in range(0, len(abh.tPos)):
			ft = time.time()*3 + i*(2*np.pi)/12
			abh.tPos[i] = (.5*np.sin(ft)+.5)*45+15
		abh.tPos[5] = -abh.tPos[5]

		abh.writePos()
		print(abh.rPos)

		time.sleep(0.01)

except KeyboardInterrupt:
	print("stopping")
	abh.soc.close()

