from AbilityHandClient import *
import time

abh = AbilityHandClient()
abh.reply_mode=1
abh.block_read=False

print("beginning test")
try:
	fpos = np.array([30,30,30,30,30,-30])
	while(True):

		abh.writeVoltageDuty()
		if(len(abh.rPos) != 0):
			abh.tVoltageDuty = (fpos-abh.rPos)*0.05 - abh.rVelocity*.0001
			print(abh.rVelocity*.0001)	#on windows, may BSOD if there is no delay. print is blocking IO delay with a shorter duration than time.sleep
		# time.sleep(0.01)	

except KeyboardInterrupt:
	print("stopping")
	abh.soc.close()
