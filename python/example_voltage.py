from AbilityHandClient import *
import time

abh_api_client = AbilityHandClient()
abh_api_client.reply_mode=0

print("beginning test")
try:
	fpos = np.array([15,15,15,15,15,-15])
	while(True):

		# create position targets
		for i in range(0, len(abh_api_client.tPos)):
			ft = time.time()*3 + i*(2*np.pi)/12
			fpos[i] = (.5*np.sin(ft)+.5)*45+15
		fpos[5] = -fpos[5]


		abh_api_client.writeVoltageDuty()
		if(len(abh_api_client.rPos) != 0):
			abh_api_client.tVoltageDuty = (fpos-abh_api_client.rPos)*0.05
			print(abh_api_client.rPos)
		# time.sleep(0.01)

except KeyboardInterrupt:
	print("stopping")
	abh_api_client.soc.close()
