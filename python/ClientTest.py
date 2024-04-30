from AbilityHandClient import *
import time

abh_api_client = AbilityHandClient()
abh_api_client.reply_mode=0

print("beginning test")
try:
	while(True):

		for i in range(0, len(abh_api_client.tPos)):
			ft = time.time()*3 + i*(2*np.pi)/12
			abh_api_client.tPos[i] = (.5*np.sin(ft)+.5)*45+15
		abh_api_client.tPos[5] = -abh_api_client.tPos[5]

		abh_api_client.writePos()
		print(abh_api_client.rCurrent*abh_api_client.currentConversionRatio)

		time.sleep(0.01)

except KeyboardInterrupt:
	print("stopping")
	abh_api_client.soc.close()
