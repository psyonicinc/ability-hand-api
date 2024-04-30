import socket
import numpy as np
from AbilityHandClient import *
import time

abh_api_client = AbilityHandClient()
print("beginning test")

try:
	vd = np.array([0,0,0,0,0,0])
	while(True):
		for i in range(0, len(abh_api_client.tPos)):
			ft = time.time()*3 + i*(2*np.pi)/12
			abh_api_client.tPos[i] = (.5*np.sin(ft)+.5)*45+15
		abh_api_client.tPos[5] = -abh_api_client.tPos[5]

		abh_api_client.writePos()
		print(abh_api_client.rPos)

		# time.sleep(1.0)

except KeyboardInterrupt:
	print("stopping")
	abh_api_client.soc.close()
