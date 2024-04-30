import socket
import numpy as np
from AbilityHandClient import *
import time

abh_api_client = AbilityHandClient()
print("beginning test")

try:
	while(True):
		for i in range(0, len(abh_api_client.tPos)):
			ft = time.time()*3 + i*(2*np.pi)/12
			abh_api_client.tPos[i] = (.5*np.sin(ft)+.5)*45+15
		abh_api_client.tPos[5] = -abh_api_client.tPos[5]

		abh_api_client.writePos()
		abh_api_client.read()

		print(abh_api_client.rPos)

		time.sleep(0.01)
except KeyboardInterrupt:
	print("stopping")
	abh_api_client.soc.close()
