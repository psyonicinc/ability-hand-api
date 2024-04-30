import socket
import numpy as np
from PPP_stuffing import *
from abh_api_core import *
import time


client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
client_socket.settimeout(0.0)
fpos=[15.,15.,15.,15.,15.,-15.]
print("beginning test")
try:
	while(True):
		for i in range(0, len(fpos)):
			ft = time.time()*3 + i*(2*np.pi)/12
			fpos[i] = (.5*np.sin(ft)+.5)*45+15
		fpos[5] = -fpos[5]
		
		msg = farr_to_dposition(0x50, fpos, 1)
		stuffed_payload = PPP_stuff(bytearray(msg))

		client_socket.sendto(stuffed_payload, ('127.0.0.1', 5006))

		time.sleep(0.01)
except KeyboardInterrupt:
	print("stopping")
	client_socket.close()
	