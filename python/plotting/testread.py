from serialtools import *

ser,retv = setupSerial(460800)
write_pl = create_misc_msg(0x50, 0x40)

while(1):
	recieved_response = 0
	print("Writing: ", write_pl)
	ser.write(write_pl)
	while recieved_response == 0:
		if(ser.inWaiting() > 0):
			data = ser.read(ser.inWaiting())
			print("received: ", len(data), "bytes of data")
			# for i in range(0,len(data)):
				# print(hex(data[i]),", ",end='')
			recieved_response = 1
			parse_abh_response(data)
	time.sleep(1)
ser.close()
print("done")