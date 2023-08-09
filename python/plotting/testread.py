from serialtools import *

ser,retv = setupSerial(460800)
write_pl = create_misc_msg(0x50, 0x40)

frames = 0
successful_frames = 0
while successful_frames < 1000:
	
	recieved_response = 0
	# print("Writing: ", write_pl)
	ser.write(write_pl)
	while recieved_response == 0:
		if(ser.inWaiting() > 0):
			data = ser.read(ser.inWaiting())
			recieved_response = 1
			pdata, current, velocity, fsrdata = parse_abh_response(data)
			if(len(pdata) > 0):
				print(pdata)
				successful_frames = successful_frames + 1
	time.sleep(0.001)
	frames = frames + 1

ser.close()
print(successful_frames, "successful frames out of ", frames, "total")
print("done")

