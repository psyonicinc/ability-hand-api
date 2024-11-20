from AbilityHandSerialClient import *
import time

abh = AbilityHandSerialClient(baudrate=460800)
abh.reply_mode=2	#1, 2, or 3
abh.create_read_thread()

gripdict = {
	"OPEN": 0,
	"POWER":1,
	"KEY":2,
	"PINCH":3,
	"CHUCK_OK":4,
	"SIGN_OF_THE_HORNS":5,
	"UTILITY":6,
	"MOUSE_GRASP":7,
	"MODE_SWITCH_CLOSE":8,
	"POINT_GRASP":9,
	"RUDE_POINT":10,
	"HOOK_GRASP":11,
	"RELAX":12,
	"SLEEVE_GRASP":13,
	"PEACE_GRASP":14,
	"CHUCK_GRASP":15,
	"HANGLOOSE_GRASP":16,
	"HANDSHAKE":17,
	"PINCH_FIXED_GRASP":18,
	"UGRIP_7":19,
	"UGRIP_8":20,
	"UGRIP_9":21,
	"UGRIP_10":22,
	"UGRIP_11":23,
	"TRIGGER":24,
	"UGRIP_12":25,
	"UGRIP_13":26,
	"UGRIP_14":27,
	"UGRIP_15":28,
	"UGRIP_16":29,
	"UGRIP_17":30,
	"UGRIP_18":31,
	"WAGGLE":32
}

print("beginning test")
try:
	while(True):
		abh.writeGripCommand(gripdict["POWER"], 0xFF)
		time.sleep(1)
		abh.writeGripCommand(gripdict["OPEN"], 0xFF)
		time.sleep(1)


except KeyboardInterrupt:
	abh.close()
	print("stopping")
