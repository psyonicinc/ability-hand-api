from AbilityHandClient import *
import time

abh = AbilityHandClient()
abh.reply_mode = 2  # 1,2, or 3
abh.block_read = False
abh.dest_addr = ("192.168.123.164", 34345)
abh.create_read_thread()
abh.soc.sendto(bytearray("activate_hose", encoding="utf8"), abh.dest_addr)

print("beginning test")
try:
    while True:

        for i in range(0, len(abh.tPos)):
            ft = time.time() * 3 + i * (2 * np.pi) / 12
            abh.tPos[i] = (0.5 * np.sin(ft) + 0.5) * 45 + 15
        abh.tPos[5] = -abh.tPos[5]

        abh.writePos()
        with abh.readlock:
            print(abh.rVelocity)

        time.sleep(0.01)

except KeyboardInterrupt:
    abh.close()
    print("stopping")
