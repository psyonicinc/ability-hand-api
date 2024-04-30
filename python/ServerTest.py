from SerialForward import *

ser = SerialForwarder()
try:
    ser.run()
except KeyboardInterrupt:
    print("done.")
