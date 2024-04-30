from SerialForwarder import *

ser = SerialForwarder()
try:
    ser.run()
except KeyboardInterrupt:
    print("done.")
