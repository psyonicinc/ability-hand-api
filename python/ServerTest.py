from SerialForwarder import *

ser = SerialForwarder(baudrate=921600)
try:
    ser.run()
except KeyboardInterrupt:
    print("done.")
