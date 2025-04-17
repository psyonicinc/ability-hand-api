# C++ Ability Hand Wrapper

This wrapper is a synchronous implementation that can send position, velocity,
current and duty messages to the hand and parse it's feedback.  See main.cpp
for how to use.

Given the non-blocking/sleeping nature of how this code handles reads and writes
it is best to use a high as possible baudrate.

Common baudrates are

    0x0F: 250000
    0x10: 460800
    0x11: 921600
    0x12: 1000000 (RS485 Only)

To change the baud-rate on the hand to 921600 enter the following using the 
Developer terminal via the PSYONIC app.  

```Wp001:2:11```

Or to reset it back to the default 460800:  

```Wp001:2:10```

You will also have to modify the *BAUD_RATE* const argument in linux_serial.cpp

# Linux Build Instructions

```
mkdir build && cmake ..
make
./main
```


