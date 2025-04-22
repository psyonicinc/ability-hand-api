# C++ Ability Hand Wrapper

This wrapper is a synchronous single threaded implementation that can send 
position, velocity, current and duty messages to the hand and parse its feedback.  

See [main.cpp](https://github.com/psyonicinc/ability-hand-api/blob/master/cpp/main.cpp) 
for how to use.

### Enable UART & Byte Stuffing using App

The C++ wrapper uses serial communication and requires [byte stuffing](https://www.tutorialspoint.com/data_communication_computer_network/byte_stuffing.htm) 
to be enabled.  It is recommended to use byte stuffing, fore more info see 
section 3.5.5 in the [API Documentation](https://github.com/psyonicinc/ability-hand-api/blob/master/Documentation/ABILITY-HAND-ICD.pdf).  

To enable UART and byte stuffing from the PSYONIC app select:

Scan ➡️ SELECT HAND ➡️ Gear Icon ⚙️
(Top Right) ➡️ Troubleshoot ➡️ Developer Mode

and issue the following commands 
individually.

We16  
We46  
We47

### Increase Baud Rate

Given the non-blocking and non-sleeping nature of how this code handles reads 
and writes it is best to use a high as possible baudrate (921600 recommended).

Common baudrates are on the ability hand are:

    0x0F: 250000
    0x10: 460800
    0x11: 921600
    0x12: 1000000 (RS485 Only)

To change the baud-rate on the hand to 921600 enter the following using the 
Developer terminal via the PSYONIC app.  

```Wp001:2:11```

Or to reset it back to the default 460800:  

```Wp001:2:10```

The hand will need to be reset for the baud rate to change

### Disable BLE Radio

Leaving bluetooth enabled can cause packets to intermittently become lost.  To 
avoid lost packets use [send_misc_msg.py](https://github.com/psyonicinc/ability-hand-api/blob/master/python/send_misc_msg.py)
to send the command to disable BLE, you should see the hand reset after.

```python3 send_misc_msg.py 0x08```

To re-enable bluetooth

```python3 send_misc_msg.py 0x07```

## Linux Build Instructions
- Using a terminal navigate to cpp directory of API

```
mkdir build && cd build && cmake ..
make
./main
./hand_wave
```

## Windows Build Instructions

- Install [CMake](https://github.com/Kitware/CMake/releases/download/v4.0.1/cmake-4.0.1-windows-x86_64.msi) and [Visual Studio](https://visualstudio.microsoft.com/thank-you-downloading-visual-studio/?sku=Community&channel=Release&version=VS2022&source=VSLandingPage&cid=2030&passive=false).
- In Visual Studio install C/C++ development tools 
- Using a terminal navigate to cpp directory of API
```
mkdir build
cd build
cmake ..
```

Build and launch programs using Visual Studio

