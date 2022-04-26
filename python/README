# Ability Hand API - Python Examples

### System Requirements 

These examples require Python 3.10 or newer. They have been tested on Windows 10 and Ubuntu. Nothing should prevent them from working on Mac, however this is untested. Before running the examples, install the required modules using pip: `pip install -r requirements.txt` 

### Hand Setup

These examples are set up to communicate with the Ability Hand over a Serial to UART Dongle, such as a CP210x dongle. Connect Ground, TX (SDA), and RX (SCL) from the hand to the dongle. 

Before running, please ensure the hand is in UART communication mode - this can be configured over Bluetooth with the binary setting command `We16`

## Included Examples

### `finger_wave_live_plot.py`

This example uses the Ability Hand Extended API in mode `0x12`. It sends position commands and plots the received position from the hand as they are received. The position commands sent can be controlled via the terminal window to either do a finger wave, open/close the fingers, or do nothing. Instructions will be printed to the console. 

