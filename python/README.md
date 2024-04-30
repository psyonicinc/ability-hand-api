# Ability Hand API - Python Examples

### System Requirements 

These examples require Python 3.10 or newer. They have been tested on Windows 10 and Ubuntu. Nothing should prevent them from working on Mac, however this is untested. Before running the examples, install the required modules using pip: `pip install -r requirements.txt` 

### Hand Setup

These examples are set up to communicate with the Ability Hand over a Serial to UART Dongle, such as a CP210x dongle. Connect Ground, TX (SDA), and RX (SCL) from the hand to the dongle. 

Before running, please ensure the hand is in UART communication mode - this can be configured over Bluetooth with the binary setting command `We16`

## Included Examples

