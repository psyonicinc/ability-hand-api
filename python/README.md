# Ability Hand API - Python Examples

### System Requirements 

These examples require Python 3.10 or newer. They have been tested on Windows 10 and Ubuntu. Nothing should prevent them from working on Mac, however this is untested. Before running the examples, install the required modules using pip: `pip install -r requirements.txt` 

### Hand Setup

These examples are set up to communicate with the Ability Hand over a Serial to UART Dongle, such as a CP210x dongle. Connect Ground, TX (SDA), and RX (SCL) from the hand to the dongle. 

Before running, please ensure the hand is in UART communication mode - this can be configured over Bluetooth with the binary setting command `We16`

## Included Examples

### `live_plot_demo.py`

This example uses the Ability Hand Extended API to live plot data from the hand while providing the option to do basic hand movements with keyboard controls. It can plot either touch or position data using command line parameters to specify:
- `python3 live_plot_demo.py --touch`
- `python3 live_plot_demo.py --position`

When plotting touch data it uses the extended API in mode 0x10, for position data it uses 0x12. Instructions on how to move the hand will be printed to the console. This demo is set-up to be work with an Ability Hand with default out of the box settings. However, you can configure this with command line options - use `python3 live_plot_demo.py -h` for a full list of options. 

