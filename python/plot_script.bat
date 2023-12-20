@echo off
echo ------------------------------------------------------------------------
echo Run live_plot.py with recommended settings
echo For this script to work properly, you must send the following commands to the Ability Hand over BLE
echo using the developer console in the Ability Hand app, or a 3rd party GATT/BLE application such as nRF toolbox:
echo Enable UART:                          We16
echo If using RS485, enable RS485:         We35
echo Enable byte stuffing (TX):            We46
echo Enable byte unstuffing (RX):          We47
echo ------------------------------------------------------------------------
echo Note: it is recommended to increase the baud rate to 1000000 if using RS485
echo Set baud rate to max: 		Wp001:2:0x12
echo If you set baud to max, ensure you call live_plot.py with -b 1000000

python live_plot.py --position --stuff
@REM sudo python3 live_plot.py --position --stuff -b 1000000
