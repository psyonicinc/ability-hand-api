import numpy as np
import pyqtgraph as pg
from PyQt5.QtWidgets import QApplication, QComboBox, QVBoxLayout, QWidget, QHBoxLayout, QLabel, QPushButton, QLineEdit
from PyQt5.QtCore import QTimer, QRegExp
from PyQt5.QtGui import QRegExpValidator
import sys
import socket
from threading import *
import Serial_Loop
import json
import Combobox_Defines
import struct
import math
import keyboard
import time


# Global variables
MAX_BUFFER_SIZE = 100  
NUM_FINGERS = 5
#hand Varibles
Index = 25
Middle = 15
Ring = 10
Pinky = 40
Thumb = 15
Thumb_rot = -15
Close_Window = Event()
isFingerWave = False
LastCommand = [15] * 6
LastCommand[5] = -LastCommand[5]
Pos = [15] * 6
Pos[5] = -Pos[5]

class HexLineEdit(QLineEdit):
    def __init__(self, parent=None):
        super().__init__(parent)
        
        regex = QRegExp("[0-9A-Fa-f]{2}")
        validator = QRegExpValidator(regex, self)
        self.setValidator(validator)


class Comm_to_hand:
    def __init__(self,UDP_IP,UDP_PORT_Receiver, UDP_PORT_SENDER) -> None:
        self.UDP_IP = UDP_IP
        self.UDP_PORT_RECEIVER = UDP_PORT_Receiver
        self.UDP_PORT_SENDER = UDP_PORT_SENDER
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.UDP_IP, self.UDP_PORT_RECEIVER))


    def UDP_Data_receiver(self):
       global Pos 
       while True:
           data, addr = self.sock.recvfrom(1024)
           Datos = data.decode('utf-8')
           Decoded = eval(Datos)# redundant, but good to keep.
           update_graph(Decoded)
           if(Close_Window.is_set()):
               self.sock.close()
               break
            


    def Close_Port(self):
        self.sock.close()

    def Send_UDP_Data(self,Message):
        self.sock.sendto(Message,(self.UDP_IP, self.UDP_PORT_SENDER))
         
CLASS_UDP = Comm_to_hand("Localhost",5005,5006) #Invert the port order in Serial_Loop.py      


# Initialize buffers
x = np.arange(MAX_BUFFER_SIZE)
y_position_buffer = np.zeros((6,MAX_BUFFER_SIZE))
y_Var1_buffer = np.zeros((6, MAX_BUFFER_SIZE))
y_Var2_buffer = np.zeros((6, MAX_BUFFER_SIZE))
y_touch_buffer = np.zeros((30, MAX_BUFFER_SIZE)) # this is set for touch data, if necesary


def update_graph(data):
    global x, y_Var1_buffer, y_Var2_buffer, y_position_buffer,y_touch_buffer
    # Generate sample data, change later for UDP data
    y_var1_new = data[1]
    y_position_new = data[0]

    # Shift existing data in buffers to make room for new data
    y_Var1_buffer[:, :-1] = y_Var1_buffer[:, 1:]
    y_position_buffer[:, :-1] = y_position_buffer[:, 1:]

    # Add new data to the end of buffers
    try:
        y_Var1_buffer[:, -1] = y_var1_new
    except:
        y_Var1_buffer = np.zeros((6, MAX_BUFFER_SIZE))
    
    y_position_buffer[:, -1] = y_position_new

    # Update plots
    for i in range(6):
        curve_Plot2[i].setData(x, y_Var1_buffer[i])
    # curve_touch[0].setData(x, y_current_buffer)
    for i in range(6):
        curve_pos[i].setData(x, y_position_buffer[i])

    reply_variant = np.bitwise_and(Combobox_Defines.Modes[Req_data.currentText()] , 0x0F) + 1

    if(reply_variant == 1 or reply_variant == 2):
        y_touch_new = data[2] 
        y_touch_buffer[:, :-1] = y_touch_buffer[:, 1:]
        try:
            y_touch_buffer[:, -1] = y_touch_new
        except:
            y_touch_buffer = np.zeros((30, MAX_BUFFER_SIZE))
        for i in range(30):
            curve_touch[i].setData(x, y_touch_buffer[i])
    if(reply_variant == 3):
        y_Var2_new = data[2]  
        y_Var2_buffer[:, :-1] = y_Var2_buffer[:, 1:]
        try:
            y_Var2_buffer[:, -1] = y_Var2_new
        except:
            y_Var2_buffer = np.zeros((6, MAX_BUFFER_SIZE))
        for i in range(6):
            curve_touch[i].setData(x, y_Var2_buffer[i])

def Send_INIT_params():
    timer.start(100) 
    Array = []
    Array.append(Combobox_Defines.Comm_Type[Comms.currentText()])
    Array.append(int(Bauds.currentText()))
    Array.append(Combobox_Defines.ByteStuff[ByteStuff.currentText()])
    Array.append(1)# this number indicates the Backloop to start.
    Checksum = (sum(Array) % 256)
    Checksum = (-Checksum) & 0xFF
    Array.append(Checksum) 
    CLASS_UDP.Send_UDP_Data(json.dumps(Array).encode('utf-8'))     
    Start_button.setDisabled(True)
    Comms.setDisabled(True)
    Bauds.setDisabled(True)
    ByteStuff.setDisabled(True)
    Address.setDisabled(True)
    # Update_Control_Mode()

def Exit_function(): 
    Array = []
    Array.append(Combobox_Defines.Comm_Type[Comms.currentText()])
    Array.append(int(Bauds.currentText()))
    Array.append(Combobox_Defines.ByteStuff[ByteStuff.currentText()])  
    Array.append(0) # this number indicates the Backloop to stop.
    Checksum = (sum(Array) % 256)
    Checksum = (-Checksum) & 0xFF
    Array.append(Checksum) 
    CLASS_UDP.Send_UDP_Data(json.dumps(Array).encode('utf-8')) 
    Close_Window.set()
    print("adios")

def generateTX(hand_address,reply_mode, positions):
        txBuf = []

        txBuf.append((struct.pack('<B',hand_address))[0])
        txBuf.append((struct.pack('<B',reply_mode))[0])

        for i in range(0,6):
            posFixed = int(positions[i] * 32767 / 150)
            txBuf.append((struct.pack('<B',(posFixed & 0xFF)))[0])
            txBuf.append((struct.pack('<B',(posFixed >> 8) & 0xFF))[0])

        cksum = 0
        for b in txBuf:
            cksum = cksum + b
        cksum = (-cksum) & 0xFF
        txBuf.append((struct.pack('<B', cksum))[0])

        return txBuf
## Generate Message to send to hand from array of positions (floating point)
def calculate_positions(current_positions, last_command):
    isFingerWave = True
    moveUp = False
    moveDown = False
    
    # if keyboard.is_pressed('esc'):
    #     isFingerWave = False
    # elif keyboard.is_pressed('up') or keyboard.is_pressed('w'):
    #     isFingerWave = False
    #     moveUp = True
    # elif keyboard.is_pressed('down') or keyboard.is_pressed('s'):
    #     isFingerWave = False
    #     moveDown = True
    # elif keyboard.is_pressed('space'):
    #     isFingerWave = True
    
    positions = [0.0] * 6
    
    for i in range(0, 6):
        if isFingerWave:
            ft = time.time() * 3 + i
            positions[i] = (0.5 * math.sin(ft) + 0.5) * 45 + 15
        elif moveUp:
            positions[i] = abs(last_command[i]) - 0.5
            if positions[i] < 5:
                positions[i] = 5
        elif moveDown:
            positions[i] = abs(last_command[i]) + 0.5
            if positions[i] > 90:
                positions[i] = 90
            if i == 4 or i == 5:
                if positions[i] > 50:
                    positions[i] = 50
        else:
            positions[i] = current_positions[i]
    
    positions[5] = -positions[5]  # Invert thumb rotator
    return positions

def Send_Hand_Data():
 
    global LastCommand
    global Pos
    # Arr = calculatePositions()

    LastCommand = calculate_positions(Pos,LastCommand)

    
    Array = []
    Array.append(int(Address.text(), 16))
    Array.append(Combobox_Defines.Modes[Req_data.currentText()])
    for i in range(6):
            Array.append(int(LastCommand[i]))
    Checksum = (sum(Array) % 256)
    Checksum = (-Checksum) & 0xFF
    Array.append(Checksum)
    CLASS_UDP.Send_UDP_Data(json.dumps(Array).encode('utf-8')) 

def Update_Control_Mode():
    mode = Req_data.currentText()
    # Split the mode string by "-"
    parts = mode.split("-")
    # The last part contains the "Var" information
    ctrl_info = parts[0]
    # Clear the combobox before repopulating it
    Req_data.clear()
    Req_data.update()
    # Repopulate the combobox based on the control information
    for value in Combobox_Defines.Modes:
        if ctrl_info in value:
            Req_data.addItem(value)

def extract_var():
    global x,y_position_buffer,y_Var1_buffer,y_Var2_buffer,y_touch_buffer
    x = np.arange(MAX_BUFFER_SIZE)
    y_position_buffer = np.zeros((6,MAX_BUFFER_SIZE))
    y_Var1_buffer = np.zeros((6, MAX_BUFFER_SIZE))
    y_Var2_buffer = np.zeros((6, MAX_BUFFER_SIZE))
    y_touch_buffer = np.zeros((30, MAX_BUFFER_SIZE)) # this is set for touch data, if necesary

    reply_variant = np.bitwise_and(Combobox_Defines.Modes[Req_data.currentText()] , 0x0F) + 1

    if(reply_variant == 1):
        plot2.setTitle("Current")
        plot1.setTitle("Touch")
    if(reply_variant == 2):
        plot2.setTitle("Velocity")
        plot1.setTitle("Touch")
    if(reply_variant == 3):
        plot1.setTitle("Current")
        plot2.setTitle("Velocity")        

## Read keyboard input and calculate positions


app = QApplication(sys.argv)

# Create main window
window = QWidget()

# Create layout for comboboxes
combobox_layout = QHBoxLayout()

# Create layout for Button Start
Button_layout = QHBoxLayout()

# Create buttons
Start_button = QPushButton("Start")
Start_button.clicked.connect(Send_INIT_params)

# Add buttons to the button layout
Button_layout.addWidget(Start_button)

# Create labels for the comboboxes
label_comm = QLabel("Communication:")
label_baud = QLabel("Baudrate:")
label_Addr = QLabel("Address:")
label_ByteStuff = QLabel("ByteStuffing:")
label_Req_data = QLabel("Data Type:")

# Create comboboxs, comboboxes, idk how to say it 
Comms = QComboBox()
for value in Combobox_Defines.Comm_Type:
    Comms.addItem(value)
Comms.setFixedSize(120,30)

Bauds = QComboBox()
for value in Combobox_Defines.Baudrate:
    Bauds.addItem(str(value))
Bauds.setFixedSize(120,30)

ByteStuff = QComboBox()
for value in Combobox_Defines.ByteStuff:
    ByteStuff.addItem(value)
ByteStuff.setFixedSize(120,30)

Req_data = QComboBox()
for value in Combobox_Defines.Modes:
    Req_data.addItem(value)
Req_data.setFixedSize(150,30)
Req_data.currentIndexChanged.connect(extract_var)

Address = HexLineEdit()
Address.setText("50")
# Add the labels and comboboxes to the combobox layout
combobox_layout.addWidget(label_comm)
combobox_layout.addWidget(Comms)
combobox_layout.addWidget(label_baud)
combobox_layout.addWidget(Bauds)
combobox_layout.addWidget(label_ByteStuff)
combobox_layout.addWidget(ByteStuff)
combobox_layout.addWidget(label_Addr)
combobox_layout.addWidget(Address)
combobox_layout.addWidget(label_Req_data)
combobox_layout.addWidget(Req_data)
combobox_layout.addStretch()

# Create layout for the graphs
graph_layout = QVBoxLayout()

# Create plot widgets
win1 = pg.GraphicsLayoutWidget()
plot1 = win1.addPlot(title="Touch")
curve_touch = []

for i in range(30):
    curve = plot1.plot(pen=(i, 6), name=f"Curve {i+1}")  # Assigning different colors to curves
    curve_touch.append(curve)

graph_layout.addWidget(win1)

win2 = pg.GraphicsLayoutWidget()
plot2 = win2.addPlot(title="Current")
curve_Plot2 = []

for i in range(30):
    curve = plot2.plot(pen=(i, 6), name=f"Curve {i+1}")  # Assigning different colors to curves
    curve_Plot2.append(curve)

graph_layout.addWidget(win2)

win3 = pg.GraphicsLayoutWidget()
plot3 = win3.addPlot(title="Position")
curve_pos = []

for i in range(30):
    curve = plot3.plot(pen=(i, 6), name=f"Curve {i+1}")  # Assigning different colors to curves
    curve_pos.append(curve)
graph_layout.addWidget(win3)

# Create main layout
layout = QVBoxLayout(window)
layout.addLayout(combobox_layout)
layout.addLayout(Button_layout)
layout.addLayout(graph_layout)

# Update graphs initially
# update_graph()

# Create a QTimer to Comunicate with the hands periodically
timer = QTimer()
timer.timeout.connect(Send_Hand_Data)



if __name__=='__main__':
    # Ability_Hand = Serial_Loop.AbilityHand(460800,"localhost",5006,5005,True)
    # T2 = Thread(target= Ability_Hand.run)
    # T2.start()
    T1 = Thread(target = CLASS_UDP.UDP_Data_receiver) 
    T1.setDaemon(True)                   
    T1.start()   
    window.show()
    
    T2 = Thread(target= Serial_Loop)
    try:
        sys.exit(app.exec_())
    finally:        
        Exit_function()



