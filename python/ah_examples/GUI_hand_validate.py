#!/usr/bin/env python3
"""
GUI TCP client for hand validation testing
"""

import sys
import socket
import json
import threading
import time
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QTextEdit, QLineEdit, 
                             QLabel, QGroupBox, QGridLayout, QMessageBox)
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtGui import QFont, QPixmap
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import numpy as np
import base64
import io

class TCPClientThread(QThread):
    """Thread for handling TCP communication"""
    message_received = pyqtSignal(str)
    data_received = pyqtSignal(dict)
    connection_status = pyqtSignal(bool)
    
    def __init__(self, host='localhost', port=12345):
        super().__init__()
        self.host = host
        self.port = port
        self.socket = None
        self.running = False
        
    def connect(self):
        """Connect to the validation server"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.host, self.port))
            self.running = True
            self.connection_status.emit(True)
            return True
        except Exception as e:
            self.connection_status.emit(False)
            return False
            
    def send_command(self, command: str):
        """Send a command to the server"""
        if self.socket and self.running:
            try:
                self.socket.send((command + "\n").encode())
                return True
            except Exception as e:
                self.message_received.emit(f"Error sending command: {e}")
                return False
        return False
        
    def receive_response(self) -> str:
        """Receive a response from the server"""
        if self.socket and self.running:
            try:
                data = self.socket.recv(1024).decode().strip()
                return data
            except Exception as e:
                self.message_received.emit(f"Error receiving response: {e}")
                return ""
        return ""
        
    def run(self):
        """Main thread loop for receiving messages"""
        while self.running:
            try:
                response = self.receive_response()
                if response:
                    # Check if it's JSON data
                    try:
                        data = json.loads(response)
                        # Check if this is plot data - don't show in messages
                        if data.get('Plot', False):
                            self.data_received.emit(data)
                        else:
                            # Regular JSON data - show in messages
                            self.message_received.emit(f"Received data: {data.get('test_type', 'unknown')}")
                            self.data_received.emit(data)
                    except json.JSONDecodeError:
                        # Regular message
                        self.message_received.emit(response)
                        
            except Exception as e:
                if self.running:
                    self.message_received.emit(f"Connection error: {e}")
                break
                
    def stop(self):
        """Stop the thread"""
        self.running = False
        if self.socket:
            self.socket.close()


class HandValidationGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.tcp_thread = None
        self.current_test = "None"
        self.test_passed = "Unknown"
        self.init_ui()
        
    def init_ui(self):
        """Initialize the GUI"""
        self.setWindowTitle("Hand Validation GUI")
        self.setGeometry(100, 100, 1200, 800)
        
        # Create central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Main layout
        main_layout = QHBoxLayout()
        central_widget.setLayout(main_layout)
        
        # Left panel for controls
        left_panel = QWidget()
        left_layout = QVBoxLayout()
        left_panel.setLayout(left_layout)
        
        # Tech ID section
        tech_group = QGroupBox("Tech ID")
        tech_layout = QVBoxLayout()
        self.tech_id_input = QLineEdit()
        self.tech_id_input.setPlaceholderText("Enter Tech ID")
        tech_layout.addWidget(self.tech_id_input)
        tech_group.setLayout(tech_layout)
        left_layout.addWidget(tech_group)
        
        # Connection section
        conn_group = QGroupBox("Connection")
        conn_layout = QVBoxLayout()
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.connect_to_server)
        self.status_label = QLabel("Status: Disconnected")
        conn_layout.addWidget(self.connect_btn)
        conn_layout.addWidget(self.status_label)
        conn_group.setLayout(conn_layout)
        left_layout.addWidget(conn_group)
        
        # Clinical hand section
        clinical_group = QGroupBox("Clinical Hand")
        clinical_layout = QHBoxLayout()
        self.clinical_yes_btn = QPushButton("Clinical (Y)")
        self.clinical_no_btn = QPushButton("Not Clinical (N)")
        self.clinical_yes_btn.clicked.connect(lambda: self.send_command("y"))
        self.clinical_no_btn.clicked.connect(lambda: self.send_command("n"))
        clinical_layout.addWidget(self.clinical_yes_btn)
        clinical_layout.addWidget(self.clinical_no_btn)
        clinical_group.setLayout(clinical_layout)
        left_layout.addWidget(clinical_group)
        
        # Command buttons
        cmd_group = QGroupBox("Commands")
        cmd_layout = QGridLayout()
        
        self.next_btn = QPushButton("Next")
        self.ready_btn = QPushButton("Ready")


        
        self.next_btn.clicked.connect(lambda: self.send_command("next"))
        self.ready_btn.clicked.connect(lambda: self.send_command("ready"))
        
        cmd_layout.addWidget(self.next_btn, 0, 0)
        cmd_layout.addWidget(self.ready_btn, 0, 1)
        
        cmd_group.setLayout(cmd_layout)
        left_layout.addWidget(cmd_group)
        
        # Test status section
        status_group = QGroupBox("Test Status")
        status_layout = QVBoxLayout()
        
        self.current_test_label = QLabel("Current Test: None")
        self.test_passed_label = QLabel("Test Result: Unknown")
        
        status_layout.addWidget(self.current_test_label)
        status_layout.addWidget(self.test_passed_label)
        
        status_group.setLayout(status_layout)
        left_layout.addWidget(status_group)
        
        # Server messages
        msg_group = QGroupBox("Server Messages")
        msg_layout = QVBoxLayout()
        self.message_display = QTextEdit()
        self.message_display.setMaximumHeight(200)
        self.message_display.setReadOnly(True)
        msg_layout.addWidget(self.message_display)
        msg_group.setLayout(msg_layout)
        left_layout.addWidget(msg_group)
        
        # Add left panel to main layout
        main_layout.addWidget(left_panel, 1)
        
        # Right panel for plotting
        right_panel = QWidget()
        right_layout = QVBoxLayout()
        right_panel.setLayout(right_layout)
        
        # Plot area
        plot_group = QGroupBox("Test Data Plot")
        plot_layout = QVBoxLayout()
        
        # Create image label for displaying plots
        self.plot_label = QLabel()
        self.plot_label.setMinimumSize(600, 400)
        self.plot_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.plot_label.setText("No plot data available")
        plot_layout.addWidget(self.plot_label)
        
        plot_group.setLayout(plot_layout)
        right_layout.addWidget(plot_group)
        
        # Add right panel to main layout
        main_layout.addWidget(right_panel, 2)
        
        # Set up button states
        self.set_button_states(False)
        
    def set_button_states(self, connected: bool):
        """Enable/disable buttons based on connection status"""
        self.next_btn.setEnabled(connected)
        self.ready_btn.setEnabled(connected)
        self.clinical_yes_btn.setEnabled(connected)
        self.clinical_no_btn.setEnabled(connected)
        
    def connect_to_server(self):
        """Connect to the validation server"""
        if self.tcp_thread is None or not self.tcp_thread.running:
            self.tcp_thread = TCPClientThread()
            self.tcp_thread.message_received.connect(self.handle_message)
            self.tcp_thread.data_received.connect(self.handle_data)
            self.tcp_thread.connection_status.connect(self.handle_connection_status)
            
            if self.tcp_thread.connect():
                self.tcp_thread.start()
                self.connect_btn.setText("Disconnect")
            else:
                QMessageBox.critical(self, "Connection Error", "Failed to connect to server")
        else:
            self.disconnect_from_server()
            
    def disconnect_from_server(self):
        """Disconnect from the validation server"""
        if self.tcp_thread:
            self.tcp_thread.send_command("quit")
            self.tcp_thread.stop()
            self.tcp_thread.wait()
            self.tcp_thread = None
            self.connect_btn.setText("Connect")
            self.set_button_states(False)
            self.status_label.setText("Status: Disconnected")
            
    def send_command(self, command: str):
        """Send a command to the server"""
        if self.tcp_thread and self.tcp_thread.running:
            self.tcp_thread.send_command(command)
            self.add_message(f"Sent: {command}")
            
    def handle_message(self, message: str):
        """Handle incoming server messages"""
        self.add_message(f"Server: {message}")
        
        # # Auto-respond to certain messages
        if "Send 'start' to begin" in message or "Starting FSR validation test" in message:
            # Auto-send start for tests
            self.send_command("start")
        elif "Grip validation complete" in message or "FSR validation complete" in message:
            # Auto-send start for tests
            self.send_command("next")
        elif "Press ready to run hand wave" in message:
            # Auto-send start for tests
            self.send_command("ready")
        elif "Starting" in message and "test" in message:
            # Update current test status
            if "grip" in message.lower():
                self.current_test = "Grip Validation"
            elif "fsr" in message.lower():
                self.current_test = "FSR Validation"
            elif "velocity" in message.lower():
                self.current_test = "Velocity Validation"
            elif "position" in message.lower():
                self.current_test = "Position Validation"
            elif "torque" in message.lower():
                self.current_test = "Torque Validation"
            self.update_test_status()
        elif "complete" in message.lower():
            # Auto-send next when test completes
            # self.send_command("next")
            self.test_passed = "Passed"
            self.update_test_status()
        elif "skipping" in message.lower():
            self.test_passed = "Skipped"
            self.update_test_status()
        elif "error" in message.lower():
            self.test_passed = "Failed"
            self.update_test_status()
            
    def handle_data(self, data: dict):
        """Handle incoming JSON data and plot it"""
        try:
            if 'image_file' in data:
                self.add_message("Received plot image file")
                self.display_plot_image(data['image_file'])
            else:
                self.add_message("Received raw plot data")
                self.plot_data(data)
        except Exception as e:
            self.add_message(f"Error displaying plot: {e}")
        
    def handle_connection_status(self, connected: bool):
        """Handle connection status changes"""
        if connected:
            self.status_label.setText("Status: Connected")
            self.set_button_states(True)
        else:
            self.status_label.setText("Status: Disconnected")
            self.set_button_states(False)
            
    def add_message(self, message: str):
        """Add a message to the display"""
        self.message_display.append(message)
        # Keep only the last 50 messages
        lines = self.message_display.toPlainText().split('\n')
        if len(lines) > 50:
            self.message_display.setPlainText('\n'.join(lines[-50:]))
            
    def update_test_status(self):
        """Update the test status display"""
        self.current_test_label.setText(f"Current Test: {self.current_test}")
        self.test_passed_label.setText(f"Test Result: {self.test_passed}")
        
    def display_plot_image(self, image_file: str):
        """Display a plot image from file"""
        try:
            # Create QPixmap from file
            pixmap = QPixmap(image_file)
            if pixmap.isNull():
                raise Exception(f"Failed to load image from file: {image_file}")
            
            # Get the label size
            label_size = self.plot_label.size()
            if label_size.width() <= 0 or label_size.height() <= 0:
                # Use default size if label hasn't been sized yet
                label_size = self.plot_label.minimumSize()
            
            # Scale to fit the label while maintaining aspect ratio
            scaled_pixmap = pixmap.scaled(
                label_size, 
                Qt.AspectRatioMode.KeepAspectRatio, 
                Qt.TransformationMode.SmoothTransformation
            )
            
            # Display the image
            self.plot_label.setPixmap(scaled_pixmap)
            self.add_message(f"Plot image displayed successfully from {image_file}")
            
        except Exception as e:
            self.add_message(f"Error displaying plot image: {e}")
            self.plot_label.setText(f"Error displaying plot: {str(e)}")
            
    def plot_data(self, data: dict):
        """Plot the received data (fallback for raw data)"""
        try:
            # Create a simple matplotlib figure for raw data
            fig = Figure(figsize=(8, 6))
            ax = fig.add_subplot(111)
            
            if 'positions' in data and 'timestamps' in data:
                positions = np.array(data['positions'])
                timestamps = np.array(data['timestamps'])
                
                # Plot each finger position
                colors = ['red', 'green', 'blue', 'magenta', 'orange', 'purple']
                labels = ['Index', 'Middle', 'Ring', 'Pinky', 'Thumb Flexor', 'Thumb Rotator']
                
                for i in range(min(len(positions[0]), len(colors))):
                    if len(positions) > 0:
                        ax.plot(timestamps, positions[:, i], color=colors[i], label=labels[i])
                
                ax.set_xlabel('Time (s)')
                ax.set_ylabel('Position')
                ax.set_title(f"{data.get('test_type', 'Test')} - Positions")
                ax.legend()
                ax.grid(True)
                
            elif 'currents' in data and 'timestamps' in data:
                currents = np.array(data['currents'])
                timestamps = np.array(data['timestamps'])
                
                # Plot each finger current
                colors = ['red', 'green', 'blue', 'magenta', 'orange', 'purple']
                labels = ['Index', 'Middle', 'Ring', 'Pinky', 'Thumb Flexor', 'Thumb Rotator']
                
                for i in range(min(len(currents[0]), len(colors))):
                    if len(currents) > 0:
                        ax.plot(timestamps, currents[:, i], color=colors[i], label=labels[i])
                
                ax.set_xlabel('Time (s)')
                ax.set_ylabel('Current (A)')
                ax.set_title(f"{data.get('test_type', 'Test')} - Currents")
                ax.legend()
                ax.grid(True)
                
            else:
                # No recognizable data format
                ax.text(0.5, 0.5, f"No plot data available for {data.get('test_type', 'unknown')} test", 
                       ha='center', va='center', transform=ax.transAxes)
                ax.set_title("No Data to Plot")
            
            # Convert figure to image and display
            buf = io.BytesIO()
            fig.savefig(buf, format='png', dpi=100, bbox_inches='tight')
            buf.seek(0)
            
            # Create QPixmap from bytes
            pixmap = QPixmap()
            pixmap.loadFromData(buf.getvalue())
            
            # Scale to fit the label while maintaining aspect ratio
            scaled_pixmap = pixmap.scaled(
                self.plot_label.size(), 
                Qt.AspectRatioMode.KeepAspectRatio, 
                Qt.TransformationMode.SmoothTransformation
            )
            
            # Display the image
            self.plot_label.setPixmap(scaled_pixmap)
            
            # Clean up
            buf.close()
            plt.close(fig)
            
        except Exception as e:
            self.add_message(f"Error plotting data: {e}")
            self.plot_label.setText("Error plotting data")
            
    def closeEvent(self, event):
        """Handle window close event"""
        if self.tcp_thread and self.tcp_thread.running:
            self.send_command("quit")
            self.disconnect_from_server()
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')  # Modern look
    
    # Set application font
    font = QFont("Arial", 9)
    app.setFont(font)
    
    window = HandValidationGUI()
    window.show()
    
    sys.exit(app.exec_())


if __name__ == "__main__":
    main() 