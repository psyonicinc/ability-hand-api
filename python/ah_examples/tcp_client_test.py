#!/usr/bin/env python3
"""
Simple TCP client to test the validation server
"""

import socket
import json
import threading
import time

class TCPValidationClient:
    def __init__(self, host='localhost', port=12345):
        self.host = host
        self.port = port
        self.socket = None
        
    def connect(self):
        """Connect to the validation server"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.host, self.port))
            print(f"Connected to validation server at {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"Failed to connect: {e}")
            return False
            
    def send_command(self, command: str):
        """Send a command to the server"""
        if self.socket:
            try:
                self.socket.send((command + "\n").encode())
                return True
            except Exception as e:
                print(f"Error sending command: {e}")
                return False
        return False
        
    def receive_response(self) -> str:
        """Receive a response from the server"""
        if self.socket:
            try:
                data = self.socket.recv(1024).decode().strip()
                return data
            except Exception as e:
                print(f"Error receiving response: {e}")
                return ""
        return ""
        
    def receive_data(self) -> dict:
        """Receive JSON data from the server"""
        if self.socket:
            try:
                data = self.socket.recv(4096).decode().strip()
                if data:
                    return json.loads(data)
            except Exception as e:
                print(f"Error receiving data: {e}")
        return {}
        
    def close(self):
        """Close the connection"""
        if self.socket:
            self.socket.close()
            
    def listen_for_messages(self):
        """Listen for messages from the server in a separate thread"""
        while True:
            try:
                response = self.receive_response()
                if response:
                    print(f"Server: {response}")
                    
                    # Check if it's JSON data
                    try:
                        data = json.loads(response)
                        print(f"Received data: {data.get('test_type', 'unknown')}")
                    except json.JSONDecodeError:
                        pass  # Not JSON data, just a regular message
                        
            except Exception as e:
                print(f"Error in listener: {e}")
                break


def main():
    client = TCPValidationClient()
    
    if not client.connect():
        return
        
    # Start listener thread
    listener_thread = threading.Thread(target=client.listen_for_messages, daemon=True)
    listener_thread.start()
    
    try:
        # Wait for initial message
        time.sleep(1)
        
        # Example interaction
        print("\n=== Validation Test Client ===")
        print("Available commands:")
        print("- 'start': Start a test")
        print("- 'skip': Skip a test")
        print("- 'next': Move to next test")
        print("- 'y': Yes (for clinical hand question)")
        print("- 'n': No (for clinical hand question)")
        print("- 'ready': Ready for next step")
        print("- 'continue': Continue FSR test after timeout")
        print("- 'quit': Exit client and stop validation")
        
        while True:
            command = input("\nEnter command: ").strip()
            
            if command.lower() == 'quit':
                break
                
            if client.send_command(command):
                print(f"Sent: {command}")
            else:
                print("Failed to send command")
                
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        client.close()
        print("Client disconnected")


if __name__ == "__main__":
    main() 