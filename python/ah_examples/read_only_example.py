#!/usr/bin/env python3
"""
Example demonstrating how to use the read-only mode functionality.

This example shows how to:
1. Put the hand in read-only mode with different reply variants
2. Read sensor data without sending control commands
"""

import time
from ah_wrapper.ah_serial_client import AHSerialClient


def main():
    """Demonstrate read-only mode functionality"""
    
    # Initialize the hand client
    print("Initializing Ability Hand client...")
    hand = AHSerialClient(simulated=True)  # Use simulated=True for testing without hardware
    
    print("\n=== Read-Only Mode Examples ===")
    
    # Example 1: Read-only mode with reply variant 1 (0xA0) - Pos. Cur. Touch
    print("\n1. Setting read-only mode with reply variant 1 (0xA0) - Pos. Cur. Touch")
    hand.set_read_only(reply_mode=0)
    time.sleep(0.1)  # Give time for command to be sent
    
    # Read some data
    print("Reading data in read-only mode...")
    for i in range(3):
        pos = hand.hand.get_position()
        cur = hand.hand.get_current()
        fsr = hand.hand.get_fsr()
        
        print(f"  Read {i+1}:")
        if pos:
            print(f"    Positions: {[f'{p:.2f}' for p in pos]}")
        if cur:
            print(f"    Currents: {[f'{c:.2f}' for c in cur]}")
        if fsr:
            print(f"    FSR: {[f'{f:.2f}' for f in fsr[:6]]}")  # Show first 6 FSR values
        time.sleep(0.5)
    
    # Example 2: Read-only mode with reply variant 2 (0xA1) - Pos. Vel. Touch
    print("\n2. Setting read-only mode with reply variant 2 (0xA1) - Pos. Vel. Touch")
    hand.set_read_only(reply_mode=1)
    time.sleep(0.1)
    
    # Read some data
    print("Reading data in read-only mode...")
    for i in range(3):
        pos = hand.hand.get_position()
        vel = hand.hand.get_velocity()
        fsr = hand.hand.get_fsr()
        
        print(f"  Read {i+1}:")
        if pos:
            print(f"    Positions: {[f'{p:.2f}' for p in pos]}")
        if vel:
            print(f"    Velocities: {[f'{v:.2f}' for v in vel]}")
        if fsr:
            print(f"    FSR: {[f'{f:.2f}' for f in fsr[:6]]}")  # Show first 6 FSR values
        time.sleep(0.5)
    
    # Example 3: Read-only mode with reply variant 3 (0xA2) - Pos. Cur. Vel.
    print("\n3. Setting read-only mode with reply variant 3 (0xA2) - Pos. Cur. Vel.")
    hand.set_read_only(reply_mode=2)
    time.sleep(0.1)
    
    # Read some data
    print("Reading data in read-only mode...")
    for i in range(3):
        pos = hand.hand.get_position()
        cur = hand.hand.get_current()
        vel = hand.hand.get_velocity()
        
        print(f"  Read {i+1}:")
        if pos:
            print(f"    Positions: {[f'{p:.2f}' for p in pos]}")
        if cur:
            print(f"    Currents: {[f'{c:.2f}' for c in cur]}")
        if vel:
            print(f"    Velocities: {[f'{v:.2f}' for v in vel]}")
        time.sleep(0.5)
    
    # Example 4: Return to normal control mode
    print("\n4. Returning to normal control mode (position control)")
    hand.set_position(30)  # This will switch back to position control mode
    time.sleep(0.1)
    
    print("\n=== Read-Only Mode Examples Complete ===")
    print("\nNote: In read-only mode, the hand will only send feedback data")
    print("and will not respond to position, velocity, or torque commands.")
    print("Use set_position(), set_velocity(), or set_torque() to return to control mode.")
    
    # Close the connection
    hand.close()


if __name__ == "__main__":
    main()
