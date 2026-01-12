#!/usr/bin/env python3
"""
###############################################################################
# GADGET ACTIVATION SCRIPT (Relay Pulse Sequence)
# 
# DESCRIPTION:
# This script performs a specific double-pulse sequence on a GPIO-connected 
# relay to activate an external gadget. 
#
# CIRCUIT LOGIC:
# - Relay "Close" usually means GPIO High (if logic is Active High).
# - Relay "Open" usually means GPIO Low.
#
# SEQUENCE:
# 1. Close circuit (T1)
# 2. Open circuit  (T2)
# 3. Close circuit (T3)
# 4. Open circuit  (T4)
#
# HARDWARE SETUP:
# - GPIO Pin: 17 (Default)
# - Power: Connect Relay VCC/GND to Pi 5V/GND.
# - Signal: Connect Relay IN to GPIO 17.
###############################################################################
"""

from gpiozero import OutputDevice
from time import sleep
import sys

# CONFIGURATION
RELAY_PIN = 17
T1 = 0.5  # Duration of first pulse (seconds)
T2 = 0.5  # Duration of pause (seconds)
T3 = 0.5  # Duration of second pulse (seconds)
T4 = 0.1  # Final settling time (seconds)

def activate_gadget():
    # active_high=True means 1 is 'on' (circuit closed), 0 is 'off' (circuit open)
    # If your relay is active-low, change to active_high=False
    relay = OutputDevice(RELAY_PIN, active_high=True, initial_value=False)
    
    print(f"🎬 Starting Activation Sequence on GPIO {RELAY_PIN}...")
    
    # 1. Close circuit
    print("Step 1: Closing circuit...")
    relay.on()
    sleep(T1)
    
    # 2. Open circuit
    print("Step 2: Opening circuit...")
    relay.off()
    sleep(T2)
    
    # 3. Close circuit again
    print("Step 3: Closing circuit...")
    relay.on()
    sleep(T3)
    
    # 4. Open circuit
    print("Step 4: Opening circuit (Sequence Complete).")
    relay.off()
    sleep(T4)

if __name__ == "__main__":
    try:
        activate_gadget()
    except KeyboardInterrupt:
        print("\nAborted by user.")
    except Exception as e:
        print(f"Error: {e}")
