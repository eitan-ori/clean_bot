#!/usr/bin/env python3
"""
###############################################################################
# GADGET DEACTIVATION SCRIPT (Relay Pulse Sequence)
# 
# DESCRIPTION:
# This script performs a specific single-pulse sequence on a GPIO-connected 
# relay to turn off an external gadget.
#
# SEQUENCE:
# 1. Close circuit (T_off_1)
# 2. Open circuit  (T_off_2)
#
# HARDWARE SETUP:
# - GPIO Pin: 17 (Default)
###############################################################################
"""

from gpiozero import OutputDevice
from time import sleep

# CONFIGURATION
RELAY_PIN = 17
T_OFF_1 = 1.0  # Duration of the 'off' pulse (seconds)
T_OFF_2 = 0.1  # Final settling time (seconds)

def deactivate_gadget():
    relay = OutputDevice(RELAY_PIN, active_high=True, initial_value=False)
    
    print(f"🎬 Starting Deactivation Sequence on GPIO {RELAY_PIN}...")
    
    # 1. Close circuit
    print("Step 1: Closing circuit...")
    relay.on()
    sleep(T_OFF_1)
    
    # 2. Open circuit
    print("Step 2: Opening circuit (Sequence Complete).")
    relay.off()
    sleep(T_OFF_2)

if __name__ == "__main__":
    try:
        deactivate_gadget()
    except KeyboardInterrupt:
        print("\nAborted by user.")
    except Exception as e:
        print(f"Error: {e}")
