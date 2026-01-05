#!/usr/bin/env python3
import serial
import glob
import time

import subprocess

def check_port(port, baudrate):
    try:
        ser = serial.Serial(port, baudrate, timeout=2, write_timeout=2)
        print(f"Checking {port} at {baudrate}...", end='', flush=True)
        
        # Toggle DTR/RTS to try and reset the unit
        ser.dtr = False
        ser.rts = False
        time.sleep(0.1)
        ser.dtr = True
        ser.rts = True
        time.sleep(0.1)
        
        # Reset buffer
        ser.reset_input_buffer()
        
        # Try to send GET_HEALTH command (0xA5 0x50) to wake it up
        try:
            ser.write(b'\xA5\x50')
            ser.flush()
        except:
            pass
            
        # Try to read some bytes
        data = ser.read(100)
        ser.close()
        
        if len(data) > 0:
            print(f" SUCCESS! Received {len(data)} bytes.")
            print(f"Hex dump: {data[:20].hex()}")
            return True
        else:
            print(" No data received.")
            return False
            
    except serial.SerialException as e:
        print(f" Failed to open: {e}")
        return False
    except Exception as e:
        print(f" Error: {e}")
        return False

def main():
    print("Scanning for Serial Ports...")
    
    # Check dmesg for USB issues
    try:
        print("\n--- Kernel Log (Last 5 lines for ttyUSB) ---")
        subprocess.run("dmesg | grep ttyUSB | tail -n 5", shell=True)
        print("--------------------------------------------\n")
    except:
        pass

    ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
    
    if not ports:
        print("No serial ports found! Check USB connection.")
        return

    baudrates = [115200, 256000, 1000000] # Common Lidar baudrates

    found = False
    for port in ports:
        print(f"\n--- Testing Port: {port} ---")
        for baud in baudrates:
            if check_port(port, baud):
                print(f"\n>>> FOUND POSSIBLE LIDAR AT {port} WITH BAUDRATE {baud} <<<")
                print("Update your launch file with these values.")
                found = True
                break # Move to next port if found
    
    if not found:
        print("\nNo active data stream found on any port.")

if __name__ == '__main__':
    main()
