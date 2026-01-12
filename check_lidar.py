import serial
import time
import sys

port = '/dev/ttyUSB0'
# Common RPLidar baud rates
bauds = [115200, 256000]

def check_baud(baud):
    print(f"Testing {port} at {baud}...")
    try:
        ser = serial.Serial(port, baud, timeout=2)
        
        # Reset mechanics for RPLidar (DTR toggling often controls motor/reset)
        ser.dtr = False
        time.sleep(0.1)
        ser.dtr = True 
        time.sleep(1)
        
        # Request health (A5 52) or simpler, just listen for info on startup
        # We will try to send GET_HEALTH command: 0xA5 0x52
        ser.reset_input_buffer()
        ser.write(b'\xA5\x52') 
        
        # Read response
        data = ser.read(100)
        ser.close()
        
        if len(data) > 0:
            print(f"SUCCESS: Received {len(data)} bytes at {baud} baud.")
            # Check for header 0xA5 0x5A (response descriptor)
            if b'\xa5\x5a' in data:
                print("  -> Valid RPLidar signature found!")
            else:
                print(f"  -> Data: {data}")
            return True
        else:
            print(f"FAILURE: No data received at {baud} baud.")
            return False
            
    except Exception as e:
        print(f"ERROR: {e}")
        return False

print("--- Lidar Serial Diagnostic [Requires 'pyserial' installed] ---")
print("Press Ctrl+C to stop if stuck.")

success = False
try:
    for b in bauds:
        if check_baud(b):
            success = True
            break
except KeyboardInterrupt:
    print("\nAborted.")

if not success:
    print("\nDIAGNOSIS: The Lidar is not responding. Checklist:")
    print("1. Power: Does the lidar spin?")
    print("2. Port: Is /dev/ttyUSB0 correct? (ls -la /dev/ttyUSB*)")
    print("3. Permission: sudo chmod 666 /dev/ttyUSB0")
    print("4. Hardware: Cable or unit might be defective")
else:
    print("\nDIAGNOSIS: Hardware responded! Please verify the Baud Rate in your launch file.")
