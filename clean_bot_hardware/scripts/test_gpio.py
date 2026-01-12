#!/usr/bin/env python3
"""
GPIO Test Script - Test servo and relay directly
Run on Raspberry Pi to diagnose hardware issues.

BCM Pin Mapping (default gpiozero):
  - BCM 18 = Physical Pin 12 (PWM0)
  - BCM 17 = Physical Pin 11

Usage:
    python3 test_gpio.py
"""

import time

print("=" * 60)
print("🔧 GPIO Hardware Test Script")
print("=" * 60)

# Check if we're on a Pi
try:
    from gpiozero import Servo, OutputDevice, LED
    from gpiozero.pins.pigpio import PiGPIOFactory
    print("✅ gpiozero imported successfully")
except ImportError as e:
    print(f"❌ Failed to import gpiozero: {e}")
    exit(1)

# Try to use pigpio for better PWM (less jitter)
try:
    factory = PiGPIOFactory()
    print("✅ pigpio factory available (better PWM)")
    USE_PIGPIO = True
except Exception as e:
    print(f"⚠️ pigpio not available, using software PWM: {e}")
    print("   To install: sudo apt install pigpio && sudo systemctl enable pigpiod && sudo systemctl start pigpiod")
    factory = None
    USE_PIGPIO = False

print()
print("=" * 60)
print("📌 Pin Configuration:")
print("   Servo: BCM 18 (Physical Pin 12)")
print("   Relay: BCM 17 (Physical Pin 11)")
print("=" * 60)
print()

# ============== TEST 1: Relay ==============
print("🔌 TEST 1: RELAY on GPIO 17")
print("-" * 40)

try:
    relay = OutputDevice(17, active_high=True, initial_value=False)
    print(f"   Relay object created: {relay}")
    print(f"   Initial state: is_active={relay.is_active}")
    
    print("\n   👀 Watch/listen for relay click!")
    print("   Turning relay ON...")
    relay.on()
    print(f"   Relay is_active: {relay.is_active}")
    time.sleep(1)
    
    print("   Turning relay OFF...")
    relay.off()
    print(f"   Relay is_active: {relay.is_active}")
    time.sleep(0.5)
    
    print("   Toggling 3 times (should hear 3 clicks)...")
    for i in range(3):
        relay.on()
        time.sleep(0.3)
        relay.off()
        time.sleep(0.3)
    
    print("✅ Relay test complete")
    relay.close()
except Exception as e:
    print(f"❌ Relay test failed: {e}")
    import traceback
    traceback.print_exc()

print()

# ============== TEST 2: Servo ==============
print("⚙️ TEST 2: SERVO on GPIO 18")
print("-" * 40)

try:
    if USE_PIGPIO:
        servo = Servo(18, pin_factory=factory)
        print("   Using pigpio for PWM (better quality)")
    else:
        servo = Servo(18)
        print("   Using software PWM (may have jitter)")
    
    print(f"   Servo object created: {servo}")
    
    print("\n   👀 Watch for servo movement!")
    
    print("   Moving to MIN position (-1)...")
    servo.min()
    print(f"   Servo value: {servo.value}")
    time.sleep(1)
    
    print("   Moving to MID position (0)...")
    servo.mid()
    print(f"   Servo value: {servo.value}")
    time.sleep(1)
    
    print("   Moving to MAX position (1)...")
    servo.max()
    print(f"   Servo value: {servo.value}")
    time.sleep(1)
    
    print("   Moving back to MID...")
    servo.mid()
    time.sleep(0.5)
    
    print("   Stopping PWM signal...")
    servo.value = None
    
    print("✅ Servo test complete")
    servo.close()
except Exception as e:
    print(f"❌ Servo test failed: {e}")
    import traceback
    traceback.print_exc()

print()

# ============== TEST 3: Simple GPIO toggle ==============
print("💡 TEST 3: Simple GPIO toggle (LED test)")
print("-" * 40)
print("   Testing GPIO 17 and 18 as simple on/off outputs")

for pin in [17, 18]:
    try:
        print(f"\n   GPIO {pin}:")
        led = LED(pin)
        print(f"      Turning ON...")
        led.on()
        time.sleep(0.5)
        print(f"      Turning OFF...")
        led.off()
        time.sleep(0.5)
        led.close()
        print(f"   ✅ GPIO {pin} toggle OK")
    except Exception as e:
        print(f"   ❌ GPIO {pin} failed: {e}")

print()
print("=" * 60)
print("🔍 TROUBLESHOOTING:")
print("=" * 60)
print("""
If relay didn't click:
  1. Check wiring: Signal to GPIO 17 (physical pin 11)
  2. Check power: Relay module needs 5V (or 3.3V depending on model)
  3. Check ground: Common GND between Pi and relay module
  4. Try different GPIO pin

If servo didn't move:
  1. Check wiring: Signal (orange/yellow) to GPIO 18 (physical pin 12)
  2. Check power: Servo needs 5V power (red wire) - NOT from Pi GPIO!
  3. Check ground: Common GND (brown/black wire)
  4. Install pigpio for better PWM:
     sudo apt install pigpio
     sudo systemctl enable pigpiod
     sudo systemctl start pigpiod

Physical Pin Layout (looking at Pi with USB ports down):
  
  3V3  (1)  (2)  5V
  SDA  (3)  (4)  5V     <-- Use for servo power
  SCL  (5)  (6)  GND    <-- Common ground
  GP4  (7)  (8)  TX
  GND  (9)  (10) RX
  GP17 (11) (12) GP18   <-- Relay / Servo signal
  ...
""")
