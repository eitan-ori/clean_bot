# Battery Indicator — How It Works

## What You See

The battery indicator appears in the top-right corner of the web control panel, showing a percentage (0–100%) with a color-coded bar:

| Color | Level |
|-------|-------|
| 🟢 Green | > 50% |
| 🟡 Yellow | 20–50% |
| 🔴 Red | < 20% |

---

## Where Does the Data Come From?

**The battery level is a software simulation, not a real hardware sensor reading.**

The Clean Bot hardware does not include a battery voltage sensor or fuel gauge IC — there is no physical component that measures the actual battery level. Instead, the web app backend (`app.py`) runs a **time-and-activity-based drain simulation** that estimates how much battery has been consumed since the app started.

### How the Simulation Works

The code is in `clean_bot_mission/clean_bot_mission/webapp/app.py`, inside the odometry callback (`_on_odom`):

```python
# Battery simulation
now = time.monotonic()
dt = now - self._battery_last_update      # Time since last update (seconds)
self._battery_last_update = now

speed = abs(self.linear_vel) + abs(self.angular_vel) * 0.3
drain = dt * (0.02 + speed * 0.15)        # base drain + velocity drain
self.battery_level = max(0.0, self.battery_level - drain)
```

**The formula:**

```
drain_per_second = 0.02 + (|linear_vel| + |angular_vel| × 0.3) × 0.15
```

- **Base drain rate:** 0.02% per second (~83 minutes from 100% to 0% when idle)
- **Movement drain:** Additional drain proportional to how fast the robot is moving
- **Angular movement** is weighted at 30% of linear (turning uses less energy than driving)

### Key Details

| Property | Value |
|----------|-------|
| Start value | 100% (every time the web app starts) |
| Updates | Every odometry message (~5 Hz) |
| Minimum | 0% (clamped, never goes negative) |
| Reset | Restarting the web app resets to 100% |

### Arduino Battery Override

The code also includes a placeholder for a real battery reading:

```python
self._arduino_battery = None   # Set from Arduino if hardware supports it

# In status push:
battery = self._arduino_battery if self._arduino_battery is not None else round(self.battery_level, 1)
```

If you add a voltage divider and analog reading on the Arduino, and send the battery percentage over serial, you can set `_arduino_battery` and the indicator will show the real value instead of the simulation.

---

## Why a Simulation?

1. **User awareness** — Even an estimated battery gives the user a sense that the robot has been running for a long time and may need charging soon.
2. **Upgrade-ready** — The architecture supports swapping in a real sensor without changing the frontend. Just set `_arduino_battery` from a serial reading.
3. **No hardware cost** — Adding a real battery sensor (e.g., INA219 or voltage divider + ADC) requires extra wiring and an Arduino analog pin. The simulation works out of the box.

---

## How to Add a Real Battery Sensor

If you want to replace the simulation with real data:

### Option A: Voltage Divider on Arduino

1. Wire a voltage divider from the motor battery to an Arduino analog pin (e.g., A0).
2. In your Arduino sketch, read the voltage and send it over serial:
   ```cpp
   float voltage = analogRead(A0) * (5.0 / 1023.0) * DIVIDER_RATIO;
   float percentage = map(voltage, MIN_VOLTAGE, MAX_VOLTAGE, 0, 100);
   Serial.println("BAT:" + String(percentage));
   ```
3. In `arduino_driver.py`, parse the `BAT:` line and publish it (or store it for the web app to read).
4. In `app.py`, set `self._arduino_battery = <value>` when the reading arrives.

### Option B: INA219 on I2C

1. Connect an INA219 current/voltage sensor to the Pi's I2C bus.
2. Read voltage using the `ina219` Python library.
3. Publish a custom ROS 2 topic (e.g., `/battery_level`) and subscribe in the web app.

Either way, once `_arduino_battery` is set to a real value, the frontend automatically displays it instead of the simulation — no frontend changes needed.
