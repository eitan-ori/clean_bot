#!/usr/bin/env python3
"""
Simple IMU Driver for Grove IMU 9DOF (ICM20600 + AK09918)
Direct I2C communication using smbus2
"""

import time
import math
from smbus2 import SMBus

# I2C Addresses
ICM20600_ADDR = 0x69
AK09918_ADDR = 0x0C

# ICM20600 Registers
ICM_PWR_MGMT_1 = 0x6B
ICM_ACCEL_XOUT_H = 0x3B
ICM_GYRO_XOUT_H = 0x43

# AK09918 Registers
AK_CNTL2 = 0x31
AK_HXL = 0x11
AK_ST1 = 0x10


class SimpleIMU:
    """Simple IMU driver for ICM20600 + AK09918"""
    
    def __init__(self, bus_num=1):
        self.bus = SMBus(bus_num)
        
        # Initialize ICM20600 (Accelerometer + Gyroscope)
        # Wake up from sleep mode
        self.bus.write_byte_data(ICM20600_ADDR, ICM_PWR_MGMT_1, 0x00)
        time.sleep(0.1)
        
        # Initialize AK09918 (Magnetometer)
        # Set to Continuous Measurement Mode 4 (100Hz)
        try:
            self.bus.write_byte_data(AK09918_ADDR, AK_CNTL2, 0x08)
        except Exception as e:
            print(f"Magnetometer init failed (check connection): {e}")

    def read_bytes_block(self, addr, reg, length):
        """Read a block of bytes from I2C device"""
        return self.bus.read_i2c_block_data(addr, reg, length)

    def _conv(self, high, low):
        """Convert two bytes to signed 16-bit integer"""
        val = (high << 8) + low
        if val >= 0x8000:
            return -((65535 - val) + 1)
        else:
            return val

    def get_accel_data(self):
        """
        Get accelerometer data in m/s^2
        Returns: tuple (ax, ay, az)
        """
        data = self.read_bytes_block(ICM20600_ADDR, ICM_ACCEL_XOUT_H, 6)
        
        x = self._conv(data[0], data[1])
        y = self._conv(data[2], data[3])
        z = self._conv(data[4], data[5])
        
        # Convert to m/s^2 (default sensitivity +/- 2g is 16384 LSB/g)
        scale = 16384.0
        return (x / scale) * 9.81, (y / scale) * 9.81, (z / scale) * 9.81

    def get_gyro_data(self):
        """
        Get gyroscope data in rad/s
        Returns: tuple (gx, gy, gz)
        """
        data = self.read_bytes_block(ICM20600_ADDR, ICM_GYRO_XOUT_H, 6)
        
        x = self._conv(data[0], data[1])
        y = self._conv(data[2], data[3])
        z = self._conv(data[4], data[5])
        
        # Convert to radians (default sensitivity +/- 250dps is 131 LSB/dps)
        scale = 131.0
        return math.radians(x / scale), math.radians(y / scale), math.radians(z / scale)

    def get_mag_data(self):
        """
        Get magnetometer data in Tesla
        Returns: tuple (mx, my, mz)
        """
        try:
            data = self.read_bytes_block(AK09918_ADDR, AK_HXL, 6)
            # Magnetometer sends Low Byte first!
            x = self._conv(data[1], data[0])
            y = self._conv(data[3], data[2])
            z = self._conv(data[5], data[4])
            
            # End read (required by AK09918 to refresh register)
            self.bus.read_byte_data(AK09918_ADDR, 0x18)
            
            # Convert to Tesla (sensitivity: 0.15 uT/LSB)
            scale = 0.15
            return (x * scale) * 1e-6, (y * scale) * 1e-6, (z * scale) * 1e-6
        except:
            return 0.0, 0.0, 0.0


if __name__ == '__main__':
    # Test the driver
    imu = SimpleIMU()
    
    while True:
        ax, ay, az = imu.get_accel_data()
        gx, gy, gz = imu.get_gyro_data()
        mx, my, mz = imu.get_mag_data()
        
        print(f"Accel: ({ax:.2f}, {ay:.2f}, {az:.2f}) m/s^2")
        print(f"Gyro:  ({gx:.4f}, {gy:.4f}, {gz:.4f}) rad/s")
        print(f"Mag:   ({mx:.2e}, {my:.2e}, {mz:.2e}) T")
        print("-" * 40)
        
        time.sleep(0.5)
