#!/usr/bin/env python3
"""
###############################################################################
# FILE DESCRIPTION:
# This node interfaces with the Grove IMU 9DOF (comprised of ICM20600 and 
# AK09918 chips) via I2C. It reads raw physical measurements and converts 
# them into standard ROS 2 sensor messages.
#
# MAIN FUNCTIONS:
# 1. Periodically polls the IMU for acceleration, angular velocity, and 
#    magnetic field data.
# 2. Publishes raw IMU data to 'imu/data_raw' (sensor_msgs/Imu).
# 3. Publishes magnetic field data to 'imu/mag' (sensor_msgs/MagneticField).
#
# PARAMETERS & VALUES:
# - i2c_bus: 1 (The I2C bus index on the host SBC, usually Raspberry Pi).
# - frame_id: imu_link (The TF frame name associated with the physical sensor).
# - publish_rate: 50.0 Hz (Sampling and publishing frequency).
#
# ASSUMPTIONS:
# - The Grove IMU 9DOF sensor is correctly wired to the I2C pins.
# - The 'smbus2' or similar library is used by the underlying SimpleIMU driver.
# - Magnetic field data is raw and not yet calibrated for local interference.
###############################################################################
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField

from clean_bot_hardware.simple_imu_driver import SimpleIMU


class ImuPublisherNode(Node):
    """ROS 2 Node that publishes IMU data"""
    
    def __init__(self):
        super().__init__('imu_publisher_node')
        
        # Declare parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 50.0)  # Hz
        
        # Get parameters
        i2c_bus = self.get_parameter('i2c_bus').value
        self.frame_id = self.get_parameter('frame_id').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # Create publishers
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)
        
        # Initialize IMU
        try:
            self.sensor = SimpleIMU(bus_num=i2c_bus)
            self.get_logger().info(f"IMU initialized on I2C bus {i2c_bus}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize IMU: {e}")
            return
        
        # Create timer for publishing
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.publish_data)
        
        self.get_logger().info(f"IMU Publisher started at {publish_rate} Hz")

    def publish_data(self):
        """Read IMU data and publish messages"""
        now = self.get_clock().now().to_msg()
        
        # Read sensor data
        try:
            ax, ay, az = self.sensor.get_accel_data()
            gx, gy, gz = self.sensor.get_gyro_data()
            mx, my, mz = self.sensor.get_mag_data()
        except Exception as e:
            self.get_logger().warn(f"Failed to read IMU: {e}")
            return
        
        # Publish IMU message
        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = self.frame_id
        
        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az
        
        imu_msg.angular_velocity.x = gx
        imu_msg.angular_velocity.y = gy
        imu_msg.angular_velocity.z = gz
        
        # Set covariance to -1 to indicate unknown
        imu_msg.orientation_covariance[0] = -1.0
        
        self.imu_pub.publish(imu_msg)
        
        # Publish MagneticField message
        mag_msg = MagneticField()
        mag_msg.header.stamp = now
        mag_msg.header.frame_id = self.frame_id
        
        mag_msg.magnetic_field.x = mx
        mag_msg.magnetic_field.y = my
        mag_msg.magnetic_field.z = mz
        
        self.mag_pub.publish(mag_msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = ImuPublisherNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
