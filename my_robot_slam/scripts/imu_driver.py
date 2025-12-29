#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

# Placeholder for I2C communication
# You might need to install smbus2: pip3 install smbus2
try:
    import smbus2
    SMBUS_AVAILABLE = True
except ImportError:
    SMBUS_AVAILABLE = False


class ImuDriver(Node):
    def __init__(self):
        super().__init__('imu_driver')

        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x69)  # Common address for ICM20600
        self.declare_parameter('frame_id', 'imu_link')

        self.i2c_bus = self.get_parameter('i2c_bus').value
        self.i2c_addr = self.get_parameter('i2c_address').value
        self.frame_id = self.get_parameter('frame_id').value

        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)

        if SMBUS_AVAILABLE:
            try:
                self.bus = smbus2.SMBus(self.i2c_bus)
                self.init_imu()
                self.get_logger().info(
                    f'IMU initialized on bus {self.i2c_bus} address {hex(self.i2c_addr)}')
            except Exception as e:
                self.get_logger().error(f'Failed to connect to IMU: {e}')
                self.bus = None
        else:
            self.get_logger().warn('smbus2 not installed. IMU will publish dummy data.')
            self.bus = None

        self.create_timer(0.02, self.publish_imu)  # 50Hz

    def init_imu(self):
        # TODO: Write configuration registers for ICM20600
        # Power management, Gyro config, Accel config
        pass

    def read_raw_data(self):
        if self.bus:
            try:
                # TODO: Read actual registers
                # This is a placeholder returning zero
                acc_x = 0.0
                acc_y = 0.0
                acc_z = 9.81
                gyro_x = 0.0
                gyro_y = 0.0
                gyro_z = 0.0
                return (acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z)
            except Exception:
                return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        else:
            return (0.0, 0.0, 9.81, 0.0, 0.0, 0.0)

    def publish_imu(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        ax, ay, az, gx, gy, gz = self.read_raw_data()

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz

        # Orientation is usually computed by an AHRS filter (like Madgwick)
        # or provided by the IMU's DMP.
        # For now, we leave orientation as 0 (identity quaternion) or unknown covariance
        msg.orientation.w = 1.0
        msg.orientation_covariance[0] = -1.0  # Indicate no orientation data

        self.imu_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
