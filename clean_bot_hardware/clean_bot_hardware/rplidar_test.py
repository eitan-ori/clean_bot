#!/usr/bin/env python3
"""
RPLidar A1M8 Test Node
Tests the physical RPLidar sensor and displays diagnostics.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import time


class RPLidarTest(Node):
    """Node to test and diagnose RPLidar A1M8 sensor."""

    def __init__(self):
        super().__init__('rplidar_test')
        
        # Parameters
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('display_interval', 1.0)  # seconds
        
        scan_topic = self.get_parameter('scan_topic').value
        self.display_interval = self.get_parameter('display_interval').value
        
        # Statistics
        self.scan_count = 0
        self.last_display_time = time.time()
        self.min_range_ever = float('inf')
        self.max_range_ever = 0.0
        self.valid_readings = 0
        self.invalid_readings = 0
        
        # Subscribe to laser scan
        self.scan_sub = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            10
        )
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('🔴 RPLidar A1M8 Test Node Started')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Listening on topic: {scan_topic}')
        self.get_logger().info('Waiting for scan data...')
        self.get_logger().info('')

    def scan_callback(self, msg: LaserScan):
        """Process incoming laser scan data."""
        self.scan_count += 1
        current_time = time.time()
        
        # Analyze scan data
        ranges = [r for r in msg.ranges if not math.isinf(r) and not math.isnan(r)]
        valid_count = len(ranges)
        invalid_count = len(msg.ranges) - valid_count
        
        self.valid_readings += valid_count
        self.invalid_readings += invalid_count
        
        if ranges:
            min_range = min(ranges)
            max_range = max(ranges)
            avg_range = sum(ranges) / len(ranges)
            
            self.min_range_ever = min(self.min_range_ever, min_range)
            self.max_range_ever = max(self.max_range_ever, max_range)
        else:
            min_range = max_range = avg_range = 0.0
        
        # Display at interval
        if current_time - self.last_display_time >= self.display_interval:
            self.display_diagnostics(msg, ranges, min_range, max_range, avg_range, 
                                    valid_count, invalid_count)
            self.last_display_time = current_time

    def display_diagnostics(self, msg, ranges, min_range, max_range, avg_range,
                           valid_count, invalid_count):
        """Display diagnostic information."""
        
        # Calculate angles
        angle_min_deg = math.degrees(msg.angle_min)
        angle_max_deg = math.degrees(msg.angle_max)
        angle_inc_deg = math.degrees(msg.angle_increment)
        
        # Calculate scan frequency
        scan_time = msg.scan_time
        freq = 1.0 / scan_time if scan_time > 0 else 0
        
        # Find closest obstacle direction
        if ranges:
            min_idx = msg.ranges.index(min(r for r in msg.ranges 
                                          if not math.isinf(r) and not math.isnan(r)))
            closest_angle = math.degrees(msg.angle_min + min_idx * msg.angle_increment)
        else:
            closest_angle = 0
        
        # Print diagnostics
        print('\033[2J\033[H')  # Clear screen
        print('=' * 60)
        print('🔴 RPLidar A1M8 V6 - Live Diagnostics')
        print('=' * 60)
        print(f'')
        print(f'📊 Scan Statistics:')
        print(f'   Total scans received: {self.scan_count}')
        print(f'   Scan frequency: {freq:.1f} Hz')
        print(f'   Points per scan: {len(msg.ranges)}')
        print(f'   Valid readings: {valid_count} ({100*valid_count/len(msg.ranges):.1f}%)')
        print(f'   Invalid readings: {invalid_count}')
        print(f'')
        print(f'📐 Angular Info:')
        print(f'   Angle range: {angle_min_deg:.1f}° to {angle_max_deg:.1f}°')
        print(f'   Angular resolution: {angle_inc_deg:.3f}°')
        print(f'   Field of view: {angle_max_deg - angle_min_deg:.1f}°')
        print(f'')
        print(f'📏 Range Info:')
        print(f'   Configured range: {msg.range_min:.2f}m - {msg.range_max:.2f}m')
        print(f'   Current min: {min_range:.3f}m')
        print(f'   Current max: {max_range:.3f}m')
        print(f'   Current avg: {avg_range:.3f}m')
        print(f'')
        print(f'🎯 Closest Obstacle:')
        print(f'   Distance: {min_range:.3f}m')
        print(f'   Direction: {closest_angle:.1f}°')
        print(f'')
        print(f'📈 Session Stats:')
        print(f'   Min range ever: {self.min_range_ever:.3f}m')
        print(f'   Max range ever: {self.max_range_ever:.3f}m')
        print(f'   Total valid readings: {self.valid_readings}')
        print(f'')
        print(f'Frame: {msg.header.frame_id}')
        print('=' * 60)
        print('Press Ctrl+C to exit')
        
        # Simple visualization - top-down view
        self.print_simple_map(msg)

    def print_simple_map(self, msg):
        """Print a simple ASCII top-down visualization."""
        print('')
        print('📍 Simple Top-Down View (R = Robot):')
        
        # Create a simple grid
        grid_size = 21
        grid = [[' ' for _ in range(grid_size)] for _ in range(grid_size)]
        center = grid_size // 2
        
        # Mark robot position
        grid[center][center] = 'R'
        
        # Plot obstacles
        scale = 2.0  # meters per grid cell
        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r) or r > 5.0:
                continue
            
            angle = msg.angle_min + i * msg.angle_increment
            x = int(center + (r * math.cos(angle)) / scale * (grid_size / 5))
            y = int(center - (r * math.sin(angle)) / scale * (grid_size / 5))
            
            if 0 <= x < grid_size and 0 <= y < grid_size:
                grid[y][x] = '█'
        
        # Print grid
        print('   ' + '-' * (grid_size + 2))
        for row in grid:
            print('   |' + ''.join(row) + '|')
        print('   ' + '-' * (grid_size + 2))
        print(f'   Scale: ~{scale}m per character')


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    node = RPLidarTest()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n')
        print('=' * 60)
        print('📊 Final Statistics:')
        print(f'   Total scans: {node.scan_count}')
        print(f'   Total valid readings: {node.valid_readings}')
        print(f'   Total invalid readings: {node.invalid_readings}')
        if node.min_range_ever != float('inf'):
            print(f'   Min range recorded: {node.min_range_ever:.3f}m')
            print(f'   Max range recorded: {node.max_range_ever:.3f}m')
        print('=' * 60)
        print('✅ RPLidar test completed')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
