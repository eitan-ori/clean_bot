#!/usr/bin/env python3

import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import String


class MissionCommandPublisher(Node):
    def __init__(self, topic_name: str):
        super().__init__('mission_command_publisher')

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE

        self._pub = self.create_publisher(String, topic_name, qos)

    def wait_for_subscribers(self, timeout_sec: float) -> int:
        deadline = time.time() + max(0.0, timeout_sec)
        subs = self._pub.get_subscription_count()
        while subs == 0 and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            subs = self._pub.get_subscription_count()
        return subs

    def publish(self, command: str):
        msg = String()
        msg.data = command
        self._pub.publish(msg)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Publish a /mission_command reliably (waits for subscribers, can repeat).'
    )
    parser.add_argument('command', help='Command string, e.g. start_scan, stop_scan, start_clean')
    parser.add_argument('--topic', default='/mission_command', help='Topic name (default: /mission_command)')
    parser.add_argument('--wait', type=float, default=2.0, help='Seconds to wait for subscribers (default: 2.0)')
    parser.add_argument('--repeat', type=int, default=5, help='How many times to publish (default: 5)')
    parser.add_argument('--interval', type=float, default=0.2, help='Seconds between repeats (default: 0.2)')
    args = parser.parse_args(argv)

    rclpy.init(args=None)
    node = MissionCommandPublisher(args.topic)
    try:
        cmd = (args.command or '').strip()
        if not cmd:
            raise SystemExit('command must be non-empty')

        subs = node.wait_for_subscribers(args.wait)
        node.get_logger().info(
            f'Publishing "{cmd}" on {args.topic} (subscribers={subs}, repeat={args.repeat})'
        )

        for i in range(max(1, args.repeat)):
            node.publish(cmd)
            rclpy.spin_once(node, timeout_sec=0.0)
            if i < args.repeat - 1:
                time.sleep(max(0.0, args.interval))

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
