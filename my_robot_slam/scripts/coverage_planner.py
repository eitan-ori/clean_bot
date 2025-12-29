#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid


class CoveragePlanner(Node):
    def __init__(self):
        super().__init__('coverage_planner')

        self.declare_parameter('robot_width', 0.3)
        self.robot_width = self.get_parameter('robot_width').value

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.map_sub = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)

        self.map_data = None
        self.goals = []
        self.current_goal_index = 0
        self.is_working = False

        self.get_logger().info('Coverage Planner initialized. Waiting for map...')

    def map_callback(self, msg):
        if self.map_data is None:
            self.get_logger().info('Map received. Generating coverage path...')
            self.map_data = msg
            self.generate_coverage_path()
            self.start_mission()

    def generate_coverage_path(self):
        # Simplified Lawnmower Pattern
        # 1. Find map bounds
        width = self.map_data.info.width
        height = self.map_data.info.height
        resolution = self.map_data.info.resolution
        origin_x = self.map_data.info.origin.position.x
        origin_y = self.map_data.info.origin.position.y

        # 2. Define coverage area (simple bounding box of the map for now)
        # In a real scenario, we would process the occupancy grid to find free space.
        # Here we generate a path and let Nav2 handle obstacle avoidance.

        min_x = origin_x
        max_x = origin_x + (width * resolution)
        min_y = origin_y
        max_y = origin_y + (height * resolution)

        step_size = self.robot_width

        x = min_x + step_size / 2
        going_up = True

        self.goals = []

        while x < max_x:
            if going_up:
                y_start = min_y + step_size / 2
                y_end = max_y - step_size / 2
                self.goals.append((x, y_start))
                self.goals.append((x, y_end))
            else:
                y_start = max_y - step_size / 2
                y_end = min_y + step_size / 2
                self.goals.append((x, y_start))
                self.goals.append((x, y_end))

            x += step_size
            going_up = not going_up

        self.get_logger().info(f'Generated {len(self.goals)} waypoints.')

    def start_mission(self):
        if not self.goals:
            return

        self.is_working = True
        self.send_next_goal()

    def send_next_goal(self):
        if self.current_goal_index >= len(self.goals):
            self.get_logger().info('Mission Complete!')
            self.is_working = False
            return

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 Action Server not available!')
            return

        x, y = self.goals[self.current_goal_index]

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0  # Facing forward (simplified)

        self.get_logger().info(
            f'Sending goal {self.current_goal_index + 1}/{len(self.goals)}: ({x:.2f}, {y:.2f})')

        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted.')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.get_logger().info(f'Goal finished with status: {future.result().status}')

        # Move to next goal regardless of success (simple retry logic could be added)
        self.current_goal_index += 1
        self.send_next_goal()


def main(args=None):
    rclpy.init(args=args)
    node = CoveragePlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
