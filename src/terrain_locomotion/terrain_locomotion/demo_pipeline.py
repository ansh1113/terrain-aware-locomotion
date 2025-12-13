# File:
# ~/terrain_locomotion_ws/src/terrain_locomotion/terrain_locomotion/demo_pipeline.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import threading


class DemoPipelineNode(Node):
    def __init__(self):
        super().__init__('demo_pipeline')

        # Publisher for robot commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Demo sequence
        self.demo_commands = [
            {'vel': [0.3, 0.0, 0.0], 'duration': 3.0, 'description': 'Move forward on flat terrain'},
            {'vel': [0.0, 0.0, 0.0], 'duration': 2.0, 'description': 'Stop and assess terrain'},
            {'vel': [0.2, 0.0, 0.0], 'duration': 4.0, 'description': 'Move forward on slope (slower)'},
            {'vel': [0.0, 0.0, 0.0], 'duration': 2.0, 'description': 'Stop before rubble'},
            {'vel': [0.1, 0.0, 0.0], 'duration': 5.0, 'description': 'Navigate rubble (very slow)'},
            {'vel': [0.0, 0.0, 0.0], 'duration': 2.0, 'description': 'Stop before stairs'},
            {'vel': [0.15, 0.0, 0.0], 'duration': 6.0, 'description': 'Climb stairs (precise)'},
            {'vel': [0.0, 0.0, 0.0], 'duration': 2.0, 'description': 'Mission complete'},
        ]

        self.current_command_index = 0
        self.demo_start_time = None
        self.is_running = False

        # Timer for demo execution
        self.demo_timer = self.create_timer(0.1, self.demo_callback)

        self.get_logger().info('Demo Pipeline initialized')
        self.get_logger().info('Demo will showcase terrain-aware locomotion')

    def start_demo(self):
        """Start the demo sequence"""
        self.demo_start_time = time.time()
        self.current_command_index = 0
        self.is_running = True
        self.get_logger().info('🎬 Starting terrain locomotion demo!')

    def demo_callback(self):
        """Execute demo sequence"""
        if not self.is_running or self.demo_start_time is None:
            return

        current_time = time.time()
        elapsed_time = current_time - self.demo_start_time

        # Calculate total time for current command
        if self.current_command_index < len(self.demo_commands):
            current_cmd = self.demo_commands[self.current_command_index]

            # Check if it's time to move to next command
            if elapsed_time >= current_cmd['duration']:
                self.current_command_index += 1
                self.demo_start_time = current_time

                if self.current_command_index >= len(self.demo_commands):
                    # Demo complete
                    self.is_running = False
                    self.get_logger().info('🏁 Demo completed!')
                    return
                else:
                    # Log next command
                    next_cmd = self.demo_commands[self.current_command_index]
                    self.get_logger().info(f'📍 {next_cmd["description"]}')

            # Send current velocity command
            self.send_velocity_command(current_cmd['vel'])

    def send_velocity_command(self, velocity):
        """Send velocity command to robot"""
        twist_msg = Twist()
        twist_msg.linear.x = velocity[0]
        twist_msg.linear.y = velocity[1]
        twist_msg.angular.z = velocity[2]

        self.cmd_vel_pub.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DemoPipelineNode()

    # Start demo after a short delay
    def delayed_start():
        time.sleep(3.0)
        node.start_demo()

    demo_thread = threading.Thread(target=delayed_start)
    demo_thread.daemon = True
    demo_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
