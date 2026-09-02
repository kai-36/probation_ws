#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('task_publisher')
        self.publisher_ = self.create_publisher(
            Twist, 
            "/mavros/setpoint_velocity/cmd_vel_unstamped", 
            10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Twist()

        msg.linear.x = 0.0  # Move forward
        msg.linear.y = 0.0  # Strate left/right (used for holonomic/omni robots)
        msg.linear.z = 0.0  # Move up/down (used for drones/underwater vehicles)

        # Assign angular (rotation) velocities in radians per second
        msg.angular.x = 0.0 # Roll
        msg.angular.y = 0.0 # Pitch
        msg.angular.z = 1.5 # Yaw (turn left)

        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing angular_z: {msg.angular.z}")


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()