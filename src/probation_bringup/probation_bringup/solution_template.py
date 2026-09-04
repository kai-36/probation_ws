#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from vision_msgs.msg import BoundingBoxArray
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, DurabilityPolicy
from mavros_msgs.srv import SetMode


class GateNavigator(Node):

    def __init__(self):
        super().__init__('gate_navigator')

        self.vel_cmd = Twist()
        self.gateless_frames = 0
        self.altitude = 0
        self.mode = "GUIDED"

        self.STANDBY = "STANDBY"
        self.DESCENDING = "DESCENDING"
        self.NAVIGATING = "NAVIGATING"

        self.operation = self.STANDBY

        # Client to change mode
        self.mode_client = self.create_client(
            SetMode, 
            '/mavros/set_mode')

        while not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.set_mode(self.mode)

        # publisher for velocity commands
        vel_cmd_Qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.publisher_ = self.create_publisher(
            Twist, 
            '/mavros/setpoint_velocity/cmd_vel_unstamped', 
            vel_cmd_Qos)
        
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.vel_cmd_publisher_callback)

        # subscriber to bounding box
        self.subscription = self.create_subscription(
            BoundingBoxArray,
            '/main_camera/detection/bounding_boxes',
            self.camera_listener_callback,
            10)

        # subscriber to rel_alt
        self.subscription = self.create_subscription(
            Float64,
            '/mavros/global_position/rel_alt',
            self.alt_listener_callback,
            10)

    def set_mode(self, mode):
        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = mode

        future = self.mode_client.call_async(request)

        future.add_done_callback(self.mode_response_callback)

    def mode_response_callback(self, future):
        try:
            response = future.result()

            if response.mode_sent:
                self.get_logger().info(f'Mode changed to {self.mode}.')
            else:
                self.get_logger().warn(f'Mode change to {self.mode} failed.')

        except Exception as e:
            self.get_logger().heading_error(f'Mode change service call failed: {e}')

    def vel_cmd_publisher_callback(self):
        
        msg = self.vel_cmd

        if self.operation == self.STANDBY:
            self.publisher_.publish(Twist())
        
        else:
            self.publisher_.publish(msg)

    def camera_listener_callback(self, msg):

        if self.operation != self.NAVIGATING:
            return

        bounding_boxes = msg.bounding_boxes

        if not bounding_boxes:
            self.get_logger().info("No objects detected")
            return

        for box in bounding_boxes:

            # if gate
            if box.label_id == 3:

                self.generate_cmd(box)

            else:
                # self.get_logger().info("Obstacle")
                pass

    def alt_listener_callback(self, msg):

        TARGET_ALT = -1.7
        DESCENDING_VEL = -0.5

        self.altitude = msg.data

        if self.altitude > TARGET_ALT:

            if self.operation != self.DESCENDING:
                self.get_logger().info(f"Operation: {self.operation} -> {self.DESCENDING}")
                self.operation = self.DESCENDING

            vel_cmd = Twist()
            vel_cmd.linear.z = DESCENDING_VEL

            self.vel_cmd = vel_cmd

        elif self.operation == self.DESCENDING or self.operation == self.STANDBY:
            
            self.get_logger().info(f"Operation: {self.operation} -> {self.NAVIGATING}")
            self.operation = self.NAVIGATING
            self.vel_cmd = Twist()
        

    def generate_cmd(self, box):    

        HEADING_ERROR_TOL = 0.05
        FRAME_CENTER_X = 0.5
        YAW_SPEED = 0.5
        direction = 1
        
        heading_error = box.x - FRAME_CENTER_X

        if heading_error > 0:
            direction = -1

        self.get_logger().info(f"heading_error: {heading_error}")
        if abs(heading_error) > HEADING_ERROR_TOL:
  
            self.vel_cmd = Twist()
            self.vel_cmd.angular.z = direction*YAW_SPEED

        else:
            self.vel_cmd = Twist()
        


def main(args=None):
    rclpy.init(args=args)
    node = GateNavigator()

    try:
        # spinning is blocking 
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
