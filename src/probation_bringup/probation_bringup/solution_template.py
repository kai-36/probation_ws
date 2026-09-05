#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from vision_msgs.msg import BoundingBoxArray
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, DurabilityPolicy
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode


class GateNavigator(Node):

    def __init__(self):
        super().__init__('gate_navigator')

        self.vel_cmd = Twist()
        self.gateless_frames = 0
        self.altitude = 0
        self.heading = 0
        self.charge_forward = False
        self.mode = ""
        self.pending_mode_change = False

        self.STANDBY = "STANDBY"
        self.DESCENDING = "DESCENDING"
        self.SEARCHING = "SEARCHING"
        self.NAVIGATING = "NAVIGATING"

        self.operation = self.STANDBY

        # Client to change mode
        self.mode_client = self.create_client(
            SetMode, 
            '/mavros/set_mode')

        while not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

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

        # subscriber to bounding_box
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

        # subscriber to compass_hdg
        self.subscription = self.create_subscription(
            Float64,
            '/mavros/global_position/compass_hdg',
            self.heading_listener_callback,
            10)

        # subscriber to state
        self.subscription = self.create_subscription(
            State,
            '/mavros/state',
            self.state_listener_callback,
            10)

    def set_mode(self, mode):
        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = mode

        future = self.mode_client.call_async(request)

        future.add_done_callback(lambda future: self.mode_response_callback(future, mode))

        self.pending_mode_change = True

    def mode_response_callback(self, future, mode):
        try:
            response = future.result()

            if response.mode_sent:
                self.get_logger().info(f'Mode change request to {mode} ACCEPTED.')
                self.pending_mode_change = False
            else:
                self.get_logger().warn(f'Mode change request to {mode} REJECTED.')

        except Exception as e:
            self.get_logger().error(f'Mode change service call failed: {e}')

    def vel_cmd_publisher_callback(self):
        
        msg = self.vel_cmd

        if self.operation == self.STANDBY:
            self.publisher_.publish(Twist())
        
        else:
            self.publisher_.publish(msg)

    def camera_listener_callback(self, msg):

        if (self.operation != self.SEARCHING) and (self.operation != self.NAVIGATING):
            return

        if self.operation == self.SEARCHING:

            SEARCH_YAW_VEL = 0.3
            direction = 1
            self.vel_cmd = Twist()

            if self.heading < 180:
                direction = -1
            
            self.vel_cmd.angular.z = direction*SEARCH_YAW_VEL
        
        GATELESS_FRAMES_THRESHOLD = 50
        BOX_MAX_HEIGHT = 0.8
        BOX_MAX_WIDTH = 0.5

        #  if the AUV has passed through the gate
        if (self.operation == self.NAVIGATING) and (self.gateless_frames > GATELESS_FRAMES_THRESHOLD):
            self.vel_cmd = Twist()
            self.get_logger().info(f"Operation: {self.operation} -> {self.STANDBY}")
            self.operation = self.STANDBY
            self.charge_forward = False
            return

        bounding_boxes = msg.bounding_boxes

        if not bounding_boxes:
            # self.get_logger().info("No objects detected")
            self.gateless_frames += 1
            return

        gate_detected = False

        for box in bounding_boxes:

            # if gate
            if box.label_id == 3:

                if self.operation == self.SEARCHING:
                    self.get_logger().info(f"Operation: {self.operation} -> {self.NAVIGATING}")
                    self.operation = self.NAVIGATING
                
                gate_detected = True
                self.gateless_frames = 0

                if (box.h > BOX_MAX_HEIGHT and box.w > BOX_MAX_WIDTH):
                    self.charge_forward = True
                    self.move_towards_gate()

                self.generate_cmd(box)

            else:
                # self.get_logger().info("Obstacle")
                pass

        if not gate_detected:

            self.gateless_frames += 1
        
    def alt_listener_callback(self, msg):

        TARGET_ALT = -1.3
        DESCENDING_VEL = -1.0

        self.altitude = msg.data

        if self.mode != "GUIDED":
            return

        if self.altitude > TARGET_ALT:

            if self.operation != self.DESCENDING:
                self.get_logger().info(f"Operation: {self.operation} -> {self.DESCENDING}")
                self.operation = self.DESCENDING

            vel_cmd = Twist()
            vel_cmd.linear.z = DESCENDING_VEL

            self.vel_cmd = vel_cmd

        elif self.operation == self.DESCENDING:
            
            self.get_logger().info(f"Operation: {self.operation} -> {self.SEARCHING}")
            self.operation = self.SEARCHING
            self.vel_cmd = Twist()

    def heading_listener_callback(self, msg):

        if self.operation != self.DESCENDING:
            return

        self.heading = msg.data

    def state_listener_callback(self, msg):

        self.mode = msg.mode

        if self.mode == "GUIDED":
            return

        if not self.pending_mode_change:
            self.set_mode("GUIDED")

    
    def generate_cmd(self, box):

        if self.charge_forward:
            return

        HEADING_ERROR_TOL = 0.05
        FRAME_CENTER_X = 0.5
        
        heading_error = box.x - FRAME_CENTER_X

        if abs(heading_error) > HEADING_ERROR_TOL:

            self.correct_heading(heading_error)

        else:

            self.move_towards_gate()
        

    def correct_heading(self, heading_error):

        YAW_SPEED = 0.2
        direction = 1

        if heading_error > 0:
            direction = -1

        self.vel_cmd = Twist()
        self.vel_cmd.angular.z = direction*YAW_SPEED

    def move_towards_gate(self):

        FORWARD_VEL = 0.8

        self.vel_cmd = Twist()
        self.vel_cmd.linear.x = FORWARD_VEL
        

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
