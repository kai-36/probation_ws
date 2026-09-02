#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from vision_msgs.msg import BoundingBoxArray


class GateNavigator(Node):

    def __init__(self):
        super().__init__('gate_navigator')

        # TODO: Implement service call to change mode
        # self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

        # publisher for velocity commands
        self.publisher_ = self.create_publisher(Twist, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publisher_callback)

        # subscriber to bounding box
        self.subscription = self.create_subscription(
            BoundingBoxArray,
            '/main_camera/detection/bounding_boxes',
            self.listener_callback,
            10)


    def publisher_callback(self):
        msg = Twist()    
        pass

    def listener_callback(self, msg):

        bounding_boxes = msg.bounding_boxes

        if not bounding_boxes:
            self.get_logger().info("None")
            return

        for box in bounding_boxes:

            # if gate
            if box.label_id == 3:

                self.get_logger().info(
                    f"\nC: ({box.x}, {box.y})\n"
                    f"W: {box.w}\n"
                    f"H: {box.h}\n"
                    f"conf: {box.conf}\n"
                    f"name: {box.label_name}"
                )

            else:
                self.get_logger().info("No gate")


def main(args=None):
    rclpy.init(args=args)
    node = GateNavigator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
