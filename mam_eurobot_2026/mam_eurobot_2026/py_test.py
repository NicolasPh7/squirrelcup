#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

class TestPyNode(Node):
    def __init__(self):
        super().__init__("test_py_node")
        self.get_logger().info("Hello from Python!")
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.count = 0
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        flag_ = False
        if self.count < 10:
            msg.linear.x = 0.50  # forward velocity
            msg.angular.z = 1.57  # angular velocity
            self.get_logger().info('Moving in a circle...')
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.get_logger().info('Stopping the robot.')
            flag_ = True
        self.publisher_.publish(msg)
        if flag_:
            self.get_logger().info('Shutting down node.')
            self.timer.cancel()
            rclpy.get_global_executor().create_task(self._shutdown())
            return
        self.count += 1

    async def _shutdown(self):
        """Clean shutdown scheduled outside the callback."""
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = TestPyNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException) as e:
        print(e)
        print("Shutting down...")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
