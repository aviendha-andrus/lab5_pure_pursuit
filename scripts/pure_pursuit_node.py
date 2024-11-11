#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


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

# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node

# import numpy as np
# from sensor_msgs.msg import LaserScan
# from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
# # TODO CHECK: include needed ROS msg type headers and libraries

# class PurePursuit(Node):
#     """ 
#     Implement Pure Pursuit on the car
#     This is just a template, you are free to implement your own node!
#     """
#     def __init__(self):
#         super().__init__('pure_pursuit_node')
#         # TODO: create ROS subscribers and publishers

#     def pose_callback(self, pose_msg):
#         pass
#         # TODO: find the current waypoint to track using methods mentioned in lecture

#         # TODO: transform goal point to vehicle frame of reference

#         # TODO: calculate curvature/steering angle

#         # TODO: publish drive message, don't forget to limit the steering angle.

# def main(args=None):
#     rclpy.init(args=args)
#     print("PurePursuit Initialized")
#     pure_pursuit_node = PurePursuit()
#     rclpy.spin(pure_pursuit_node)

#     pure_pursuit_node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
