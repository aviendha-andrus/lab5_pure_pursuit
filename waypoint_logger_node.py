#!/usr/bin/env python3
"""Module for waypoint logging."""

import argparse
import csv
from collections import deque
import numpy as np
import rclpy
import tf_transformations
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from geometry_msgs.msg import Point, Pose
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

# LOG_FILE_NAME = '/sim_ws/src/pure_pursuit/logs/waypoints.csv'

RED = ColorRGBA()
RED.r = 1.0
RED.a = 1.0


class WaypointLoggerNode(Node):
    """Node for logging waypoints."""
    
    def __init__(
        self,
        log_file_name: str,
        debug: bool = False,
        time_between_logs: float = 0.01,
    ):
        """Creates a new WaypointLogger"""
        super().__init__('waypoint_logger_node')

        self.debug: bool = debug
        self.time_between_logs: float = time_between_logs
        self.last_log_time: float = -1.0

        #
        # Publishers and subscribers
        #
        self.pf_pose_odom_sub = self.create_subscription(
            msg_type=Odometry,
            topic="/ego_racecar/odom",
            callback=self.pf_pose_odom_callback,
            qos_profile=10,
        )
        self.waypoints_pub = self.create_publisher(
            msg_type=Marker,
            topic="/waypoints",
            qos_profile=5,
        )

        #
        # Open file for writing
        #
        self.log_file = open(log_file_name, 'w+')
        self.csv_writer = csv.DictWriter(
            self.log_file,
            fieldnames=['time', 'x', 'y', 'theta', 'speed']
        )
        self.csv_writer.writeheader()

        #
        # Marker for visual display
        #
        self.dots = Marker()
        self.dots.header.frame_id = "map"
        self.dots.ns = 'waypoint_logger'
        self.dots.action = Marker.ADD
        self.dots.pose.orientation.w = 1.0
        self.dots.id = 1
        self.dots.type = Marker.POINTS
        self.dots.scale.x = 0.1
        self.dots.scale.y = 0.1
        self.dots.points = []
        self.dots.colors = []
        
        print(f"{self.__class__.__name__} initialized with\n{log_file_name=!r}")

    def pf_pose_odom_callback(self, msg):
        """Called when pf/pose/odom receives a message."""
        
        stamp = msg.header.stamp
        time = stamp.sec + 1.0e-9 * stamp.nanosec
        
        if time - self.last_log_time < self.time_between_logs:
            return
        self.last_log_time = time

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        speed = np.linalg.norm(
            np.array(
                [
                    msg.twist.twist.linear.x, 
                    msg.twist.twist.linear.y, 
                    msg.twist.twist.linear.z,
                ]
            ),
            2,
        )
        quaternion = np.array([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        ])
        euler = tf_transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]

        data = {
            'time': time,
            'x': position.x,
            'y': position.y,
            'theta': yaw,
            'speed': speed,
        }
        self.csv_writer.writerow(data)

        #
        # Write visual data
        #
        if self.debug:
            print(f"Logged:\n{data}")
            point = Point()
            point.x = position.x
            point.y = position.y
            point.z = 0.0
            self.dots.points.append(point)
            self.dots.colors.append(RED)
            self.waypoints_pub.publish(self.dots)

    def destroy_node(self, *args, **kwargs) -> None:
        self.log_file.close()
        super().destroy_node(*args, **kwargs)


def main(args=None):
    """Main entry point function."""

    parser = argparse.ArgumentParser(
        description="Logs the waypoints given by the particle filter "
            "and logs them into the file specified by the given file "
            "name."
    )
    parser.add_argument(
        'log_file_name',
        help="Name of the log file.",
    )
    parser.add_argument(
        '-d',
        '--debug',
        action='store_true',
        help="If true, print and publish debug messages."
    )
    parser.add_argument(
        '-t',
        '--time-between-logs',
        type=float,
        default=0.01,
    )

    parser_args = parser.parse_args()

    rclpy.init(args=args)
    
    node = WaypointLoggerNode(
        log_file_name=parser_args.log_file_name,
        debug=parser_args.debug,
        time_between_logs=parser_args.time_between_logs,
    )
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()