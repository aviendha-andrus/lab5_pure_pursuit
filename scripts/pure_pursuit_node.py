
#!/usr/bin/env python3
import numpy as np
from scipy.spatial import distance, transform
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry

class PurePursuit(Node):
    """ 
    Implement Pure Pursuit on the car using Odometry (is_real=False)
    """
    def __init__(self):
        super().__init__('pure_pursuit_node')

        self.waypoint_asc = True  # waypoint indices are ascending during tracking

        # Topics & Subs, Pubs
        drive_topic = '/drive'
        # odom_topic = '/ego_racecar/odom' 
        odom_topic = '/pf/pose/odom'

        # Subscribe to Odometry
        self.subscribe_to_odom = self.create_subscription(
            Odometry, 
            odom_topic, 
            self.odom_callback, 1)

        # Publish to drive
        self.publish_to_drive = self.create_publisher(
            AckermannDriveStamped, 
            drive_topic, 
            1)
        self.drive_msg = AckermannDriveStamped()

        # Hardcoded waypoint file path
        # self.file = '/sim_ws/src/f1tenth_gym_ros/pure_pursuit/logs/waypoints.csv' 
        # self.file = '/sim_ws/src/pure_pursuit/logs/waypoints.csv' 
        self.file = '/poppin_tires/lab5_pure_pursuit/logs/waypoints.csv'

        csv_data = np.loadtxt(self.file, delimiter=',', skiprows=1)  # Adjust delimiter if needed
        self.waypoints = csv_data[:, [1, 2, 3, 4]]  # Extract x, y, theta (yaw), and speed (columns 1, 2, 3, 4)
        self.numWaypoints = self.waypoints.shape[0]
        
        # Reference speed for the waypoints (adjust as needed)
        # self.speed = self.waypoints[:, 4]  # Assuming speed is in column 4 (index 4)
        
        # self.speed = 0.7
        # self.lookahead = 1.0
        # self.max_turn_angle = 0.5

        self.declare_parameter('speed', 0.0)
        self.speed = self.get_parameter('speed').get_parameter_value().double_value

        self.declare_parameter('lookahead', 0.0)
        self.lookahead = self.get_parameter('lookahead').get_parameter_value().double_value

        self.declare_parameter('max_turn', 0.0)
        self.max_turn_angle = self.get_parameter('max_turn').get_parameter_value().double_value

    def odom_callback(self, odom_msg):
        # Get current pose from Odometry message
        self.current_x = odom_msg.pose.pose.position.x
        self.current_y = odom_msg.pose.pose.position.y
        self.current_position = np.array([self.current_x, self.current_y]).reshape((1, 2))

        # Transform quaternion pose message to rotation matrix
        quat = odom_msg.pose.pose.orientation
        quat = [quat.x, quat.y, quat.z, quat.w]
        R = transform.Rotation.from_quat(quat)
        self.rot = R.as_matrix()

        # Find closest waypoint to where we are
        self.car_dist = distance.cdist(self.current_position, self.waypoints[:, :2], 'euclidean').reshape((self.numWaypoints))  # Only x, y for distance
        self.closest_index = np.argmin(self.car_dist)
        self.closestPoint = self.waypoints[self.closest_index]

        # Find target point beyond lookahead distance
        target = self.get_closest_point_beyond_lookahead_dist(self.lookahead)

        # Homogeneous transformation to the target point
        trans_target_point = self.translatePoint(target[:2])  # Ignore theta for transformation
        
        # Calculate curvature/steering angle
        y = trans_target_point[1]
        gamma = self.max_turn_angle * (2 * y / self.lookahead**2)

        # Publish drive message, don't forget to limit the steering angle.
        gamma = np.clip(gamma, -0.35, 0.35)
        self.drive_msg.drive.steering_angle = gamma
        self.drive_msg.drive.speed = self.speed
        self.publish_to_drive.publish(self.drive_msg)
        # error checking, delete later 
        # print("steering = {}, speed = {}".format(round(self.drive_msg.drive.steering_angle, 2), round(self.drive_msg.drive.speed, 2)))

    def get_closest_point_beyond_lookahead_dist(self, threshold):
        current_index = self.closest_index
        dist = self.car_dist[current_index]
        
        while dist < threshold:
            if self.waypoint_asc:
                current_index += 1
                if current_index >= len(self.waypoints):
                    current_index = 0
                dist = self.car_dist[current_index]
            else:
                current_index -= 1
                if current_index < 0:
                    current_index = len(self.waypoints) - 1
                dist = self.car_dist[current_index]

        point = self.waypoints[current_index]

        return point

    def translatePoint(self, target):
        H = np.zeros((4, 4))
        H[0:3, 0:3] = np.linalg.inv(self.rot)
        H[0, 3] = self.current_x
        H[1, 3] = self.current_y
        H[3, 3] = 1.0
        pvect = target - self.current_position
        convertedTarget = (H @ np.array((pvect[0, 0], pvect[0, 1], 0, 0))).reshape((4))
        
        return convertedTarget


def main(args=None):
    rclpy.init(args=args)
    print("PurePursuit Initialized")
   
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)

    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
