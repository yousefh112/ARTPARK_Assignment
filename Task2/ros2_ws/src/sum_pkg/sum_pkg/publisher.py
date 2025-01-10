#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math

class RobotStatePublisher(Node):
    def __init__(self):
        # Pass the node name to the parent class's __init__() method
        super().__init__('robot_state_publisher')

        # Create a publisher for the /robot_pose topic
        self.publisher_ = self.create_publisher(PoseStamped, '/robot_pose', 10)

        # Set the publishing rate (e.g., 1 Hz)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize simulated robot pose (x, y, theta)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

    def timer_callback(self):
        # Create a PoseStamped message
        pose_msg = PoseStamped()

        # Fill in the header
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'  # Assuming the pose is in the map frame

        # Fill in the position (x, y, z)
        pose_msg.pose.position.x = self.robot_x
        pose_msg.pose.position.y = self.robot_y
        pose_msg.pose.position.z = 0.0  # Assuming 2D motion, z is 0

        # Fill in the orientation (quaternion)
        # Convert theta (yaw) to a quaternion
        quaternion = self.yaw_to_quaternion(self.robot_theta)
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]

        # Publish the message
        self.publisher_.publish(pose_msg)

        # Log the published pose
        self.get_logger().info(f'Publishing Pose: x={self.robot_x}, y={self.robot_y}, theta={self.robot_theta}')

        # Update the simulated robot pose (for demonstration purposes)
        self.update_robot_pose()

    def yaw_to_quaternion(self, yaw):
        # Convert yaw (in radians) to a quaternion
        return [
            0.0,  # x
            0.0,  # y
            math.sin(yaw / 2),  # z
            math.cos(yaw / 2)   # w
        ]

    def update_robot_pose(self):
        # Simulate robot motion (for demonstration purposes)
        self.robot_x += 0.1  # Move forward in x
        self.robot_y += 0.05  # Move slightly in y
        self.robot_theta += 0.1  # Rotate counterclockwise

def main(args=None):
    # Initialize ROS 2
    rclpy.init(args=args)

    # Create the node
    robot_state_publisher = RobotStatePublisher()

    # Spin the node to keep it running
    rclpy.spin(robot_state_publisher)

    # Destroy the node explicitly when done
    robot_state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()