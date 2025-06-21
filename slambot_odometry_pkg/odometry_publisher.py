#!/usr/bin/env python3

import math

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

from slambot_interfaces.msg import EncoderTicks

import tf_transformations

# Physical robot constants
WHEEL_BASE = 0.233
WHEEL_RADIUS = 0.036
WHEEL_CIRCUMFERENCE = 2 * math.pi * WHEEL_RADIUS
TICKS_PER_REV = 292

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__("odometry_publisher")
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Initial robot pose
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_theta = 0.0

        # Last encoder tick values
        self.last_left_encoder_ticks = 0
        self.last_right_encoder_ticks = 0

        self.encoder_ticks_subscriber_ = self.create_subscription(EncoderTicks, "encoder_ticks", self.callback_update_pose, 10)
        self.odom_publisher_ = self.create_publisher(Odometry, 'odom', 10)

        self.get_logger().info("Running odometry publisher node")
    
    def callback_update_pose(self, msg: EncoderTicks):
        left_encoder_ticks = msg.left_encoder
        right_encoder_ticks = msg.right_encoder
        self.get_logger().info(f"Left: {left_encoder_ticks}, Right: {right_encoder_ticks}")

        left_wheel_distance = WHEEL_CIRCUMFERENCE * ((left_encoder_ticks - self.last_left_encoder_ticks) / TICKS_PER_REV)
        right_wheel_distance = WHEEL_CIRCUMFERENCE * ((right_encoder_ticks - self.last_right_encoder_ticks) / TICKS_PER_REV)

        self.last_left_encoder_ticks = left_encoder_ticks
        self.last_right_encoder_ticks = right_encoder_ticks

        delta_s = (left_wheel_distance + right_wheel_distance) / 2.0
        delta_theta = (right_wheel_distance - left_wheel_distance) / WHEEL_BASE

        # Update robot pose
        self.pose_x += delta_s * math.cos(self.pose_theta + delta_theta / 2.0)
        self.pose_y += delta_s * math.sin(self.pose_theta + delta_theta / 2.0)
        self.pose_theta += delta_theta

        self.get_logger().info(f"{self.pose_x}, {self.pose_y}, {self.pose_theta}")

        self.publish_odometry()

    def publish_odometry(self):
        # Publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = self.pose_x
        odom_msg.pose.pose.position.y = self.pose_y
        odom_msg.pose.pose.position.z = 0.0

        # Convert yaw (theta) to quaternion
        q = tf_transformations.quaternion_from_euler(0, 0, self.pose_theta)
        odom_msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        self.odom_publisher_.publish(odom_msg)

        t = TransformStamped()
        t.header.stamp = odom_msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.pose_x
        t.transform.translation.y = self.pose_y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom_msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()