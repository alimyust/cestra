from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class DiffDriveOdom(Node):
    def __init__(self):
        super().__init__('diff_drive_odom')
        
        # inside your class
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.last_time = self.get_clock().now()
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

    # timer callback at ~20 Hz
    def update_odom(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        # read actual wheel angles from /joint_states for realism
        # or integrate v_left/v_right if you trust commands
        v = (self.v_right + self.v_left) * 0.5 * self.wheel_radius
        w = (self.v_right - self.v_left) * self.wheel_radius / self.wheel_base

        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt
        self.yaw += w * dt
        q = quaternion_from_euler(0, 0, self.yaw)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)

        # broadcast TF
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)