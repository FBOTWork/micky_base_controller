#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import math


class ImuOdomNode(Node):
    def __init__(self):
        super().__init__("imu_odom_node")

        
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        
        self.vx = 0.0
        self.vy = 0.0
        self.v_yaw = 0.0

        self.last_time = self.get_clock().now()

        
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)

        self.cmd_vel_sub = self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        self.imu_sub = self.create_subscription(Imu, "/imu/data_raw", self.imu_callback, 10)

        
        self.timer = self.create_timer(0.02, self.update_odometry)

        self.get_logger().info("Nó de odometria iniciado.")

    def cmd_vel_callback(self, msg: Twist):
        
        self.vx = msg.linear.x
        self.vy = msg.linear.y

    def imu_callback(self, msg: Imu):
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        
        self.yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )
        
        self.v_yaw = msg.angular_velocity.z

    def update_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt <= 0.0:
            return

        
        delta_x = (self.vx * math.cos(self.yaw) - self.vy * math.sin(self.yaw)) * dt
        delta_y = (self.vx * math.sin(self.yaw) + self.vy * math.cos(self.yaw)) * dt

        self.x += delta_x
        self.y += delta_y

        
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"

        # Posição
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)

        
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.v_yaw

        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = ImuOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()