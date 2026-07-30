#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class CmdVelToOdom(Node):
    def __init__(self):
        super().__init__("cmd_vel_to_odom_node")

        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("cmd_timeout", 0.3)
        self.declare_parameter("linear_accel_limit", 0.5)
        self.declare_parameter("angular_accel_limit", 0.5)

        self.odom_frame = self.get_parameter("odom_frame").get_parameter_value().string_value
        self.base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        publish_rate = self.get_parameter("publish_rate").get_parameter_value().double_value
        self.cmd_timeout = self.get_parameter("cmd_timeout").get_parameter_value().double_value
        self.linear_accel_limit = self.get_parameter("linear_accel_limit").get_parameter_value().double_value
        self.angular_accel_limit = self.get_parameter("angular_accel_limit").get_parameter_value().double_value

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        # velocidade "real" estimada (rampeada) usada pra integrar a pose
        self.vx = 0.0
        self.vy = 0.0
        self.vw = 0.0
        # ultimo /cmd_vel recebido: alvo que a rampa persegue, nao o valor usado direto
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_vw = 0.0
        self.last_cmd_time = self.get_clock().now()
        self.last_update_time = self.get_clock().now()

        # sem encoder nas rodas: essa e a unica fonte de "odometria" possivel hoje,
        # integrando o /cmd_vel comandado como se fosse a velocidade real executada
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        self.create_timer(1.0 / publish_rate, self.update)

    def cmd_vel_callback(self, msg):
        # usa os valores crus do /cmd_vel (convencao padrao do ROS): a inversao de
        # eixos em cmd_vel_to_serial existe so pra bater com a fiacao do firmware,
        # o deslocamento fisico real segue o que foi originalmente comandado
        self.target_vx = msg.linear.x
        self.target_vy = msg.linear.y
        self.target_vw = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    @staticmethod
    def _ramp(current, target, accel_limit, dt):
        # sem encoder pra saber a velocidade real, isso so aproxima a inercia do
        # motor: sem essa rampa, uma mudanca de comando vira deslocamento
        # instantaneo na odometria, que sempre corre na frente do robo real
        max_step = accel_limit * dt
        diff = target - current
        if diff > max_step:
            return current + max_step
        if diff < -max_step:
            return current - max_step
        return target

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds / 1e9
        self.last_update_time = now

        elapsed_since_cmd = (now - self.last_cmd_time).nanoseconds / 1e9
        if elapsed_since_cmd > self.cmd_timeout:
            self.target_vx = self.target_vy = self.target_vw = 0.0

        self.vx = self._ramp(self.vx, self.target_vx, self.linear_accel_limit, dt)
        self.vy = self._ramp(self.vy, self.target_vy, self.linear_accel_limit, dt)
        self.vw = self._ramp(self.vw, self.target_vw, self.angular_accel_limit, dt)

        self.x += (self.vx * math.cos(self.theta) - self.vy * math.sin(self.theta)) * dt
        self.y += (self.vx * math.sin(self.theta) + self.vy * math.cos(self.theta)) * dt
        self.theta += self.vw * dt

        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom_msg.twist.twist.linear.x = self.vx
        odom_msg.twist.twist.linear.y = self.vy
        odom_msg.twist.twist.angular.z = self.vw

        # sem encoder, o erro cresce sem limite com a distancia percorrida;
        # covariancia alta avisa o EKF pra confiar pouco nessa estimativa
        odom_msg.pose.covariance[0] = 0.05
        odom_msg.pose.covariance[7] = 0.05
        odom_msg.pose.covariance[35] = 0.1
        odom_msg.twist.covariance[0] = 0.05
        odom_msg.twist.covariance[7] = 0.05
        odom_msg.twist.covariance[35] = 0.1

        self.odom_pub.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
