#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import time


class ImuNode(Node):
    def __init__(self):
        super().__init__("imu_node")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("frame_id", "imu_link")

        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value

        self.ser = serial.Serial(port, baud, timeout=1)
        
        self.ser.setDTR(False)
        self.ser.setRTS(False)
        time.sleep(2.0)
        self.ser.reset_input_buffer()

        self.pub_imu = self.create_publisher(Imu, "/imu/data_raw", 10)

        self.get_logger().info("IMU node started")

        
        self.timer = self.create_timer(1.0 / 50.0, self.read_and_publish)  # 50Hz

    def read_and_publish(self):
        try:
            line = self.ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                return
            self.process_line(line)
        except Exception as e:
            self.get_logger().warn(f"Serial error: {e}", throttle_duration_sec=5.0)

    def process_line(self, line):
        try:
            if "IMU:" not in line:
                return

            line = line.split("IMU:")[-1]
            line = line.replace(";", "").replace(" ", "").strip()
            parts = line.split(",")

            if len(parts) != 10:
                self.get_logger().warn(f"Invalid IMU line: {line}", throttle_duration_sec=5.0)
                return

            try:
                data = [float(x) for x in parts]
            except ValueError:
                self.get_logger().warn(f"Invalid numeric value in IMU line: {line}", throttle_duration_sec=5.0)
                return

            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id

            msg.orientation.x = data[0]
            msg.orientation.y = data[1]
            msg.orientation.z = data[2]
            msg.orientation.w = data[3]
            msg.orientation_covariance = [
                0.05, 0.0, 0.0,
                0.0, 0.05, 0.0,
                0.0, 0.0, 0.30,
            ]

            msg.angular_velocity.x = data[4]
            msg.angular_velocity.y = data[5]
            msg.angular_velocity.z = data[6]
            msg.angular_velocity_covariance = [
                0.02, 0.0, 0.0,
                0.0, 0.02, 0.0,
                0.0, 0.0, 0.02,
            ]

            msg.linear_acceleration.x = data[7]
            msg.linear_acceleration.y = data[8]
            msg.linear_acceleration.z = data[9]
            msg.linear_acceleration_covariance = [
                0.04, 0.0, 0.0,
                0.0, 0.04, 0.0,
                0.0, 0.0, 0.04,
            ]

            self.pub_imu.publish(msg)

        except Exception as e:
            self.get_logger().warn(f"Parsing error: {e} | line: {line}", throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()