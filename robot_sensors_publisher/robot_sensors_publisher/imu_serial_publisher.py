import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

import serial
import time
import math
import os
import glob


class ImuSerialPublisher(Node):

    def __init__(self):
        super().__init__("imu_serial_publisher")

        # =========================
        # PARAMETROS
        # =========================
        self.declare_parameter("port", "/dev/arduino_robo")
        self.declare_parameter("baudrate", 115200)

        port = self.get_parameter("port").value
        baudrate = self.get_parameter("baudrate").value

        # =========================
        # SERIAL
        # =========================
        self.serial_port = self.find_serial_port(port)
        self.serial_conn = self.open_serial(self.serial_port, baudrate)

        self.serial_conn.setDTR(False)
        self.serial_conn.setRTS(False)

        time.sleep(2)
        self.serial_conn.reset_input_buffer()

        self.get_logger().info(f"Serial conectada em {self.serial_port}")

        # =========================
        # PUBLICADORES
        # =========================
        self.pub_imu = self.create_publisher(Imu, "/imu/data", 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer 100Hz
        self.timer = self.create_timer(0.01, self.read_and_publish)

    # =========================
    # SERIAL HELPERS
    # =========================
    def find_serial_port(self, port):
        if port and os.path.exists(port):
            return port

        while rclpy.ok():
            devices = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
            if devices:
                return devices[0]
            time.sleep(1)

    def open_serial(self, port, baudrate):
        while rclpy.ok():
            try:
                return serial.Serial(port, baudrate, timeout=0.1)
            except:
                time.sleep(1)

    # =========================
    # EULER → QUATERNION
    # =========================
    def euler_to_quaternion(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return qx, qy, qz, qw

    # =========================
    # LOOP
    # =========================
    def read_and_publish(self):

        if self.serial_conn.in_waiting > 0:

            raw = self.serial_conn.read_until(b";")
            line = raw.decode("utf-8", errors="ignore").strip().replace(";", "")

            if not line:
                return

            parts = line.split(",")

            # NOVO FORMATO → 12 VALORES
            if len(parts) == 12:
                try:
                    roll = float(parts[0])
                    pitch = float(parts[1])
                    yaw = float(parts[2])

                    ax = float(parts[3])
                    ay = float(parts[4])
                    az = float(parts[5])

                    gx = float(parts[6])
                    gy = float(parts[7])
                    gz = float(parts[8])

                    mx = float(parts[9])
                    my = float(parts[10])
                    mz = float(parts[11])

                    # CONVERTER PRA RAD
                    roll = math.radians(roll)
                    pitch = math.radians(pitch)
                    yaw = math.radians(yaw)

                    # =========================
                    # IMU MESSAGE
                    # =========================
                    imu_msg = Imu()
                    imu_msg.header.stamp = self.get_clock().now().to_msg()
                    imu_msg.header.frame_id = "base_link"

                    qx, qy, qz, qw = self.euler_to_quaternion(roll, pitch, yaw)

                    imu_msg.orientation.x = qx
                    imu_msg.orientation.y = qy
                    imu_msg.orientation.z = qz
                    imu_msg.orientation.w = qw

                    imu_msg.angular_velocity.x = gx
                    imu_msg.angular_velocity.y = gy
                    imu_msg.angular_velocity.z = gz

                    imu_msg.linear_acceleration.x = ax
                    imu_msg.linear_acceleration.y = ay
                    imu_msg.linear_acceleration.z = az

                    self.pub_imu.publish(imu_msg)

                    # =========================
                    # TF
                    # =========================
                    t = TransformStamped()
                    t.header.stamp = imu_msg.header.stamp
                    t.header.frame_id = "base_link"
                    t.child_frame_id = "imu_link"

                    t.transform.rotation.x = qx
                    t.transform.rotation.y = qy
                    t.transform.rotation.z = qz
                    t.transform.rotation.w = qw

                    self.tf_broadcaster.sendTransform(t)

                except ValueError:
                    pass


def main(args=None):
    rclpy.init(args=args)

    node = ImuSerialPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.serial_conn.is_open:
            node.serial_conn.close()

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
