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
        self.serial_conn = None

        if self.serial_port:
            self.serial_conn = self.open_serial(self.serial_port, baudrate)

        if self.serial_conn is not None:
            self.serial_conn.setDTR(False)
            self.serial_conn.setRTS(False)

            time.sleep(2)
            self.serial_conn.reset_input_buffer()

            self.get_logger().info(f"Serial conectada em {self.serial_port}")
        else:
            self.get_logger().error(
                "Não foi possível abrir a porta serial. O tópico não será publicado até que o dispositivo esteja disponível."
            )

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

        devices = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
        if devices:
            return devices[0]

        self.get_logger().warn(
            "Nenhuma porta serial compatível foi encontrada em /dev/ttyACM* ou /dev/ttyUSB*."
        )
        return None

    def open_serial(self, port, baudrate):
        baudrates = [baudrate] + [rate for rate in (115200, 57600, 9600, 38400, 230400) if rate != baudrate]

        for rate in baudrates:
            try:
                conn = serial.Serial(port, rate, timeout=0.1)
                self.get_logger().info(f"Porta serial aberta em {port} com baudrate {rate}")
                return conn
            except serial.SerialException as exc:
                self.get_logger().warning(f"Falha ao abrir {port} com baudrate {rate}: {exc}")
            except Exception as exc:
                self.get_logger().warning(f"Erro inesperado ao abrir {port}: {exc}")

        return None

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
        if self.serial_conn is None:
            return

        try:
            if self.serial_conn.in_waiting <= 0:
                return

            raw = self.serial_conn.readline()
            line = raw.decode("utf-8", errors="ignore").strip().replace(";", "")

            if not line:
                return

            parts = line.split(",")

            # Aceita mensagens com pelo menos 12 valores e ignora campos extras.
            if len(parts) < 12:
                self.get_logger().warning(f"Formato inesperado recebido na serial: {line}")
                return

            values = parts[:12]

            try:
                roll = float(values[0])
                pitch = float(values[1])
                yaw = float(values[2])

                ax = float(values[3])
                ay = float(values[4])
                az = float(values[5])

                gx = float(values[6])
                gy = float(values[7])
                gz = float(values[8])

                mx = float(values[9])
                my = float(values[10])
                mz = float(values[11])

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

            except ValueError as exc:
                self.get_logger().warning(f"Erro ao converter os valores recebidos: {line} ({exc})")
        except serial.SerialException as exc:
            self.get_logger().warning(f"Erro de leitura na serial: {exc}")


def main(args=None):
    rclpy.init(args=args)

    node = ImuSerialPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if getattr(node, "serial_conn", None) is not None and node.serial_conn.is_open:
            node.serial_conn.close()

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
