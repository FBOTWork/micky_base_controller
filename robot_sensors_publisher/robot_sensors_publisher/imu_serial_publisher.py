import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from std_msgs.msg import Bool

import serial
import time
import math
import os
import glob

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class ImuSerialPublisher(Node):

    def __init__(self):

        super().__init__('imu_serial_publisher')

        # =========================
        # PARAMETROS
        # =========================

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value

        # =========================
        # TF BROADCASTER
        # =========================

        self.tf_broadcaster = TransformBroadcaster(self)

        # =========================
        # TOPICOS IMU
        # =========================

        self.pub_front_roll = self.create_publisher(Float32, '/imu/front/roll', 10)
        self.pub_front_pitch = self.create_publisher(Float32, '/imu/front/pitch', 10)

        self.pub_back_roll = self.create_publisher(Float32, '/imu/back/roll', 10)
        self.pub_back_pitch = self.create_publisher(Float32, '/imu/back/pitch', 10)

        # =========================
        # TOPICO BOTAO ESTOP
        # =========================

        self.pub_estop = self.create_publisher(Bool, '/robot/estop', 10)

        # =========================
        # ESTADO DO MOTOR
        # =========================

        self.pub_motor = self.create_publisher(Bool, '/robot/estado_motor', 10)

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

        # Timer 100Hz
        self.timer = self.create_timer(0.01, self.read_and_publish)

    def find_serial_port(self, port):
        """Retorna porta serial válida ou aguarda Arduino conectar."""
        if port and os.path.exists(port):
            self.get_logger().info(f"Porta especificada existe: {port}")
            return port

        self.get_logger().info(f"Porta {port} não encontrada. Procurando dispositivos USB...")

        while rclpy.ok():
            candidates = sorted(glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*'))

            if candidates:
                selected = candidates[0]
                self.get_logger().info(f"Encontrado dispositivo serial: {selected}")
                return selected

            self.get_logger().info("Nenhum dispositivo serial encontrado. Conecte o Arduino e aguarde...")
            time.sleep(1)

        raise SystemExit

    def open_serial(self, port, baudrate):
        while rclpy.ok():
            try:
                ser = serial.Serial(port, baudrate, timeout=0.1)
                return ser
            except serial.SerialException as e:
                self.get_logger().error(f"Erro abrindo porta {port}: {e}")
                self.get_logger().info("Tentando detectar novamente a porta...")
                time.sleep(1)
                port = self.find_serial_port(port)

        raise SystemExit

    # =========================
    # CONVERTE EULER -> QUATERNION
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
    # PUBLICA TF
    # =========================

    def publish_tf(self, frame_name, roll, pitch):

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "wx200/base_link"
        t.child_frame_id = frame_name

        if frame_name == "imu_front":

            t.transform.translation.x = 0.5
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0

        else:

            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0

        roll = math.radians(roll)
        pitch = math.radians(pitch)

        qx, qy, qz, qw = self.euler_to_quaternion(roll, pitch, 0.0)

        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

    # =========================
    # LE SERIAL E PUBLICA
    # =========================

    def read_and_publish(self):

        if self.serial_conn.in_waiting > 0:

            raw = self.serial_conn.read_until(b';')

            line = raw.decode('utf-8', errors='ignore').strip().replace(';', '')

            if line:

                parts = line.split(',')

                if len(parts) == 20:

                    try:

                        # IMU FRONT
                        roll1 = float(parts[0])
                        pitch1 = float(parts[1])

                        # IMU BACK
                        roll2 = float(parts[9])
                        pitch2 = float(parts[10])

                        # BOTAO
                        botao = int(parts[18])

                        # MOTOR
                        motor = int(parts[19])

                        msg = Float32()

                        msg.data = roll1
                        self.pub_front_roll.publish(msg)

                        msg.data = pitch1
                        self.pub_front_pitch.publish(msg)

                        msg.data = roll2
                        self.pub_back_roll.publish(msg)

                        msg.data = pitch2
                        self.pub_back_pitch.publish(msg)

                        # PUBLICA TF

                        self.publish_tf("imu_front", roll1, pitch1)
                        self.publish_tf("imu_back", roll2, pitch2)

                        # PUBLICA ESTOP

                        estop_msg = Bool()

                        if botao == 1:
                            estop_msg.data = True #aqui o
                        else:
                            estop_msg.data = False

                        self.pub_estop.publish(estop_msg)

                        # PUBLICA ESTADO DO MOTOR

                        motor_msg = Bool()

                        if motor == 1:
                            motor_msg.data = True
                        else:
                            motor_msg.data = False

                        self.pub_motor.publish(motor_msg)

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


if __name__ == '__main__':
    main()