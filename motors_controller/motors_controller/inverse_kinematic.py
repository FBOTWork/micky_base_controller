#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import serial
import os
import glob
import time
import math


class InverseKinematic(Node):
    def __init__(self):
        super().__init__("inverse_kinematic_node")

        self.CMD_TIMEOUT_S = 0.5
        self.last_cmd_time = self.get_clock().now()
        self.last_cmd_dt_time = self.get_clock().now()
        self.stop_sent = True

        distancia_entre_eixos = 0.41
        bitola = 0.41

        self.lx = distancia_entre_eixos / 2.0
        self.ly = bitola / 2.0
        self.L = self.lx + self.ly

        self.KP_YAW = 2.0
        self.KI_YAW = 0.1
        self.KD_YAW = 0.05
        self.YAW_INTEGRAL_LIMIT = 1.0
        self.VW_LOCK_THRESHOLD = 0.02
        self.VW_TURN_OVERRIDE_THRESHOLD = 0.25
        self.YAW_ERROR_DEADBAND = 0.015
        self.IMU_WARMUP_S = 2.0
        self.IMU_TIMEOUT_S = 0.25
        self.imu_first_sample_time = None
        self.last_imu_time = None
        self.imu_valid = False
        self.imu_rx_buffer = bytearray()
        self.IMU_RX_BUFFER_LIMIT = 2048
        self.imu_ultima_amostra = {"IMU_F": None, "IMU_R": None}
        self.imu_ultimo_tempo = {"IMU_F": None, "IMU_R": None}
        self.imu_nova_amostra = False

        self.current_yaw = 0.0
        self.target_heading = 0.0
        self.heading_locked = False
        self.yaw_integral = 0.0
        self.yaw_last_error = 0.0

        self.KP_VX = 0.8
        self.KI_VX = 0.3
        self.VX_INTEGRAL_LEAK = 0.95
        self.VX_INTEGRAL_LIMIT = 0.3
        self.VX_CORRECTION_LIMIT = 0.15
        self.VY_STRAFE_THRESHOLD = 0.05
        self.VX_INTENT_THRESHOLD = 0.02
        self.ACCEL_LOWPASS_ALPHA = 0.15
        self.BUMP_Z_THRESHOLD = 2.0
        self.GRAVITY = 9.81

        self.accel_x_filtered = 0.0
        self.vx_integral = 0.0
        self.strafe_locked = False

        self.calibrating_accel = True
        self.accel_calib_samples = []
        self.ACCEL_CALIB_SAMPLES_N = 50
        self.accel_x_bias = 0.0

        self.declare_parameter("motor_port", "/dev/base_controller")
        self.declare_parameter("motor_baud", 115200)
        self.declare_parameter("imu_front_frame_id", "imu_front_link")
        self.declare_parameter("imu_rear_frame_id", "imu_rear_link")

        motor_port = self.get_parameter("motor_port").get_parameter_value().string_value
        motor_baud = self.get_parameter("motor_baud").get_parameter_value().integer_value
        self.imu_frame_ids = {
            "IMU_F": self.get_parameter("imu_front_frame_id").get_parameter_value().string_value,
            "IMU_R": self.get_parameter("imu_rear_frame_id").get_parameter_value().string_value,
        }

        self.motor_serial = self.open_serial(self.find_serial_port(motor_port), motor_baud)
        self.get_logger().info(f"Motor (STM32) conectado em {self.motor_serial.port} a {motor_baud} bps")

        self.subscription = self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        self.imu_publishers = {
            "IMU_F": self.create_publisher(Imu, "/imu/front/data_raw", 10),
            "IMU_R": self.create_publisher(Imu, "/imu/rear/data_raw", 10),
        }

        self.watchdog_timer = self.create_timer(0.1, self.watchdog_check)
        self.imu_read_timer = self.create_timer(0.02, self.read_imu_serial)

    def find_serial_port(self, preferred_port):
        self.get_logger().info(f"Aguardando a porta {preferred_port} conectar...")
        while rclpy.ok():
            if preferred_port and os.path.exists(preferred_port):
                return preferred_port
            time.sleep(1)
        raise SystemExit

    def open_serial(self, port, baud):
        while rclpy.ok():
            try:
                conn = serial.Serial(port, baud, timeout=0)
                conn.setDTR(False)
                conn.setRTS(False)
                time.sleep(2)
                conn.reset_input_buffer()
                return conn
            except Exception:
                time.sleep(1)
                port = self.find_serial_port(port)
        raise SystemExit

    def read_imu_serial(self):
        try:
            available = self.motor_serial.in_waiting
            if available:
                self.imu_rx_buffer.extend(self.motor_serial.read(available))

            while b"\n" in self.imu_rx_buffer:
                raw_line, _, remainder = self.imu_rx_buffer.partition(b"\n")
                self.imu_rx_buffer = bytearray(remainder)
                line = raw_line.decode("utf-8", errors="ignore").strip()
                if line:
                    self.process_imu_line(line)

            if self.imu_nova_amostra:
                self.imu_nova_amostra = False
                self.atualizar_yaw_fundido()

            if len(self.imu_rx_buffer) > self.IMU_RX_BUFFER_LIMIT:
                self.imu_rx_buffer.clear()
                self.get_logger().warn("Buffer serial da IMU excedeu o limite; dados descartados.")
        except serial.SerialException as exc:
            self.imu_valid = False
            self.get_logger().warn(f"Erro lendo IMU pela serial do STM32: {exc}", throttle_duration_sec=5.0)

    def process_imu_line(self, line):
        if line.startswith("STATUS:"):
            self.get_logger().info(f"STM32 {line}", throttle_duration_sec=5.0)
            return

        prefixo, separador, payload = line.partition(":")
        if not separador or prefixo not in self.imu_publishers:
            return

        parts = payload.replace(";", "").strip().split(",")
        if len(parts) != 10:
            self.get_logger().warn(f"Linha de IMU inválida: {line}", throttle_duration_sec=5.0)
            return

        try:
            data = [float(value) for value in parts]
        except ValueError:
            self.get_logger().warn(f"Valores inválidos na linha da IMU: {line}", throttle_duration_sec=5.0)
            return

        if not all(math.isfinite(value) for value in data):
            self.get_logger().warn(f"Valores não finitos na linha da IMU: {line}", throttle_duration_sec=5.0)
            return

        quaternion = self.normalizar_quaternion(*data[:4])
        if quaternion is None:
            return
        data[:4] = quaternion

        self.imu_ultima_amostra[prefixo] = data
        self.imu_ultimo_tempo[prefixo] = self.get_clock().now()
        self.imu_nova_amostra = True
        self.imu_publishers[prefixo].publish(self.montar_msg_imu(data, self.imu_frame_ids[prefixo]))

    def montar_msg_imu(self, data, frame_id):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = data[:4]
        msg.orientation_covariance = [
            0.05, 0.0, 0.0,
            0.0, 0.05, 0.0,
            0.0, 0.0, 0.30,
        ]
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = data[4:7]
        msg.angular_velocity_covariance = [
            0.02, 0.0, 0.0,
            0.0, 0.02, 0.0,
            0.0, 0.0, 0.02,
        ]
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = data[7:10]
        msg.linear_acceleration_covariance = [
            0.04, 0.0, 0.0,
            0.0, 0.04, 0.0,
            0.0, 0.0, 0.04,
        ]
        return msg

    def atualizar_yaw_fundido(self):
        # Média circular do yaw das IMUs com amostra recente; se uma cair, segue só com a outra
        now = self.get_clock().now()
        yaws = []
        for prefixo, tempo in self.imu_ultimo_tempo.items():
            if tempo is None or (now - tempo).nanoseconds / 1e9 > self.IMU_TIMEOUT_S:
                continue
            qx, qy, qz, qw = self.imu_ultima_amostra[prefixo][:4]
            yaws.append(math.atan2(
                2.0 * (qw * qz + qx * qy),
                1.0 - 2.0 * (qy * qy + qz * qz),
            ))
        if not yaws:
            return

        self.current_yaw = math.atan2(
            sum(math.sin(yaw) for yaw in yaws),
            sum(math.cos(yaw) for yaw in yaws),
        )
        if self.imu_first_sample_time is None:
            self.imu_first_sample_time = now
        self.last_imu_time = now
        self.imu_valid = True

    @staticmethod
    def normalizar_angulo(angulo):
        return math.atan2(math.sin(angulo), math.cos(angulo))

    @staticmethod
    def normalizar_quaternion(qx, qy, qz, qw):
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if not math.isfinite(norm) or norm < 1e-9:
            return None
        return qx / norm, qy / norm, qz / norm, qw / norm

    def cinematica_inversa(self, vx, vy, vw):
        L = self.L
        v_fl = vx + vy + (vw * L)
        v_fr = vx - vy - (vw * L)
        v_rl = vx - vy + (vw * L)
        v_rr = vx + vy - (vw * L)
        return v_fl, v_fr, v_rl, v_rr

    def send_serial(self, v_fl, v_fr, v_rl, v_rr):
        comando = f"{v_fl:.3f},{v_fr:.3f},{v_rl:.3f},{v_rr:.3f}\n"
        try:
            self.motor_serial.write(comando.encode("utf-8"))
        except Exception as e:
            self.get_logger().warn(f"Erro na serial do motor: {e}")

    def calcular_correcao_heading(self, vw_comandado, dt):
        imu_pronta = (
            self.imu_first_sample_time is not None
            and self.last_imu_time is not None
            and (self.get_clock().now() - self.imu_first_sample_time).nanoseconds / 1e9 >= self.IMU_WARMUP_S
            and (self.get_clock().now() - self.last_imu_time).nanoseconds / 1e9 <= self.IMU_TIMEOUT_S
        )

        if not imu_pronta or not self.imu_valid:
            self.heading_locked = False
            self.yaw_integral = 0.0
            self.yaw_last_error = 0.0
            return vw_comandado

        if abs(vw_comandado) > self.VW_TURN_OVERRIDE_THRESHOLD:
            self.heading_locked = False
            self.yaw_integral = 0.0
            self.yaw_last_error = 0.0
            return vw_comandado

        if not self.heading_locked:
            self.target_heading = self.current_yaw
            self.heading_locked = True
            self.yaw_integral = 0.0
            self.yaw_last_error = 0.0

        erro = self.normalizar_angulo(self.current_yaw - self.target_heading)
        if abs(erro) < self.YAW_ERROR_DEADBAND:
            erro = 0.0

        self.yaw_integral += erro * dt
        self.yaw_integral = max(-self.YAW_INTEGRAL_LIMIT, min(self.YAW_INTEGRAL_LIMIT, self.yaw_integral))

        derivada = (erro - self.yaw_last_error) / dt if dt > 0 else 0.0
        self.yaw_last_error = erro

        return (self.KP_YAW * erro) + (self.KI_YAW * self.yaw_integral) + (self.KD_YAW * derivada)

    def cmd_vel_callback(self, msg):
        now = self.get_clock().now()
        dt = (now - self.last_cmd_dt_time).nanoseconds / 1e9
        dt = dt if 0.0 < dt < 0.5 else 0.02

        self.last_cmd_dt_time = now
        self.last_cmd_time = now

        vx_comandado = msg.linear.x
        vy_comandado = -msg.linear.y
        vw_comandado = -msg.angular.z

        if vx_comandado == 0.0 and vy_comandado == 0.0 and vw_comandado == 0.0:
            vw_effective = 0.0
            self.heading_locked = False
            self.stop_sent = True
        else:
            vw_effective = self.calcular_correcao_heading(vw_comandado, dt)
            self.stop_sent = False

        v_fl, v_fr, v_rl, v_rr = self.cinematica_inversa(vx_comandado, vy_comandado, vw_effective)
        self.send_serial(v_fl, v_fr, v_rl, v_rr)

    def watchdog_check(self):
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.CMD_TIMEOUT_S and not self.stop_sent:
            self.get_logger().warn("Watchdog: Sinal de /cmd_vel perdido. Parada de emergência!")
            self.send_serial(0.0, 0.0, 0.0, 0.0)
            self.stop_sent = True
            self.heading_locked = False


def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send_serial(0.0, 0.0, 0.0, 0.0)
        node.motor_serial.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()