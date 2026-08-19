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
        self.YAW_ERROR_DEADBAND = 0.015
        self.IMU_WARMUP_S = 2.0
        self.imu_first_sample_time = None

        self.current_yaw = 0.0
        self.target_heading = 0.0
        self.heading_locked = False
        self.yaw_integral = 0.0
        self.yaw_last_error = 0.0

        self.declare_parameter("motor_port", "/dev/ttyUSB1")
        self.declare_parameter("motor_baud", 115200)

        motor_port = self.get_parameter("motor_port").get_parameter_value().string_value
        motor_baud = self.get_parameter("motor_baud").get_parameter_value().integer_value

        self.motor_serial = self.open_serial(self.find_serial_port(motor_port), motor_baud)
        self.get_logger().info(f"Motor (STM32) conectado em {self.motor_serial.port} a {motor_baud} bps")

        self.subscription = self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        self.imu_subscription = self.create_subscription(Imu, "/imu/data_raw", self.imu_callback, 10)

        self.watchdog_timer = self.create_timer(0.1, self.watchdog_check)

    def find_serial_port(self, preferred_port):
        if preferred_port and os.path.exists(preferred_port):
            return preferred_port
        while rclpy.ok():
            candidates = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
            if candidates:
                return candidates[0]
            time.sleep(1)
        raise SystemExit

    def open_serial(self, port, baud):
        while rclpy.ok():
            try:
                conn = serial.Serial(port, baud, timeout=0.05)
                conn.setDTR(False)
                conn.setRTS(False)
                time.sleep(2)
                conn.reset_input_buffer()
                return conn
            except Exception:
                time.sleep(1)
                port = self.find_serial_port(port)
        raise SystemExit

    def imu_callback(self, msg: Imu):
        if self.imu_first_sample_time is None:
            self.imu_first_sample_time = self.get_clock().now()

        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        self.current_yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )

    @staticmethod
    def normalizar_angulo(angulo):
        return math.atan2(math.sin(angulo), math.cos(angulo))

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
            and (self.get_clock().now() - self.imu_first_sample_time).nanoseconds / 1e9 >= self.IMU_WARMUP_S
        )

        if abs(vw_comandado) > self.VW_LOCK_THRESHOLD or not imu_pronta:
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
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()