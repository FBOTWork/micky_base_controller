#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import serial
import os
import glob
import time
import math


class CmdVelConverter(Node):
    def __init__(self):
        super().__init__("cmd_vel_converter_node")

        self.CMD_TIMEOUT_S = 0.2
        self.last_cmd_time = self.get_clock().now()
        self.stop_sent = True

        # Geometria do robô
        self.L = 0.38

        # --- Heading lock / PID de yaw (giro em torno de Z) ---
        self.KP_YAW = 2.0
        self.KI_YAW = 0.1
        self.KD_YAW = 0.05
        self.YAW_INTEGRAL_LIMIT = 1.0
        self.VW_LOCK_THRESHOLD = 0.02

        self.current_yaw = 0.0
        self.target_heading = 0.0
        self.heading_locked = False
        self.yaw_integral = 0.0
        self.yaw_last_error = 0.0
        self.last_cmd_dt_time = self.get_clock().now()

        # --- Strafe lock / correção de Vx (deriva linear durante strafe) ---
        # IMPORTANTE: diferente do PID de yaw, isso NÃO é uma malha confiável
        # a longo prazo — acelerômetro integrado vira velocidade que diverge
        # rápido, e o solavanco vertical (roletes) contamina o eixo X sempre
        # que o chassi inclina no impacto. Por isso: integrador com "vazamento"
        # (decai sozinho), filtro passa-baixa forte, rejeição de amostra
        # durante pico de solavanco (via accel Z), e correção com teto baixo.
        # Trate como um "trim" de curto prazo, não uma odometria de verdade.
        self.KP_VX = 0.8
        self.KI_VX = 0.3
        self.VX_INTEGRAL_LEAK = 0.95        # a cada ciclo, o acumulado decai 5%
        self.VX_INTEGRAL_LIMIT = 0.3
        self.VX_CORRECTION_LIMIT = 0.15     # nunca corrige mais que isso (m/s)
        self.VY_STRAFE_THRESHOLD = 0.05     # abaixo disso, não considera "strafe"
        self.VX_INTENT_THRESHOLD = 0.02     # se Vx pedido > isso, não é strafe puro
        self.ACCEL_LOWPASS_ALPHA = 0.15     # filtro passa-baixa (0=trava, 1=sem filtro)
        self.BUMP_Z_THRESHOLD = 2.0         # m/s² de desvio em Z = descarta a amostra
        self.GRAVITY = 9.81

        self.accel_x_filtered = 0.0
        self.vx_integral = 0.0
        self.strafe_locked = False

        # calibração de bias do acelerômetro X no boot (robô parado)
        self.calibrating_accel = True
        self.accel_calib_samples = []
        self.ACCEL_CALIB_SAMPLES_N = 50
        self.accel_x_bias = 0.0

        # --- Parâmetros de conexão ---
        self.declare_parameter("motor_port", "/dev/ttyUSB1")
        self.declare_parameter("motor_baud", 115200)

        motor_port = self.get_parameter("motor_port").get_parameter_value().string_value
        motor_baud = self.get_parameter("motor_baud").get_parameter_value().integer_value

        self.motor_serial = self.open_serial(self.find_serial_port(motor_port), motor_baud)
        self.get_logger().info(f"Motor (STM32) conectado em {self.motor_serial.port} a {motor_baud} bps")

        self.subscription = self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        self.imu_subscription = self.create_subscription(Imu, "/imu/data_raw", self.imu_callback, 10)
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_check)

    # ---------- Conexão serial ----------
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

    # ---------- IMU: yaw (para heading) + accel X filtrado (para strafe) ----------
    def imu_callback(self, msg: Imu):
        qx, qy, qz, qw = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        self.current_yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )

        ax_raw = msg.linear_acceleration.x
        az_raw = msg.linear_acceleration.z

        # calibração de bias no boot (robô precisa estar parado nesse momento)
        if self.calibrating_accel:
            self.accel_calib_samples.append(ax_raw)
            if len(self.accel_calib_samples) >= self.ACCEL_CALIB_SAMPLES_N:
                self.accel_x_bias = sum(self.accel_calib_samples) / len(self.accel_calib_samples)
                self.calibrating_accel = False
                self.get_logger().info(f"Bias do acelerômetro X: {self.accel_x_bias:.4f} m/s²")
            return

        ax = ax_raw - self.accel_x_bias

        # rejeita a amostra inteira se estiver no meio de um solavanco vertical —
        # nesses instantes o eixo X vem contaminado pela inclinação do impacto
        if abs(az_raw - self.GRAVITY) > self.BUMP_Z_THRESHOLD:
            return

        # filtro passa-baixa (média móvel exponencial), reduz ruído residual
        self.accel_x_filtered = (
            self.ACCEL_LOWPASS_ALPHA * ax
            + (1.0 - self.ACCEL_LOWPASS_ALPHA) * self.accel_x_filtered
        )

    @staticmethod
    def normalizar_angulo(angulo):
        return math.atan2(math.sin(angulo), math.cos(angulo))

    # ---------- Cinemática inversa mecanum ----------
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

    # ---------- PID de heading (giro em torno de Z) ----------
    def calcular_correcao_heading(self, vw_comandado, dt):
        if abs(vw_comandado) > self.VW_LOCK_THRESHOLD:
            self.heading_locked = False
            self.yaw_integral = 0.0
            self.yaw_last_error = 0.0
            return vw_comandado

        if not self.heading_locked:
            self.target_heading = self.current_yaw
            self.heading_locked = True
            self.yaw_integral = 0.0
            self.yaw_last_error = 0.0

        erro = self.normalizar_angulo(self.target_heading - self.current_yaw)

        self.yaw_integral += erro * dt
        self.yaw_integral = max(-self.YAW_INTEGRAL_LIMIT, min(self.YAW_INTEGRAL_LIMIT, self.yaw_integral))

        derivada = (erro - self.yaw_last_error) / dt if dt > 0 else 0.0
        self.yaw_last_error = erro

        return (self.KP_YAW * erro) + (self.KI_YAW * self.yaw_integral) + (self.KD_YAW * derivada)

    # ---------- Correção de Vx durante strafe (deriva linear) ----------
    def calcular_correcao_strafe(self, vx_comandado, vy_comandado, dt):
        eh_strafe = (
            abs(vy_comandado) > self.VY_STRAFE_THRESHOLD
            and abs(vx_comandado) < self.VX_INTENT_THRESHOLD
        )

        if not eh_strafe:
            self.strafe_locked = False
            self.vx_integral = 0.0
            return 0.0

        if not self.strafe_locked:
            # começando um novo strafe — zera o acumulado, não carrega erro antigo
            self.strafe_locked = True
            self.vx_integral = 0.0

        # integrador com vazamento: soma a aceleração filtrada, mas decai um
        # pouco a cada ciclo — evita divergência de longo prazo típica de
        # integrar acelerômetro puro
        self.vx_integral = (self.vx_integral * self.VX_INTEGRAL_LEAK) + (self.accel_x_filtered * dt)
        self.vx_integral = max(-self.VX_INTEGRAL_LIMIT, min(self.VX_INTEGRAL_LIMIT, self.vx_integral))

        # alvo é sempre "sem deriva linear" (velocidade residual em X = 0)
        correcao = -(self.KP_VX * self.accel_x_filtered) - (self.KI_VX * self.vx_integral)

        # teto baixo — isso é um trim, não pode dominar o movimento comandado
        correcao = max(-self.VX_CORRECTION_LIMIT, min(self.VX_CORRECTION_LIMIT, correcao))
        return correcao

    # ---------- /cmd_vel ----------
    def cmd_vel_callback(self, msg):
        now = self.get_clock().now()
        dt = (now - self.last_cmd_dt_time).nanoseconds / 1e9
        dt = dt if 0.0 < dt < 0.5 else 0.02
        self.last_cmd_dt_time = now
        self.last_cmd_time = now

        vx_comandado = msg.linear.x
        vy_comandado = -msg.linear.y
        vw_comandado = -msg.angular.z

        vw_effective = self.calcular_correcao_heading(vw_comandado, dt)
        vx_correcao = self.calcular_correcao_strafe(vx_comandado, vy_comandado, dt)
        vx_effective = vx_comandado + vx_correcao

        self.stop_sent = (vx_comandado == 0.0 and vy_comandado == 0.0 and vw_comandado == 0.0)

        v_fl, v_fr, v_rl, v_rr = self.cinematica_inversa(vx_effective, vy_comandado, vw_effective)
        self.send_serial(v_fl, v_fr, v_rl, v_rr)

    def watchdog_check(self):
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.CMD_TIMEOUT_S and not self.stop_sent:
            self.get_logger().warn("Sem /cmd_vel há muito tempo — enviando parada de segurança")
            self.send_serial(0.0, 0.0, 0.0, 0.0)
            self.stop_sent = True
            self.heading_locked = False
            self.strafe_locked = False


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelConverter()
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