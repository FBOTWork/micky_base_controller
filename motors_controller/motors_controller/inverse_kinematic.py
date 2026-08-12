#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import os
import glob
import time


class CmdVelConverter(Node):
    def __init__(self):
        super().__init__("cmd_vel_converter_node")

        self.YAW_DRIFT_COMP = 0.05   # fração de Vy somada a W para corrigir deriva no strafe
        self.CMD_TIMEOUT_S = 0.2     # tempo sem /cmd_vel até mandar parada de segurança
        self.last_cmd_time = self.get_clock().now()
        self.stop_sent = True        # evita reenviar zeros repetidamente

        # Geometria do robô (precisa bater com o L usado antes no firmware, #define L 0.38f)
        self.L = 0.38

        self.declare_parameter("port", "/dev/arduino_robo")
        self.declare_parameter("baud", 115200)
        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value

        self.serial_port = self.find_serial_port(port)
        self.arduino = self.open_serial(self.serial_port, baud)
        self.get_logger().info(f"Conectado ao Arduino na porta {self.serial_port} a {baud} bps")

        self.subscription = self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_check)  # checa silêncio 10x mais rápido que o timeout

    def find_serial_port(self, preferred_port):
        # usa a porta fixa se existir; senão, procura qualquer ACM/USB disponível
        if preferred_port and os.path.exists(preferred_port):
            return preferred_port
        while rclpy.ok():
            candidates = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
            if candidates:
                return candidates[0]
            time.sleep(1)
        raise SystemExit

    def open_serial(self, port, baud):
        # tenta reconectar indefinidamente caso o Arduino não esteja pronto ainda
        while rclpy.ok():
            try:
                return serial.Serial(port, baud, timeout=0.1)
            except Exception:
                time.sleep(1)
                port = self.find_serial_port(port)
        raise SystemExit

    def cinematica_inversa(self, vx, vy, vw):
        # Mesma matriz que estava em Mover_Robo() no firmware:
        # FL = Vx+Vy+W*L | FR = Vx-Vy-W*L | RL = Vx-Vy+W*L | RR = Vx+Vy-W*L
        L = self.L
        v_fl = vx + vy + (vw * L)
        v_fr = vx - vy - (vw * L)
        v_rl = vx - vy + (vw * L)
        v_rr = vx + vy - (vw * L)
        return v_fl, v_fr, v_rl, v_rr

    def send_serial(self, v_fl, v_fr, v_rl, v_rr):
        # novo protocolo: velocidade linear (m/s) por roda, ordem FL,FR,RL,RR
        # firmware agora só faz Aplicar_Movimento() direto por roda, sem Mover_Robo()
        comando = f"{v_fl:.3f},{v_fr:.3f},{v_rl:.3f},{v_rr:.3f}\n"
        try:
            self.arduino.write(comando.encode("utf-8"))
        except Exception as e:
            self.get_logger().warn(f"Erro na Serial: {e}")

    def cmd_vel_callback(self, msg):
        self.last_cmd_time = self.get_clock().now()

        vx = msg.linear.x
        vy = -msg.linear.y    # eixo Y do ROS invertido pra bater com o strafe físico do robô
        vw = -msg.angular.z   # yaw do ROS invertido pra bater com o giro físico do robô
        vw_effective = vw + (vy * self.YAW_DRIFT_COMP)  # injeta a correção de deriva no giro

        self.stop_sent = (vx == 0.0 and vy == 0.0 and vw_effective == 0.0)

        v_fl, v_fr, v_rl, v_rr = self.cinematica_inversa(vx, vy, vw_effective)
        self.send_serial(v_fl, v_fr, v_rl, v_rr)

    def watchdog_check(self):
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.CMD_TIMEOUT_S and not self.stop_sent:
            # /cmd_vel parou de chegar (ex: teleop_twist_keyboard só publica no keypress) — força parada
            self.get_logger().warn("Sem /cmd_vel há muito tempo — enviando parada de segurança")
            self.send_serial(0.0, 0.0, 0.0, 0.0)
            self.stop_sent = True


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send_serial(0.0, 0.0, 0.0, 0.0)  # garante parada mesmo em Ctrl+C
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()