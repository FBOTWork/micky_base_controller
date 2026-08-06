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
        self.stop_sent = True        # evita reenviar "0,0,0" repetidamente

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

    def send_serial(self, vx, vy, vw):
        # protocolo esperado pelo firmware: "Vx,Vy,W\n" — cinemática fica no Arduino
        comando = f"{vx:.2f},{vy:.2f},{vw:.2f}\n"
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
        self.send_serial(vx, vy, vw_effective)

    def watchdog_check(self):
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.CMD_TIMEOUT_S and not self.stop_sent:
            # /cmd_vel parou de chegar (ex: teleop_twist_keyboard só publica no keypress) — força parada
            self.get_logger().warn("Sem /cmd_vel há muito tempo — enviando parada de segurança")
            self.send_serial(0.0, 0.0, 0.0)
            self.stop_sent = True


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send_serial(0.0, 0.0, 0.0)  # garante parada mesmo em Ctrl+C
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()