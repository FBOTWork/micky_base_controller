#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import os
import glob
import time

class CmdVelToSerial(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_serial_node')
        
        # Parâmetros de conexão
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value

        self.serial_port = self.find_serial_port(port)
        self.arduino = self.open_serial(self.serial_port, baud)

        self.get_logger().info(f'Conectado ao Arduino na porta {self.serial_port} a {baud} bps')

        # Inscrição no tópico cmd_vel (padrão do ROS 2)
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10) # QoS History Depth

    def find_serial_port(self, preferred_port):
        if preferred_port and os.path.exists(preferred_port):
            self.get_logger().info(f'Porta especificada existe: {preferred_port}')
            return preferred_port

        self.get_logger().info(f'Porta {preferred_port} não encontrada. Procurando /dev/ttyACM* e /dev/ttyUSB*...')
        while rclpy.ok():
            candidates = sorted(glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*'))
            if candidates:
                selected = candidates[0]
                self.get_logger().info(f'Encontrado dispositivo serial: {selected}')
                return selected
            self.get_logger().info('Nenhum dispositivo serial encontrado. Conecte o Arduino e aguarde...')
            time.sleep(1)

        raise SystemExit

    def open_serial(self, port, baud):
        while rclpy.ok():
            try:
                ser = serial.Serial(port, baud, timeout=0.1)
                return ser
            except Exception as e:
                self.get_logger().error(f'Erro abrindo porta {port}: {e}')
                self.get_logger().info('Tentando detectar novamente a porta...')
                time.sleep(1)
                port = self.find_serial_port(port)

        raise SystemExit

    def cmd_vel_callback(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        vw = msg.angular.z
        
        # Formata a string idêntica ao que o Arduino Mega espera
        comando = f"<{vx:.3f},{vy:.3f},{vw:.3f}>\n"
        
        try:
            self.arduino.write(comando.encode('utf-8'))
        except Exception as e:
            self.get_logger().warn(f'Erro ao enviar para Serial: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToSerial()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
