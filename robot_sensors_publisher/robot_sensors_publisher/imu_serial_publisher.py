import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu

import serial
import time
import math
import os
import glob


class ImuSerialPublisher(Node):

    # Firmware manda telemetria de DOIS MPU9250 numa linha so:
    # roll1,pitch1,yaw1,accX1,accY1,accZ1,gyroX1,gyroY1,gyroZ1,temp1,
    # roll2,pitch2,accX2,accY2,accZ2,gyroX2,gyroY2,gyroZ2,temp2,botao,motor;
    # so usamos os 10 primeiros campos (MPU1), que e o que esta fisicamente
    # alinhado com o chassi - o resto (MPU2/botao/motor) e ignorado aqui.
    # yaw1 vem do magnetometro do MPU1 (AK8963), ja tilt-compensado pelo
    # firmware - e o unico heading absoluto real que existe no robo hoje.
    MIN_FIELDS = 10

    # firmware manda giro em graus/s e acel em g; sensor_msgs/Imu exige rad/s e m/s^2
    DEG_TO_RAD = math.pi / 180.0
    G_TO_MS2 = 9.80665

    # variancia do giro Z medida em bancada com o robo parado (rad/s)^2 - se
    # trocar a IMU ou a montagem, remedir com o robo parado e atualizar aqui
    GYRO_Z_VARIANCE = 3.5e-6

    # variancia do heading do magnetometro (rad)^2 - valor de partida
    # conservador (~13 graus de desvio padrao), ja que motor de passo/chassi
    # metalico perto da IMU pode distorcer a leitura. Meça girando o robo
    # devagar num heading conhecido e comparando com o valor lido; se o
    # ruido/salto for maior que isso, aumente ainda mais.
    YAW_VARIANCE = 0.05

    CALIBRATION_DURATION_S = 2.0

    def __init__(self):
        super().__init__("imu_serial_publisher")

        # =========================
        # PARAMETROS
        # =========================
        self.declare_parameter("port", "/dev/arduino_imu")
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

        self.gyro_z_bias_dps = 0.0

        # calibracao do bias do giro Z agora e feita em segundo plano, dentro
        # do proprio timer de publicacao (ver read_and_publish/_update_gyro_calibration)
        # em vez de bloquear o __init__ por CALIBRATION_DURATION_S: bloquear
        # aqui deixava /imu/data sem publicar nada por 2s logo na largada,
        # enquanto /odom ja estava de pe - quando a primeira leitura (com
        # heading do magnetometro) finalmente chegava, o EKF via um "salto"
        # de uma vez e se corrigia bruscamente, desalinhando o mapa que o
        # slam_toolbox ja tinha comecado a desenhar com a referencia antiga.
        self.calibrating = False
        self.calib_samples = []
        self.calib_deadline = None

        if self.serial_conn is not None:
            self.serial_conn.setDTR(False)
            self.serial_conn.setRTS(False)

            time.sleep(2)
            self.serial_conn.reset_input_buffer()

            self.get_logger().info(f"Serial conectada em {self.serial_port}")
            self.get_logger().info("Calibrando bias do giro Z em segundo plano - mantenha o robô parado...")
            self.calibrating = True
            self.calib_deadline = time.time() + self.CALIBRATION_DURATION_S
        else:
            self.get_logger().error(
                "Não foi possível abrir a porta serial. O tópico não será publicado até que o dispositivo esteja disponível."
            )

        # =========================
        # PUBLICADORES
        # =========================
        self.pub_imu = self.create_publisher(Imu, "/imu/data", 10)

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

    def _update_gyro_calibration(self, gz_dps):
        # acumula amostras de gyroZ1 cru (bias ainda nao descontado) enquanto
        # calibrando; ao passar do deadline, fecha a media e passa a usar o
        # bias real dali em diante. Publicacao continua rolando o tempo todo
        # (com bias=0 ate a calibracao fechar), sem gap de /imu/data.
        self.calib_samples.append(gz_dps)

        if time.time() >= self.calib_deadline:
            self.gyro_z_bias_dps = sum(self.calib_samples) / len(self.calib_samples)
            self.get_logger().info(
                f"Bias do giro Z: {self.gyro_z_bias_dps:.4f}°/s ({len(self.calib_samples)} amostras)"
            )
            self.calibrating = False
            self.calib_samples = []

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

            if len(parts) < self.MIN_FIELDS:
                self.get_logger().warning(f"Formato inesperado recebido na serial: {line}")
                return

            try:
                roll_deg, pitch_deg, yaw_deg, ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, _temp1 = (
                    float(v) for v in parts[: self.MIN_FIELDS]
                )

                roll = math.radians(roll_deg)
                pitch = math.radians(pitch_deg)
                yaw = math.radians(yaw_deg)

                if self.calibrating:
                    self._update_gyro_calibration(gz_dps)

                # =========================
                # IMU MESSAGE
                # =========================
                imu_msg = Imu()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = "imu_link"

                # roll/pitch vem do acelerometro; yaw agora vem do magnetometro
                # (AK8963 do MPU1), ja tilt-compensado pelo firmware - e o unico
                # heading absoluto real do robo. O EKF (ekf.yaml) fica configurado
                # pra fundir esse yaw absoluto + a velocidade angular (vyaw) do
                # giro, e ignorar o heading da odometria de comando.
                qx, qy, qz, qw = self.euler_to_quaternion(roll, pitch, yaw)

                imu_msg.orientation.x = qx
                imu_msg.orientation.y = qy
                imu_msg.orientation.z = qz
                imu_msg.orientation.w = qw

                # roll/pitch nao sao fundidos pelo EKF (robo 2D, two_d_mode: true
                # no ekf.yaml) - covariancia alta so pra nao serem levados a serio
                # por outro consumidor eventual desse topico. yaw usa o valor medido.
                imu_msg.orientation_covariance[0] = 999.0
                imu_msg.orientation_covariance[4] = 999.0
                imu_msg.orientation_covariance[8] = self.YAW_VARIANCE

                imu_msg.angular_velocity.x = gx_dps * self.DEG_TO_RAD
                imu_msg.angular_velocity.y = gy_dps * self.DEG_TO_RAD
                imu_msg.angular_velocity.z = (gz_dps - self.gyro_z_bias_dps) * self.DEG_TO_RAD

                # covariancia medida em bancada - deixa o EKF confiar nessa leitura
                # pra rotacao em vez da odometria de comando (que e so uma estimativa)
                imu_msg.angular_velocity_covariance[0] = self.GYRO_Z_VARIANCE
                imu_msg.angular_velocity_covariance[4] = self.GYRO_Z_VARIANCE
                imu_msg.angular_velocity_covariance[8] = self.GYRO_Z_VARIANCE

                imu_msg.linear_acceleration.x = ax_g * self.G_TO_MS2
                imu_msg.linear_acceleration.y = ay_g * self.G_TO_MS2
                imu_msg.linear_acceleration.z = az_g * self.G_TO_MS2

                self.pub_imu.publish(imu_msg)

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
