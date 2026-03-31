# Micky_Base_Controller

## Tutorial de Configuração e Uso

Este repositório contém o controle base do robô Micky, incluindo firmware e nós ROS 2 para controle de motores e publicação de sensores.

### 1. Requisitos

- Linux (Ubuntu)
- ROS 2 (Humble)
- Dependências Python: instaladas via `pip` usando `setup.py`
- Microcontrolador com firmware (arquivo `firmware_micro_controller/firmware.ino`)

### 2. Compilar e carregar o firmware

O processo abaixo é para o primeiro uso com um Arduino novo. Se o seu Arduino já tiver o firmware `micky_base_controller` gravado, você pode pular a etapa de upload e ir direto para a configuração do ROS 2 e do link `/dev/arduino_robo`.

2.1. Primeiro uso (Arduino novo)

1. Instale o Arduino IDE ou PlatformIO:
   - Arduino IDE: https://www.arduino.cc/en/software
   - PlatformIO: https://platformio.org/
2. Abra o Arduino IDE ou PlatformIO.
3. Abra `firmware_micro_controller/firmware.ino`.
4. Selecione a placa (por exemplo, "Arduino Uno") e a porta serial (`/dev/ttyACM0` ou `/dev/ttyUSB0`).
5. Clique em Upload.

2.2. Uso subsequente (firmware já carregado)

- Se você já tiver o firmware no Arduino e não precisar recarregar, pode pular a etapa de upload e seguir direto para a seção 2.a (permissão porta USB) e depois 3 (build ROS 2).

### 2.a Permissão da porta USB e nome fixo `arduino_robo`

Por padrão em Linux, o dispositivo USB do Arduino pode ser criado como `/dev/ttyUSB0` ou `/dev/ttyACM0`, dependendo do modelo. Para evitar problemas de permissão e manter sempre o mesmo nome de dispositivo, use uma regra udev:

1. Crie arquivo de regras: `/etc/udev/rules.d/99-arduino_robo.rules`.
2. Insira (substitua os IDs do fabricante/produto conforme seu dispositivo):

```bash
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0043", MODE="0666", SYMLINK+="arduino_robo"
```

- `idVendor` e `idProduct` podem ser obtidos com `lsusb`.
- `MODE="0666"` garante leitura/gravação para todos usuários (ajuste conforme segurança desejada).
- `SYMLINK+="arduino_robo"` cria `/dev/arduino_robo` apontando para o dispositivo real.

3. Recarrregue regras udev e reconecte o Arduino:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

4. Verifique:

```bash
ls -l /dev/arduino_robo
```

5. No código ou launch ROS, use `/dev/arduino_robo` como porta serial.

### 3. Instalar os pacotes ROS 2

No workspace principal `/home/fbot/micky_base_controller`:

```bash
source /opt/ros/<distro>/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 4. Executar o controlador de motores

```bash
ros2 launch motors_controller motors_controller.launch.py
```

### 5. Executar o publicador de sensores IMU

```bash
ros2 launch robot_sensors_publisher imu_sensor_publisher.launch.py
```

### 6. Teste de envio de velocidade

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.2, y: 0.0, z: 0.0} angular: {x: 0.0, y: 0.0, z: 0.1}" --once
```

### 7. Verificar tópicos

```bash
ros2 topic echo /imu/data
ros2 topic echo /wheel_encoders
```

### 8. Estrutura do repositório

- `firmware_micro_controller/`: firmware do MCU
- `motors_controller/`: nodo ROS 2 para motor via serial
- `robot_sensors_publisher/`: nodo ROS 2 para IMU via serial

