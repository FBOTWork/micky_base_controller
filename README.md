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

Passo a passo completo para identificar o Arduino, criar regra udev e manter um link estável `/dev/arduino_robo`.

#### 1. Descobrir o Arduino no sistema

Conecte o Arduino e rode:

```bash
ls /dev/tty*
```

Normalmente aparece `/dev/ttyUSB0` ou `/dev/ttyACM0`.

Agora pegue mais detalhes:

```bash
udevadm info -a -n /dev/ttyACM0
```

(substitua pelo dispositivo que apareceu no seu sistema).

#### 2. Identificar atributos únicos

Procure por `idVendor`, `idProduct` e `serial`:

```text
ATTRS{idVendor}=="2341"
ATTRS{idProduct}=="0043"
```

Se nenhum `idVendor/idProduct` aparecer no primeiro bloco, pode estar em um dos níveis seguintes. Verifique com:

```bash
udevadm info -a -n /dev/ttyACM0 | grep -E 'idVendor|idProduct|serial'
```

#### 3. Criar a regra udev

Crie o arquivo:

```bash
sudo nano /etc/udev/rules.d/99-arduino_robo.rules
```

Insira:

```bash
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0043", MODE="0666", SYMLINK+="arduino_robo"
```

Isso criará:

- `/dev/arduino_robo`

Se quiser evitar `sudo` para todos, use `MODE="0666"` ou `MODE="0660", GROUP="dialout"` e adicione seu usuário ao grupo dialout:

```bash
sudo usermod -a -G dialout $USER
```

#### 4. Recarregar regras

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

#### 5. Testar

Desconecte e reconecte o Arduino:

```bash
ls /dev/arduino_robo
```

Se aparecer, está funcionando.

#### 6. Casos especiais e dicas

- Clones CH340 e CP2102 usam outros IDs:
  - CH340: `ATTRS{idVendor}=="1a86"`, `ATTRS{idProduct}=="7523"`
  - CP2102: `ATTRS{idVendor}=="10c4"`, `ATTRS{idProduct}=="ea60"`

- Se não houver `idVendor/idProduct`, use parâmetros de caminho/mestre:

```bash
SUBSYSTEM=="tty", KERNEL=="ttyACM*", SYMLINK+="arduino_robo"
```

ou (mais preciso):

```bash
SUBSYSTEM=="tty", ATTRS{devpath}=="1.3", SYMLINK+="arduino_robo"
```

- Para distinguir múltiplos Arduinos:

```bash
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0043", ATTRS{serial}=="ABC123", SYMLINK+="arduino_robo"
```

#### 7. Uso no ROS

No `motors_controller`/`robot_sensors_publisher`, use `/dev/arduino_robo` como porta serial padrão.

Exemplo no ROS2 launch ou parâmetro do node:

```bash
ros2 param set /imu_serial_publisher port /dev/arduino_robo
```

---

Use este fluxo para deixar a conexão robusta entre reboot/conexões e evitar falhas pelo nome dinâmico `/dev/ttyACM*`.

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

