# 🤖 Micky Base Controller

## 📘 Tutorial de Configuração e Uso

Este repositório contém o controle base do robô Micky, incluindo firmware e nós ROS 2 para controle de motores e publicação de sensores.

---

## 🧠 Visão geral

* **ROS2** → controle e tomada de decisão
* **Arduino Mega** → execução (motores)

O Arduino roda o `firmware.ino`, responsável por interpretar comandos recebidos e controlar o robô.

---

## 📦 Estrutura do repositório

```bash
micky_base_controller/
│── firmware_micro_controller/   # Firmware do Arduino
│── motors_controller/           # Controle dos motores (ROS2)
│── robot_sensors_publisher/     # Publicação de sensores (ROS2)
│── README.md
```

---

## 🚀 Funcionalidades

* Controle de robô diferencial
* Integração com ROS2
* Controle via tópico `/cmd_vel`
* Publicação de dados de sensores (IMU)
* Arquitetura modular

---

## 🛠️ Requisitos

* Linux (Ubuntu)
* ROS 2 (Humble)
* Arduino IDE ou PlatformIO
* Dependências Python (`setup.py`)

---

## ⚙️ 1. Instalar Arduino IDE

1. Acesse: https://www.arduino.cc/en/software
2. Baixe e instale normalmente

---

## ⚙️ 2. Upload do firmware

1. Abra:

```bash
firmware_micro_controller/firmware.ino
```

2. Conecte o Arduino

3. Configure na IDE:

* Placa: **Arduino Mega 2560**
* Porta: `/dev/ttyACM0` ou `/dev/ttyUSB0`

4. Clique em **Upload**

---

## ⚙️ 3. Configurar porta USB (udev)

### 🔹 Descobrir dispositivo

```bash
ls /dev/tty*
```

---

### 🔹 Obter informações

```bash
udevadm info -a -n /dev/ttyACM0 | grep -E 'idVendor|idProduct|serial'
```

---

### 🔹 Criar regra udev

```bash
sudo nano /etc/udev/rules.d/99-arduino_robo.rules
```

Adicione:

```bash
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0043", MODE="0666", SYMLINK+="arduino_robo"
```

---

### 🔹 Recarregar regras

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

---

### 🔹 Testar

```bash
ls /dev/arduino_robo
```

---

### 🔹 Permissão (recomendado)

```bash
sudo usermod -aG dialout $USER
```

(Reinicie ou faça logout/login)

---

### 🔹 Dicas

* CH340:

```bash
idVendor=1a86 idProduct=7523
```

* CP2102:

```bash
idVendor=10c4 idProduct=ea60
```

---

## ⚙️ 4. Build do ROS 2

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## ⚙️ 5. Executar o sistema

### 🔸 Motores

```bash
ros2 launch motors_controller motors_controller.launch.py
```

### 🔸 Sensores (IMU)

```bash
ros2 launch robot_sensors_publisher imu_sensor_publisher.launch.py
```

---

## ⚙️ 6. Controle manual (Teleop)

Instale:

```bash
sudo apt install ros-humble-teleop-twist-keyboard
```

Execute:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Controles:

```
i → frente
, → ré
j → esquerda
l → direita
k → parar
```

---

## ⚙️ 7. Teste rápido

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.1}}" --once
```

---

## ⚙️ 8. Verificar dados

```bash
ros2 topic echo /imu/data
ros2 topic echo /wheel_encoders
```

---

## ⚡ Firmware (`firmware.ino`)

Responsável por:

* Receber comandos
* Interpretar velocidade linear e angular
* Controlar motores com PWM

---

## 🎮 Controle do robô

* Frente / Ré → velocidade linear
* Giro → velocidade angular
* Curvas → combinação dos dois

---
