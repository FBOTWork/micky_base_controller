# 🤖 Micky Base Controller

Sistema de controle para robô diferencial utilizando **Arduino Mega** como controlador de baixo nível e **ROS2** para controle de alto nível.

---

## 🧠 Visão geral

* **ROS2** → controle e tomada de decisão
* **Arduino Mega** → execução (motores)

O Arduino roda o `firmware.ino`, responsável por interpretar comandos recebidos e controlar o robô.

---

## 📦 Estrutura do repositório

```bash id="qz9k2p"
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

## 🛠️ Tecnologias

* Arduino Mega 2560
* ROS2
* C++
* Comunicação Serial

---

## ⚙️ Como usar

### 🔹 1. Instalar Arduino IDE

1. Acesse: https://www.arduino.cc/en/software
2. Baixe e instale normalmente

---

### 🔹 2. Configurar permissão da porta USB (Linux)

Para evitar erros de permissão ao acessar o Arduino:

#### Adicionar usuário ao grupo `dialout`

```bash id="q8d3vp"
sudo usermod -aG dialout $USER
```

Depois disso:

* **Reinicie o sistema** ou faça logout/login

---

#### (Opcional) Criar regra udev

Crie o arquivo:

```bash id="m8w4zf"
sudo nano /etc/udev/rules.d/99-arduino.rules
```

Adicione:

```id="z5c7yr"
KERNEL=="ttyACM*", MODE="0666"
```

Salve e aplique:

```bash id="f2x9hc"
sudo udevadm control --reload-rules
sudo udevadm trigger
```

---

### 🔹 3. Upload do firmware

1. Vá até:

```id="1j3p9k"
firmware_micro_controller/
```

2. Abra `firmware.ino` na Arduino IDE

3. Conecte o Arduino Mega

4. Configure:

* Placa: **Arduino Mega 2560**
* Porta correta

5. Clique em **Upload**

---

### 🔹 4. Build do ROS2

```bash id="r6k1vd"
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

### 🔹 5. Executar o sistema

#### 🔸 Controle dos motores

```bash id="c7m2xo"
ros2 launch motors_controller motors_controller.launch.py
```

#### 🔸 Sensores (IMU)

```bash id="v9n5re"
ros2 launch robot_sensors_publisher imu_sensor_publisher.launch.py
```

---

### 🔹 6. Controle manual (Teleop)

Instale:

```bash id="k4t8dz"
sudo apt install ros-humble-teleop-twist-keyboard
```

Execute:

```bash id="y2p6xn"
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Controles:

```id="b6r4mw"
i → frente
, → ré
j → esquerda
l → direita
k → parar
```

---

### 🔹 7. Enviar comandos manualmente (alternativo)

```bash id="g8v2qk"
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.1}}"
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
