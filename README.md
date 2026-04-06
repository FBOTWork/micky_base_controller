# 🤖 Micky Base Controller

Sistema de controle para robô diferencial utilizando **Arduino Mega** como controlador de baixo nível e **ROS2** para controle de alto nível.

O projeto é dividido em módulos responsáveis pelo controle dos motores e publicação de sensores, permitindo integração completa com o ecossistema ROS2.

---

## 🧠 Visão geral

Este sistema segue a arquitetura clássica de robótica:

* **ROS2** → tomada de decisão (alto nível)
* **Arduino Mega** → execução (baixo nível)

O Arduino roda um firmware responsável por interpretar comandos recebidos via serial e controlar diretamente os motores do robô.

---

## 📦 Estrutura do repositório

```bash
micky_base_controller/
│── firmware_micro_controller/   # Firmware do Arduino (firmware.ino)
│── motors_controller/           # Controle dos motores via ROS2
│── robot_sensors_publisher/     # Publicação de sensores no ROS2
│── README.md
```

---

## 🚀 Funcionalidades

* Controle de robô diferencial
* Comunicação ROS2 ↔ Arduino via serial
* Controle de velocidade linear e angular (`cmd_vel`)
* Modularização do sistema (motores + sensores)
* Base pronta para expansão com autonomia

---

## 🛠️ Tecnologias utilizadas

* Arduino Mega 2560
* ROS2 (Humble ou similar)
* C++ (ROS2 nodes)
* Arduino (C/C++)
* Comunicação Serial (USB)

---

## ⚙️ Como usar

### 🔹 1. Upload do firmware no Arduino

1. Acesse a pasta:

```bash
firmware_micro_controller/
```

2. Abra o arquivo `firmware.ino` na Arduino IDE

3. Configure:

* Placa: **Arduino Mega 2560**
* Porta correta

4. Clique em **Upload**

---

### 🔹 2. Build do ROS2

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

### 🔹 3. Executar os nós

#### Controle de motores:

```bash
ros2 run motors_controller <nome_do_node>
```

#### Sensores:

```bash
ros2 run robot_sensors_publisher <nome_do_node>
```

---

### 🔹 4. Enviar comandos para o robô

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

---

## 📡 Comunicação Serial

O ROS2 envia comandos para o Arduino no formato:

```bash
Vx;Wz
```

Exemplo:

```bash
0.2;0.1
```

Onde:

* `Vx` → velocidade linear
* `Wz` → velocidade angular

---

## ⚡ Funcionamento do firmware

O `firmware.ino` é responsável por:

* Ler dados da serial
* Interpretar comandos recebidos
* Calcular velocidades das rodas
* Aplicar PWM nos motores

Fluxo:

1. Recebe comando via serial
2. Faz parsing (`Vx;Wz`)
3. Converte para controle diferencial
4. Aciona motores

---

## 🎮 Controle do robô

O robô utiliza cinemática diferencial:

* Frente / Ré → velocidade linear
* Giro → velocidade angular
* Curvas → combinação dos dois

---

## 🔧 Possíveis melhorias

* Implementação de odometria
* Uso de encoders
* Controle PID nos motores
* Integração com IMU
* Migração para micro-ROS
* Adição de câmera / visão computacional

---

## 🐛 Problemas conhecidos

* Dependência de comunicação serial USB
* Possível latência nos comandos
* Sem feedback de posição atualmente

---

## 📄 Licença

Projeto para fins educacionais e desenvolvimento em robótica.

---

## 🤝 Contribuição

Contribuições são bem-vindas!
Abra uma issue ou envie um pull request.

---
