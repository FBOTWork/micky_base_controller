# 🤖 Micky Base Controller

## 📘 Setup and Usage Tutorial

This repository contains the base control for the Micky robot, including firmware and ROS 2 nodes for motor control and sensor publishing.

---

## 🧠 Overview

* **ROS2** → control and decision making
* **Arduino Mega** → execution (motors)

The Arduino runs the `firmware.ino`, responsible for interpreting received commands and controlling the robot.

---

## 📦 Repository Structure

```bash
micky_base_controller/
│── firmware_micro_controller/   # Arduino Firmware
│── motors_controller/           # Motor Control (ROS2)
│── robot_sensors_publisher/     # Sensor Publishing (ROS2)
│── README.md
```

---

## 🚀 Features

* Differential robot control
* ROS2 integration
* Control via `/cmd_vel` topic
* Sensor data publishing (IMU)
* Modular architecture

---

## 🛠️ Requirements

* Linux (Ubuntu)
* ROS 2 (Humble)
* Arduino IDE or PlatformIO
* Python dependencies (`setup.py`)

---

## ⚙️ 1. Install Arduino IDE

1. Access: https://www.arduino.cc/en/software
2. Download and install normally

---

## ⚙️ 2. Firmware Upload

1. Open:

```bash
firmware_micro_controller/firmware.ino
```

2. Connect the Arduino

3. Configure in IDE:

* Board: **Arduino Mega 2560**
* Port: `/dev/ttyACM0` or `/dev/ttyUSB0`

4. Click **Upload**

---

## ⚙️ 3. Configure USB Port (udev)

### 🔹 Discover Device

```bash
ls /dev/tty*
```

---

### 🔹 Get Information

```bash
udevadm info -a -n /dev/ttyACM0 | grep -E 'idVendor|idProduct|serial'
```

---

### 🔹 Create udev Rule

```bash
sudo nano /etc/udev/rules.d/99-arduino_robo.rules
```

Add:

```bash
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0043", MODE="0666", SYMLINK+="arduino_robo"
```

---

### 🔹 Reload Rules

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

---

### 🔹 Test

```bash
ls /dev/arduino_robo
```

---

### 🔹 Permission (recommended)

```bash
sudo usermod -aG dialout $USER
```

(Restart or logout/login)

---

### 🔹 Tips

* CH340:

```bash
idVendor=1a86 idProduct=7523
```

* CP2102:

```bash
idVendor=10c4 idProduct=ea60
```

---

## ⚙️ 4. ROS 2 Build

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## ⚙️ 5. Run the System

### 🔸 Motors

```bash
ros2 launch motors_controller motors_controller.launch.py
```

### 🔸 Sensors (IMU)

```bash
ros2 launch robot_sensors_publisher imu_sensor_publisher.launch.py
```

---

## ⚙️ 6. Manual Control (Teleop)

Install:

```bash
sudo apt install ros-humble-teleop-twist-keyboard
```

Run:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Controls:

```
i → forward
, → reverse
j → left
l → right
k → stop
```

---

## ⚙️ 7. Quick Test

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.1}}" --once
```

---

## ⚙️ 8. Check Data

```bash
ros2 topic echo /imu/data
ros2 topic echo /wheel_encoders
```

---

## ⚡ Firmware (`firmware.ino`)

Responsible for:

* Receiving commands
* Interpreting linear and angular velocity
* Controlling motors with PWM

---

## 🎮 Robot Control

* Forward / Reverse → linear velocity
* Turn → angular velocity
* Curves → combination of both

---
