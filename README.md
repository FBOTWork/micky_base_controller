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

The robot has **two** USB-serial devices that must resolve to different,
stable names: the motors Arduino (`/dev/arduino_robo`) and the IMU
(`/dev/arduino_imu`).

⚠️ **Both currently use the same CH340 USB-serial chip**
(`idVendor=1a86`, `idProduct=7523`), and CH340 clones don't expose a
unique per-device serial number. A rule that matches only on
`idVendor`/`idProduct` will match **both** devices and they will fight
over the same symlink name (whichever enumerates last wins, and the
other is left without a stable name). To tell them apart, the rules
also match on the **physical USB port** (`KERNELS`), which stays stable
as long as each cable stays plugged into the same physical port.

### 🔹 Discover Device

```bash
ls /dev/tty*
```

---

### 🔹 Get Information

```bash
udevadm info -a -n /dev/ttyUSB0 | grep -E 'idVendor|idProduct|serial|KERNELS'
```

Note the `KERNELS=="X-Y"` value closest to the top of the output (the
USB device itself, e.g. `KERNELS=="8-1"`) — that identifies the
physical port this device is plugged into. Repeat for every connected
`ttyUSB*`/`ttyACM*` device and note which one is the motors Arduino and
which one is the IMU (e.g. unplug one cable at a time and see which
device disappears from `ls /dev/ttyUSB*`).

---

### 🔹 Install the udev Rule

The rules for this robot are versioned in [`udev/99-micky-usb-serial.rules`](udev/99-micky-usb-serial.rules).
Install it with:

```bash
sudo cp udev/99-micky-usb-serial.rules /etc/udev/rules.d/
```

If your motors Arduino / IMU are plugged into different physical ports
than the ones already in the file, edit the `KERNELS=="..."` values to
match the ports you found in the previous step.

---

### 🔹 Reload Rules

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

---

### 🔹 Test

```bash
ls -l /dev/arduino_robo /dev/arduino_imu
```

Both should exist and point to different `ttyUSB*`/`ttyACM*` devices.

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

* Genuine Arduino (e.g. Mega 2560):

```bash
idVendor=2341 idProduct=0043
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
