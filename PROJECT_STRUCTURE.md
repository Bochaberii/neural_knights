# Neural Knights - Project Structure Guide

## 📋 Overview

**Neural Knights** is a ROS 2-based autonomous mobile robot platform featuring:

- Arduino Mega-based motor and encoder control
- Raspberry Pi 5 running ROS 2 Humble
- Gazebo simulation environment
- URDF/Xacro robot description with camera and LIDAR sensors
- Real-time motor feedback using IBT-2 motor drivers

---

## 📁 Core Project Structure

```
neural_knights/
├── config/                      # ROS 2 controller configuration
│   ├── empty.yaml              # Empty configuration template
│   └── my_controllers.yaml      # Differential drive controller setup
│
├── description/                # URDF/Xacro robot descriptions
│   ├── robot.urdf.xacro       # Main robot description
│   ├── robot_core.xacro        # Base robot geometry
│   ├── ros2_control.xacro      # ros2_control integration
│   ├── gazebo_control.xacro    # Gazebo-specific simulation
│   ├── camera.xacro            # Camera sensor definition
│   ├── lidar.xacro             # LIDAR sensor definition
│   ├── rover_properties.xacro  # Robot properties (wheels, mass)
│   ├── rover_macros.xacro      # Reusable macro definitions
│   ├── inertial_macros.xacro   # Inertia calculation macros
│   └── gazebo_control.xacro    # Gazebo plugin configuration
│
├── launch/                      # ROS 2 launch files
│   ├── rsp.launch.py           # Robot State Publisher launch
│   └── launch_sim.launch.py    # Gazebo simulation + control launch
│
├── motors/                      # Arduino firmware for motor control
│   ├── platformio.ini          # PlatformIO configuration (Arduino Mega 2560)
│   └── src/
│       ├── motor.cpp           # Motor control implementation
│       ├── packets.h           # Communication packet definitions
│       └── pins.h              # GPIO pin definitions
│
├── ros_arduino_bridge/          # Arduino-ROS 2 communication
│   ├── README.md               # Setup and usage guide
│   ├── arduino_bridge_node/    # ROS 2 Python node
│   ├── ROSArduinoBridge/       # Main Arduino sketch
│   ├── ROSArduinoBridge from Brian Fundi/  # Reference implementation
│   ├── serial_motor_demo/      # Serial communication demo
│   ├── srv/                    # ROS service definitions
│   ├── tools/                  # Utility scripts
│   │
│   └── Test Sketches:
│       ├── continuous_encoder_monitor.ino
│       ├── encoder_test.ino
│       ├── ibt2_encoder_test.ino
│       ├── motor_with_live_feedback.ino
│       └── right_encoder_diagnostic.ino
│
├── worlds/                      # Gazebo world files
│   └── empty.world             # Empty simulation world
│
├── images/                      # Documentation, photos, videos
│   ├── Technical Documentation:
│   │   ├── NEURAL KNIGHTS technical poster.pdf
│   │   └── Neural Knights technical report (2).pdf
│   │
│   ├── Assembly & Setup:
│   │   ├── robot first assembly.png
│   │   ├── robot.jpeg
│   │   ├── robot2.jpeg
│   │   ├── pi image.jpg
│   │   └── design iterations.png
│   │
│   ├── Simulation & Testing:
│   │   ├── rviz + gazebo mapping in real world.png
│   │   ├── rviz mapping.png
│   │   ├── robot in rviz and gazebo.mp4
│   │   │
│   │   ├── Pi Video Feed:
│   │   │   └── pi video feed.mp4
│   │   │
│   │   └── Field Testing:
│   │       ├── VID-20250904-WA0003.mp4
│   │       ├── VID-20250904-WA0004.mp4
│   │       ├── IMG-20250904-WA0030.jpg
│   │       └── IMG-20250904-WA0031.jpg
│   │
│   ├── Resources:
│   │   ├── map.jpeg
│   │   └── diseased.jpg
│
└── worlds/                      # Gazebo simulation environments
    └── empty.world             # Default empty world
```

---

## 📦 Project Files

| File               | Purpose                             |
| ------------------ | ----------------------------------- |
| `CMakeLists.txt`   | Build configuration (ROS 2 package) |
| `package.xml`      | ROS 2 package metadata              |
| `encoder_test.ino` | Quick test sketch for encoders      |
| `LICENSE.md`       | Project license                     |
| `LOCAL_README.md`  | Local development notes             |
| `README.md`        | Package template documentation      |

---

## 🏗️ Additional Components

### Supporting Projects & References

- **my_robot_package/** - Alternative/older robot package variant
- **Neural Knights Gamefield_map/** - Gamification/mapping system configurations
- **plant-detetction/** - Plant detection ML model (separate sub-project)
- **RABUnique/** - Legacy Arduino bridge reference

### Test & Reference Code (in `ros_arduino_bridge/`)

- **ROSArduinoBridge from Brian Fundi/** - Reference implementation
- **serial_motor_demo/** - Standalone serial communication demo
- **Test sketches** - Individual encoder/motor testing utilities

---

## 🔧 Hardware Architecture

```
Raspberry Pi 5 (ROS 2 Master)
    │
    ├─→ Serial USB ─→ Arduino Mega 2560
    │                    │
    │                    ├─→ IBT-2 Motor Driver L (GPIO 5,6)
    │                    ├─→ IBT-2 Motor Driver R (GPIO 7,8)
    │                    ├─→ Encoder Readers (GPIO 2,3,18,19)
    │                    └─→ Other Sensors
    │
    ├─→ Camera (USB/CSI)
    └─→ LIDAR (USB/Serial)
```

**Motor Control Flow:**

1. ROS 2 node publishes velocity commands
2. `ros2_control` converts to motor commands
3. Arduino receives via serial protocol
4. Arduino outputs PWM to IBT-2 drivers
5. Motor encoders send feedback via serial

---

## 🚀 Quick Start

### 1. Build the ROS 2 Package

```bash
cd ~/dev_ws/src
git clone <this-repo>
cd ..
colcon build
source install/setup.bash
```

### 2. Upload Arduino Firmware

```bash
cd motors/
platformio run -t upload --upload-port COM3  # or /dev/ttyACM0
```

### 3. Launch Simulation

```bash
ros2 launch neural_knights launch_sim.launch.py
```

### 4. Launch with Real Robot

```bash
ros2 launch neural_knights rsp.launch.py
```

---

## 📊 Key Technologies

| Component         | Technology                   |
| ----------------- | ---------------------------- |
| Robot OS          | ROS 2 Humble                 |
| Simulation        | Gazebo Ignition Fortress     |
| Visualization     | RViz2                        |
| Motor Control     | ros2_control + IBT-2 drivers |
| Arduino Firmware  | PlatformIO (C++)             |
| Firmware Language | C++ for Arduino              |
| ROS Nodes         | Python                       |
| Robot Description | URDF/Xacro                   |

---

## 🔌 Pin Configuration (Arduino Mega)

```
Motor Control (IBT-2):
  LEFT_PWM:  GPIO 5
  LEFT_EN:   GPIO 6
  RIGHT_PWM: GPIO 7
  RIGHT_EN:  GPIO 8

Encoders:
  LEFT_A:    GPIO 2
  LEFT_B:    GPIO 3
  RIGHT_A:   GPIO 18
  RIGHT_B:   GPIO 19

Serial:
  RX1:       GPIO 19 (Serial1)
  TX1:       GPIO 18 (Serial1)
```

_(Update in `motors/src/pins.h` if using different pins)_

---

## 📝 Configuration Files

### `config/my_controllers.yaml`

Configures the differential drive controller:

- Update rate: 30 Hz
- Wheel separation: 0.254 m
- Wheel radius: 0.0425 m
- Base frame: `base_link`

### `description/robot.urdf.xacro`

Main robot description includes:

- Base geometry and inertia
- Motor and encoder references
- Camera and LIDAR mounts
- Gazebo simulation parameters

---

## 🎯 Typical Development Workflow

1. **Modify URDF** → Edit `description/` files
2. **Change Motor Control** → Edit `motors/src/` and re-upload
3. **Adjust Controllers** → Edit `config/my_controllers.yaml`
4. **Test in Gazebo** → Use `launch_sim.launch.py`
5. **Field Testing** → Use `rsp.launch.py` + real robot
6. **Debugging** → Check encoder/motor test sketches

---

## 🐛 Troubleshooting

### Arduino won't upload:

- Check COM port in `motors/platformio.ini`
- Verify ATmega2560 board is selected
- Try different USB cable

### Robot won't move:

- Check serial connection with Arduino
- Verify pin configurations in `motors/src/pins.h`
- Test with encoder diagnostic sketches
- Check IBT-2 motor driver wiring

### Gazebo/RViz not starting:

- Ensure ROS 2 environment sourced: `source install/setup.bash`
- Check package.xml dependencies installed
- Verify URDF syntax: `check_urdf description/robot.urdf.xacro`

---

## 📚 References

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Ignition Fortress](https://gazebosim.org/)
- [ros2_control](https://control.ros.org/)
- [URDF Standard](http://wiki.ros.org/urdf)
- [PlatformIO Documentation](https://platformio.org/)

---

## 📄 License

See `LICENSE.md` for details.

---

## 👥 Project Information

**Maintainer:** Collins Chemweno  
**Email:** collinskiprop59@gmail.com  
**Current Version:** 0.0.0 (Development)

---

_Last Updated: March 2026_
