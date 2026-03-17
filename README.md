# Neural Knights - Autonomous Mobile Robot

Neural Knights is a ROS 2-based autonomous mobile robot platform designed for educational and research purposes. It features real-time motor control, camera and LIDAR integration, and full simulation support in Gazebo.

## 🎯 Quick Overview

**What is Neural Knights?**

- A differential-drive mobile robot platform
- Built on ROS 2 Humble for robotics software
- Simulated in Gazebo Ignition Fortress
- Real-time hardware control via Arduino Mega 2560
- Complete with sensor integration (camera, LIDAR, encoders)

**Key Features:**

- ✅ URDF/Xacro robot description with multiple sensors
- ✅ Gazebo simulation environment with physics
- ✅ Arduino-based real-time motor and encoder control
- ✅ ROS 2 ros2_control differential drive controller
- ✅ RViz2 visualization and monitoring
- ✅ Modular and extensible architecture

## 📚 Documentation

| Document                                                         | Purpose                                             |
| ---------------------------------------------------------------- | --------------------------------------------------- |
| **[PROJECT_STRUCTURE.md](PROJECT_STRUCTURE.md)**                 | Complete file organization and component guide      |
| **[LOCAL_README.md](LOCAL_README.md)**                           | Local development and deployment instructions       |
| **[ros_arduino_bridge/README.md](ros_arduino_bridge/README.md)** | Arduino-ROS communication setup                     |
| **images/**                                                      | Technical reports, photos, and video demonstrations |

## 🏗️ System Architecture

```
                    Raspberry Pi 5
        ┌──────────────────────────────────┐
        │   ROS 2 Humble Environment       │
        ├──────────────────────────────────┤
        │ ros2_control Framework           │
        │ └─→ Differential Drive Manager   │
        │                                  │
        │ Launch System                    │
        │ └─→ [launch_sim.launch.py]       │
        │ └─→ [rsp.launch.py]              │
        │                                  │
        │ Visualization                    │
        │ └─→ RViz2 / Gazebo              │
        └──────────────┬───────────────────┘
                      │ Serial (USB)
        ┌─────────────▼────────────────┐
        │   Arduino Mega 2560          │
        │                              │
        │ Motor Control:               │
        │ ├─ PWM Driver L (GPIO 5,6)  │
        │ └─ PWM Driver R (GPIO 7,8)  │
        │                              │
        │ Feedback:                    │
        │ ├─ Encoder L (GPIO 2,3)     │
        │ └─ Encoder R (GPIO 18,19)   │
        └──────────────┬───────────────┘
                      │
        ┌─────────────┴──────────────┐
        │                            │
    [Left Motor]              [Right Motor]
    [IBT-2 Driver]            [IBT-2 Driver]
    [Quadrature Encoder]      [Quadrature Encoder]
```

## 🚀 Quick Start

### 1. Build the Package

```bash
# On your development machine or Raspberry Pi
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src
git clone <repository-url>
cd ~/dev_ws
colcon build
source install/setup.bash
```

### 2. Upload Arduino Code

```bash
cd neural_knights/motors
platformio run -t upload --upload-port COM3
# (Replace COM3 with your Arduino's port: COM3, /dev/ttyACM0, etc.)
```

### 3. Run Simulation

```bash
# Terminal 1: Start ROS and simulation
ros2 launch neural_knights launch_sim.launch.py

# Terminal 2: Start RViz visualization (optional)
rviz2

# Terminal 3: Send movement commands
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}, angular: {z: 0.2}}"
```

### 4. Run on Real Hardware

```bash
# Terminal 1: Start robot state publisher
ros2 launch neural_knights rsp.launch.py

# Terminal 2: Ensure Arduino is connected via USB
# The serial bridge will auto-detect and connect

# Terminal 3: Send commands
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}}"
```

## 🔌 Hardware Specifications

### Compute Platform

| Component       | Model                | Role              |
| --------------- | -------------------- | ----------------- |
| Main Computer   | Raspberry Pi 5 (8GB) | ROS 2 runtime     |
| Microcontroller | Arduino Mega 2560    | Real-time I/O     |
| Motor Driver    | IBT-2 (2x)           | PWM motor control |
| Encoders        | Quadrature (2x)      | Wheel feedback    |
| Camera          | USB/CSI              | Visual input      |
| LIDAR           | USB Serial           | Distance scanning |

### Pin Configuration

```
MOTOR CONTROL (IBT-2):
  Arduino Pin 5  → Left Motor PWM
  Arduino Pin 6  → Left Motor Enable
  Arduino Pin 7  → Right Motor PWM
  Arduino Pin 8  → Right Motor Enable

ENCODERS (Quadrature):
  Arduino Pin 2  → Left Encoder A
  Arduino Pin 3  → Left Encoder B
  Arduino Pin 18 → Right Encoder A
  Arduino Pin 19 → Right Encoder B

SERIAL COMMUNICATION:
  USB ↔ Serial1 (TX/RX on pins 18/19)
```

_(Edit these in `motors/src/pins.h` if different)_

## 📋 Project Structure

```
neural_knights/
├── config/                    # Controller configurations
│   ├── empty.yaml
│   └── my_controllers.yaml    # Differential drive setup
│
├── description/               # Robot URDF/Xacro definitions
│   ├── robot.urdf.xacro      # Main robot
│   ├── robot_core.xacro       # Base geometry
│   ├── ros2_control.xacro     # Motor control config
│   ├── camera.xacro
│   ├── lidar.xacro
│   ├── gazebo_control.xacro   # Simulation-specific
│   └── [other macro files]
│
├── launch/                    # ROS 2 launch files
│   ├── rsp.launch.py         # Real robot startup
│   └── launch_sim.launch.py  # Gazebo simulation
│
├── motors/                    # Arduino firmware
│   ├── platformio.ini        # Build configuration
│   └── src/
│       ├── motor.cpp
│       ├── pins.h            # GPIO pin definitions
│       └── packets.h         # Serial protocol
│
├── ros_arduino_bridge/        # Arduino-ROS bridge
│   ├── README.md             # Setup guide
│   ├── arduino_bridge_node/  # Python node
│   ├── ROSArduinoBridge/     # Main firmware sketch
│   ├── serial_motor_demo/    # Demo code
│   └── [test sketches]
│
├── worlds/                    # Gazebo worlds
│   └── empty.world
│
└── images/                    # Documentation & media
    ├── Technical reports & posters
    ├── Assembly photos
    ├── Test videos
    └── Field testing footage
```

**For complete file descriptions, see [PROJECT_STRUCTURE.md](PROJECT_STRUCTURE.md)**

## 🎮 Control Interface

### Published Topics

| Topic           | Type                     | Description                          |
| --------------- | ------------------------ | ------------------------------------ |
| `/cmd_vel`      | `geometry_msgs/Twist`    | Velocity commands (linear + angular) |
| `/odom`         | `nav_msgs/Odometry`      | Odometry estimate                    |
| `/joint_states` | `sensor_msgs/JointState` | Motor positions/velocities           |

### Subscribed Topics

| Topic      | Type                  | Description                |
| ---------- | --------------------- | -------------------------- |
| `/cmd_vel` | `geometry_msgs/Twist` | Incoming velocity commands |

### Services

- Check `ros_arduino_bridge/srv/` for custom service definitions

## 🧪 Testing & Diagnostics

### Motor & Encoder Tests

Located in `ros_arduino_bridge/`:

- `encoder_test.ino` - Basic encoder polling
- `continuous_encoder_monitor.ino` - Real-time monitoring
- `motor_with_live_feedback.ino` - Motor + feedback integration
- `ibt2_encoder_test.ino` - IBT-2 specific testing
- `right_encoder_diagnostic.ino` - Right motor diagnostics

### ROS 2 Testing

```bash
# Monitor encoder feedback
ros2 topic echo /joint_states

# Check odometry
ros2 topic echo /odom

# Monitor raw serial data (if configured)
ros2 topic echo /serial_data
```

## 🐛 Troubleshooting

| Problem                  | Solution                                                                            |
| ------------------------ | ----------------------------------------------------------------------------------- |
| **Arduino won't upload** | Check COM port in platformio.ini, verify ATmega2560 board selected                  |
| **Robot won't respond**  | Verify USB connection, check serial permissions (`chmod 666 /dev/ttyACM0` on Linux) |
| **Motors not moving**    | Test with diagnostic sketch, verify IBT-2 wiring, check PWM pins                    |
| **No encoder feedback**  | Verify quadrature encoder pinout, check pin configuration in pins.h                 |
| **Gazebo crashes**       | Verify URDF syntax (`check_urdf description/robot.urdf.xacro`), check dependencies  |
| **ROS 2 won't start**    | Ensure `source install/setup.bash` executed, check ROS 2 Humble installation        |

## 📊 Configuration Parameters

### Motor Controller (`config/my_controllers.yaml`)

```yaml
update_rate: 30 # Control loop frequency (Hz)
use_sim_time: true # Use simulated time in Gazebo

diff_cont:
  base_frame_id: base_link
  left_wheel_names: ["left_wheel_joint"]
  right_wheel_names: ["right_wheel_joint"]
  wheel_separation: 0.254 # Distance between wheels (m)
  wheel_radius: 0.0425 # Wheel radius (m)
```

Adjust these for your specific robot geometry!

## 🔄 Development Workflow

1. **Modify Robot Geometry**
   - Edit `description/robot.urdf.xacro`
   - Recompile: `colcon build`

2. **Change Motor Control**
   - Edit `motors/src/`
   - Recompile and upload: `platformio run -t upload`

3. **Adjust Control Parameters**
   - Edit `config/my_controllers.yaml`
   - No recompile needed, reload with `colcon build`

4. **Test Changes**
   - First: Gazebo simulation (`launch_sim.launch.py`)
   - Then: Real hardware (ensure connection stable)

## 📚 External Resources

- **[ROS 2 Humble Docs](https://docs.ros.org/en/humble/)**
- **[ros2_control Documentation](https://control.ros.org/)**
- **[Gazebo Ignition](https://gazebosim.org/)**
- **[URDF Format Specification](http://wiki.ros.org/urdf)**
- **[PlatformIO Documentation](https://platformio.org/)**
- **[Arduino Mega Reference](https://docs.arduino.cc/hardware/mega-2560/)**

## 📄 License

See [LICENSE.md](LICENSE.md) for license details.

## 👤 About

**Project Name:** Neural Knights  
**Maintainer:** Collins Chemweno  
**Email:** collinskiprop59@gmail.com  
**Status:** ✅ Active Development  
**Version:** 0.0.0 (Development)  
**ROS 2 Version:** Humble

## 📸 Gallery

Check the `images/` folder for:

- Technical documentation (PDF)
- Assembly photos
- Simulation screenshots (RViz + Gazebo)
- Real-world testing videos (Pi camera feed, field tests)

---

**For detailed project structure and file descriptions, see [PROJECT_STRUCTURE.md](PROJECT_STRUCTURE.md)**

**For local setup and deployment guide, see [LOCAL_README.md](LOCAL_README.md)**
