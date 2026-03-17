# Neural Knights - Developer Quick Reference

## 🚀 Common Commands

### Build & Deploy

```bash
# Build the package
colcon build
source install/setup.bash

# Upload Arduino firmware
cd motors
platformio run -t upload --upload-port COM3

# Build + upload (PlatformIO)
cd motors
pio run -t upload
```

### Launch Modes

```bash
# Gazebo simulation
ros2 launch neural_knights launch_sim.launch.py

# Real robot
ros2 launch neural_knights rsp.launch.py

# With custom parameters
ros2 launch neural_knights launch_sim.launch.py use_rviz:=false
```

### Monitor & Debug

```bash
# Watch encoder feedback
watch -n 0.1 'ros2 topic echo /joint_states | head -15'

# Monitor odometry
ros2 topic echo /odom

# List all topics
ros2 topic list

# Check node communication
rqt_graph

# ROS system info
ros2 daemon status
```

## 🎮 Movement Commands

```bash
# Move forward 0.5 m/s
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}}"

# Rotate 0.2 rad/s (counterclockwise)
ros2 topic pub /cmd_vel geometry_msgs/Twist "{angular: {z: 0.2}}"

# Combined (forward + rotate)
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}, angular: {z: 0.2}}"

# Stop
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"

# Continuous publishing (different approach)
ros2 topic pub -r 10 /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.3}, angular: {z: 0.0}}"
```

## 📁 Key Files Quick Location

| Need to...               | Go to...              | File(s)                                  |
| ------------------------ | --------------------- | ---------------------------------------- |
| Add motors/actuators     | `description/`        | `robot_core.xacro`, `ros2_control.xacro` |
| Change pin assignments   | `motors/src/`         | `pins.h`                                 |
| Adjust controller tuning | `config/`             | `my_controllers.yaml`                    |
| Modify Arduino behavior  | `motors/src/`         | `motor.cpp`, `packets.h`                 |
| Change simulation world  | `worlds/`             | `empty.world`                            |
| Add new ROS node         | `ros_arduino_bridge/` | Create new Python file                   |

## 🔍 Verification Checklist

- [ ] URDF loads without errors: `check_urdf description/robot.urdf.xacro`
- [ ] Arduino compiles: `cd motors && pio run`
- [ ] Package builds: `colcon build`
- [ ] Topics visible: `ros2 topic list`
- [ ] Simulation starts: `ros2 launch neural_knights launch_sim.launch.py`
- [ ] Motor responds: `ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}}"`

## 📦 Hardware Connection Checklist

- [ ] USB cable connected (Arduino → Raspberry Pi)
- [ ] Arduino powered via USB or external power
- [ ] Motors connected to IBT-2 drivers
- [ ] Encoders wired to Arduino pins 2, 3, 18, 19
- [ ] 12V power supply connected (if needed)
- [ ] All ground connections solid

## 🐛 Quick Diagnostics

```bash
# Is Arduino detected?
dmesg | grep ttyACM

# Can we communicate?
cat /dev/ttyACM0  # (shows serial data - Ctrl+C to stop)

# ROS 2 health check
ros2 daemon status
ros2 topic list
ros2 node list

# Check actual vs expected pins
cd motors/src && grep "#define" pins.h
```

## 🔧 Editing Configurations

### Motor Parameters

**File:** `config/my_controllers.yaml`

```yaml
wheel_separation: 0.254 # Adjust to actual wheel distance (meters)
wheel_radius: 0.0425 # Adjust to actual wheel radius (meters)
update_rate: 30 # Control loop frequency
```

### Pin Mappings

**File:** `motors/src/pins.h`

```cpp
#define MOTOR_LEFT_PWM   5   // GPIO pin for left motor PWM
#define MOTOR_RIGHT_PWM  7   // GPIO pin for right motor PWM
#define ENC_LEFT_A       2   // Left encoder channel A
#define ENC_RIGHT_A      18  // Right encoder channel A
// ... etc
```

### Robot Physical Properties

**File:** `description/robot.urdf.xacro`

```xml
<!-- Adjust wheel radius -->
<xacro:property name="wheel_radius" value="0.0425"/>

<!-- Adjust wheel separation -->
<xacro:property name="wheel_separation" value="0.254"/>

<!-- Adjust base dimensions -->
<xacro:property name="base_width" value="0.3"/>
```

## 📊 Performance Tuning

### Slow Response?

1. Check `update_rate` in `config/my_controllers.yaml` (default: 30 Hz)
2. Verify serial baud rate in Arduino code (typical: 9600)
3. Monitor CPU load on Raspberry Pi

### Drift in Odometry?

1. Verify encoder values: `ros2 topic echo /joint_states`
2. Check `wheel_separation` and `wheel_radius` are accurate
3. Calibrate encoders (run diagnostic tests)

### Overshooting/Oscillation?

1. Adjust PID gains (if tuning possible)
2. Reduce `use_stamped_vel` setting if needed
3. Check motor dead-band compensation

## 🔄 Git Workflow

```bash
# See what changed
git status

# Stage changes
git add <file>
git add .  # all files

# Commit
git commit -m "Describe your changes"

# Push to remote
git push origin main

# Pull latest
git pull origin main
```

## 📝 Documentation Practice

When modifying code, update:

1. **Code comments** - Explain WHY not just WHAT
2. **README.md** - If behavioral changes
3. **Header files** - Document new functions/defines
4. **Git commit messages** - Clear, descriptive

## 🆘 Emergency Procedures

### Robot Won't Stop

```bash
# Publish zero velocity repeatedly
while true; do ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0}, angular: {z: 0}}"; sleep 0.1; done
```

### Arduino Disconnected

```bash
# Check connection
lsusb | grep Arduino

# Re-enumerate
dmesg | tail -20

# Reboot Arduino
# (Power cycle or upload blank sketch)
```

### ROS 2 Unresponsive

```bash
# Kill zombie nodes
pkill -f ros2

# Clear daemon
ros2 daemon stop
ros2 daemon start

# Restart
source install/setup.bash
```

---

**Version:** March 2026  
**For more details, see README.md and PROJECT_STRUCTURE.md**
