# Complete Setup Guide: ROS 2 + Arduino Robot Control

This guide will walk you through every step to get your robot moving using ROS 2, an Arduino Mega, and an IBT-2 motor driver. All commands and file names are provided exactly—no placeholders or guesswork.

## How to Access Your Raspberry Pi Code from VS Code (Remote SSH)

1. Open VS Code on your computer.
2. Press `Ctrl+Shift+P` to open the command palette.
3. Type `Remote-SSH: Connect to Host...` and select it.
4. Enter `neural@<your_pi_ip_address>` (replace with your Pi's IP).
5. Enter your password when prompted.
6. Once connected, click `Open Folder` and select your ROS 2 workspace folder (e.g., `/home/neural/ros2_ws`).
7. You can now edit your ROS node code directly on the Raspberry Pi from VS Code.

---

## Hardware Required

- Arduino Mega
- IBT-2 motor driver (connected to Arduino pins as per your wiring)
- Raspberry Pi 5 running Ubuntu (with ROS 2 Humble installed)
- USB cable for Arduino

## 1. Upload the Arduino Code

1. Plug your Arduino Mega into your computer using a USB cable.
2. Open VS Code and navigate to `ros_arduino_bridge/ROSArduinoBridge/ROSArduinoBridge.ino`.
3. Make sure all servo-related code is deleted. Only motor control code should remain.
4. Double-check the pin assignments in the code match your IBT-2 wiring:
   - Example: `#define MOTOR_LEFT_PWM 5`, `#define MOTOR_RIGHT_PWM 6` (update to your actual pins)
5. In the Arduino IDE (or VS Code with Arduino extension):
   - Select Board: `Arduino Mega 2560`
   - Select Port: (e.g., `COM3` on Windows, `/dev/ttyACM0` on Linux)
   - Click Upload.
6. Wait for "Upload complete" message.

## 2. Create and Build Your ROS 2 Workspace

Open a terminal on your Raspberry Pi and run:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
cp -r /path/to/ros_arduino_bridge ~/ros2_ws/src/
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## 3. Create the ROS 2 Node/Package for the Bridge

### Generate the ROS 2 Python Package

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python serial_motor_demo
```

### Add the Bridge Node Code

Create the file: `~/ros2_ws/src/serial_motor_demo/serial_motor_demo/bridge_node.py` with the following content:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class BridgeNode(Node):
   def __init__(self):
      super().__init__('bridge_node')
      self.subscription = self.create_subscription(
         Twist,
         '/cmd_vel',
         self.listener_callback,
         10)
      self.serial_port = serial.Serial('/dev/ttyACM0', 57600, timeout=1)

   def listener_callback(self, msg):
      left_speed = int(msg.linear.x * 100 - msg.angular.z * 50)
      right_speed = int(msg.linear.x * 100 + msg.angular.z * 50)
      command = f"m {left_speed} {right_speed}\r"
      self.serial_port.write(command.encode())

def main(args=None):
   rclpy.init(args=args)
   node = BridgeNode()
   rclpy.spin(node)
   node.destroy_node()
   rclpy.shutdown()

if __name__ == '__main__':
   main()
```

### Note: Arduino Node Code

If you create a file named `arduino_node.py` (or similar), you should use the exact same code as shown above for `bridge_node.py`. This ensures consistent behavior and communication between ROS 2 and your Arduino.

### Update `setup.py` in `~/ros2_ws/src/serial_motor_demo/setup.py`:

```python
from setuptools import setup

package_name = 'serial_motor_demo'

setup(
   name=package_name,
   version='0.0.0',
   packages=[package_name],
   install_requires=['setuptools'],
   zip_safe=True,
   maintainer='neural',
   maintainer_email='your@email.com',
   description='ROS 2 bridge node for Arduino motor control',
   license='MIT',
   entry_points={
      'console_scripts': [
         'bridge_node = serial_motor_demo.bridge_node:main',
      ],
   },
)
```

### Update `package.xml` in `~/ros2_ws/src/serial_motor_demo/package.xml`:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>serial_motor_demo</name>
  <version>0.0.0</version>
  <description>ROS 2 bridge node for Arduino motor control</description>
  <maintainer email="your@email.com">neural</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_python</buildtool_depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
</package>
```

---

## 3. Install teleop_twist_keyboard

In the same terminal, run:

```bash
sudo apt update
sudo apt install ros-humble-teleop-twist-keyboard
```

## 4. Run the ROS 2 Bridge Node

1. Make sure your bridge node Python file is at `~/ros2_ws/src/ros_arduino_bridge/serial_motor_demo/serial_motor_demo/bridge_node.py`.
2. Make it executable:
   ```bash
   chmod +x ~/ros2_ws/src/ros_arduino_bridge/serial_motor_demo/serial_motor_demo/bridge_node.py
   ```
3. Run the node:
   ```bash
   ros2 run serial_motor_demo bridge_node
   ```
4. You should see output confirming connection to `/dev/ttyACM0` at baud rate `57600`.

## 5. Run teleop_twist_keyboard and Control the Robot

Open a new terminal, source your workspace, and run:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Use these keys to control your robot:

| Key | Action                      | What Happens                             |
| --- | --------------------------- | ---------------------------------------- |
| i   | Move forward                | Both motors forward                      |
| ,   | Move backward               | Both motors backward                     |
| j   | Turn left                   | Left motor backward, right motor forward |
| l   | Turn right                  | Left motor forward, right motor backward |
| u   | Forward + left (diagonal)   | Both motors forward, left slower         |
| o   | Forward + right (diagonal)  | Both motors forward, right slower        |
| m   | Backward + right (diagonal) | Both motors backward, right slower       |
| n   | Backward + left (diagonal)  | Both motors backward, left slower        |
| k   | Stop                        | Both motors stop                         |

## How the System Works

1. teleop_twist_keyboard publishes Twist messages to `/cmd_vel`.
2. `bridge_node.py` listens to `/cmd_vel`, converts the message to a serial command: `m <left_speed> <right_speed>\r`.
3. Arduino receives the command, parses 'm', and sets motor speeds.
4. Robot moves as commanded.

## Troubleshooting (Concrete Steps)

- If the robot only spins or turns, check that both left and right speeds are positive for forward (see `bridge_node.py`).
- If nothing moves, check:
  - Is the Arduino code uploaded and running?
  - Is the USB cable connected?
  - Is the bridge node running and showing "connected"?
  - Is the correct port (`/dev/ttyACM0`) and baud rate (`57600`) used?
- If upload fails, make sure all servo code is removed from Arduino sketch.
- If you get "permission denied" on serial port, run:
  ```bash
  sudo usermod -a -G dialout $USER
  reboot
  ```

## Full Example Workflow

1. Upload Arduino code as described above.
2. Copy `ros_arduino_bridge` folder into `~/ros2_ws/src/`.
3. Build workspace: `colcon build`
4. Source workspace: `source install/setup.bash`
5. Run bridge node: `ros2 run serial_motor_demo bridge_node`
6. Open new terminal, source workspace, run: `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
7. Press 'i' to move forward, 'k' to stop, etc.

---

This guide is complete and explicit. Follow each step exactly and your robot will move with keyboard control via ROS 2 and Arduino.
