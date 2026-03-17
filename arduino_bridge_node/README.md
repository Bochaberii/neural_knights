# Arduino Unload Node

This ROS node listens for a color-match result and sends an unload command to the Arduino via serial when the result indicates a successful match.

## Usage

1. Install dependencies on the Raspberry Pi:

```bash
sudo apt update
sudo apt install python3-serial python3-rospy
```

2. Configure parameters either via rosparam or in a launch file:

- `~port` (string): serial device path, default `/dev/ttyACM0`
- `~baud` (int): serial baud rate, default `57600`
- `~color_result_topic` (string): topic name to subscribe for results, default `/color_result`
- `~success_token` (string): token indicating successful match, default `SUCCESS`
- `~unload_command` (string): command sent to Arduino to trigger unload, default `z`
- `~ack_timeout` (float): seconds to wait for Arduino acknowledgement

3. Run the node:

```bash
rosrun arduino_bridge_node arduino_unload_node.py _port:=/dev/ttyACM0 _baud:=57600 _color_result_topic:=/color_result _success_token:=SUCCESS
```

4. The node will log serial output from the Arduino and warn if no acknowledgement is received.

## Integration

- Make sure the Arduino sketch listens for the `z` (or configured) command and performs the unload sequence, printing status lines to serial.
- The color matching node should publish a `std_msgs/String` message containing the configured success token (e.g., `SUCCESS`) when a match is found.

## Testing

- You can simulate a match from the Pi with:

```bash
rostopic pub /color_result std_msgs/String "data: 'SUCCESS'" -1
```

- The node will send the `z` command to the Arduino and log the response.
