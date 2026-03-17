#!/usr/bin/env python3
"""
arduino_unload_node.py

ROS node that listens for color-match success messages and tells the Arduino
to perform the unload sequence via serial. Configurable via ROS params.

Publishers/Subscriptions:
- Subscribes to `~color_result_topic` (std_msgs/String) by default

Parameters:
- ~port (string): serial device path, default '/dev/ttyACM0'
- ~baud (int): serial baud, default 57600
- ~color_result_topic (string): topic name to subscribe for results
- ~success_token (string): token indicating successful match; default 'SUCCESS'
- ~unload_command (string): command to send to Arduino; default 'z' (followed by newline)
- ~ack_timeout (float): seconds to wait for Arduino acknowledgement

This node sends the unload command when it receives a message on the color result
topic whose payload contains the success token. It optionally waits for an
acknowledgement line from the Arduino. If ack isn't received within timeout the
node logs a warning.

Note: This script expects ROS (rospy) to be installed on the Pi.
"""

import rospy
from std_msgs.msg import String
import serial
import threading


class ArduinoUnloadNode(object):
    def __init__(self):
        rospy.init_node('arduino_unload_node')

        self.port = rospy.get_param('~port', '/dev/ttyACM0')
        self.baud = rospy.get_param('~baud', 57600)
        self.color_result_topic = rospy.get_param('~color_result_topic', '/color_result')
        self.success_token = rospy.get_param('~success_token', 'SUCCESS')
        self.unload_command = rospy.get_param('~unload_command', 'z')
        self.ack_timeout = rospy.get_param('~ack_timeout', 2.0)

        self.lock = threading.Lock()
        self.ser = None

        rospy.loginfo("ArduinoUnloadNode starting, will listen on %s for token '%s'", self.color_result_topic, self.success_token)

        # Try to open the serial port
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            # allow Arduino reset
            rospy.sleep(2.0)
            rospy.loginfo('Opened serial port %s @ %d', self.port, self.baud)
        except Exception as e:
            rospy.logerr('Failed to open serial port %s: %s', self.port, e)
            self.ser = None

        # Subscribe to color result topic
        self.sub = rospy.Subscriber(self.color_result_topic, String, self.on_color_result)

    def on_color_result(self, msg):
        payload = msg.data if isinstance(msg.data, str) else str(msg)
        rospy.loginfo('Color result received: %s', payload)
        if self.success_token in payload:
            rospy.loginfo('Success token matched; requesting unload')
            self.request_unload()

    def request_unload(self):
        if not self.ser:
            rospy.logerr('Serial port not available. Cannot send unload command')
            return

        cmd = (self.unload_command + '\n').encode('ascii')
        with self.lock:
            try:
                self.ser.reset_input_buffer()
                self.ser.write(cmd)
                rospy.loginfo('Sent unload command: %s', self.unload_command)
            except Exception as e:
                rospy.logerr('Failed to write to serial port: %s', e)
                return

            # Optionally read ack
            ack = None
            deadline = rospy.Time.now() + rospy.Duration(self.ack_timeout)
            while rospy.Time.now() < deadline:
                try:
                    line = self.ser.readline().decode('utf-8', errors='replace').strip()
                    if line:
                        rospy.loginfo('Arduino: %s', line)
                        # treat any non-empty line as an ack
                        ack = line
                        break
                except Exception as e:
                    rospy.logwarn('Error reading ack: %s', e)
                    break

            if ack:
                rospy.loginfo('Unload acknowledged: %s', ack)
            else:
                rospy.logwarn('No ack received from Arduino after %0.2f seconds', self.ack_timeout)

    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    node = ArduinoUnloadNode()
    node.spin()
