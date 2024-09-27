#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import sys
import termios
import tty

class KeyboardJoyPublisher(Node):

    def __init__(self):
        super().__init__('keyboard_joy_publisher')
        self.publisher_ = self.create_publisher(Joy, 'joy', 10)
        self.joy_msg = Joy()
        self.joy_msg.axes = [0.0] * 8
        self.joy_msg.buttons = [0] * 12
        self.get_logger().info('Node has been started. Use keyboard to control the Joy message.')

    def publish_joy_message(self):
        self.joy_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.joy_msg)

    def run(self):
        self.get_logger().info('Use arrow keys to control axes, space to stop, q to quit.')
        while rclpy.ok():
            key = self.get_key()
            if key == 'q':
                break
            elif key == '\x1b[A':  # Up arrow
                self.joy_msg.axes[1] += 0.1
            elif key == '\x1b[B':  # Down arrow
                self.joy_msg.axes[1] -= 0.1
            elif key == '\x1b[C':  # Right arrow
                self.joy_msg.axes[0] += 0.1
            elif key == '\x1b[D':  # Left arrow
                self.joy_msg.axes[0] -= 0.1
            elif key == ' ':  # Space bar
                self.joy_msg.axes = [0.0] * 8
                self.joy_msg.buttons = [0] * 12

            self.publish_joy_message()

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(3)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

def main(args=None):
    global settings
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = KeyboardJoyPublisher()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
