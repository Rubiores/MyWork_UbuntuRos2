#!/usr/bin/env python3
import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Teleop listo. Usa W A S D X")

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        twist = Twist()
        speed = 0.4
        turn = 0.4

        while rclpy.ok():
            key = self.get_key()

            if key == 'w':
                twist.linear.x = speed
                twist.angular.z = 0.0

            elif key == 's':
                twist.linear.x = -speed
                twist.angular.z = 0.0

            elif key == 'a':
                twist.linear.x = speed
                twist.angular.z = turn

            elif key == 'd':
                twist.linear.x = -speed
                twist.angular.z = -turn

            elif key == 'x':  # STOP
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            self.pub.publish(twist)

def main():
    rclpy.init()
    node = TeleopKeyboard()
    try:
        node.run()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
