#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

import sys
import termios
import tty
import select

# ----------------------------
# Key bindings
# ----------------------------
LINEAR_BINDINGS = {
    'w': ( 1,  0,  0),
    's': (-1,  0,  0),
    'a': ( 0,  1,  0),
    'd': ( 0, -1,  0),
    'q': ( 0,  0,  1),
    'e': ( 0,  0, -1),
}

ANGULAR_BINDINGS = {
    'i': ( 1,  0,  0),
    'k': (-1,  0,  0),
    'j': ( 0,  1,  0),
    'l': ( 0, -1,  0),
    'u': ( 0,  0,  1),
    'o': ( 0,  0, -1),
}

LINEAR_SPEED  = 0.10
ANGULAR_SPEED = 0.50


def get_key():
    """Blocking-ish key read with short timeout"""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.02)  # 20ms
    key = sys.stdin.read(1) if rlist else None
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class KeyboardServo(Node):

    def __init__(self):
        super().__init__('keyboard_servo')

        self.pub = self.create_publisher(
            TwistStamped,
            '/servo_node/delta_twist_cmds',
            10
        )

        self.timer = self.create_timer(0.02, self.loop)   # 50 Hz

        self.last_key = None

        self.get_logger().info("Keyboard Servo Teleop Started")
        self.get_logger().info("Hold keys for continuous motion")
        self.get_logger().info("CTRL+C to quit")

    def loop(self):

        key = get_key()

        lin = [0.0, 0.0, 0.0]
        ang = [0.0, 0.0, 0.0]

        if key:
            key = key.lower()
            self.last_key = key

            if key in LINEAR_BINDINGS:
                dx, dy, dz = LINEAR_BINDINGS[key]
                lin = [dx * LINEAR_SPEED,
                       dy * LINEAR_SPEED,
                       dz * LINEAR_SPEED]

            if key in ANGULAR_BINDINGS:
                rx, ry, rz = ANGULAR_BINDINGS[key]
                ang = [rx * ANGULAR_SPEED,
                       ry * ANGULAR_SPEED,
                       rz * ANGULAR_SPEED]
        else:
            # no key → stop motion
            self.last_key = None

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        msg.twist.linear.x  = lin[0]
        msg.twist.linear.y  = lin[1]
        msg.twist.linear.z  = lin[2]
        msg.twist.angular.x = ang[0]
        msg.twist.angular.y = ang[1]
        msg.twist.angular.z = ang[2]

        self.pub.publish(msg)


def main():
    global settings

    settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    node = KeyboardServo()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    main()
