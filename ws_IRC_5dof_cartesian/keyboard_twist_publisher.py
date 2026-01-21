
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import sys
import termios
import tty
import select

# Keyboard mapping for linear and angular motion
MOVE_BINDINGS = {
    'w': (0.1, 0.0, 0.0),  # +X
    's': (-0.1, 0.0, 0.0), # -X
    'a': (0.0, 0.1, 0.0),  # +Y
    'd': (0.0, -0.1, 0.0), # -Y
    'q': (0.0, 0.0, 0.1),  # +Z
    'e': (0.0, 0.0, -0.1)  # -Z
}

ROTATE_BINDINGS = {
    'i': (0.1, 0.0, 0.0),  # Roll +
    'k': (-0.1, 0.0, 0.0), # Roll -
    'j': (0.0, 0.1, 0.0),  # Pitch +
    'l': (0.0, -0.1, 0.0), # Pitch -
    'u': (0.0, 0.0, 0.1),  # Yaw +
    'o': (0.0, 0.0, -0.1)  # Yaw -
}


def get_key():
    """Non-blocking key reader"""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class KeyboardTwistPublisher(Node):

    def __init__(self):
        super().__init__('keyboard_twist_publisher')
        self.publisher = self.create_publisher(TwistStamped,
                                               '/servo_node/delta_twist_cmds',
                                               10)

        self.get_logger().info("Keyboard control started (100Hz publishing)!")
        self.get_logger().info("W/S: X,  A/D: Y,  Q/E: Z")
        self.get_logger().info("I/K: roll, J/L: pitch, U/O: yaw")
        self.get_logger().info("CTRL+C to quit.")

        self.timer = self.create_timer(0.01, self.timer_callback)  # 100 Hz
        self.reset_twist()

    def reset_twist(self):
        self.linear = [0.0, 0.0, 0.0]
        self.angular = [0.0, 0.0, 0.0]

    def timer_callback(self):
        key = get_key()

        # Modify velocities based on key
        if key in MOVE_BINDINGS:
            self.linear = list(MOVE_BINDINGS[key])
        elif key in ROTATE_BINDINGS:
            self.angular = list(ROTATE_BINDINGS[key])
        else:
            self.reset_twist()

        # Construct message
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = self.linear
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = self.angular

        self.publisher.publish(msg)


if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = KeyboardTwistPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

