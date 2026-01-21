#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from collections import deque


class LimitFilter(Node):

    def __init__(self):
        super().__init__('limit_filter')

        # Buffers for each topic
        self.buff1 = deque(maxlen=3)
        self.buff2 = deque(maxlen=3)
        self.buff3 = deque(maxlen=3)

        # ---------- Subscriptions ----------
        self.sub_ls1 = self.create_subscription(
            Int16, '/limit_switch_1',
            self.limit1_callback, 10)

        self.sub_ls2 = self.create_subscription(
            Int16, '/limit_switch_2',
            self.limit2_callback, 10)

        self.sub_debug = self.create_subscription(
            Int16, '/manipulator_debug',
            self.debug_callback, 10)

        # ---------- Publishers ----------
        self.pub_bevel_up = self.create_publisher(
            Int16, '/bevel_limit_up', 10)

        self.pub_bevel_down = self.create_publisher(
            Int16, '/bevel_limit_down', 10)

        self.pub_link1_limit = self.create_publisher(
            Int16, '/link1_limit', 10)

        self.get_logger().info("Limit Filter Running (3 topics RAW → Filtered)")

    # ---------------------------
    # Generic 3-sample filter
    # ---------------------------
    def apply_filter(self, buffer: deque, value: int):
        buffer.append(value)
        if len(buffer) < 3:
            return None

        prev, curr, next_ = buffer[0], buffer[1], buffer[2]

        if prev != 0 and curr == 0 and next_ != 0:
            return prev
        return curr

    # ---------------------------
    # Topic-specific Callbacks
    # ---------------------------
    def limit1_callback(self, msg: Int16):
        filtered = self.apply_filter(self.buff1, msg.data)
        if filtered is not None:
            out = Int16()
            out.data = filtered
            self.pub_bevel_up.publish(out)

    def limit2_callback(self, msg: Int16):
        filtered = self.apply_filter(self.buff2, msg.data)
        if filtered is not None:
            out = Int16()
            out.data = filtered
            self.pub_bevel_down.publish(out)

    def debug_callback(self, msg: Int16):
        filtered = self.apply_filter(self.buff3, msg.data)
        if filtered is not None:
            out = Int16()
            out.data = filtered
            self.pub_link1_limit.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LimitFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

