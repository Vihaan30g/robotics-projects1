# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Int16, Float32MultiArray
# from rclpy.time import Time

# # ----------------------------
# # FSM STATES
# # ----------------------------
# STATE_HOME = 0
# STATE_BEVEL_START = 1
# STATE_LINK1_DOWN = 2
# STATE_BEVEL_LIMIT_DOWN = 3
# STATE_LINK1_UP_FOR_SCOOP = 4
# STATE_CARRY = 5
# STATE_BEVEL_DUMP = 6
# STATE_LINK1_HOME = 7
# STATE_RECOVERY = 10

# class AutoNode(Node):

#     def __init__(self):
#         super().__init__('auto_node')

#         self.state = STATE_HOME

#         # ----------------------------
#         # Publishers
#         # ----------------------------
#         self.pub_bevel_cmd = self.create_publisher(Int16, '/bevel_cmd', 10)
#         self.pub_link1_cmd = self.create_publisher(Int16, '/manipulator_cmd', 10)
#         self.pub_flag = self.create_publisher(Int16, '/vihaan', 10)

#         # ----------------------------
#         # Subscribers
#         # ----------------------------
#         self.create_subscription(Int16, '/rohan', self.rohan_callback, 10)
        
#         # Updated to MultiArray and specific indices
#         self.create_subscription(Float32MultiArray, '/imu_orientation', self.link1_imu_callback, 10)
#         self.create_subscription(Float32MultiArray, '/bevel_orientation', self.bevel_imu_callback, 10)
        
#         # Consolidated Bevel Limit
#         self.create_subscription(Int16, '/limit_hit', self.bevel_limit_callback, 10)
        
#         # Link 1 Debug topic (Value 10 = limit hit)
#         self.create_subscription(Int16, '/manipulator_debug', self.link1_limit_callback, 10)

#         # ----------------------------
#         # FSM Timer (20 ms)
#         # ----------------------------
#         self.timer = self.create_timer(0.02, self.update)

#         # ----------------------------
#         # Sensor state
#         # ----------------------------
#         self.link1_angle = 0
#         self.bevel_angle = 0

#         self.bevel_limit_down = False
#         self.bevel_limit_up = False
#         self.link1_limit_home = False

#         # ----------------------------
#         # Parameters
#         # ----------------------------
#         self.target_bevel_scoop_angle = 22
#         self.target_link1_down_angle = -264
#         self.target_link1_up_scoop_angle = 50
#         self.link1_recovery_step = 10
#         self.recovery_timeout = 4.0

#         self.recover_start_time = None
#         self.recovery_timer_started = False

#         self.get_logger().info('AutoNode Ready. Awaiting trigger...')

#     # ==================================================
#     # Callbacks
#     # ==================================================
#     def rohan_callback(self, msg: Int16):
#         if msg.data == 1 and self.state == STATE_HOME:
#             self.state = STATE_BEVEL_START
#             self.get_logger().info('START SCOOPING (State 1)')
#         if msg.data == 2 and self.state == STATE_CARRY:
#             self.state = STATE_BEVEL_DUMP
#             self.get_logger().info('START DROPPING (State 6)')

#     def link1_imu_callback(self, msg: Float32MultiArray):
#         # Pull angle from Index 1
#         if len(msg.data) > 1:
#             self.link1_angle = msg.data[1]

#     def bevel_imu_callback(self, msg: Float32MultiArray):
#         # Pull angle from Index 1
#         if len(msg.data) > 1:
#             self.bevel_angle = msg.data[1]

#     def bevel_limit_callback(self, msg: Int16):
#         # 1 = Up limit, 2 = Down limit, 0 = None
#         self.bevel_limit_up = (msg.data == 1)
#         self.bevel_limit_down = (msg.data == 2)

#     def link1_limit_callback(self, msg: Int16):
#         # 10 = Home limit hit
#         self.link1_limit_home = (msg.data == 10)

#     # ==================================================
#     # FSM UPDATE
#     # ==================================================
#     def update(self):
#         cmd = Int16()

#         if self.state == STATE_HOME:
#             self.stop_all()

#         elif self.state == STATE_BEVEL_START:
#             if self.bevel_angle < self.target_bevel_scoop_angle:
#                 self.pub_bevel_cmd.publish(self.int_msg(8)) # Move Bevel Pos
#             else:
#                 self.stop_all()
#                 self.state = STATE_LINK1_DOWN
#                 self.get_logger().info('L1 Down...')

#         elif self.state == STATE_LINK1_DOWN:
#             if self.link1_angle < self.target_link1_down_angle:
#                 self.pub_link1_cmd.publish(self.int_msg(2)) # Move L1 Pos
#             else:
#                 self.stop_all()
#                 self.state = STATE_BEVEL_LIMIT_DOWN
#                 self.recovery_timer_started = False

#         elif self.state == STATE_BEVEL_LIMIT_DOWN:
#             if self.bevel_limit_down: # Checks for '2' from callback
#                 self.stop_all()
#                 self.state = STATE_LINK1_UP_FOR_SCOOP
#                 self.get_logger().info('Ground reached.')
#             else:
#                 self.pub_bevel_cmd.publish(self.int_msg(8))
#                 if not self.recovery_timer_started:
#                     self.recover_start_time = self.get_clock().now()
#                     self.recovery_timer_started = True

#                 elapsed = (self.get_clock().now() - self.recover_start_time).nanoseconds * 1e-9
#                 if elapsed > self.recovery_timeout:
#                     self.state = STATE_RECOVERY

#         elif self.state == STATE_RECOVERY:
#             self.pub_bevel_cmd.publish(self.int_msg(8))
#             self.pub_link1_cmd.publish(self.int_msg(1)) # Move L1 Neg
#             if self.link1_angle <= (self.target_link1_down_angle - self.link1_recovery_step):
#                 self.state = STATE_BEVEL_LIMIT_DOWN
#                 self.recovery_timer_started = False

#         elif self.state == STATE_LINK1_UP_FOR_SCOOP:
#             if self.link1_angle > self.target_link1_up_scoop_angle:
#                 self.pub_link1_cmd.publish(self.int_msg(1))
#             else:
#                 self.stop_all()
#                 self.state = STATE_CARRY
#                 self.get_logger().info('Scooped. Ready to carry.')

#         elif self.state == STATE_CARRY:
#             self.pub_flag.publish(self.int_msg(1))

#         elif self.state == STATE_BEVEL_DUMP:
#             if self.bevel_limit_up: # Checks for '1' from callback
#                 self.stop_all()
#                 self.state = STATE_LINK1_HOME
#             else:
#                 self.pub_bevel_cmd.publish(self.int_msg(4))

#         elif self.state == STATE_LINK1_HOME:
#             if self.link1_limit_home: # Checks for '10' from callback
#                 self.stop_all()
#                 self.state = STATE_HOME
#                 self.get_logger().info('Task Finished.')
#             else:
#                 self.pub_link1_cmd.publish(self.int_msg(1))
            
#             self.pub_flag.publish(self.int_msg(2))

#     # ==================================================
#     # Utilities
#     # ==================================================
#     def int_msg(self, value: int) -> Int16:
#         msg = Int16()
#         msg.data = value
#         return msg

#     def stop_all(self):
#         self.pub_bevel_cmd.publish(self.int_msg(0))
#         self.pub_link1_cmd.publish(self.int_msg(0))

# def main(args=None):
#     rclpy.init(args=args)
#     node = AutoNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

















# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Int16, Float32MultiArray
# from rclpy.time import Time

# # ----------------------------
# # FSM STATES
# # ----------------------------
# STATE_HOME = 0
# STATE_BEVEL_START = 1
# STATE_LINK1_DOWN = 2
# STATE_BEVEL_LIMIT_DOWN = 3
# STATE_LINK1_UP_FOR_SCOOP = 4
# STATE_CARRY = 5
# STATE_BEVEL_DUMP = 6
# STATE_LINK1_HOME = 7
# STATE_RECOVERY = 10

# class AutoNode(Node):

#     def __init__(self):
#         super().__init__('auto_manip')

#         self.state = STATE_HOME

#         # Publishers
#         self.pub_bevel_cmd = self.create_publisher(Int16, 'lift_angle', 10)  # /bevel_cmd
#         self.pub_link1_cmd = self.create_publisher(Int16, '/manipulator_cmd', 10)
#         self.pub_flag = self.create_publisher(Int16, '/vihaan', 10)

#         # Subscribers
#         self.create_subscription(Int16, '/rohan', self.start_callback, 10)
#         self.create_subscription(Float32MultiArray, '/imu_orientation', self.link1_imu_callback, 10)
#         self.create_subscription(Float32MultiArray, '/bevel_orientation', self.bevel_imu_callback, 10)
#         self.create_subscription(Int16, '/bevel_limit_down', self.bevel_limit_down_callback, 10)
#         self.create_subscription(Int16, '/bevel_limit_up', self.bevel_limit_up_callback, 10)
#         self.create_subscription(Int16, '/link1_limit', self.link1_limit_callback, 10)

#         # FSM update timer
#         self.timer = self.create_timer(0.02, self.update)

#         # Internal state tracking
#         self.link1_angle = 0.0
#         self.bevel_angle = 0.0

#         self.bevel_limit_down = False
#         self.bevel_limit_up = False
#         self.link1_limit_home = True  # Start at home

#         # Target angles (you will tune later)
#         self.target_bevel_scoop_angle = 22.0
#         self.target_link1_down_angle = 96.0  # should be 96+6, but it goes to -270 after 96, so ece need to change this logic
#         self.target_link1_up_scoop_angle = 50.0
#         self.link1_recovery_step = 5.0
#         self.recovery_timeout = 5.0

#         self.recovery_start = None
#         self.recovery_active = False

#         self.get_logger().info("Auto Manipulator FSM Ready: Waiting for command...")

#     # -------------------------------------
#     # Subscriber Callbacks
#     # -------------------------------------
#     def start_callback(self, msg):
#         if msg.data == 1:
#             if self.state == STATE_HOME:
#                 self.state = STATE_BEVEL_START
#                 self.get_logger().info("START SCOOPING (STATE 1)")
#         elif msg.data == 2:
#             if self.state == STATE_CARRY:
#                 self.state = STATE_BEVEL_DUMP
#                 self.get_logger().info("START DROPPING (STATE 6)")

#     def link1_imu_callback(self, msg):
#         if len(msg.data) >= 2:
#             self.link1_angle = msg.data[1]

#     def bevel_imu_callback(self, msg):
#         if len(msg.data) >= 2:
#             self.bevel_angle = msg.data[1]

#     def bevel_limit_down_callback(self, msg):
#         self.bevel_limit_down = (msg.data == 1)

#     def bevel_limit_up_callback(self, msg):
#         self.bevel_limit_up = (msg.data == 1)

#     def link1_limit_callback(self, msg):
#         self.link1_limit_home = (msg.data == 1)

#     # -------------------------------------
#     # Utility
#     # -------------------------------------
#     def cmd(self, value):
#         msg = Int16()
#         msg.data = value
#         return msg
    
#     def int_msg(self, value: int) -> Int16:
#         msg = Int16()
#         msg.data = value
#         return msg

#     def stop_all(self):
#         self.pub_bevel_cmd.publish(self.cmd(0))
#         self.pub_link1_cmd.publish(self.cmd(0))

#     # -------------------------------------
#     # FSM UPDATE LOOP
#     # -------------------------------------
#     def update(self):
#         cmd = Int16()

#         # ================================ STATE 0 =================================
#         if self.state == STATE_HOME:
#             self.stop_all()

#         # ================================ STATE 1 =================================
#         elif self.state == STATE_BEVEL_START:
#             if self.bevel_angle < self.target_bevel_scoop_angle:
#                 self.pub_bevel_cmd.publish(self.int_msg(8))  # +ve down
#             else:
#                 self.stop_all()
#                 self.state = STATE_LINK1_DOWN
#                 self.get_logger().info("STATE 2 - Link1 Lowering...")

#         # ================================ STATE 2 =================================
#         elif self.state == STATE_LINK1_DOWN:
#             if self.link1_angle < self.target_link1_down_angle:
#                 self.pub_link1_cmd.publish(self.int_msg(2))  # +ve down
#             else:
#                 self.stop_all()
#                 self.state = STATE_BEVEL_LIMIT_DOWN
#                 self.recovery_timer_started = False
#                 self.get_logger().info("STATE 3 - Bevel to Ground Limit...")

#         # ================================ STATE 3 =================================
#         elif self.state == STATE_BEVEL_LIMIT_DOWN:
#             if self.bevel_limit_down:
#                 self.stop_all()
#                 self.state = STATE_LINK1_UP_FOR_SCOOP
#                 self.get_logger().info("STATE 4 - Raise Link1 for Load")
#             else:
#                 self.pub_bevel_cmd.publish(self.int_msg(8))  # keep trying

#                 if not self.recovery_timer_started:
#                     self.recover_start_time = self.get_clock().now()
#                     self.recovery_timer_started = True

#                 elapsed = (self.get_clock().now() - self.recover_start_time).nanoseconds * 1e-9
#                 if elapsed > self.recovery_timeout:
#                     self.state = STATE_RECOVERY
#                     self.get_logger().warn("STATE 10 - Recovery Mode")

#         # ================================ STATE 10 ================================
#         elif self.state == STATE_RECOVERY:
#             self.pub_bevel_cmd.publish(self.int_msg(8))  # keep pushing
#             self.pub_link1_cmd.publish(self.int_msg(1))  # move up
        
#             if self.link1_angle <= (self.target_link1_down_angle - self.link1_recovery_step):
#                 self.state = STATE_BEVEL_LIMIT_DOWN
#                 self.recovery_timer_started = False
#                 self.get_logger().info("Retry STATE 3 - Ground Limit")

#         # ================================ STATE 4 =================================
#         elif self.state == STATE_LINK1_UP_FOR_SCOOP:
#             if self.link1_angle > self.target_link1_up_scoop_angle:
#                 self.pub_link1_cmd.publish(self.int_msg(1))  # up
#             else:
#                 self.stop_all()
#                 self.state = STATE_CARRY
#                 self.get_logger().info("STATE 5 - Carrying Sand")

#         # ================================ STATE 5 =================================
#         elif self.state == STATE_CARRY:
#             self.pub_flag.publish(self.int_msg(1))

#         # ================================ STATE 6 =================================
#         elif self.state == STATE_BEVEL_DUMP:
#             if self.bevel_limit_up:
#                 self.stop_all()
#                 self.state = STATE_LINK1_HOME
#                 self.get_logger().info("STATE 7 - Returning Link1 to Home")
#             else:
#                 self.pub_bevel_cmd.publish(self.int_msg(4))

#         # ================================ STATE 7 =================================
#         elif self.state == STATE_LINK1_HOME:
#             if self.link1_limit_home:
#                 self.stop_all()
#                 self.state = STATE_HOME
#                 self.get_logger().info("STATE 0 - Finished. Back Home ✔")
#             else:
#                 self.pub_link1_cmd.publish(self.int_msg(1))

#             self.pub_flag.publish(self.int_msg(2))


# def main(args=None):
#     rclpy.init(args=args)
#     node = AutoNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == "__main__":
#     main()














































#####################################################################################














# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Int16, Float32MultiArray
# from rclpy.time import Time

# # ----------------------------
# # FSM STATES
# # ----------------------------
# STATE_HOME = 0
# STATE_BEVEL_START = 1
# STATE_LINK1_DOWN = 2
# STATE_BEVEL_LIMIT_DOWN = 3
# STATE_LINK1_UP_FOR_SCOOP = 4
# STATE_CARRY = 5
# STATE_BEVEL_DUMP = 6
# STATE_LINK1_HOME = 7
# STATE_RECOVERY = 10

# class AutoNode(Node):

#     def __init__(self):
#         super().__init__('auto_manip')

#         self.state = STATE_HOME

#         # Publishers
#         self.pub_bevel_cmd = self.create_publisher(Int16, 'lift_angle', 10)  # /bevel_cmd
#         self.pub_link1_cmd = self.create_publisher(Int16, '/manipulator_cmd', 10)
#         self.pub_flag = self.create_publisher(Int16, '/vihaan', 10)

#         # Subscribers
#         self.create_subscription(Int16, '/rohan', self.start_callback, 10)
#         self.create_subscription(Float32MultiArray, '/imu_orientation', self.link1_imu_callback, 10)
#         self.create_subscription(Float32MultiArray, '/bevel_orientation', self.bevel_imu_callback, 10)
#         self.create_subscription(Int16, '/bevel_limit_down', self.bevel_limit_down_callback, 10)
#         self.create_subscription(Int16, '/bevel_limit_up', self.bevel_limit_up_callback, 10)
#         self.create_subscription(Int16, '/link1_limit', self.link1_limit_callback, 10)

#         # FSM update timer
#         self.timer = self.create_timer(0.02, self.update)

#         # Internal state tracking
#         self.link1_angle = 0.0
#         self.bevel_angle = 0.0

#         self.bevel_limit_down = False
#         self.bevel_limit_up = False
#         self.link1_limit_home = True  # Start at home

#         # Target angles (you will tune later)
#         self.target_bevel_scoop_angle = 22.0
#         self.target_link1_down_angle = 96.0  # should be 96+6, but it goes to -270 after 96, so ece need to change this logic
#         self.target_link1_up_scoop_angle = 50.0
#         self.link1_recovery_step = 5.0
#         self.recovery_timeout = 5.0

#         self.recovery_start = None
#         self.recovery_active = False

#         self.get_logger().info("Auto Manipulator FSM Ready: Waiting for command...")

#     # -------------------------------------
#     # Subscriber Callbacks
#     # -------------------------------------
#     def start_callback(self, msg):
#         if msg.data == 1:
#             if self.state == STATE_HOME:
#                 self.state = STATE_BEVEL_START
#                 self.get_logger().info("START SCOOPING (STATE 1)")
#         elif msg.data == 2:
#             if self.state == STATE_CARRY:
#                 self.state = STATE_BEVEL_DUMP
#                 self.get_logger().info("START DROPPING (STATE 6)")

#     def link1_imu_callback(self, msg):
#         if len(msg.data) >= 2:
#             self.link1_angle = msg.data[1]

#     def bevel_imu_callback(self, msg):
#         if len(msg.data) >= 2:
#             self.bevel_angle = msg.data[1]

#     def bevel_limit_down_callback(self, msg):
#         self.bevel_limit_down = (msg.data == 1)

#     def bevel_limit_up_callback(self, msg):
#         self.bevel_limit_up = (msg.data == 1)

#     def link1_limit_callback(self, msg):
#         self.link1_limit_home = (msg.data == 1)

#     # -------------------------------------
#     # Utility
#     # -------------------------------------
#     def cmd(self, value):
#         msg = Int16()
#         msg.data = value
#         return msg
    
#     def int_msg(self, value: int) -> Int16:
#         msg = Int16()
#         msg.data = value
#         return msg

#     def stop_all(self):
#         self.pub_bevel_cmd.publish(self.cmd(0))
#         self.pub_link1_cmd.publish(self.cmd(0))

#     # -------------------------------------
#     # FSM UPDATE LOOP
#     # -------------------------------------
#     def update(self):
#         cmd = Int16()

#         # ================================ STATE 0 =================================
#         if self.state == STATE_HOME:
#             self.stop_all()

#         # ================================ STATE 1 =================================
#         elif self.state == STATE_BEVEL_START:
#             if self.bevel_angle < self.target_bevel_scoop_angle:
#                 self.pub_bevel_cmd.publish(self.int_msg(8))  # +ve down
#             else:
#                 self.stop_all()
#                 self.state = STATE_LINK1_DOWN
#                 self.get_logger().info("STATE 2 - Link1 Lowering...")

#         # ================================ STATE 2 =================================
#         elif self.state == STATE_LINK1_DOWN:
#             if self.link1_angle < self.target_link1_down_angle:
#                 self.pub_link1_cmd.publish(self.int_msg(2))  # +ve down
#             else:
#                 self.stop_all()
#                 self.state = STATE_BEVEL_LIMIT_DOWN
#                 self.recovery_timer_started = False
#                 self.get_logger().info("STATE 3 - Bevel to Ground Limit...")

#         # ================================ STATE 3 =================================
#         elif self.state == STATE_BEVEL_LIMIT_DOWN:
#             if self.bevel_limit_down:
#                 self.stop_all()
#                 self.state = STATE_LINK1_UP_FOR_SCOOP
#                 self.get_logger().info("STATE 4 - Raise Link1 for Load")
#             else:
#                 self.pub_bevel_cmd.publish(self.int_msg(8))  # keep trying

#                 if not self.recovery_timer_started:
#                     self.recover_start_time = self.get_clock().now()
#                     self.recovery_timer_started = True

#                 elapsed = (self.get_clock().now() - self.recover_start_time).nanoseconds * 1e-9
#                 if elapsed > self.recovery_timeout:
#                     self.state = STATE_RECOVERY
#                     self.get_logger().warn("STATE 10 - Recovery Mode")


#         # ================================ STATE 10 ================================
#         elif self.state == STATE_RECOVERY:
    
#             # If we already reached bevel ground limit → stop and continue FSM
#             if self.bevel_limit_down:
#                 self.stop_all()
#                 self.state = STATE_LINK1_UP_FOR_SCOOP
#                 self.get_logger().info("Recovered! → STATE 4 - Raise Link1 for Load")
#                 return

#             # Otherwise keep recovering motion
#             self.pub_bevel_cmd.publish(self.int_msg(8))
#             self.pub_link1_cmd.publish(self.int_msg(1))

#             # Exit recovery once link1 moved UP enough
#             if self.link1_angle >= (self.target_link1_down_angle + self.link1_recovery_step):
#                 self.state = STATE_BEVEL_LIMIT_DOWN
#                 self.recovery_timer_started = False
#                 self.get_logger().info("Retry STATE 3 - Ground Limit after recovery")


#         # ================================ STATE 4 =================================
#         elif self.state == STATE_LINK1_UP_FOR_SCOOP:
#             if self.link1_angle > self.target_link1_up_scoop_angle:
#                 self.pub_link1_cmd.publish(self.int_msg(1))  # up
#             else:
#                 self.stop_all()
#                 self.state = STATE_CARRY
#                 self.get_logger().info("STATE 5 - Carrying Sand")

#         # ================================ STATE 5 =================================
#         elif self.state == STATE_CARRY:
#             self.pub_flag.publish(self.int_msg(1))

#         # ================================ STATE 6 =================================
#         elif self.state == STATE_BEVEL_DUMP:
#             if self.bevel_limit_up:
#                 self.stop_all()
#                 self.state = STATE_LINK1_HOME
#                 self.get_logger().info("STATE 7 - Returning Link1 to Home")
#             else:
#                 self.pub_bevel_cmd.publish(self.int_msg(4))

#         # ================================ STATE 7 =================================
#         elif self.state == STATE_LINK1_HOME:
#             if self.link1_limit_home:
#                 self.stop_all()
#                 self.state = STATE_HOME
#                 self.get_logger().info("STATE 0 - Finished. Back Home ✔")
#             else:
#                 self.pub_link1_cmd.publish(self.int_msg(1))

#             self.pub_flag.publish(self.int_msg(2))


# def main(args=None):
#     rclpy.init(args=args)
#     node = AutoNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == "__main__":
#     main()


























# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Int16, Float32MultiArray

# # ----------------------------
# # FSM STATES
# # ----------------------------
# STATE_HOME = 0
# STATE_BEVEL_START = 1
# STATE_LINK1_DOWN = 2
# STATE_BEVEL_LIMIT_DOWN = 3
# STATE_LINK1_UP_FOR_SCOOP = 4
# STATE_CARRY = 5
# STATE_BEVEL_DUMP = 6
# STATE_LINK1_HOME = 7
# STATE_RECOVERY = 10


# class AutoNode(Node):

#     def __init__(self):
#         super().__init__('auto_manip')

#         self.state = STATE_HOME

#         # Publishers
#         self.pub_bevel_cmd = self.create_publisher(Int16, 'lift_angle', 10)
#         self.pub_link1_cmd = self.create_publisher(Int16, '/manipulator_cmd', 10)
#         self.pub_flag = self.create_publisher(Int16, '/vihaan', 10)

#         # Subscribers
#         self.create_subscription(Int16, '/rohan', self.start_callback, 10)
#         self.create_subscription(Float32MultiArray, '/imu_orientation', self.link1_imu_callback, 10)
#         self.create_subscription(Float32MultiArray, '/bevel_orientation', self.bevel_imu_callback, 10)
#         self.create_subscription(Int16, '/bevel_limit_down', self.bevel_limit_down_callback, 10)
#         self.create_subscription(Int16, '/bevel_limit_up', self.bevel_limit_up_callback, 10)
#         self.create_subscription(Int16, '/link1_limit', self.link1_limit_callback, 10)

#         # FSM Timer
#         self.timer = self.create_timer(0.02, self.update)

#         # State Tracking Variables
#         self.link1_angle = 0.0
#         self.bevel_angle = 0.0

#         # Limit switches
#         self.bevel_limit_down = False
#         self.bevel_limit_up = False
#         self.link1_limit_home = False  # FIX: do not start at home!

#         # Target angles
#         self.target_bevel_scoop_angle = 22.0
#         self.target_link1_down_angle = 96.0
#         self.target_link1_up_scoop_angle = 50.0

#         # Recovery timing
#         self.recovery_timeout = 5.0
#         self.recovery_timer_started = False
#         self.recover_start_time = None
#         self.link1_recovery_step = 5.0

#         self.get_logger().info("Auto Manip Ready ✓ Waiting for /rohan command.")

#     # ----------------------- Callbacks -----------------------
#     def start_callback(self, msg):
#         if msg.data == 1 and self.state == STATE_HOME:
#             self.state = STATE_BEVEL_START
#             self.link1_limit_home = False  # reset!
#             self.get_logger().info("STATE 1: Begin Scooping")
#         elif msg.data == 2 and self.state == STATE_CARRY:
#             self.state = STATE_BEVEL_DUMP
#             self.get_logger().info("STATE 6: Begin Dumping")

#     def link1_imu_callback(self, msg):
#         if len(msg.data) >= 2:
#             self.link1_angle = msg.data[1]

#     def bevel_imu_callback(self, msg):
#         if len(msg.data) >= 2:
#             self.bevel_angle = msg.data[1]

#     def bevel_limit_down_callback(self, msg):
#         self.bevel_limit_down = (msg.data == 1)

#     def bevel_limit_up_callback(self, msg):
#         self.bevel_limit_up = (msg.data == 1)

#     def link1_limit_callback(self, msg):
#         self.link1_limit_home = (msg.data == 1)

#     # ----------------------- Helper -----------------------
#     def int_msg(self, v): return Int16(data=v)

#     def stop_all(self):
#         self.pub_bevel_cmd.publish(self.int_msg(0))
#         self.pub_link1_cmd.publish(self.int_msg(0))

#     # ----------------------- FSM LOOP -----------------------
#     def update(self):

#         # ---- STATE 0 ----
#         if self.state == STATE_HOME:
#             self.stop_all()
#             self.pub_flag.publish(self.int_msg(2))  # Finished cycle signal
#             return

#         # ---- STATE 1 ----
#         elif self.state == STATE_BEVEL_START:
#             if self.bevel_angle < self.target_bevel_scoop_angle:
#                 self.pub_bevel_cmd.publish(self.int_msg(8))
#             else:
#                 self.stop_all()
#                 self.state = STATE_LINK1_DOWN
#                 self.get_logger().info("STATE 2: Lower Link1")

#         # ---- STATE 2 ----
#         elif self.state == STATE_LINK1_DOWN:
#             if self.link1_angle < self.target_link1_down_angle:
#                 self.pub_link1_cmd.publish(self.int_msg(2))
#             else:
#                 self.stop_all()
#                 self.state = STATE_BEVEL_LIMIT_DOWN
#                 self.recovery_timer_started = False
#                 self.get_logger().info("STATE 3: Move Bevel Until Ground Limit")

#         # ---- STATE 3 ----
#         elif self.state == STATE_BEVEL_LIMIT_DOWN:

#             if self.bevel_limit_down:
#                 self.stop_all()
#                 self.state = STATE_LINK1_UP_FOR_SCOOP
#                 self.recovery_timer_started = False
#                 self.get_logger().info("STATE 4: Lift Link1 for Load")
#                 return

#             self.pub_bevel_cmd.publish(self.int_msg(8))

#             # Timer
#             if not self.recovery_timer_started:
#                 self.recover_start_time = self.get_clock().now()
#                 self.recovery_timer_started = True

#             elapsed = (self.get_clock().now() - self.recover_start_time).nanoseconds * 1e-9

#             if elapsed > self.recovery_timeout:
#                 self.state = STATE_RECOVERY
#                 self.recovery_timer_started = False
#                 self.get_logger().warn("STATE 10: Recovery due to timeout")

#         # ---- STATE 10 ----
#         elif self.state == STATE_RECOVERY:

#             # If bevel limit hit now → success!
#             if self.bevel_limit_down:
#                 self.stop_all()
#                 self.state = STATE_LINK1_UP_FOR_SCOOP
#                 self.get_logger().info("Recovered → STATE 4")
#                 return
            
#             # Recovery motion
#             self.pub_bevel_cmd.publish(self.int_msg(8))
#             self.pub_link1_cmd.publish(self.int_msg(1))

#             # Retry after Link1 up slightly
#             if self.link1_angle >= (self.target_link1_down_angle + self.link1_recovery_step):
#                 self.state = STATE_BEVEL_LIMIT_DOWN
#                 self.recovery_timer_started = False
#                 self.get_logger().info("Retry → STATE 3")

#         # ---- STATE 4 ----
#         elif self.state == STATE_LINK1_UP_FOR_SCOOP:
#             if self.link1_angle > self.target_link1_up_scoop_angle:
#                 self.pub_link1_cmd.publish(self.int_msg(1))
#             else:
#                 self.stop_all()
#                 self.state = STATE_CARRY
#                 self.get_logger().info("STATE 5: Carrying Sand")
        
#         # ---- STATE 5 ----
#         elif self.state == STATE_CARRY:
#             self.pub_flag.publish(self.int_msg(1))

#         # ---- STATE 6 ----
#         elif self.state == STATE_BEVEL_DUMP:
#             if self.bevel_limit_up:
#                 self.stop_all()
#                 self.state = STATE_LINK1_HOME
#                 self.get_logger().info("STATE 7: Return Link1 to Home")
#             else:
#                 self.pub_bevel_cmd.publish(self.int_msg(4))

#         # ---- STATE 7 ----
#         elif self.state == STATE_LINK1_HOME:
#             self.pub_flag.publish(self.int_msg(2))  # signal return in progress
#             if not self.link1_limit_home:
#                 self.pub_link1_cmd.publish(self.int_msg(1))
#             else:
#                 self.stop_all()
#                 self.state = STATE_HOME
#                 self.get_logger().info("✔ STATE 0: Finished Cycle")


# def main(args=None):
#     rclpy.init(args=args)
#     node = AutoNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()


























import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, Float32MultiArray


# FSM STATES
STATE_HOME = 0
STATE_BEVEL_START = 1
STATE_LINK1_DOWN = 2
STATE_BEVEL_LIMIT_DOWN = 3
STATE_LINK1_UP_FOR_SCOOP = 4
STATE_CARRY = 5
STATE_BEVEL_DUMP = 6
STATE_LINK1_HOME = 7
STATE_RECOVERY = 10


class AutoNode(Node):

    def __init__(self):
        super().__init__("auto_manip")

        self.state = STATE_HOME

        # Publishers
        self.pub_bevel_cmd = self.create_publisher(Int16, "bevel_cmd", 10)
        self.pub_link1_cmd = self.create_publisher(Int16, "/manipulator_cmd", 10)
        self.pub_flag = self.create_publisher(Int16, "/vihaan", 10)

        # Subscribers
        self.create_subscription(Int16, "/rohan", self.start_callback, 10)
        self.create_subscription(Float32MultiArray, "/imu_orientation", self.link1_imu_callback, 10)
        self.create_subscription(Float32MultiArray, "/bevel_orientation", self.bevel_imu_callback, 10)
        self.create_subscription(Int16, "/bevel_limit_down", self.bevel_limit_down_callback, 10)
        self.create_subscription(Int16, "/bevel_limit_up", self.bevel_limit_up_callback, 10)
        self.create_subscription(Int16, "/link1_limit", self.link1_limit_callback, 10)

        self.timer = self.create_timer(0.02, self.update)

        # State variables initialization
        self.reset_cycle_state()

        # Target parameters
        self.target_bevel_scoop_angle = 22.0
        self.target_link1_down_angle = 90.0    # 90.0
        self.target_link1_up_scoop_angle = 50.0
        self.recovery_timeout = 10.0  
        self.link1_recovery_step = 5.0

        self.get_logger().info("Ready, Waiting for /rohan command")

    # Reset states when a new pick cycle begins
    def reset_cycle_state(self):
        self.link1_angle = 0.0
        self.bevel_angle = 0.0
        self.bevel_limit_down = False
        self.bevel_limit_up = False
        self.link1_limit_home = True
        self.recovery_timer_started = False
        self.recover_start_time = None

    # ---- CALLBACKS ----
    def start_callback(self, msg):
        if msg.data == 1 and self.state == STATE_HOME:
            self.reset_cycle_state()
            self.link1_limit_home = False
            self.state = STATE_BEVEL_START
            self.get_logger().info("STATE 1 → Starting Scoop Sequence")

        elif msg.data == 2 and self.state == STATE_CARRY:
            self.state = STATE_BEVEL_DUMP
            self.get_logger().info("STATE 6 → Starting Dump Sequence")

    def link1_imu_callback(self, msg):
        if len(msg.data) >= 2:
            self.link1_angle = msg.data[1]

    def bevel_imu_callback(self, msg):
        if len(msg.data) >= 2:
            self.bevel_angle = msg.data[1]

    def bevel_limit_down_callback(self, msg):
        self.bevel_limit_down = (msg.data == 1)

    def bevel_limit_up_callback(self, msg):
        self.bevel_limit_up = (msg.data == 1)

    def link1_limit_callback(self, msg):
        self.link1_limit_home = (msg.data == 1)

    # Utility
    def int_msg(self, value): return Int16(data=value)

    def stop_all(self):
        self.pub_bevel_cmd.publish(self.int_msg(0))
        self.pub_link1_cmd.publish(self.int_msg(0))

    # ---- FSM LOOP ----
    def update(self):

        # STATE 0 : HOME
        if self.state == STATE_HOME:
            self.stop_all()
            self.pub_flag.publish(self.int_msg(2))  # Still ready after last dump
            return

        # STATE 1 : BEVEL START SCOOPING
        elif self.state == STATE_BEVEL_START:
            if self.bevel_angle < self.target_bevel_scoop_angle:
                self.pub_bevel_cmd.publish(self.int_msg(8))
            else:
                self.stop_all()
                self.state = STATE_LINK1_DOWN
                self.get_logger().info("STATE 2: Lower Link1")

        # STATE 2 : LINK1 DOWN
        elif self.state == STATE_LINK1_DOWN:
            if (self.link1_angle >=0 and self.link1_angle < self.target_link1_down_angle) or (self.link1_angle < -265):
            #if self.link1_angle < self.target_link1_down_angle:
                self.pub_link1_cmd.publish(self.int_msg(2))
            else:
                self.stop_all()
                self.state = STATE_BEVEL_LIMIT_DOWN
                self.recovery_timer_started = False
                self.get_logger().info("STATE 3: Search Bevel Limit")

        # STATE 3 : BEVEL GND SEARCH
        elif self.state == STATE_BEVEL_LIMIT_DOWN:

            if self.bevel_limit_down:
                self.stop_all()
                self.state = STATE_LINK1_UP_FOR_SCOOP
                self.get_logger().info("STATE 4: Lift Link1")
                return

            self.pub_bevel_cmd.publish(self.int_msg(8))

            if not self.recovery_timer_started:
                self.recover_start_time = self.get_clock().now()
                self.recovery_timer_started = True

            elapsed = (self.get_clock().now() - self.recover_start_time).nanoseconds * 1e-9
            if elapsed > self.recovery_timeout:
                self.state = STATE_RECOVERY
                self.recovery_timer_started = False
                self.get_logger().warn("⚠ Recovery Mode")

        # STATE 10 : RECOVERY
        elif self.state == STATE_RECOVERY:

            if self.bevel_limit_down:
                self.stop_all()
                self.state = STATE_LINK1_UP_FOR_SCOOP
                self.get_logger().info("Recovered → STATE 4")
                return

            self.pub_bevel_cmd.publish(self.int_msg(8))
            self.pub_link1_cmd.publish(self.int_msg(1))

            if self.link1_angle >= (self.target_link1_down_angle + self.link1_recovery_step):
                self.state = STATE_BEVEL_LIMIT_DOWN
                self.get_logger().info("Retry → STATE 3")

        # STATE 4 : LINK1 UP FOR SCOOP
        elif self.state == STATE_LINK1_UP_FOR_SCOOP:
            if self.link1_angle > self.target_link1_up_scoop_angle:
                self.pub_link1_cmd.publish(self.int_msg(1))
            else:
                self.stop_all()
                self.state = STATE_CARRY
                self.get_logger().info("STATE 5: Carry")

        # STATE 5 : CARRY
        elif self.state == STATE_CARRY:
            self.pub_flag.publish(self.int_msg(1))

        # STATE 6 : BEVEL DUMP
        elif self.state == STATE_BEVEL_DUMP:
            if self.bevel_limit_up:
                self.stop_all()
                self.state = STATE_LINK1_HOME
                self.get_logger().info("STATE 7: Return to Home")
            else:
                self.pub_bevel_cmd.publish(self.int_msg(4))

        # STATE 7 : RETURN LINK HOME
        elif self.state == STATE_LINK1_HOME:

            if not self.link1_limit_home:
                self.pub_link1_cmd.publish(self.int_msg(1))
            else:
                self.stop_all()
                self.state = STATE_HOME
                self.get_logger().info("Dump Complete → STATE HOME")
                return


def main(args=None):
    rclpy.init(args=args)
    node = AutoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
