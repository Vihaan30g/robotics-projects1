
ABOUT THE PKG : 

This package is for real-time Cartesian control of a 5-DOF robotic arm.

The URDF used here is not the exact physical URDF of the real manipulator.
It is a simplified model i created to improve planning performance in MoveIt.

With the original (real) URDF, MoveIt Servo was unable to consistently find valid IK solutions.

The simplified URDF preserves the same link lengths and joint structure as the real manipulator, ensuring that the inverse kinematics solutions remain equivalent.

Since the IK solutions are the same for both models, the joint angle outputs from MoveIt Servo can be directly applied to the real manipulator.

Tool used: MoveIt Servo

Control mode: Position-based control (Servo outputs joint angles)



Tested On : 

- Ubuntu 22.04
- ROS 2 Humble
- RViz2
- MoveIt 2




HOW TO VISUALIZE THE RESULT OF THIS PKG : 



Makesure that all the required dependencies are installed(use rosdep) : 
cd ~/your_workspace
rosdep install --from-paths src --ignore-src -r -y


source the workspace and while being within the workspace folder :- 

After building the pkg in a ROS2 workspace, run following command in a terminal : 
1. ros2 launch manipulator1_moveit_pkg cartesian.launch.py 


To move the manipulator using keyboard, run following command in a separate terminal : 
2. python3 keyboard_twist_publisher.py 