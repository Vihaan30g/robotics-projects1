
Caterpillar Autonomy Challenge 2025 – Manipulator Control Package

This package contains the control software developed for the Caterpillar Autonomy Challenge 2025, conducted by IIT Madras.

The competition task required an autonomous robot to repeatedly perform the following cycle:
Navigate to a designated excavation zone.
Precisely scoop sand using the manipulator.
Travel to a different dump zone.
Accurately deposit the scooped sand.
Repeat the cycle continuously to form berms.
All operations were performed fully autonomously, without manual intervention.


Objective
The primary objective of this package was to implement reliable and safe control logic for the manipulator such that:
The manipulator scoops sufficient sand while avoiding excessive forces that could damage the hardware.
The dumping operation occurs at the correct spatial location.
The system can handle real-world uncertainties such as minor positioning errors, sensor noise, and actuator delays.
The robot can automatically recover from abnormal conditions.


System Design
The control logic is structured as a Finite State Machine (FSM).
Each state represents a specific operational phase such as positioning, scooping, carrying, dumping, and recovery.
Multiple recovery behaviors are implemented to handle unexpected situations and ensure robustness during continuous operation.


Hardware Validation
The software was successfully deployed and validated on the physical robot hardware.
Extensive real-world testing was performed to ensure reliability, safety, and repeatability of the manipulation cycle.
This project provided valuable exposure to:
Hardware integration and actuator behavior.
Practical testing methodologies on real robotic systems.
Debugging and validation under real operating conditions.
Collaboration with multidisciplinary teams (mechanical, electronics, and controls).


Note
This package is focused purely on control logic and behavior execution.
Visualization is not included — no URDF or simulation models are provided in this repository.
