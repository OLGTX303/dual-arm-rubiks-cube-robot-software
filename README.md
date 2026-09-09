# Dual-arm Rubik's Cube Robot Software

Software for the dual-arm Rubik's-cube robot using EMM-V5 TTL motor control.

The main motion controller is `cube_motion.py`. The Linux host uses `/dev/ttyS9` at 115200 baud by default when launched by the robot backend. Motor IDs are 1–4; verify the mechanical pose and wiring before enabling motors.

This repository contains only the robot software directory. Hardware-specific calibration values in `points_config.json` may need adjustment for another machine.
