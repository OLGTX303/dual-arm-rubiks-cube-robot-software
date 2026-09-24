# linux_web_debug retired

This legacy runtime was removed because it returned fabricated UI values
(including fixed HSV colors and fixed calibration points) and used the legacy
Python motor bridge.

Use the maintained Linux runtime in:

- repository: `OLGTX303/cube_robot_esp32_v2_linux`
- branch: `fix/full-esp32-linux-port`
- service: `linux_system_backend.py`

The maintained runtime reads USB camera frames, calibration/configuration,
and RS485 motor data from the system and executes the pinned upstream ESP32 C
motion/color/solver implementation.
