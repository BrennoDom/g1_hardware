# Unitree G1 ROS2 Control - Arm Interface

https://github.com/BrennoDom/g1_hardware/blob/main/docs/Demonstration_G1.mp4

## Current Capabilities
- **7-DOF Arm Control** (Both left/right arms + waist joints)
- **Position Control** (Primary mode)
- **Real-time Communication** (DDS)
- **Joint State Monitoring** (Position/Velocity/Effort)

## Known Limitations
- Velocity/Effort control still in development (See TODOs)
- Requires manual PID tuning per joint
- SDK installation must be done manually

## TODO

- PID gains in yaml
- seamless activation for motors
- velocity control && effort control
- Installation procedure with unitree_sdk 
