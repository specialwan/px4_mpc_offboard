# PX4 MPC Offboard Control

Model Predictive Control (MPC) based offboard controller for PX4 autopilot via MAVROS.

## Features

- ✅ Pure MPC implementation with quadratic programming (QP) solver
- ✅ MAVROS interface (works with Pixhawk 1/2/4 without uXRCE-DDS)
- ✅ Attitude + thrust control (no PX4 position controller)
- ✅ Waypoint following with custom publisher
- ✅ Data logging for analysis
- ✅ Tested on hardware (Pixhawk + GPS)

## Dependencies

### ROS2 Packages
```bash
sudo apt install ros-humble-mavros ros-humble-mavros-extras
