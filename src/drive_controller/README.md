# Drive Controller

This package contains launch files and controller configuration for bringing up the 4-wheel drive system.

## Usage

### Launch with Real Hardware (CubeMars motors via CAN)

First, make sure your CAN interface is up:

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

Then launch with real hardware:

```bash
ros2 launch drive_controller cubemars_test.launch.py
```

This launches the controller manager with the drive controller for 4 wheels.
```

**Use the old test launch file (deprecated, kept for reference):**
```bash
ros2 launch drive_controller cubemars_test.launch.py
```

## Controllers

The default controller configuration includes:
- `joint_state_broadcaster`: Publishes joint states to `/joint_states` topic
- `forward_position_controller`: Accepts position commands for all joints

## Testing Commands

Send a position command to all joints:

```bash
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.5, -0.5, 0.0, 0.0]"
```

Monitor joint states:

```bash
ros2 topic echo /joint_states
```

List available controllers:

```bash
ros2 control list_controllers
```

List hardware interfaces:

```bash
ros2 control list_hardware_interfaces
```

