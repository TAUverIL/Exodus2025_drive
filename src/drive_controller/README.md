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

## Ackermann Drive

An Ackermann driver node converts `/cmd_vel` (`geometry_msgs/Twist`) into
the `drive_controller` (4 wheel velocities) and `pivot_controller`
(2 steering angles) commands.

Launch the full stack (controller manager + Ackermann node):

```bash
ros2 launch drive_controller ackermann.launch.py
```

Drive the rover:

```bash
# Forward 0.5 m/s, gentle left turn
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.3}}"
```

Or use teleop:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Built-in keyboard teleop

This package ships its own minimal keyboard teleop node that publishes
to `/cmd_vel`. After `ros2 launch drive_controller ackermann.launch.py`
is running, in **another terminal** run:

```bash
ros2 run drive_controller keyboard_teleop_node.py
```

Controls:

| Key      | Action                              |
|----------|-------------------------------------|
| `w`/`s`  | increase / decrease forward speed   |
| `a`/`d`  | steer left / right (yaw rate)       |
| `space`  | full stop                           |
| `x`      | zero linear only                    |
| `z`      | zero angular only                   |
| `+`/`-`  | larger / smaller linear step        |
| `]`/`[`  | larger / smaller angular step       |
| `q`      | quit (sends a zero command first)   |

The node must be run in a real terminal (not a launch-piped log) so it
can read key presses. On Linux, an alternative launch wrapper that opens
an `xterm` is provided:

```bash
ros2 launch drive_controller keyboard_teleop.launch.py
```

### Tuning

Edit `config/ackermann.yaml` to match your rover:
- `wheelbase`, `track_width`, `wheel_radius` — physical geometry.
- `max_steer_angle`, `max_wheel_speed` — actuator limits.
- `wheel_order` — permutation `[FL, FR, RL, RR]` mapping to the
  drive_controller joint list `[wheel1, wheel2, wheel3, wheel4]`.
- `wheel_direction` — per-wheel sign in case a motor is mounted mirrored.
- `pivot_right_index` / `pivot_left_index` — index of each pivot inside
  `/pivot_controller/commands` (matches the order in `controllers.yaml`).

### Math

For linear velocity $v$ and yaw rate $\omega$, the turn radius is
$R = v/\omega$ and the ICR lies on the rear axle line. With wheelbase
$L$ and track $W$:

$$\delta_\text{inner} = \arctan\!\frac{L}{|R| - W/2}, \qquad
  \delta_\text{outer} = \arctan\!\frac{L}{|R| + W/2}$$

Wheel ground speeds:
$$v_\text{rear,in/out} = \omega\,(|R| \mp W/2), \quad
  v_\text{front,in/out} = \omega\,\sqrt{(|R| \mp W/2)^2 + L^2}$$

Wheel angular velocity = ground speed / `wheel_radius`.

