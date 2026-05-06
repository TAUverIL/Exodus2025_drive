#!/usr/bin/env python3
"""
Xbox controller teleop for the Ackermann rover (Linux / Ubuntu).

Requires the ROS 2 joy driver running in a separate terminal:
    sudo apt install ros-$ROS_DISTRO-joy
    ros2 run joy joy_node

This node subscribes to /joy (sensor_msgs/Joy) and publishes
geometry_msgs/Twist on /cmd_vel, which is consumed by ackermann_drive_node.py.

Xbox One / Xbox 360 axis & button mapping on Linux (xpad / xboxdrv):
────────────────────────────────────────────────────────────────────
  Axes (value range -1.0 … +1.0, triggers 0.0 … +1.0 after scaling):
    0  Left  stick  X   (+left / -right)
    1  Left  stick  Y   (+forward / -back)
    2  Left  trigger     (0 = released, +1 = fully pressed)
    3  Right stick  X
    4  Right stick  Y
    5  Right trigger     (0 = released, +1 = fully pressed)
    6  D-pad X          (+1 = left, -1 = right)
    7  D-pad Y          (+1 = up,   -1 = down)

  Buttons:
    0  A        — full stop (linear + angular)
    1  B        — zero angular only
    2  X        — zero linear only
    3  Y        — (unused)
    4  LB       — decrease speed scale
    5  RB       — increase speed scale
    6  Back     — (unused)
    7  Start    — quit / shutdown node
    8  Xbox     — (unused)
    9  L-stick  — (unused)
   10  R-stick  — (unused)

Drive modes
───────────
  ANALOG  (default): Left stick Y/X → linear / angular directly proportional.
          Speed is scaled by the current speed_scale factor (LB / RB to adjust).

  DPAD:   D-pad accumulates setpoints exactly like the keyboard node
          (up/down = ±linear step, left/right = ±angular step).
          Switching between modes is automatic: any stick input activates ANALOG,
          any D-pad press activates DPAD.

Dead-man switch
───────────────
  Optional: set dead_man_button to a button index (e.g. 5 for RB).
  While that button is NOT held the node sends zero. Default -1 = disabled.
"""

import math
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


# ---- Xbox axis / button indices (standard Linux xpad mapping) -----------

AXIS_LEFT_X   = 0   # positive = left
AXIS_LEFT_Y   = 1   # positive = forward
AXIS_LT       = 2   # 0 released → +1 pressed
AXIS_RIGHT_X  = 3
AXIS_RIGHT_Y  = 4
AXIS_RT        = 5
AXIS_DPAD_X   = 6   # +1 = left, -1 = right
AXIS_DPAD_Y   = 7   # +1 = up,   -1 = down

BTN_A         = 0
BTN_B         = 1
BTN_X         = 2
BTN_Y         = 3
BTN_LB        = 4
BTN_RB        = 5
BTN_BACK      = 6
BTN_START     = 7
BTN_XBOX      = 8

ANALOG_DEADZONE = 0.05  # ignore stick deflections below this threshold


class XboxTeleop(Node):
    def __init__(self):
        super().__init__("xbox_teleop")

        # ---- Parameters ------------------------------------------------
        self.declare_parameter("cmd_vel_topic",    "/cmd_vel")
        self.declare_parameter("joy_topic",        "/joy")
        self.declare_parameter("max_linear",       1.5)    # m/s
        self.declare_parameter("max_angular",      1.5)    # rad/s
        self.declare_parameter("linear_step",      0.1)    # m/s per d-pad press
        self.declare_parameter("angular_step",     0.2)    # rad/s per d-pad press
        self.declare_parameter("scale_step",       0.1)    # speed_scale increment per LB/RB
        self.declare_parameter("publish_rate",     20.0)   # Hz
        self.declare_parameter("cmd_timeout",      0.5)    # s watchdog
        self.declare_parameter("dead_man_button",  -1)     # -1 = disabled

        self.max_lin     = float(self.get_parameter("max_linear").value)
        self.max_ang     = float(self.get_parameter("max_angular").value)
        self.lin_step    = float(self.get_parameter("linear_step").value)
        self.ang_step    = float(self.get_parameter("angular_step").value)
        self.scale_step  = float(self.get_parameter("scale_step").value)
        self.rate_hz     = float(self.get_parameter("publish_rate").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)
        self.dead_man    = int(self.get_parameter("dead_man_button").value)

        # ---- State -------------------------------------------------------
        self.linear      = 0.0
        self.angular     = 0.0
        self.speed_scale = 1.0      # multiplier applied on top of max_* limits
        self.last_joy    = self.get_clock().now()

        # D-pad edge detection
        self._prev_dpad_x = 0.0
        self._prev_dpad_y = 0.0

        # Button edge detection (prevent auto-repeat)
        self._prev_buttons: list[int] = []

        # ---- I/O ---------------------------------------------------------
        self.pub = self.create_publisher(
            Twist, self.get_parameter("cmd_vel_topic").value, 10
        )
        self.sub = self.create_subscription(
            Joy, self.get_parameter("joy_topic").value, self._joy_cb, 10
        )
        self.timer = self.create_timer(
            1.0 / max(self.rate_hz, 1.0), self._publish
        )

        self.get_logger().info(
            "\n"
            "Xbox teleop ready\n"
            "─────────────────────────────────────\n"
            "  Left stick Y/X : forward / steer (ANALOG mode)\n"
            "  D-pad up/down  : +/- linear  (DPAD mode)\n"
            "  D-pad left/right: +/- angular (DPAD mode)\n"
            "  A              : full stop\n"
            "  B              : zero angular\n"
            "  X              : zero linear\n"
            "  LB / RB        : speed scale -/+\n"
            "  Start          : shutdown node\n"
            "─────────────────────────────────────"
        )

    # ---------------------------------------------------------------- helpers

    @staticmethod
    def _clamp(x: float, lim: float) -> float:
        return max(-lim, min(lim, x))

    @staticmethod
    def _dead(x: float, zone: float = ANALOG_DEADZONE) -> float:
        return 0.0 if abs(x) < zone else x

    def _safe_axis(self, msg: Joy, idx: int) -> float:
        if idx < len(msg.axes):
            return float(msg.axes[idx])
        return 0.0

    def _safe_btn(self, msg: Joy, idx: int) -> int:
        if idx < len(msg.buttons):
            return int(msg.buttons[idx])
        return 0

    def _btn_rising(self, msg: Joy, idx: int) -> bool:
        """True only on the first callback where button idx becomes pressed."""
        current  = self._safe_btn(msg, idx)
        previous = self._prev_buttons[idx] if idx < len(self._prev_buttons) else 0
        return current == 1 and previous == 0

    # ---------------------------------------------------------------- joy cb

    def _joy_cb(self, msg: Joy):
        self.last_joy = self.get_clock().now()

        # Pad previous button state to message length
        prev = list(self._prev_buttons)
        while len(prev) < len(msg.buttons):
            prev.append(0)

        # ---- Shutdown ---------------------------------------------------
        if self._btn_rising(msg, BTN_START):
            self.get_logger().info("Start pressed — shutting down.")
            self._prev_buttons = list(msg.buttons)
            raise SystemExit

        # ---- Dead-man check (evaluated after shutdown so Start still works)
        dead_man_ok = (
            self.dead_man < 0 or self._safe_btn(msg, self.dead_man) == 1
        )

        if not dead_man_ok:
            self.linear  = 0.0
            self.angular = 0.0
            self._prev_buttons = list(msg.buttons)
            return

        # ---- Stop buttons -----------------------------------------------
        if self._btn_rising(msg, BTN_A):
            self.linear  = 0.0
            self.angular = 0.0

        if self._btn_rising(msg, BTN_B):
            self.angular = 0.0

        if self._btn_rising(msg, BTN_X):
            self.linear = 0.0

        # ---- Speed scale ------------------------------------------------
        if self._btn_rising(msg, BTN_RB):
            self.speed_scale = min(self.speed_scale + self.scale_step, 1.0)
            self.get_logger().info(f"Speed scale: {self.speed_scale:.2f}")

        if self._btn_rising(msg, BTN_LB):
            self.speed_scale = max(self.speed_scale - self.scale_step, 0.05)
            self.get_logger().info(f"Speed scale: {self.speed_scale:.2f}")

        # ---- Analog sticks (direct proportional) ------------------------
        ly = self._dead(self._safe_axis(msg, AXIS_LEFT_Y))
        lx = self._dead(self._safe_axis(msg, AXIS_LEFT_X))

        if abs(ly) > 0.0 or abs(lx) > 0.0:
            self.linear  = self._clamp(ly * self.max_lin  * self.speed_scale, self.max_lin)
            self.angular = self._clamp(lx * self.max_ang  * self.speed_scale, self.max_ang)

        # ---- D-pad (momentum / step accumulation) -----------------------
        dx = self._safe_axis(msg, AXIS_DPAD_X)
        dy = self._safe_axis(msg, AXIS_DPAD_Y)

        # Rising-edge on D-pad (it behaves as axis, not button)
        if dy > 0.5 and self._prev_dpad_y <= 0.5:    # up
            self.linear = self._clamp(self.linear + self.lin_step, self.max_lin)
        elif dy < -0.5 and self._prev_dpad_y >= -0.5: # down
            self.linear = self._clamp(self.linear - self.lin_step, self.max_lin)

        if dx > 0.5 and self._prev_dpad_x <= 0.5:    # left
            self.angular = self._clamp(self.angular + self.ang_step, self.max_ang)
        elif dx < -0.5 and self._prev_dpad_x >= -0.5: # right
            self.angular = self._clamp(self.angular - self.ang_step, self.max_ang)

        self._prev_dpad_x = dx
        self._prev_dpad_y = dy
        self._prev_buttons = list(msg.buttons)

        self._log_state()

    # --------------------------------------------------------------- publish

    def _publish(self):
        # Watchdog: if no Joy message for cmd_timeout seconds, send zero
        dt = (self.get_clock().now() - self.last_joy).nanoseconds * 1e-9
        if dt > self.cmd_timeout:
            v, w = 0.0, 0.0
        else:
            v, w = self.linear, self.angular

        msg = Twist()
        msg.linear.x  = v
        msg.angular.z = w
        self.pub.publish(msg)

    def _log_state(self):
        sys.stdout.write(
            f"\r v={self.linear:+.2f} m/s   w={self.angular:+.2f} rad/s   "
            f"scale={self.speed_scale:.2f}    "
        )
        sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    node = XboxTeleop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        # Send zero before exit
        stop = Twist()
        node.pub.publish(stop)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
