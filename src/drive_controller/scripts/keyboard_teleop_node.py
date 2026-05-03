#!/usr/bin/env python3
"""
Simple keyboard teleop for the Ackermann rover.

Publishes geometry_msgs/Twist on /cmd_vel which is consumed by
ackermann_drive_node.py.

Controls (hold-to-send, momentum-style):
    w / s : increase / decrease forward speed
    a / d : increase / decrease yaw rate (left / right)
    space : stop linear AND angular immediately
    x     : zero linear speed only
    z     : zero angular rate only
    +/-   : scale linear step
    [ / ] : scale angular step
    q     : quit

Each key press changes a setpoint that is republished at `publish_rate` Hz
so the Ackermann node's watchdog stays satisfied.
"""

import sys
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


# ---- cross-platform single-char reader ---------------------------------

if sys.platform == "win32":
    import msvcrt

    def get_key(timeout: float) -> str:
        # Poll for a key with the given timeout (seconds).
        # msvcrt has no native timeout; emulate by short sleeps.
        import time

        end = time.time() + timeout
        while time.time() < end:
            if msvcrt.kbhit():
                ch = msvcrt.getwch()
                return ch
            time.sleep(0.01)
        return ""
else:
    import select
    import termios
    import tty

    _orig_settings = None

    def _init_terminal():
        global _orig_settings
        _orig_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    def _restore_terminal():
        if _orig_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, _orig_settings)

    def get_key(timeout: float) -> str:
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            return sys.stdin.read(1)
        return ""


HELP = """
Keyboard teleop for Ackermann rover
-----------------------------------
  w / s : forward / reverse  (linear.x)
  a / d : steer left / right (angular.z)
  space : full stop
  x     : zero linear
  z     : zero angular
  + / - : larger / smaller linear step
  [ / ] : smaller / larger angular step
  q     : quit
"""


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__("keyboard_teleop")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("linear_step", 0.1)        # m/s per keypress
        self.declare_parameter("angular_step", 0.2)       # rad/s per keypress
        self.declare_parameter("max_linear", 1.5)         # m/s
        self.declare_parameter("max_angular", 1.5)        # rad/s
        self.declare_parameter("publish_rate", 20.0)      # Hz

        self.lin_step = float(self.get_parameter("linear_step").value)
        self.ang_step = float(self.get_parameter("angular_step").value)
        self.max_lin = float(self.get_parameter("max_linear").value)
        self.max_ang = float(self.get_parameter("max_angular").value)
        self.rate_hz = float(self.get_parameter("publish_rate").value)

        self.pub = self.create_publisher(
            Twist, self.get_parameter("cmd_vel_topic").value, 10
        )

        self.linear = 0.0
        self.angular = 0.0
        self._lock = threading.Lock()
        self._stop = False

        self.timer = self.create_timer(1.0 / max(self.rate_hz, 1.0), self._publish)

        self.get_logger().info(HELP)

    @staticmethod
    def _clamp(x: float, lim: float) -> float:
        return max(-lim, min(lim, x))

    def _publish(self):
        with self._lock:
            v, w = self.linear, self.angular
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.pub.publish(msg)

    def _print_state(self):
        sys.stdout.write(
            f"\r v={self.linear:+.2f} m/s   w={self.angular:+.2f} rad/s   "
            f"step(lin={self.lin_step:.2f}, ang={self.ang_step:.2f})    "
        )
        sys.stdout.flush()

    def run(self):
        try:
            if sys.platform != "win32":
                _init_terminal()
            while rclpy.ok() and not self._stop:
                key = get_key(0.1)
                if not key:
                    continue
                k = key.lower()
                with self._lock:
                    if k == "w":
                        self.linear = self._clamp(self.linear + self.lin_step, self.max_lin)
                    elif k == "s":
                        self.linear = self._clamp(self.linear - self.lin_step, self.max_lin)
                    elif k == "a":
                        self.angular = self._clamp(self.angular + self.ang_step, self.max_ang)
                    elif k == "d":
                        self.angular = self._clamp(self.angular - self.ang_step, self.max_ang)
                    elif k == " ":
                        self.linear = 0.0
                        self.angular = 0.0
                    elif k == "x":
                        self.linear = 0.0
                    elif k == "z":
                        self.angular = 0.0
                    elif k == "+" or k == "=":
                        self.lin_step = min(self.lin_step * 1.25, self.max_lin)
                    elif k == "-" or k == "_":
                        self.lin_step = max(self.lin_step * 0.8, 0.01)
                    elif k == "]":
                        self.ang_step = min(self.ang_step * 1.25, self.max_ang)
                    elif k == "[":
                        self.ang_step = max(self.ang_step * 0.8, 0.01)
                    elif k == "q" or k == "\x03":  # q or Ctrl-C
                        self._stop = True
                        self.linear = 0.0
                        self.angular = 0.0
                self._print_state()
        finally:
            # Send a final zero so the rover stops cleanly.
            stop_msg = Twist()
            self.pub.publish(stop_msg)
            if sys.platform != "win32":
                _restore_terminal()
            sys.stdout.write("\n")


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
