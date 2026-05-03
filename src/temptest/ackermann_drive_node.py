#!/usr/bin/env python3
"""
Ackermann drive node for the 4-wheel / 2-pivot rover.

Subscribes:
  /cmd_vel  (geometry_msgs/Twist)
      linear.x  : forward speed [m/s]
      angular.z : yaw rate     [rad/s]

Publishes:
  /drive_controller/commands  (std_msgs/Float64MultiArray)
      Wheel angular velocities [rad/s] in joint order [wheel1, wheel2, wheel3, wheel4].

  /pivot_controller/commands  (std_msgs/Float64MultiArray)
      Steering angles [rad] in joint order [pivot_right, pivot_left].

Geometry (bicycle/Ackermann with front steering):
    Given linear velocity v and yaw rate w, the instantaneous turn radius is R = v/w.
    The Instantaneous Center of Rotation (ICR) lies on a line through the rear axle,
    at distance |R| from the vehicle centerline (left side if w>0, right if w<0).

    Steering (front axle, distance L from rear axle):
        delta_inner = atan( L / (|R| - track/2) )
        delta_outer = atan( L / (|R| + track/2) )

    Wheel ground speeds (sign matches forward direction):
        v_rear_inner  = w * (|R| - track/2)
        v_rear_outer  = w * (|R| + track/2)
        v_front_inner = w * sqrt( (|R| - track/2)^2 + L^2 )
        v_front_outer = w * sqrt( (|R| + track/2)^2 + L^2 )

    Converted to wheel angular velocity by dividing by wheel_radius.

Wheel index mapping is configurable via the `wheel_order` parameter, which lists
the joint indices [front_left, front_right, rear_left, rear_right] within the
drive_controller's joint list (wheel1..wheel4).
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class AckermannDrive(Node):
    def __init__(self):
        super().__init__("ackermann_drive")

        # ---- Parameters ----------------------------------------------------
        self.declare_parameter("wheelbase", 0.5)          # L  [m]
        self.declare_parameter("track_width", 0.4)        # W  [m]
        self.declare_parameter("wheel_radius", 0.1)       # r  [m]
        self.declare_parameter("max_steer_angle", 0.7854) # rad (~45 deg)
        self.declare_parameter("max_wheel_speed", 7.5)    # rad/s (motor side)

        # Mapping from logical position -> index in [wheel1, wheel2, wheel3, wheel4].
        # Default assumes wheel1=FL, wheel2=FR, wheel3=RL, wheel4=RR.
        self.declare_parameter("wheel_order", [0, 1, 2, 3])  # [FL, FR, RL, RR]

        # Pivot order published on /pivot_controller/commands.
        # Controller config has joints: [pivot_right, pivot_left] -> [0]=right, [1]=left.
        # Override if your YAML lists them in the opposite order.
        self.declare_parameter("pivot_right_index", 0)
        self.declare_parameter("pivot_left_index", 1)

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("drive_cmd_topic", "/drive_controller/commands")
        self.declare_parameter("pivot_cmd_topic", "/pivot_controller/commands")

        # Safety / behavior
        self.declare_parameter("cmd_timeout", 0.5)        # s; zero output if no cmd
        self.declare_parameter("publish_rate", 50.0)      # Hz
        # Invert wheel direction sign per wheel index in the published array
        # (e.g. if right-side motors are mounted mirrored, set [1, -1, 1, -1])
        self.declare_parameter("wheel_direction", [1.0, 1.0, 1.0, 1.0])

        self.L = float(self.get_parameter("wheelbase").value)
        self.W = float(self.get_parameter("track_width").value)
        self.r = float(self.get_parameter("wheel_radius").value)
        self.max_steer = float(self.get_parameter("max_steer_angle").value)
        self.max_wheel_speed = float(self.get_parameter("max_wheel_speed").value)
        self.wheel_order = [int(i) for i in self.get_parameter("wheel_order").value]
        self.pivot_right_index = int(self.get_parameter("pivot_right_index").value)
        self.pivot_left_index = int(self.get_parameter("pivot_left_index").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.wheel_dir = [float(s) for s in self.get_parameter("wheel_direction").value]

        if len(self.wheel_order) != 4 or sorted(self.wheel_order) != [0, 1, 2, 3]:
            raise ValueError("wheel_order must be a permutation of [0,1,2,3]")
        if len(self.wheel_dir) != 4:
            raise ValueError("wheel_direction must have 4 elements")
        if self.r <= 0.0:
            raise ValueError("wheel_radius must be > 0")

        # ---- I/O -----------------------------------------------------------
        self.last_cmd_time = self.get_clock().now()
        self.linear_x = 0.0
        self.angular_z = 0.0

        self.cmd_sub = self.create_subscription(
            Twist,
            self.get_parameter("cmd_vel_topic").value,
            self._cmd_cb,
            10,
        )
        self.drive_pub = self.create_publisher(
            Float64MultiArray,
            self.get_parameter("drive_cmd_topic").value,
            10,
        )
        self.pivot_pub = self.create_publisher(
            Float64MultiArray,
            self.get_parameter("pivot_cmd_topic").value,
            10,
        )

        period = 1.0 / max(self.publish_rate, 1.0)
        self.timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f"Ackermann drive started: L={self.L} m, W={self.W} m, r={self.r} m, "
            f"max_steer={math.degrees(self.max_steer):.1f} deg"
        )

    # -------------------------------------------------------------------- IO

    def _cmd_cb(self, msg: Twist):
        self.linear_x = float(msg.linear.x)
        self.angular_z = float(msg.angular.z)
        self.last_cmd_time = self.get_clock().now()

    def _tick(self):
        # Stale-command watchdog
        now = self.get_clock().now()
        dt = (now - self.last_cmd_time).nanoseconds * 1e-9
        if dt > self.cmd_timeout:
            v, w = 0.0, 0.0
        else:
            v, w = self.linear_x, self.angular_z

        wheels_rad_s, steer_left, steer_right = self._compute(v, w)
        self._publish(wheels_rad_s, steer_left, steer_right)

    # ------------------------------------------------------------- Ackermann

    def _compute(self, v: float, w: float):
        """Return (wheel_velocities[4], steer_left, steer_right).

        wheel_velocities is ordered as the drive_controller joint list
        [wheel1, wheel2, wheel3, wheel4].
        """
        eps = 1e-6
        L, W, r = self.L, self.W, self.r

        if abs(w) < eps:
            # Straight-line motion
            v_fl = v_fr = v_rl = v_rr = v
            steer_left = 0.0
            steer_right = 0.0
        elif abs(v) < eps:
            # Pure rotation requested. True Ackermann cannot rotate in place;
            # we stop wheels and center the steering instead of misbehaving.
            v_fl = v_fr = v_rl = v_rr = 0.0
            steer_left = 0.0
            steer_right = 0.0
        else:
            R_signed = v / w           # turn radius along the lateral axis
            R = abs(R_signed)
            turn_left = (w > 0.0)      # ICR on left side -> inner wheels are left

            # Avoid division blowups for very tight turns: clamp inner radius >= 0
            r_inner = max(R - W / 2.0, 1e-3)
            r_outer = R + W / 2.0

            delta_inner = math.atan2(L, r_inner)
            delta_outer = math.atan2(L, r_outer)

            # Cap to mechanical steering limit (cap inner; outer follows geometry).
            if delta_inner > self.max_steer:
                # Recompute inner radius corresponding to capped inner angle,
                # keep outer consistent with the same |R|.
                delta_inner = self.max_steer
                r_inner_capped = L / math.tan(self.max_steer)
                R = r_inner_capped + W / 2.0
                r_outer = R + W / 2.0
                delta_outer = math.atan2(L, r_outer)

            # Wheel ground speeds (use signed angular rate w to preserve direction)
            v_rear_inner = w * (R - W / 2.0)
            v_rear_outer = w * (R + W / 2.0)
            v_front_inner = w * math.hypot(R - W / 2.0, L)
            v_front_outer = w * math.hypot(R + W / 2.0, L)

            # Steering signs: positive angle = steer left (CCW yaw)
            if turn_left:
                steer_left = +delta_inner
                steer_right = +delta_outer
                v_fl, v_rl = v_front_inner, v_rear_inner
                v_fr, v_rr = v_front_outer, v_rear_outer
            else:
                steer_left = -delta_outer
                steer_right = -delta_inner
                v_fl, v_rl = v_front_outer, v_rear_outer
                v_fr, v_rr = v_front_inner, v_rear_inner

        # Convert linear m/s -> wheel rad/s
        w_fl = v_fl / r
        w_fr = v_fr / r
        w_rl = v_rl / r
        w_rr = v_rr / r

        # Place into joint order [wheel1, wheel2, wheel3, wheel4] using wheel_order
        wheels = [0.0, 0.0, 0.0, 0.0]
        idx_fl, idx_fr, idx_rl, idx_rr = self.wheel_order
        wheels[idx_fl] = w_fl
        wheels[idx_fr] = w_fr
        wheels[idx_rl] = w_rl
        wheels[idx_rr] = w_rr

        # Apply per-wheel direction inversion and saturation
        wheels = [self._sat(self.wheel_dir[i] * wheels[i], self.max_wheel_speed)
                  for i in range(4)]

        return wheels, steer_left, steer_right

    @staticmethod
    def _sat(x: float, lim: float) -> float:
        if x > lim:
            return lim
        if x < -lim:
            return -lim
        return x

    # ----------------------------------------------------------------- output

    def _publish(self, wheels, steer_left, steer_right):
        drive_msg = Float64MultiArray()
        drive_msg.data = [float(x) for x in wheels]
        self.drive_pub.publish(drive_msg)

        pivot_msg = Float64MultiArray()
        pivot_data = [0.0, 0.0]
        pivot_data[self.pivot_right_index] = float(steer_right)
        pivot_data[self.pivot_left_index] = float(steer_left)
        pivot_msg.data = pivot_data
        self.pivot_pub.publish(pivot_msg)


def main(args=None):
    rclpy.init(args=args)
    node = AckermannDrive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
