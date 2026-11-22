#!/usr/bin/env python3
"""
Bridge ArduPilot motor PWM outputs to Gazebo MulticopterMotorModel.

Main simulation motor chain:
SERVO_OUTPUT_RAW -> /ap/motor_pwm -> gated actuator_msgs/Actuators
-> ros_gz_bridge -> gz.msgs.Actuators -> Gazebo rotor thrust.

The bridge is intentionally gated by /ap/armed so an unarmed SITL instance
cannot spin the Gazebo rotors because of stale or default PWM values.

PX4's x500 rotor0..3 and ArduPilot's Quad-X MotorDef array share the same
front-right, rear-left, front-left, rear-right order.  The final field in each
ArduPilot MotorDef is only the motor-test order, not a SERVO/PWM permutation.
"""

import math
import time

from actuator_msgs.msg import Actuators
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray


class MotorActuatorBridge(Node):
    def __init__(self):
        super().__init__("motor_actuator_bridge")

        self.declare_parameter("motor_topic", "/drone/command/motor_speed")
        self.declare_parameter("pwm_min", 1100.0)
        self.declare_parameter("pwm_max", 1900.0)
        self.declare_parameter("max_rot_velocity", 1000.0)
        self.declare_parameter("motor_order", [0, 1, 2, 3])
        self.declare_parameter("publish_rate", 100.0)
        self.declare_parameter("pwm_mapping", "final_pwm_linear")
        self.declare_parameter("velocity_slew_rate_up", 1200.0)
        self.declare_parameter("velocity_slew_rate_down", 1800.0)
        self.declare_parameter("spin_min", 0.15)
        self.declare_parameter("spin_max", 0.95)
        self.declare_parameter("thrust_expo", 0.65)
        self.declare_parameter("stale_timeout", 0.25)
        self.declare_parameter("require_armed", True)

        self.motor_topic = str(self.get_parameter("motor_topic").value)
        self.pwm_min = float(self.get_parameter("pwm_min").value)
        self.pwm_max = float(self.get_parameter("pwm_max").value)
        self.max_rot_velocity = float(self.get_parameter("max_rot_velocity").value)
        self.motor_order = self._parse_motor_order(self.get_parameter("motor_order").value)
        publish_rate = float(self.get_parameter("publish_rate").value)
        self.pwm_mapping = str(self.get_parameter("pwm_mapping").value)
        self.velocity_slew_rate_up = float(self.get_parameter("velocity_slew_rate_up").value)
        self.velocity_slew_rate_down = float(self.get_parameter("velocity_slew_rate_down").value)
        self.spin_min = float(self.get_parameter("spin_min").value)
        self.spin_max = float(self.get_parameter("spin_max").value)
        self.thrust_expo = float(self.get_parameter("thrust_expo").value)
        self.stale_timeout = float(self.get_parameter("stale_timeout").value)
        self.require_armed = bool(self.get_parameter("require_armed").value)

        self._last_pwm = [0.0, 0.0, 0.0, 0.0]
        self._last_velocities = [0.0, 0.0, 0.0, 0.0]
        self._last_msg_time = 0.0
        self._last_publish_time = time.monotonic()
        self._last_log_time = 0.0
        self._armed = False

        self.create_subscription(Float32MultiArray, "/ap/motor_pwm", self._pwm_cb, 10)
        self.create_subscription(Bool, "/ap/armed", self._armed_cb, 10)
        self.motor_pub = self.create_publisher(Actuators, self.motor_topic, 10)
        self.create_timer(1.0 / max(publish_rate, 1.0), self._publish_actuators)

        self.get_logger().info(
            f"Motor actuator bridge: /ap/motor_pwm -> {self.motor_topic}, "
            f"armed_gate={self.require_armed}, pwm=[{self.pwm_min:.0f},{self.pwm_max:.0f}], "
            f"max={self.max_rot_velocity:.0f}rad/s, mapping={self.pwm_mapping}, "
            f"slew_up={self.velocity_slew_rate_up:.0f}rad/s^2, "
            f"slew_down={self.velocity_slew_rate_down:.0f}rad/s^2, "
            f"spin=[{self.spin_min:.2f},{self.spin_max:.2f}], expo={self.thrust_expo:.2f}, "
            f"order={self.motor_order}, rate={publish_rate:.0f}Hz"
        )

    def _parse_motor_order(self, value):
        try:
            order = [int(v) for v in value]
        except TypeError:
            order = [0, 1, 2, 3]
        if len(order) != 4 or sorted(order) != [0, 1, 2, 3]:
            self.get_logger().warn(
                f"motor_order={order} 无效, 回退为 [0, 1, 2, 3]"
            )
            return [0, 1, 2, 3]
        return order

    def _armed_cb(self, msg: Bool):
        self._armed = bool(msg.data)

    def _pwm_cb(self, msg: Float32MultiArray):
        if len(msg.data) < 4:
            return
        self._last_pwm = [float(v) for v in msg.data[:4]]
        self._last_msg_time = time.monotonic()

    def _pwm_to_velocity(self, pwm: float) -> float:
        if pwm <= self.pwm_min:
            return 0.0
        actuator = (pwm - self.pwm_min) / max(self.pwm_max - self.pwm_min, 1.0)
        actuator = max(0.0, min(1.0, actuator))
        if self.pwm_mapping == "ardupilot_motor_curve":
            thrust = self._actuator_to_thrust(actuator)
        else:
            # SERVO_OUTPUT_RAW is the final ESC PWM after ArduPilot's motor
            # curve has been applied. Gazebo needs rotor speed, so map the
            # final PWM directly to normalized thrust instead of applying
            # MOT_SPIN_MIN/MOT_THST_EXPO a second time.
            thrust = actuator
        return math.sqrt(thrust) * self.max_rot_velocity

    def _slew_limit_velocities(self, target_velocities, now):
        dt = max(0.0, min(now - self._last_publish_time, 0.1))
        self._last_publish_time = now
        if dt <= 0.0:
            return list(self._last_velocities)

        next_velocities = []
        for current, target in zip(self._last_velocities, target_velocities):
            delta = target - current
            if delta >= 0.0:
                max_delta = max(0.0, self.velocity_slew_rate_up) * dt
            else:
                max_delta = max(0.0, self.velocity_slew_rate_down) * dt

            if max_delta <= 0.0:
                next_velocities.append(target)
            elif abs(delta) <= max_delta:
                next_velocities.append(target)
            else:
                next_velocities.append(current + math.copysign(max_delta, delta))

        self._last_velocities = next_velocities
        return next_velocities

    def _actuator_to_thrust(self, actuator: float) -> float:
        span = max(self.spin_max - self.spin_min, 1.0e-6)
        throttle = (actuator - self.spin_min) / span
        throttle = max(0.0, min(1.0, throttle))

        expo = max(-0.95, min(0.95, self.thrust_expo))
        if abs(expo) < 1.0e-6:
            return throttle

        thrust = (throttle * (2.0 * expo)) - (expo - 1.0)
        thrust = (thrust * thrust) - ((1.0 - expo) * (1.0 - expo))
        thrust /= 4.0 * expo
        return max(0.0, min(1.0, thrust))

    def _publish_actuators(self):
        now = time.monotonic()
        pwm_fresh = now - self._last_msg_time <= self.stale_timeout
        enabled = pwm_fresh and (self._armed or not self.require_armed)

        if enabled:
            raw_velocities = [self._pwm_to_velocity(v) for v in self._last_pwm]
            target_velocities = [raw_velocities[i] for i in self.motor_order]
            velocities = self._slew_limit_velocities(target_velocities, now)
        else:
            velocities = [0.0, 0.0, 0.0, 0.0]
            self._last_velocities = velocities
            self._last_publish_time = now

        msg = Actuators()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.velocity = velocities
        self.motor_pub.publish(msg)

        if now - self._last_log_time > 3.0:
            self._last_log_time = now
            self.get_logger().info(
                "armed="
                f"{self._armed} fresh={pwm_fresh} pwm="
                f"{[int(v) for v in self._last_pwm]} velocity="
                f"{[round(v, 1) for v in velocities]}"
            )


def main():
    rclpy.init()
    node = MotorActuatorBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
