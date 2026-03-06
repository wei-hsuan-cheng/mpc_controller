#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import WrenchStamped

# -----------------------------------------------------------------------------
# Hardcoded config (edit here)
# -----------------------------------------------------------------------------
# Publishes a fake end-effector wrench for ZMP testing.
#
# Units:
# - Force:  Newton (N)
# - Torque: Newton-meter (N*m)
#
# Modes:
# - "static":  publish constant wrench from STATIC_WRENCH
# - "varying": publish independent sine waves for each wrench component

TOPIC = "/admittance_controller/calibrated_wrench"
FRAME_ID = "ur_arm_tool0"
RATE_HZ = 100.0

MODE = "static"  # "static" or "varying"

# Static wrench [Fx Fy Fz Tx Ty Tz]
STATIC_WRENCH = (0.0, 0.0, 500.0, 0.0, 0.0, 0.0)

# Varying wrench DOFs: AMP, FREQ, PHASE, BIAS
FX_AMP, FX_FREQ, FX_PHASE, FX_BIAS = 0.0, 0.20, 0.0, 0.0
FY_AMP, FY_FREQ, FY_PHASE, FY_BIAS = 100.0, 0.5, 0.0, 0.0
FZ_AMP, FZ_FREQ, FZ_PHASE, FZ_BIAS = 0.0, 0.5, 0.0, 0.0
TX_AMP, TX_FREQ, TX_PHASE, TX_BIAS = 0.8, 0.20, 0.0, 0.0
TY_AMP, TY_FREQ, TY_PHASE, TY_BIAS = 0.0, 0.15, 0.0, 0.0
TZ_AMP, TZ_FREQ, TZ_PHASE, TZ_BIAS = 0.0, 0.10, 0.0, 0.0


def _sine(t: float, amp: float, freq: float, phase: float, bias: float) -> float:
    return bias + amp * math.sin(2.0 * math.pi * freq * t + phase)


class PubFakeWrench(Node):
    def __init__(self):
        super().__init__("pub_fake_wrench")
        self._pub = self.create_publisher(WrenchStamped, TOPIC, 10)
        self._t0 = self.get_clock().now()

        period = 1.0 / max(1e-3, float(RATE_HZ))
        self.create_timer(period, self._on_timer)
        self.get_logger().info(
            f"Publishing fake wrench: mode={MODE} topic={TOPIC} frame_id={FRAME_ID} rate={RATE_HZ:.1f}Hz"
        )

    def _wrench_values(self, t: float):
        if MODE == "static":
            return STATIC_WRENCH
        if MODE == "varying":
            return (
                _sine(t, FX_AMP, FX_FREQ, FX_PHASE, FX_BIAS),
                _sine(t, FY_AMP, FY_FREQ, FY_PHASE, FY_BIAS),
                _sine(t, FZ_AMP, FZ_FREQ, FZ_PHASE, FZ_BIAS),
                _sine(t, TX_AMP, TX_FREQ, TX_PHASE, TX_BIAS),
                _sine(t, TY_AMP, TY_FREQ, TY_PHASE, TY_BIAS),
                _sine(t, TZ_AMP, TZ_FREQ, TZ_PHASE, TZ_BIAS),
            )
        self.get_logger().warn(f"Unknown MODE='{MODE}', fallback to static.")
        return STATIC_WRENCH

    def _on_timer(self):
        now = self.get_clock().now()
        t = (now - self._t0).nanoseconds * 1e-9

        fx, fy, fz, tx, ty, tz = self._wrench_values(t)

        msg = WrenchStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = FRAME_ID
        msg.wrench.force.x = float(fx)
        msg.wrench.force.y = float(fy)
        msg.wrench.force.z = float(fz)
        msg.wrench.torque.x = float(tx)
        msg.wrench.torque.y = float(ty)
        msg.wrench.torque.z = float(tz)
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = PubFakeWrench()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
