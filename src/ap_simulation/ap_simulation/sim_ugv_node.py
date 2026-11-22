#!/usr/bin/env python3
"""
UGV 多点导航仿真节点

功能:
  1. 发布 /mission/ugv_progress (0~1) 驱动 Gazebo 中 UGV 模型移动
  2. 发布 /mission/ugv_state (String) UGV 状态
  3. 发布 /mission/ugv_in_strike_zone (Bool) 兼容旧话题名, 表示UGV和气球已到最后门洞后扎球位置

路径: 左侧打击区 → 北侧外车道 → 东侧外车道 → 南侧外车道
      → 最后门洞后停车; GUI顶视图为顺时针，不继续绕完整一圈。
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float32
import time

from .competition_paths import (
    UGV_GATE_LABELS,
    UGV_GATE_PROGRESS,
    UGV_PATH_GAZEBO,
    polyline_length,
)


class SimUgvNode(Node):
    def __init__(self):
        super().__init__("sim_ugv_node")

        self.declare_parameter("ugv_speed", 0.3)      # m/s
        self.declare_parameter("start_delay", 0.0)     # 比赛开始后延迟启动(秒)
        self.declare_parameter("start_topic", "/mission/ugv_start")
        self.ugv_speed = float(self.get_parameter("ugv_speed").value)
        self.start_delay = float(self.get_parameter("start_delay").value)
        self.start_topic = str(self.get_parameter("start_topic").value)

        self.progress_pub = self.create_publisher(Float32, "/mission/ugv_progress", 10)
        self.state_pub = self.create_publisher(String, "/mission/ugv_state", 10)
        self.strike_pub = self.create_publisher(Bool, "/mission/ugv_in_strike_zone", 10)
        self.applied_progress = 0.
        self.last_applied_time = 0.
        self.create_subscription(Float32, '/simulation/ugv_pose_applied', self._applied_cb, 1)

        # 等待比赛开始信号。UAV与UGV并行动作, 谁先到最后门洞扎球位置谁等待。
        self.started = False
        self.start_sub = self.create_subscription(
            Bool, self.start_topic, self._start_cb, 10)

        # 模拟状态
        self._phase = 0   # 0=等待 1=行驶中 2=最后门洞扎球位置 3=完成
        self._start_time = None
        self._strike_ready = False
        self._gate_reported = set()
        self._last_state = ""

        self._path_total = polyline_length(UGV_PATH_GAZEBO)
        self._gate_thresholds = tuple(
            (float(progress), gate_id)
            for gate_id, progress in enumerate(UGV_GATE_PROGRESS, start=1)
        )
        self._tick_timer = self.create_timer(1. / 30., self._tick)

        self.get_logger().info(f"UGV仿真就绪: speed={self.ugv_speed}m/s, "
                               f"total_path={self._path_total:.1f}m, "
                               f"start_topic={self.start_topic}")

    def _reset(self):
        self.started = False
        self._phase = 0
        self._start_time = None
        self._strike_ready = False
        self.applied_progress = 0.
        self.last_applied_time = 0.
        self._gate_reported.clear()
        self.progress_pub.publish(Float32(data=0.0))
        self._publish_state("WAIT_START", force=True)
        self.strike_pub.publish(Bool(data=False))

    def _start_cb(self, msg: Bool):
        if not msg.data:
            if self.started or self._phase != 0 or self._strike_ready:
                self.get_logger().info("UGV收到复位信号, 回到左侧打击区等待比赛开始")
            self._reset()
            return

        if self.started:
            return

        self.started = True
        self._start_time = time.time()
        self._phase = 1
        self._strike_ready = False
        self._gate_reported.clear()
        self.progress_pub.publish(Float32(data=0.0))
        self._publish_state("MOVING")
        self.strike_pub.publish(Bool(data=False))
        self.get_logger().info(
            "UGV出发! 路径: 左侧打击区→北侧门1→东侧门2"
            "→南侧门3→最后门洞后扎球位置"
        )

    def _publish_state(self, state: str, force: bool = False):
        if force or state != self._last_state:
            self._last_state = state
            self.state_pub.publish(String(data=state))

    def _applied_cb(self, message):
        self.applied_progress = float(message.data)
        self.last_applied_time = time.monotonic()

    def _tick(self):
        if self._phase == 0:
            self.progress_pub.publish(Float32(data=0.0))
            self._publish_state("WAIT_START")
            self.strike_pub.publish(Bool(data=False))
            return  # 未开始

        elapsed = time.time() - (self._start_time or time.time()) - self.start_delay
        if elapsed < 0:
            self.progress_pub.publish(Float32(data=0.0))
            self._publish_state("WAIT_START_DELAY")
            self.strike_pub.publish(Bool(data=False))
            return  # 延迟启动

        # 计算进度
        distance = elapsed * self.ugv_speed
        progress = min(distance / self._path_total, 1.0)
        self.progress_pub.publish(Float32(data=float(progress)))
        self.strike_pub.publish(Bool(data=self._strike_ready))

        if self._phase == 1:
            self._publish_state("MOVING")
            for threshold, gate in self._gate_thresholds:
                if progress >= threshold and gate not in self._gate_reported:
                    self._gate_reported.add(gate)
                    label = UGV_GATE_LABELS[gate - 1] if gate <= len(UGV_GATE_LABELS) else f"GATE{gate}"
                    self._publish_state(f"GATE{gate}", force=True)
                    self.get_logger().info(f"UGV路径进度经过{label}")

        pose_applied = self.applied_progress >= .999 and time.monotonic() - self.last_applied_time < 2.
        if progress >= 1.0 and not self._strike_ready and pose_applied:
            self._phase = 2
            self._strike_ready = True
            self._publish_state("FINAL_GATE_STOP", force=True)
            self.strike_pub.publish(Bool(data=True))
            self.get_logger().info("UGV和气球已停在最后门洞后扎球位置, 允许UAV等待门禁通过")

        if progress >= 1.0 and self._phase == 2:
            self._phase = 3
            self._publish_state("STOPPED")
            self.strike_pub.publish(Bool(data=True))


def main():
    rclpy.init()
    node = SimUgvNode()
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
