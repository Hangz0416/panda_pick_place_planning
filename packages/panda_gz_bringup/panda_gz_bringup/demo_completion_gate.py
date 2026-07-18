"""Exit only after an episode result has been durably saved by the evaluator."""

from __future__ import annotations

import json
from pathlib import Path
import threading
import time

from panda_pick_place_interfaces.msg import EpisodeResult
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, UInt64


class DemoCompletionGate(Node):
    """Separate task outcome, durable result saving, and launch teardown."""

    def __init__(self) -> None:
        super().__init__("panda_demo_completion_gate")
        self.declare_parameter("output_path", "")
        self.declare_parameter("keep_open_after_result", False)
        self._result = None
        self._saved_ids = set()
        self.completed = threading.Event()
        qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(EpisodeResult, "/episode_result", self._on_result, qos)
        self.create_subscription(UInt64, "/episode_result_saved", self._on_saved, qos)
        self.shutdown_publisher = self.create_publisher(
            Bool, "/demo_application_shutdown", 10
        )

    def _on_result(self, message: EpisodeResult) -> None:
        self._result = message
        self._check_complete()

    def _on_saved(self, message: UInt64) -> None:
        self._saved_ids.add(int(message.data))
        self._check_complete()

    def _check_complete(self) -> None:
        if self._result is None or int(self._result.episode_id) not in self._saved_ids:
            return
        output_path = Path(str(self.get_parameter("output_path").value)).expanduser()
        status_path = output_path.parent / "run_status.json"
        keep_open = bool(self.get_parameter("keep_open_after_result").value)
        status = {
            "readiness": "ready",
            "episode_status": "success" if self._result.task_success else "failed",
            "episode_id": int(self._result.episode_id),
            "failure_type": str(self._result.failure_type),
            "result_saved": True,
            "result_path": str(output_path.resolve()),
            "teardown": "kept_open" if keep_open else "requested",
            "updated_unix_sec": time.time(),
        }
        status_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = status_path.with_suffix(".json.tmp")
        temporary.write_text(json.dumps(status, indent=2), encoding="utf-8")
        temporary.replace(status_path)
        self.get_logger().info(
            f"result save confirmed: episode={self._result.episode_id}, path={output_path}"
        )
        if keep_open:
            return
        self.shutdown_publisher.publish(Bool(data=True))
        self.create_timer(1.0, self.completed.set)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DemoCompletionGate()
    try:
        while rclpy.ok() and not node.completed.is_set():
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
