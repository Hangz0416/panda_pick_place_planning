"""Publish a deterministic batch one task at a time after matching results."""

from __future__ import annotations

import json
from pathlib import Path
import time

from panda_pick_place_interfaces.msg import EpisodeResult as EpisodeResultMsg
from panda_pick_place_interfaces.msg import ManipulationTask
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from panda_pick_place_planning.core.batch import BatchSequencer, batch_exit_code
from panda_pick_place_planning.core.benchmark import summarize_results
from panda_pick_place_planning.core.tasks import generate_backend_tasks
from panda_pick_place_planning.nodes.ros_conversions import (
    episode_from_ros,
    task_to_ros,
)


class BatchTaskGeneratorNode(Node):
    def __init__(self) -> None:
        super().__init__("batch_task_generator")
        self.declare_parameter("episodes_per_scene", 5)
        self.declare_parameter("seed", 42)
        self.declare_parameter("scenes_csv", "clear,obstacle")
        self.declare_parameter("planners_csv", "rrt_connect")
        self.declare_parameter("backend", "moveit")
        self.declare_parameter("simulator", "moveit_mock")
        self.declare_parameter("grasp_mode", "assisted")
        self.declare_parameter("object_variant", "cube")
        self.declare_parameter("workpiece_count", 1)
        self.declare_parameter("perception_profile", "nominal")
        self.declare_parameter("startup_delay_sec", 2.0)
        self.declare_parameter("result_timeout_sec", 120.0)
        self.declare_parameter(
            "summary_path", "results/moveit_batch_summary.json"
        )

        tasks = generate_backend_tasks(
            int(self.get_parameter("episodes_per_scene").value),
            int(self.get_parameter("seed").value),
            self._csv("scenes_csv"),
            self._csv("planners_csv"),
            backend=str(self.get_parameter("backend").value),
            simulator=str(self.get_parameter("simulator").value),
            grasp_mode=str(self.get_parameter("grasp_mode").value),
            object_variant=str(self.get_parameter("object_variant").value),
            perception_profile=str(
                self.get_parameter("perception_profile").value
            ),
            workpiece_count=int(self.get_parameter("workpiece_count").value),
        )
        self.sequencer = BatchSequencer(tasks)
        self.summary_path = Path(
            str(self.get_parameter("summary_path").value)
        ).expanduser()
        self.start_after = time.monotonic() + float(
            self.get_parameter("startup_delay_sec").value
        )
        self.result_deadline: float | None = None
        self.timed_out = False
        self.completed = False

        task_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        # A volatile subscription receives new results from the transient-local
        # publisher but not the previous run's retained final result.
        result_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.task_publisher = self.create_publisher(
            ManipulationTask, "/manipulation_task", task_qos
        )
        self.result_subscription = self.create_subscription(
            EpisodeResultMsg,
            "/episode_result",
            self.on_result,
            result_qos,
        )
        self.complete_publisher = self.create_publisher(
            Bool,
            "/benchmark_complete",
            task_qos,
        )
        self.timer = self.create_timer(0.2, self.tick)

    def _csv(self, parameter_name: str) -> list[str]:
        return [
            item.strip()
            for item in str(self.get_parameter(parameter_name).value).split(",")
            if item.strip()
        ]

    def tick(self) -> None:
        if self.completed or self.timed_out:
            return
        now = time.monotonic()
        if self.result_deadline is not None and now > self.result_deadline:
            expected = self.sequencer.inflight
            self.timed_out = True
            self.get_logger().error(
                "episode result timeout: "
                f"episode_id={getattr(expected, 'episode_id', '-')}, "
                f"received={len(self.sequencer.results)}/{len(self.sequencer.tasks)}"
            )
            self._write_summary("timeout")
            self.complete_publisher.publish(Bool(data=False))
            return
        if now < self.start_after or self.sequencer.inflight is not None:
            return
        if self.task_publisher.get_subscription_count() == 0:
            return
        task = self.sequencer.next_task()
        if task is None:
            self._complete()
            return
        message = task_to_ros(task)
        message.header.stamp = self.get_clock().now().to_msg()
        self.task_publisher.publish(message)
        self.result_deadline = now + float(
            self.get_parameter("result_timeout_sec").value
        )
        self.get_logger().info(
            f"published {task.episode_id + 1}/{len(self.sequencer.tasks)}: "
            f"scene={task.scene}, seed={task.task_seed}"
        )

    def on_result(self, message: EpisodeResultMsg) -> None:
        if self.completed or self.timed_out or self.sequencer.inflight is None:
            return
        try:
            result = episode_from_ros(message)
        except (IndexError, KeyError, TypeError, ValueError) as exc:
            self.get_logger().error(f"invalid episode result: {exc}")
            return
        if not self.sequencer.accept(result):
            self.get_logger().warning(
                f"ignored unmatched result: episode_id={result.episode_id}, "
                f"seed={result.task_seed}"
            )
            return
        self.result_deadline = None
        if self.sequencer.done:
            self._complete()

    def _complete(self) -> None:
        self.completed = True
        self._write_summary("complete")
        self.complete_publisher.publish(Bool(data=True))
        planning_successes = sum(
            result.planning_success for result in self.sequencer.results
        )
        task_successes = sum(
            result.task_success for result in self.sequencer.results
        )
        self.get_logger().info(
            f"batch complete: planning={planning_successes}/"
            f"{len(self.sequencer.results)}, task={task_successes}/"
            f"{len(self.sequencer.results)}; "
            f"summary={self.summary_path.resolve()}"
        )

    def _write_summary(self, status: str) -> None:
        summary = summarize_results(self.sequencer.results)
        summary["batch_status"] = status
        summary["expected_episodes"] = len(self.sequencer.tasks)
        summary["received_episodes"] = len(self.sequencer.results)
        planning_only = bool(
            self.sequencer.tasks
            and all(
                task.simulator == "moveit_mock"
                for task in self.sequencer.tasks
            )
        )
        summary["evaluation_mode"] = (
            "independent_planning_only" if planning_only else "full_episode"
        )
        summary["planning_regression_successes"] = sum(
            result.planning_success for result in self.sequencer.results
        )
        self.summary_path.parent.mkdir(parents=True, exist_ok=True)
        self.summary_path.write_text(
            json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )


def main(args=None) -> int:
    rclpy.init(args=args)
    node = BatchTaskGeneratorNode()
    exit_code = 0
    try:
        while rclpy.ok() and not (node.completed or node.timed_out):
            rclpy.spin_once(node, timeout_sec=0.2)
        if node.timed_out:
            exit_code = batch_exit_code("timeout")
        elif node.completed:
            exit_code = batch_exit_code("complete")
        # Give the terminal transient-local sample a brief executor cycle
        # before destroying its publisher.
        if rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        exit_code = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    main()
