"""Live ROS graph and TF health monitor for a resolved bringup profile."""

from __future__ import annotations

from pathlib import Path
import signal
import sys
import time
from typing import Iterable

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import try_shutdown
from tf2_msgs.msg import TFMessage

from .contracts import BringupContract, ContractError
from .health_core import (
    GraphSnapshot,
    PublisherObservation,
    TopicObservation,
    evaluate_health,
)


def _tf_key(parent: str, child: str) -> tuple[str, str]:
    return (parent.lstrip("/"), child.lstrip("/"))


class BringupHealthMonitor(Node):
    """Publishes one bounded diagnostic summary for the selected profile."""

    def __init__(self) -> None:
        super().__init__("greenhouse_bringup_health")
        self.declare_parameter("config_dir", "")
        self.declare_parameter("profiles", "mapping-fastlio")
        self.declare_parameter("host_profile", "simulation")
        self.declare_parameter("check_period_sec", 1.0)
        self.declare_parameter("dynamic_tf_timeout_sec", 2.0)

        config_dir = self.get_parameter("config_dir").value
        profiles = self.get_parameter("profiles").value
        host_profile = self.get_parameter("host_profile").value
        period = float(self.get_parameter("check_period_sec").value)
        self._dynamic_tf_timeout_sec = float(
            self.get_parameter("dynamic_tf_timeout_sec").value
        )
        if not config_dir:
            raise ContractError("config_dir parameter must be non-empty")
        if period <= 0.0:
            raise ContractError("check_period_sec must be positive")
        if self._dynamic_tf_timeout_sec <= 0.0:
            raise ContractError("dynamic_tf_timeout_sec must be positive")

        self._resolved = BringupContract.load(config_dir).resolve(
            profiles, host_profile
        )
        self._dynamic_tf_last_seen: dict[tuple[str, str], float] = {}
        self._static_tf_edges: set[tuple[str, str]] = set()
        self._publisher = self.create_publisher(
            DiagnosticArray, "/system/bringup_health", 10
        )
        tf_qos = QoSProfile(depth=100)
        tf_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        tf_qos.durability = DurabilityPolicy.VOLATILE
        tf_static_qos = QoSProfile(depth=100)
        tf_static_qos.reliability = ReliabilityPolicy.RELIABLE
        tf_static_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._tf_subscription = self.create_subscription(
            TFMessage, "/tf", self._observe_dynamic_tf, tf_qos
        )
        self._tf_static_subscription = self.create_subscription(
            TFMessage, "/tf_static", self._observe_static_tf, tf_static_qos
        )
        self._timer = self.create_timer(period, self._publish_health)
        self.get_logger().info(
            "health monitor profiles=%s host=%s runtime_ready=%s"
            % (
                ",".join(self._resolved.profiles),
                self._resolved.host_profile,
                self._resolved.runtime_ready,
            )
        )

    def _observe_dynamic_tf(self, message: TFMessage) -> None:
        observed_at = time.monotonic()
        for transform in message.transforms:
            edge = _tf_key(transform.header.frame_id, transform.child_frame_id)
            self._dynamic_tf_last_seen[edge] = observed_at

    def _observe_static_tf(self, message: TFMessage) -> None:
        for transform in message.transforms:
            self._static_tf_edges.add(
                _tf_key(transform.header.frame_id, transform.child_frame_id)
            )

    @staticmethod
    def _node_name(namespace: str, name: str) -> str:
        prefix = namespace.rstrip("/")
        return f"{prefix}/{name}" if prefix else f"/{name}"

    @staticmethod
    def _policy_name(value) -> str:
        name = getattr(value, "name", None)
        if not isinstance(name, str):
            return "unknown"
        return name.lower()

    def _topic_snapshot(self) -> dict[str, TopicObservation]:
        graph_types = {
            name: tuple(types) for name, types in self.get_topic_names_and_types()
        }
        result: dict[str, TopicObservation] = {}
        for requirement in self._resolved.required_topics:
            types = graph_types.get(requirement.name)
            if types is None:
                continue
            publishers = self.get_publishers_info_by_topic(requirement.name)
            result[requirement.name] = TopicObservation(
                types=types,
                publishers=tuple(
                    PublisherObservation(
                        node_name=self._node_name(
                            publisher.node_namespace, publisher.node_name
                        ),
                        reliability=self._policy_name(
                            publisher.qos_profile.reliability
                        ),
                        durability=self._policy_name(
                            publisher.qos_profile.durability
                        ),
                    )
                    for publisher in publishers
                ),
            )
        return result

    def _tf_snapshot(self) -> frozenset[tuple[str, str]]:
        now = time.monotonic()
        dynamic = {
            edge
            for edge, observed_at in self._dynamic_tf_last_seen.items()
            if now - observed_at <= self._dynamic_tf_timeout_sec
        }
        if self.get_publishers_info_by_topic("/tf_static"):
            static = self._static_tf_edges
        else:
            static = set()
        return frozenset(dynamic | static)

    def _service_snapshot(self) -> frozenset[str]:
        return frozenset(name for name, _ in self.get_service_names_and_types())

    def _device_snapshot(self) -> frozenset[str]:
        return frozenset(
            device
            for device in self._resolved.required_devices
            if Path(device).exists()
        )

    def _publish_health(self) -> None:
        snapshot = GraphSnapshot(
            topics=self._topic_snapshot(),
            services=self._service_snapshot(),
            tf_edges=self._tf_snapshot(),
            available_devices=self._device_snapshot(),
        )
        issues = evaluate_health(self._resolved, snapshot)
        status = DiagnosticStatus()
        status.name = "greenhouse_nav2_bringup/profile"
        status.hardware_id = self._resolved.host_profile
        status.level = (
            DiagnosticStatus.OK if not issues else DiagnosticStatus.ERROR
        )
        status.message = "healthy" if not issues else f"{len(issues)} contract failures"
        status.values = [
            KeyValue(key="profiles", value=",".join(self._resolved.profiles)),
            KeyValue(
                key="runtime_ready",
                value=str(self._resolved.runtime_ready).lower(),
            ),
        ]
        status.values.extend(
            KeyValue(
                key=f"{issue.code}:{issue.subject}",
                value=issue.message,
            )
            for issue in issues
        )
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self._publisher.publish(message)


def main(args: Iterable[str] | None = None) -> int:
    rclpy.init(
        args=list(args) if args is not None else None,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    node: BringupHealthMonitor | None = None
    stop_requested = False

    def request_stop(_signum, _frame) -> None:
        nonlocal stop_requested
        stop_requested = True

    try:
        node = BringupHealthMonitor()
        signal.signal(signal.SIGINT, request_stop)
        signal.signal(signal.SIGTERM, request_stop)
        while not stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except (ContractError, ValueError) as exc:
        print(f"bringup health monitor startup failed: {exc}", file=sys.stderr)
        return 2
    finally:
        if node is not None:
            node.destroy_node()
        try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
