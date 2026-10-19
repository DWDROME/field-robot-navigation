"""Pure health-contract evaluation used by the live ROS node and unit tests."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping

from .contracts import ResolvedBringup


@dataclass(frozen=True)
class PublisherObservation:
    node_name: str
    reliability: str
    durability: str


@dataclass(frozen=True)
class TopicObservation:
    types: tuple[str, ...]
    publishers: tuple[PublisherObservation, ...]

    @property
    def publisher_count(self) -> int:
        return len(self.publishers)


@dataclass(frozen=True)
class GraphSnapshot:
    topics: Mapping[str, TopicObservation]
    services: frozenset[str]
    tf_edges: frozenset[tuple[str, str]]
    available_devices: frozenset[str]


@dataclass(frozen=True)
class HealthIssue:
    code: str
    subject: str
    message: str


def evaluate_health(
    resolved: ResolvedBringup, snapshot: GraphSnapshot
) -> tuple[HealthIssue, ...]:
    """Return every failed health assertion without masking later failures."""

    issues: list[HealthIssue] = []
    for requirement in resolved.required_topics:
        observation = snapshot.topics.get(requirement.name)
        if observation is None:
            issues.append(
                HealthIssue(
                    "missing_topic",
                    requirement.name,
                    "required topic is absent from the ROS graph",
                )
            )
            continue
        if requirement.topic_type not in observation.types:
            issues.append(
                HealthIssue(
                    "topic_type_mismatch",
                    requirement.name,
                    f"expected {requirement.topic_type}, observed {observation.types}",
                )
            )
        if observation.publisher_count < requirement.minimum_publishers:
            issues.append(
                HealthIssue(
                    "too_few_publishers",
                    requirement.name,
                    "expected at least "
                    f"{requirement.minimum_publishers}, observed "
                    f"{observation.publisher_count}",
                )
            )
        if (
            requirement.maximum_publishers is not None
            and observation.publisher_count > requirement.maximum_publishers
        ):
            issues.append(
                HealthIssue(
                    "too_many_publishers",
                    requirement.name,
                    "expected at most "
                    f"{requirement.maximum_publishers}, observed "
                    f"{observation.publisher_count}",
                )
            )
        if requirement.owner_nodes:
            observed_owners = {publisher.node_name for publisher in observation.publishers}
            if observed_owners != set(requirement.owner_nodes):
                issues.append(
                    HealthIssue(
                        "publisher_owner_mismatch",
                        requirement.name,
                        f"expected {requirement.owner_nodes}, observed "
                        f"{tuple(sorted(observed_owners))}",
                    )
                )
        for publisher in observation.publishers:
            if (
                requirement.reliability is not None
                and publisher.reliability != requirement.reliability
            ):
                issues.append(
                    HealthIssue(
                        "publisher_reliability_mismatch",
                        requirement.name,
                        f"{publisher.node_name} expected {requirement.reliability}, "
                        f"observed {publisher.reliability}",
                    )
                )
            if publisher.durability != requirement.durability:
                issues.append(
                    HealthIssue(
                        "publisher_durability_mismatch",
                        requirement.name,
                        f"{publisher.node_name} expected {requirement.durability}, "
                        f"observed {publisher.durability}",
                    )
                )

    for service in resolved.required_services:
        if service not in snapshot.services:
            issues.append(
                HealthIssue(
                    "missing_service",
                    service,
                    "required lifecycle or health service is absent",
                )
            )

    for transform in resolved.required_tf:
        edge = (transform.parent, transform.child)
        if edge not in snapshot.tf_edges:
            issues.append(
                HealthIssue(
                    "missing_tf",
                    f"{transform.parent}->{transform.child}",
                    f"required TF owned by {transform.owner} has not been observed",
                )
            )

    for device in resolved.required_devices:
        if device not in snapshot.available_devices:
            issues.append(
                HealthIssue(
                    "missing_device",
                    device,
                    "required host device is not available",
                )
            )
    return tuple(issues)
