from pathlib import Path

from greenhouse_nav2_bringup.contracts import BringupContract
from greenhouse_nav2_bringup.health_core import (
    GraphSnapshot,
    PublisherObservation,
    TopicObservation,
    evaluate_health,
)


CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"


def _resolved():
    return BringupContract.load(CONFIG_DIR).resolve(
        "mapping-fastlio", "simulation"
    )


def _healthy_snapshot(resolved):
    def observation(requirement):
        owners = requirement.owner_nodes or ("/fixture_owner",)
        return TopicObservation(
            types=(requirement.topic_type,),
            publishers=tuple(
                PublisherObservation(
                    node_name=owner,
                    reliability=requirement.reliability or "reliable",
                    durability=requirement.durability,
                )
                for owner in owners[: requirement.minimum_publishers]
            ),
        )

    return GraphSnapshot(
        topics={
            requirement.name: observation(requirement)
            for requirement in resolved.required_topics
        },
        services=frozenset(resolved.required_services),
        tf_edges=frozenset(
            (requirement.parent, requirement.child)
            for requirement in resolved.required_tf
        ),
        available_devices=frozenset(resolved.required_devices),
    )


def test_complete_snapshot_is_healthy():
    resolved = _resolved()
    assert evaluate_health(resolved, _healthy_snapshot(resolved)) == ()


def test_health_reports_all_independent_failures():
    resolved = _resolved()
    snapshot = _healthy_snapshot(resolved)
    topics = dict(snapshot.topics)
    topics.pop("/perception/terrain_map")
    topics["/cmd_vel"] = TopicObservation(
        types=("geometry_msgs/msg/Twist",),
        publishers=(
            PublisherObservation("/wrong_owner", "best_effort", "volatile"),
            PublisherObservation("/second_owner", "reliable", "transient_local"),
        ),
    )
    broken = GraphSnapshot(
        topics=topics,
        services=frozenset(),
        tf_edges=frozenset({("map", "odom")}),
        available_devices=frozenset(),
    )

    issues = evaluate_health(resolved, broken)
    codes = {(issue.code, issue.subject) for issue in issues}
    assert ("missing_topic", "/perception/terrain_map") in codes
    assert ("topic_type_mismatch", "/cmd_vel") in codes
    assert ("too_many_publishers", "/cmd_vel") in codes
    assert ("publisher_owner_mismatch", "/cmd_vel") in codes
    assert ("publisher_reliability_mismatch", "/cmd_vel") in codes
    assert ("publisher_durability_mismatch", "/cmd_vel") in codes
    assert ("missing_tf", "odom->base_link") in codes
    assert ("missing_tf", "base_link->livox_frame") in codes


def test_nuc_device_is_a_real_health_gate():
    resolved = BringupContract.load(CONFIG_DIR).resolve(
        "tools", "nuc-amd64"
    )
    missing = GraphSnapshot(
        topics={},
        services=frozenset(),
        tf_edges=frozenset(),
        available_devices=frozenset(),
    )
    issues = evaluate_health(resolved, missing)
    assert [(issue.code, issue.subject) for issue in issues] == [
        ("missing_device", "/dev/dlrobot")
    ]
