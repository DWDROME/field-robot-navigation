from copy import deepcopy
from pathlib import Path

import pytest
import yaml

from greenhouse_nav2_bringup.contracts import (
    BringupContract,
    CONFIG_FILES,
    ContractError,
    REQUIRED_TF_EDGES,
    parse_profile_selection,
)


CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"


def test_mapping_simulation_resolves_complete_contract():
    resolved = BringupContract.load(CONFIG_DIR).resolve(
        "mapping-fastlio", "simulation", require_runtime_ready=True
    )

    assert resolved.runtime_ready
    assert resolved.use_sim_time
    assert resolved.network_mode == "bridge"
    assert resolved.required_devices == ()
    assert "fast_lio" in resolved.components
    assert "command_gate" in resolved.components
    assert {item.name for item in resolved.required_topics} >= {
        "/sensors/lidar/raw",
        "/localization/odometry",
        "/localization/registered_cloud",
        "/perception/terrain_map",
        "/planning/local_path",
        "/cmd_vel",
    }
    assert {(item.parent, item.child) for item in resolved.required_tf} == (
        REQUIRED_TF_EDGES
    )
    lidar = next(
        item for item in resolved.required_topics if item.name == "/sensors/lidar/raw"
    )
    assert lidar.topic_type == "sensor_msgs/msg/PointCloud2"
    assert lidar.maximum_publishers == 1
    assert lidar.owner_nodes == ("/greenhouse_sensor_bridge",)
    assert lidar.reliability == "best_effort"
    assert lidar.durability == "volatile"


def test_mapping_and_tools_are_additive():
    resolved = BringupContract.load(CONFIG_DIR).resolve(
        "mapping-fastlio,tools", "dev-ci", require_runtime_ready=True
    )
    assert resolved.profiles == ("mapping-fastlio", "tools")
    assert resolved.role == "production"
    assert "tools" in resolved.components


def test_tools_only_is_safe_and_has_no_runtime_health_requirements():
    resolved = BringupContract.load(CONFIG_DIR).resolve(
        "tools", "nuc-amd64", require_runtime_ready=True
    )
    assert resolved.role == "tools-only"
    assert resolved.required_topics == ()
    assert resolved.required_tf == ()
    assert resolved.required_devices == ("/dev/dlrobot",)


@pytest.mark.parametrize(
    "profiles,expected",
    [
        (
            "mapping-fastlio,localization-pcd",
            "conflicts",
        ),
        ("navigation-far", "require exactly one"),
        (
            "mapping-fastlio,experiment-fast-lio-super",
            "conflicts",
        ),
        (
            "navigation-far,experiment-movebase3d",
            "conflicts",
        ),
    ],
)
def test_invalid_profile_combinations_are_rejected(profiles, expected):
    with pytest.raises(ContractError, match=expected):
        BringupContract.load(CONFIG_DIR).resolve(profiles, "simulation")


def test_mapping_with_far_is_runtime_ready_in_simulation():
    contract = BringupContract.load(CONFIG_DIR)
    plan = contract.resolve(
        "mapping-fastlio,navigation-far",
        "simulation",
        require_runtime_ready=True,
    )
    assert plan.runtime_ready
    assert plan.runtime_blockers == ()
    assert "far_planner" in plan.components
    way_point = next(
        item for item in plan.required_topics if item.name == "/planning/way_point"
    )
    assert way_point.owner_nodes == ("/far_planner",)


def test_mapping_hardware_runtime_remains_blocked():
    with pytest.raises(ContractError, match="nuc-amd64"):
        BringupContract.load(CONFIG_DIR).resolve(
            "mapping-fastlio",
            "nuc-amd64",
            require_runtime_ready=True,
        )


def test_duplicate_or_unknown_profiles_do_not_fall_back():
    with pytest.raises(ContractError, match="duplicates"):
        parse_profile_selection("mapping-fastlio,mapping-fastlio")
    with pytest.raises(ContractError, match="unknown profiles"):
        BringupContract.load(CONFIG_DIR).resolve("mapping-fastlio-typo", "simulation")


def _copy_config(tmp_path: Path) -> Path:
    for name in CONFIG_FILES:
        (tmp_path / name).write_text(
            (CONFIG_DIR / name).read_text(encoding="utf-8"),
            encoding="utf-8",
        )
    return tmp_path


def test_invalid_final_command_owner_is_rejected(tmp_path):
    config_dir = _copy_config(tmp_path)
    path = config_dir / "interfaces.yaml"
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    document["command_contract"]["single_owner"] = "path_follower"
    path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")

    with pytest.raises(ContractError, match="final /cmd_vel owner"):
        BringupContract.load(config_dir)


def test_missing_tf_edge_is_rejected(tmp_path):
    config_dir = _copy_config(tmp_path)
    path = config_dir / "interfaces.yaml"
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    document["tf"]["mapping-fastlio"].pop()
    path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")

    with pytest.raises(ContractError, match="canonical tree"):
        BringupContract.load(config_dir)


def test_invalid_qos_depth_is_rejected(tmp_path):
    config_dir = _copy_config(tmp_path)
    path = config_dir / "qos.yaml"
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    document["topics"]["/cmd_vel"]["depth"] = 0
    path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")

    with pytest.raises(ContractError, match="positive integer"):
        BringupContract.load(config_dir)


def test_nuc_requires_stable_chassis_device(tmp_path):
    config_dir = _copy_config(tmp_path)
    path = config_dir / "devices.yaml"
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    document["host_profiles"]["nuc-amd64"]["required_devices"] = []
    path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")

    with pytest.raises(ContractError, match="/dev/dlrobot"):
        BringupContract.load(config_dir)
