import importlib.util
import json
from pathlib import Path

import pytest
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction

from greenhouse_nav2_bringup.contracts import ContractError


PACKAGE_DIR = Path(__file__).resolve().parents[1]
CONFIG_DIR = PACKAGE_DIR / "config"
LAUNCH_PATH = PACKAGE_DIR / "launch" / "bringup.launch.py"


def _load_launch_module():
    spec = importlib.util.spec_from_file_location(
        "greenhouse_bringup_launch", LAUNCH_PATH
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_launch_description_declares_one_validating_opaque_function():
    module = _load_launch_module()
    module.get_package_share_directory = lambda _name: str(PACKAGE_DIR)
    description = module.generate_launch_description()
    assert isinstance(description, LaunchDescription)
    entities = list(description.entities)
    assert sum(isinstance(entity, OpaqueFunction) for entity in entities) == 1
    assert sum(isinstance(entity, DeclareLaunchArgument) for entity in entities) == 11


def test_launch_description_does_not_resolve_optional_far_package():
    module = _load_launch_module()

    def package_share(name):
        assert name == "greenhouse_nav2_bringup"
        return str(PACKAGE_DIR)

    module.get_package_share_directory = package_share
    module.generate_launch_description()


def _context(**overrides):
    values = {
        "config_dir": str(CONFIG_DIR),
        "profiles": "tools",
        "host_profile": "simulation",
        "start_runtime_nodes": "false",
        "start_health_monitor": "false",
        "publish_static_tf": "false",
        "health_super_client": "false",
        "fast_lio_params": str(CONFIG_DIR / "fast-lio-generic-sim.yaml"),
        "command_gate_params": str(CONFIG_DIR / "command-gate-sim.yaml"),
        "far_planner_params": str(CONFIG_DIR / "far-planner-test.yaml"),
        "plan_output": "",
    }
    values.update(overrides)
    context = LaunchContext()
    context.launch_configurations.update(values)
    return context


def test_launch_setup_emits_a_machine_readable_plan(tmp_path):
    module = _load_launch_module()
    output = tmp_path / "plan.json"
    actions = module._launch_setup(_context(plan_output=str(output)))

    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["profiles"] == ["tools"]
    assert payload["host_profile"] == "simulation"
    assert payload["runtime_ready"] is True
    assert len(actions) == 1


def test_launch_setup_rejects_missing_plan_parent(tmp_path):
    module = _load_launch_module()
    output = tmp_path / "missing" / "plan.json"
    with pytest.raises(ContractError, match="parent directory does not exist"):
        module._launch_setup(_context(plan_output=str(output)))


def test_launch_setup_rejects_illegal_profile_combination():
    module = _load_launch_module()
    with pytest.raises(ContractError, match="conflicts"):
        module._launch_setup(
            _context(profiles="mapping-fastlio,localization-pcd")
        )


def test_far_actions_preserve_runtime_topics_and_parameters(monkeypatch):
    module = _load_launch_module()
    captured = []

    def fake_node(**kwargs):
        captured.append(kwargs)
        return kwargs

    monkeypatch.setattr(module, "Node", fake_node)
    actions = module._far_actions(
        use_sim_time=True,
        far_planner_params="/tmp/far/default.yaml",
    )

    assert actions == captured
    assert [item["package"] for item in captured] == [
        "terrain_analysis_ext",
        "far_planner",
        "graph_decoder",
    ]
    far = captured[1]
    assert far["parameters"][0] == "/tmp/far/default.yaml"
    assert far["parameters"][1]["is_static_env"] is True
    assert ("/way_point", "/planning/way_point") in far["remappings"]
    assert (
        "/navigation_boundary",
        "/planning/navigation_boundary",
    ) in far["remappings"]


@pytest.mark.parametrize("value", ["maybe", "", "truth"])
def test_launch_boolean_values_are_strict(value):
    module = _load_launch_module()
    with pytest.raises(ContractError, match="true or false"):
        module._parse_bool(value, "test")
