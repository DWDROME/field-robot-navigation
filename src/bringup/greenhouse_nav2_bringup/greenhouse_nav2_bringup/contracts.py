"""Strict, ROS-independent bringup configuration validation."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any, Iterable, Mapping

import yaml


CONFIG_FILES = (
    "profiles.yaml",
    "interfaces.yaml",
    "qos.yaml",
    "devices.yaml",
    "health.yaml",
)
REQUIRED_PROFILES = {
    "mapping-fastlio",
    "localization-pcd",
    "navigation-far",
    "experiment-fast-lio-super",
    "experiment-movebase3d",
    "tools",
}
REQUIRED_HOST_PROFILES = {"simulation", "dev-ci", "nuc-amd64"}
REQUIRED_TF_EDGES = {
    ("map", "odom"),
    ("odom", "base_link"),
    ("base_link", "livox_frame"),
}
ALLOWED_PROFILE_ROLES = {
    "production-primary",
    "production-addon",
    "experiment",
    "tools",
}
ALLOWED_RELIABILITY = {"best_effort", "reliable"}
ALLOWED_DURABILITY = {"volatile", "transient_local"}
ALLOWED_HISTORY = {"keep_last", "keep_all"}


class ContractError(ValueError):
    """Raised when a versioned bringup contract is invalid."""


def _mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ContractError(f"{label} must be a mapping")
    return value


def _string(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ContractError(f"{label} must be a non-empty string")
    return value.strip()


def _string_list(value: Any, label: str) -> tuple[str, ...]:
    if not isinstance(value, list):
        raise ContractError(f"{label} must be a list")
    result = tuple(_string(item, f"{label} item") for item in value)
    if len(set(result)) != len(result):
        raise ContractError(f"{label} contains duplicates")
    return result


def _absolute_topic(value: Any, label: str) -> str:
    topic = _string(value, label)
    if not topic.startswith("/") or topic == "/":
        raise ContractError(f"{label} must be an absolute ROS topic")
    return topic


def _load_yaml(path: Path) -> Mapping[str, Any]:
    if not path.is_file():
        raise ContractError(f"required configuration is not a file: {path}")
    try:
        parsed = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise ContractError(f"cannot parse {path}: {exc}") from exc
    root = _mapping(parsed, str(path))
    if root.get("schema_version") != 1:
        raise ContractError(f"{path} schema_version must be 1")
    return root


def parse_profile_selection(value: str | Iterable[str]) -> tuple[str, ...]:
    """Parse a comma-separated or iterable profile selection without fallback."""

    raw = value.split(",") if isinstance(value, str) else list(value)
    profiles = tuple(_string(item, "profile") for item in raw)
    if not profiles:
        raise ContractError("at least one profile is required")
    if len(set(profiles)) != len(profiles):
        raise ContractError("profile selection contains duplicates")
    return profiles


@dataclass(frozen=True)
class TopicRequirement:
    name: str
    topic_type: str
    minimum_publishers: int
    maximum_publishers: int | None
    owner_nodes: tuple[str, ...]
    reliability: str | None
    durability: str

    def as_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "type": self.topic_type,
            "minimum_publishers": self.minimum_publishers,
            "maximum_publishers": self.maximum_publishers,
            "owner_nodes": list(self.owner_nodes),
            "reliability": self.reliability,
            "durability": self.durability,
        }


@dataclass(frozen=True)
class TfRequirement:
    parent: str
    child: str
    owner: str

    def as_dict(self) -> dict[str, str]:
        return {"parent": self.parent, "child": self.child, "owner": self.owner}


@dataclass(frozen=True)
class ResolvedBringup:
    profiles: tuple[str, ...]
    host_profile: str
    role: str
    components: tuple[str, ...]
    runtime_ready: bool
    runtime_blockers: tuple[str, ...]
    use_sim_time: bool
    network_mode: str
    required_devices: tuple[str, ...]
    required_topics: tuple[TopicRequirement, ...]
    required_services: tuple[str, ...]
    required_tf: tuple[TfRequirement, ...]

    def as_dict(self) -> dict[str, Any]:
        return {
            "schema_version": 1,
            "profiles": list(self.profiles),
            "host_profile": self.host_profile,
            "role": self.role,
            "components": list(self.components),
            "runtime_ready": self.runtime_ready,
            "runtime_blockers": list(self.runtime_blockers),
            "use_sim_time": self.use_sim_time,
            "network_mode": self.network_mode,
            "required_devices": list(self.required_devices),
            "required_topics": [item.as_dict() for item in self.required_topics],
            "required_services": list(self.required_services),
            "required_tf": [item.as_dict() for item in self.required_tf],
        }

    def to_json(self) -> str:
        return json.dumps(self.as_dict(), indent=2, sort_keys=True)


class BringupContract:
    """Validated collection of all versioned bringup configuration files."""

    def __init__(self, config_dir: Path, documents: Mapping[str, Mapping[str, Any]]):
        self.config_dir = config_dir
        self.profiles_document = documents["profiles.yaml"]
        self.interfaces_document = documents["interfaces.yaml"]
        self.qos_document = documents["qos.yaml"]
        self.devices_document = documents["devices.yaml"]
        self.health_document = documents["health.yaml"]
        self._validate()

    @classmethod
    def load(cls, config_dir: str | Path) -> "BringupContract":
        resolved = Path(config_dir).expanduser().resolve()
        if not resolved.is_dir():
            raise ContractError(f"configuration directory does not exist: {resolved}")
        documents = {name: _load_yaml(resolved / name) for name in CONFIG_FILES}
        return cls(resolved, documents)

    @property
    def profiles(self) -> Mapping[str, Any]:
        return _mapping(self.profiles_document.get("profiles"), "profiles")

    @property
    def components(self) -> Mapping[str, Any]:
        return _mapping(self.profiles_document.get("components"), "components")

    @property
    def host_profiles(self) -> Mapping[str, Any]:
        return _mapping(self.devices_document.get("host_profiles"), "host_profiles")

    def _validate(self) -> None:
        profiles = self.profiles
        components = self.components
        if set(profiles) != REQUIRED_PROFILES:
            raise ContractError(
                "profiles.yaml must define exactly: "
                + ", ".join(sorted(REQUIRED_PROFILES))
            )
        if not components:
            raise ContractError("profiles.yaml components must not be empty")

        for name, raw_definition in profiles.items():
            definition = _mapping(raw_definition, f"profile {name}")
            role = _string(definition.get("role"), f"profile {name} role")
            if role not in ALLOWED_PROFILE_ROLES:
                raise ContractError(f"profile {name} has unsupported role {role}")
            selected_components = _string_list(
                definition.get("components"), f"profile {name} components"
            )
            unknown = set(selected_components) - set(components)
            if unknown:
                raise ContractError(
                    f"profile {name} references unknown components: {sorted(unknown)}"
                )
            conflicts = _string_list(
                definition.get("conflicts", []), f"profile {name} conflicts"
            )
            unknown_conflicts = set(conflicts) - set(profiles)
            if unknown_conflicts:
                raise ContractError(
                    f"profile {name} has unknown conflicts: {sorted(unknown_conflicts)}"
                )
            for conflict in conflicts:
                reciprocal = _string_list(
                    _mapping(profiles[conflict], f"profile {conflict}").get(
                        "conflicts", []
                    ),
                    f"profile {conflict} conflicts",
                )
                if name not in reciprocal:
                    raise ContractError(
                        f"profile conflict must be symmetric: {name} -> {conflict}"
                    )
            runtime_hosts = set(
                _string_list(
                    definition.get("runtime_ready_hosts", []),
                    f"profile {name} runtime_ready_hosts",
                )
            )
            unknown_hosts = runtime_hosts - REQUIRED_HOST_PROFILES
            if unknown_hosts:
                raise ContractError(
                    f"profile {name} has unknown runtime hosts: {sorted(unknown_hosts)}"
                )
            if definition.get("runtime_ready") and not runtime_hosts:
                raise ContractError(
                    f"profile {name} is runtime_ready but has no ready host"
                )
            if not definition.get("runtime_ready") and runtime_hosts:
                raise ContractError(
                    f"profile {name} has ready hosts while runtime_ready is false"
                )

        rules = _mapping(self.profiles_document.get("rules"), "profile rules")
        primary = set(
            _string_list(
                rules.get("production_primary_profiles"),
                "production_primary_profiles",
            )
        )
        if primary != {"mapping-fastlio", "localization-pcd"}:
            raise ContractError(
                "production_primary_profiles must be mapping-fastlio and localization-pcd"
            )

        self._validate_interfaces()
        self._validate_qos()
        self._validate_devices()
        self._validate_health()

    def _validate_interfaces(self) -> None:
        topics = _mapping(self.interfaces_document.get("topics"), "interface topics")
        for name, raw_contract in topics.items():
            _absolute_topic(name, f"interface topic {name}")
            contract = _mapping(raw_contract, f"interface topic {name}")
            allowed_types = contract.get("types")
            if isinstance(allowed_types, str):
                allowed = (_string(allowed_types, f"{name} type"),)
            else:
                allowed = _string_list(allowed_types, f"{name} types")
            if not all("/msg/" in item for item in allowed):
                raise ContractError(f"{name} contains a non-message ROS type")
            owners = _mapping(contract.get("owners"), f"{name} owners")
            unknown_profiles = set(owners) - REQUIRED_PROFILES
            if unknown_profiles:
                raise ContractError(
                    f"{name} has owners for unknown profiles: {sorted(unknown_profiles)}"
                )
            for profile, owner in owners.items():
                owner_name = _string(owner, f"{name} owner for {profile}")
                if owner_name not in self.components:
                    raise ContractError(f"{name} references unknown owner {owner_name}")
                profile_components = set(
                    _string_list(
                        _mapping(self.profiles[profile], f"profile {profile}").get(
                            "components"
                        ),
                        f"profile {profile} components",
                    )
                )
                if owner_name not in profile_components:
                    raise ContractError(
                        f"{name} owner {owner_name} is absent from profile {profile}"
                    )

        command = _mapping(
            self.interfaces_document.get("command_contract"), "command_contract"
        )
        if _absolute_topic(command.get("final_topic"), "final command topic") != "/cmd_vel":
            raise ContractError("final command topic must be /cmd_vel")
        if _string(command.get("single_owner"), "final command owner") != "command_gate":
            raise ContractError("greenhouse_cmd_gate must be the final /cmd_vel owner")
        sources = _string_list(command.get("allowed_sources"), "command sources")
        expected_sources = {"/cmd_vel/nav", "/cmd_vel/teleop", "/cmd_vel/test"}
        if set(sources) != expected_sources:
            raise ContractError(
                "command sources must be /cmd_vel/nav, /cmd_vel/teleop and /cmd_vel/test"
            )
        for source in sources:
            _absolute_topic(source, "command source")

        tf_profiles = _mapping(self.interfaces_document.get("tf"), "TF profiles")
        for primary in ("mapping-fastlio", "localization-pcd"):
            raw_edges = tf_profiles.get(primary)
            if not isinstance(raw_edges, list):
                raise ContractError(f"TF profile {primary} must be a list")
            edges: list[tuple[str, str]] = []
            for index, raw_edge in enumerate(raw_edges):
                edge = _mapping(raw_edge, f"TF {primary}[{index}]")
                parent = _string(edge.get("parent"), "TF parent")
                child = _string(edge.get("child"), "TF child")
                owner = _string(edge.get("owner"), "TF owner")
                if parent == child:
                    raise ContractError(f"TF edge {parent} -> {child} is cyclic")
                if owner not in self.components:
                    raise ContractError(f"TF edge references unknown owner {owner}")
                edges.append((parent, child))
            if len(set(edges)) != len(edges):
                raise ContractError(f"TF profile {primary} has duplicate edges")
            if set(edges) != REQUIRED_TF_EDGES:
                raise ContractError(
                    f"TF profile {primary} must define exactly the canonical tree"
                )

    def _validate_qos(self) -> None:
        interface_topics = _mapping(
            self.interfaces_document.get("topics"), "interface topics"
        )
        qos_topics = _mapping(self.qos_document.get("topics"), "QoS topics")
        for name, raw_qos in qos_topics.items():
            _absolute_topic(name, f"QoS topic {name}")
            if name not in interface_topics:
                raise ContractError(f"QoS topic {name} is absent from interfaces.yaml")
            qos = _mapping(raw_qos, f"QoS topic {name}")
            reliability = qos.get("reliability")
            if reliability is not None and reliability not in ALLOWED_RELIABILITY:
                raise ContractError(f"QoS topic {name} has invalid reliability")
            durability = qos.get("durability")
            if durability not in ALLOWED_DURABILITY:
                raise ContractError(f"QoS topic {name} has invalid durability")
            history = qos.get("history")
            if history is not None and history not in ALLOWED_HISTORY:
                raise ContractError(f"QoS topic {name} has invalid history")
            depth = qos.get("depth")
            if history == "keep_last" and (
                not isinstance(depth, int) or isinstance(depth, bool) or depth <= 0
            ):
                raise ContractError(
                    f"QoS topic {name} keep_last depth must be a positive integer"
                )

    def _validate_devices(self) -> None:
        hosts = self.host_profiles
        if set(hosts) != REQUIRED_HOST_PROFILES:
            raise ContractError(
                "devices.yaml must define exactly: "
                + ", ".join(sorted(REQUIRED_HOST_PROFILES))
            )
        for name, raw_host in hosts.items():
            host = _mapping(raw_host, f"host profile {name}")
            if not isinstance(host.get("use_sim_time"), bool):
                raise ContractError(f"host profile {name} use_sim_time must be boolean")
            if _string(host.get("network_mode"), f"host profile {name} network_mode") not in {
                "bridge",
                "host",
            }:
                raise ContractError(f"host profile {name} network_mode is invalid")
            devices = host.get("required_devices")
            if not isinstance(devices, list):
                raise ContractError(
                    f"host profile {name} required_devices must be a list"
                )
            for device in devices:
                path = _string(device, f"host profile {name} device")
                if not path.startswith("/dev/"):
                    raise ContractError(f"host device must be under /dev: {path}")

        simulation = _mapping(hosts["simulation"], "simulation host profile")
        if not simulation["use_sim_time"] or simulation["required_devices"]:
            raise ContractError(
                "simulation host must use simulation time and require no devices"
            )
        nuc = _mapping(hosts["nuc-amd64"], "nuc-amd64 host profile")
        if nuc["use_sim_time"] or "/dev/dlrobot" not in nuc["required_devices"]:
            raise ContractError(
                "nuc-amd64 must use wall time and require /dev/dlrobot"
            )

    def _validate_health(self) -> None:
        interface_topics = _mapping(
            self.interfaces_document.get("topics"), "interface topics"
        )
        profile_health = _mapping(
            self.health_document.get("profiles"), "health profiles"
        )
        if set(profile_health) != REQUIRED_PROFILES:
            raise ContractError("health.yaml must define every bringup profile")
        for profile, raw_health in profile_health.items():
            health = _mapping(raw_health, f"health profile {profile}")
            topics = health.get("topics")
            if not isinstance(topics, list):
                raise ContractError(f"health profile {profile} topics must be a list")
            seen: set[str] = set()
            for index, raw_topic in enumerate(topics):
                topic = _mapping(raw_topic, f"health {profile} topic {index}")
                name = _absolute_topic(topic.get("name"), "health topic name")
                if name in seen:
                    raise ContractError(
                        f"health profile {profile} repeats topic {name}"
                    )
                seen.add(name)
                if name not in interface_topics:
                    raise ContractError(
                        f"health profile {profile} uses unknown topic {name}"
                    )
                types = _mapping(
                    topic.get("type_by_host"), f"health topic {name} type_by_host"
                )
                if set(types) != REQUIRED_HOST_PROFILES:
                    raise ContractError(
                        f"health topic {name} must define every host profile type"
                    )
                allowed_raw = _mapping(
                    interface_topics[name], f"interface topic {name}"
                ).get("types")
                allowed = (
                    {_string(allowed_raw, f"{name} type")}
                    if isinstance(allowed_raw, str)
                    else set(_string_list(allowed_raw, f"{name} types"))
                )
                for host, topic_type in types.items():
                    selected_type = _string(
                        topic_type, f"health topic {name} type for {host}"
                    )
                    if selected_type not in allowed:
                        raise ContractError(
                            f"health topic {name} selects disallowed type {selected_type}"
                        )
                for field in ("minimum_publishers", "maximum_publishers"):
                    value = topic.get(field)
                    if value is not None and (
                        not isinstance(value, int)
                        or isinstance(value, bool)
                        or value < 0
                    ):
                        raise ContractError(
                            f"health topic {name} {field} must be a non-negative integer"
                        )
                owners_by_host = topic.get("owner_nodes_by_host", {})
                owners = _mapping(
                    owners_by_host, f"health topic {name} owner_nodes_by_host"
                )
                unknown_owner_hosts = set(owners) - REQUIRED_HOST_PROFILES
                if unknown_owner_hosts:
                    raise ContractError(
                        f"health topic {name} has owners for unknown hosts: "
                        f"{sorted(unknown_owner_hosts)}"
                    )
                for host, owner_nodes in owners.items():
                    for owner in _string_list(
                        owner_nodes, f"health topic {name} owners for {host}"
                    ):
                        if not owner.startswith("/"):
                            raise ContractError(
                                f"health topic {name} owner must be a fully qualified node"
                            )
            services = _string_list(
                health.get("services", []), f"health profile {profile} services"
            )
            for service in services:
                _absolute_topic(service, "health service")

    def resolve(
        self,
        profiles: str | Iterable[str],
        host_profile: str,
        *,
        require_runtime_ready: bool = False,
    ) -> ResolvedBringup:
        selected = parse_profile_selection(profiles)
        unknown = set(selected) - set(self.profiles)
        if unknown:
            raise ContractError(f"unknown profiles: {sorted(unknown)}")
        if host_profile not in self.host_profiles:
            raise ContractError(f"unknown host profile: {host_profile}")

        definitions = {
            name: _mapping(self.profiles[name], f"profile {name}") for name in selected
        }
        unavailable = [
            name for name, definition in definitions.items() if not definition.get("available")
        ]
        if unavailable:
            reasons = [
                _string(
                    definitions[name].get("unavailable_reason"),
                    f"profile {name} unavailable_reason",
                )
                for name in unavailable
            ]
            raise ContractError(
                "unavailable profiles selected: "
                + "; ".join(f"{name}: {reason}" for name, reason in zip(unavailable, reasons))
            )

        selected_set = set(selected)
        for name, definition in definitions.items():
            conflicts = set(
                _string_list(definition.get("conflicts", []), f"{name} conflicts")
            )
            active_conflicts = conflicts & selected_set
            if active_conflicts:
                raise ContractError(
                    f"profile {name} conflicts with {sorted(active_conflicts)}"
                )

        roles = {
            name: _string(definition.get("role"), f"profile {name} role")
            for name, definition in definitions.items()
        }
        primaries = [
            name for name, role in roles.items() if role == "production-primary"
        ]
        addons = [name for name, role in roles.items() if role == "production-addon"]
        experiments = [name for name, role in roles.items() if role == "experiment"]
        non_tools = [name for name, role in roles.items() if role != "tools"]

        if experiments and any(roles[name] != "experiment" for name in non_tools):
            raise ContractError("experiment profiles cannot join a production profile")
        if len(experiments) > 1:
            raise ContractError("only one experiment profile may be active")
        if addons and len(primaries) != 1:
            raise ContractError(
                "production add-ons require exactly one production localization profile"
            )
        if len(primaries) > 1:
            raise ContractError(
                "mapping-fastlio and localization-pcd are mutually exclusive"
            )
        if non_tools and not primaries and not experiments:
            raise ContractError("profile selection has no primary runtime")

        components: list[str] = []
        blockers: list[str] = []
        runtime_ready = True
        for name in selected:
            definition = definitions[name]
            for component in _string_list(
                definition.get("components"), f"profile {name} components"
            ):
                if component not in components:
                    components.append(component)
            ready_hosts = set(
                _string_list(
                    definition.get("runtime_ready_hosts", []),
                    f"profile {name} runtime_ready_hosts",
                )
            )
            if not definition.get("runtime_ready"):
                runtime_ready = False
                blockers.append(
                    _string(
                        definition.get("runtime_blocker"),
                        f"profile {name} runtime_blocker",
                    )
                )
            elif host_profile not in ready_hosts:
                runtime_ready = False
                blockers.append(
                    f"profile {name} has no qualified runtime assembly for "
                    f"host {host_profile}"
                )
        if require_runtime_ready and not runtime_ready:
            raise ContractError("runtime is not ready: " + "; ".join(blockers))

        host = _mapping(self.host_profiles[host_profile], f"host {host_profile}")
        primary = primaries[0] if primaries else None
        tf_requirements: list[TfRequirement] = []
        if primary is not None:
            tf_document = _mapping(self.interfaces_document.get("tf"), "TF profiles")
            for raw_edge in tf_document[primary]:
                edge = _mapping(raw_edge, f"TF edge for {primary}")
                tf_requirements.append(
                    TfRequirement(
                        parent=_string(edge.get("parent"), "TF parent"),
                        child=_string(edge.get("child"), "TF child"),
                        owner=_string(edge.get("owner"), "TF owner"),
                    )
                )

        topic_requirements: dict[str, TopicRequirement] = {}
        service_requirements: list[str] = []
        health_profiles = _mapping(
            self.health_document.get("profiles"), "health profiles"
        )
        qos_topics = _mapping(self.qos_document.get("topics"), "QoS topics")
        for profile in selected:
            health = _mapping(health_profiles[profile], f"health profile {profile}")
            for raw_topic in health.get("topics", []):
                topic = _mapping(raw_topic, f"health topic for {profile}")
                name = _absolute_topic(topic.get("name"), "health topic name")
                types = _mapping(topic.get("type_by_host"), f"{name} type_by_host")
                qos = _mapping(qos_topics.get(name), f"QoS topic {name}")
                owners_by_host = _mapping(
                    topic.get("owner_nodes_by_host", {}),
                    f"health topic {name} owner_nodes_by_host",
                )
                requirement = TopicRequirement(
                    name=name,
                    topic_type=_string(types[host_profile], f"{name} type"),
                    minimum_publishers=int(topic.get("minimum_publishers", 1)),
                    maximum_publishers=topic.get("maximum_publishers"),
                    owner_nodes=tuple(
                        _string_list(
                            owners_by_host.get(host_profile, []),
                            f"health topic {name} owners for {host_profile}",
                        )
                    ),
                    reliability=qos.get("reliability"),
                    durability=_string(qos.get("durability"), f"{name} durability"),
                )
                previous = topic_requirements.get(name)
                if previous is not None and previous != requirement:
                    raise ContractError(
                        f"selected profiles disagree on health contract for {name}"
                    )
                topic_requirements[name] = requirement
            for service in _string_list(
                health.get("services", []), f"health {profile} services"
            ):
                if service not in service_requirements:
                    service_requirements.append(service)

        combined_role = "tools-only"
        if experiments:
            combined_role = "experiment"
        elif primaries:
            combined_role = "production"

        return ResolvedBringup(
            profiles=selected,
            host_profile=host_profile,
            role=combined_role,
            components=tuple(components),
            runtime_ready=runtime_ready,
            runtime_blockers=tuple(blockers),
            use_sim_time=bool(host["use_sim_time"]),
            network_mode=_string(host.get("network_mode"), "network_mode"),
            required_devices=tuple(
                _string_list(host.get("required_devices"), "required_devices")
            ),
            required_topics=tuple(
                topic_requirements[name] for name in sorted(topic_requirements)
            ),
            required_services=tuple(sorted(service_requirements)),
            required_tf=tuple(tf_requirements),
        )
