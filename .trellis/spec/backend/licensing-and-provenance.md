# Licensing And Provenance Evidence

> Repository-local evidence for review routing. This is not a legal or
> redistribution conclusion.

---

## Evidence Rule

Do not treat a `package.xml` `<license>` value as sufficient authorization
evidence. For any copying, vendoring, redistribution, backend integration, or
release task, inspect:

1. the package manifest;
2. directory-level `LICENSE`, `COPYING`, and `NOTICE` files;
3. file-level SPDX identifiers and copyright headers;
4. vendored subdirectories and their own license files;
5. upstream provenance and modification records.

Conflicting or missing evidence is a blocker for making a confident licensing
claim. Record the conflict; do not resolve it by choosing the most convenient
label.

## Manifest Inventory

The 27 package manifests currently contain these values:

| Manifest value | Packages | Interpretation |
|----------------|----------|----------------|
| `BSD` | `fast_lio`, `fast_lio_localization`, `local_planner_3d`, `rover_msgs`, `rover_teleop`, `rviz_plugins`, `waypoint_generator`, `boundary_handler`, `dl_msgs`, `dlrobot`, `far_planner`, `graph_decoder`, `greenhouse_nav`, `local_planner`, `terrain_analysis`, `terrain_analysis_ext`, `visibility_graph_msg` | Ambiguous family label; not enough to distinguish BSD-2-Clause from BSD-3-Clause |
| `TODO` | `global_planner_3d`, `rover_gpr`, `rover_launch`, `rover_pcl_filter`, `rviz_multi_goals_manager_plugins`, `dlrobot_robot`, `livox_to_pointcloud2` | Metadata is unresolved; do not infer a license |
| `GPL-3.0-or-later` | `fast_lio_super`, `slam_octvox` | Must still be checked against directory and file-level evidence |
| `MIT` | `livox_ros_driver2` | Package-level evidence is consistent with its local `LICENSE.txt`, but vendored dependencies retain separate terms |

Package identity must come from `<name>`, not the directory basename. This is
especially important for `FAST_LIO*`, `rviz_plugins`, and
`waypoint_generator` when producing an inventory or SBOM.

## Confirmed Evidence Conflicts

### FAST-LIO family

| Package | Manifest | Directory license | Additional evidence | Required treatment |
|---------|----------|-------------------|---------------------|--------------------|
| `fast_lio` | `BSD` | `src/FAST_LIO/LICENSE` contains GPL version 2 | Package documentation describes BSD-3 | License is unresolved in this checkout |
| `fast_lio_localization` | `BSD` | `src/FAST_LIO_LOCALIZATION/LICENSE` contains GPL version 2 | Package documentation describes GPL-2.0 and restricts reuse | License is unresolved; do not rely on the manifest |
| `fast_lio_super` | `GPL-3.0-or-later` | `src/FAST_LIO_SUPER/LICENSE` contains GPL version 2 | Files include both GPL-3.0-or-later and BSD-2-Clause SPDX identifiers | Treat as a multi-origin tree with unresolved package-level metadata |

For `fast_lio_super`, examples of file-level evidence include:

- `src/FAST_LIO_SUPER/src/slam/map/oct_vox_map.cpp`:
  `GPL-3.0-or-later`;
- `src/FAST_LIO_SUPER/src/slam/map/ikd_tree_map.cpp`:
  `BSD-2-Clause`;
- `src/FAST_LIO_SUPER/include/slam/nn/neighbor_search.hpp`:
  `BSD-2-Clause`.

Do not replace these distinctions with a single package-wide label until
provenance and intended relicensing are established.

### Livox driver

`src/livox_ros_driver2/package.xml` and
`src/livox_ros_driver2/LICENSE.txt` both identify MIT, while the package-local
`CLAUDE.md` describes BSD-2. The manifest and license file are stronger local
evidence than generated prose, but the documentation conflict should still be
corrected before publishing a compliance statement.

The bundled RapidJSON tree has its own
`src/livox_ros_driver2/3rdparty/rapidjson/license.txt`, which describes MIT
terms plus BSD-licensed and JSON-licensed third-party components. Do not label
every file under the driver directory as uniformly MIT.

### OctVox vendoring

`slam_octvox` has aligned package-level GPL-3 evidence:

- `src/slam_octvox/package.xml`;
- `src/slam_octvox/LICENSE`;
- `src/slam_octvox/README.md`.

However, `src/slam_octvox/include/tsl/` remains MIT-licensed according to its
file headers and the README vendoring notes. The README sentence that says
all code inherits GPLv3 is therefore too broad.

## Missing Package-Level License Texts

Under `src/`, package-level license files were found only for:

- `FAST_LIO`;
- `FAST_LIO_LOCALIZATION`;
- `FAST_LIO_SUPER`;
- `slam_octvox`;
- `livox_ros_driver2`.

Most other ROS packages rely only on manifest metadata and source headers.
Seven packages use `TODO`, including the mainline hardware-facing
`dlrobot_robot`. Absence of a local license file does not prove that upstream
code is unlicensed, but it prevents this checkout from serving as complete
redistribution evidence.

License files under `other/` belong to retained firmware/reference trees.
Their presence does not license the ROS packages under `src/`.

## Change Rules

- Preserve file-level SPDX and third-party notices when moving or adapting
  code.
- Before copying code between FAST-LIO variants, compare both provenance and
  license evidence; similar APIs do not imply compatible reuse terms.
- Do not change a manifest license merely to make metadata consistent. First
  establish the intended license from authoritative provenance.
- Do not describe `TODO` as permissive, proprietary, or public domain.
- A build dependency or ROS topic connection does not itself prove that two
  packages form one distributable work.
- Escalate release, redistribution, and license-compatibility decisions for a
  dedicated provenance/legal review.

## Refresh Procedure

Inventory manifest values:

```bash
python3 - <<'PY'
from pathlib import Path
import xml.etree.ElementTree as ET

for path in sorted(Path("src").rglob("package.xml")):
    root = ET.parse(path).getroot()
    name = (root.findtext("name") or "").strip()
    licenses = [(node.text or "").strip() for node in root.findall("license")]
    print(name, path, licenses)
PY
```

Find repository-local license evidence (the globs below are inspection
patterns; they do not imply that every named file class currently exists):

```bash
find . -type f \
  \( -iname 'LICENSE*' -o -iname 'COPYING*' -o -iname 'NOTICE*' \) \
  -not -path './.git/*' -print | sort
```

## Primary Evidence

- `src/**/package.xml`
- package-local `LICENSE*` files where present
- package-local `COPYING*` files where present
- package-local `NOTICE*` files where present
- file-level SPDX and copyright headers
- `src/slam_octvox/README.md`
- `src/livox_ros_driver2/3rdparty/rapidjson/license.txt`
- package-local provenance documentation
