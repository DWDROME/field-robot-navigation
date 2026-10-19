from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _vertex_count(path):
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            fields = line.strip().split()
            if len(fields) == 3 and fields[:2] == ["element", "vertex"]:
                return int(fields[2])
            if line.strip() == "end_header":
                break
    raise AssertionError(f"missing PLY vertex count: {path}")


def test_required_path_assets_exist_and_are_nonempty():
    paths = PACKAGE_ROOT / "paths"
    required = [
        paths / "startPaths.ply",
        paths / "paths.ply",
        paths / "pathList.ply",
        paths / "correspondences.txt",
    ]
    for path in required:
        assert path.is_file(), path
        assert path.stat().st_size > 0, path


def test_path_library_contract_matches_algorithm_constants():
    paths = PACKAGE_ROOT / "paths"
    assert _vertex_count(paths / "startPaths.ply") > 0
    assert _vertex_count(paths / "paths.ply") > 0
    assert _vertex_count(paths / "pathList.ply") == 343

    correspondence_rows = sum(
        1
        for line in (paths / "correspondences.txt").read_text(
            encoding="utf-8"
        ).splitlines()
        if line.strip()
    )
    assert correspondence_rows > 0
