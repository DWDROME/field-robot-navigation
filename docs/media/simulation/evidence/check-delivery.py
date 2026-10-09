"""Read-only checks of preserved simulation results and delivery links."""
import csv
import hashlib
import json
import re
import struct
from collections import Counter
from pathlib import Path

root = Path(__file__).resolve().parents[4]
media = root/'docs/media/simulation'
evidence = media/'evidence'
summary = json.loads((evidence/'matrix-validation.json').read_text())
records = json.loads((evidence/'candidate-matrix/matrix_results.json').read_text())
rows = list(csv.DictReader((evidence/'candidate-matrix/matrix_results.csv').open()))
assert len(records) == len(rows) == 27
assert {row['run_id'] for row in rows} == {record['run_id'] for record in records}
assert Counter(record['result'] for record in records) == {'pass': 8, 'fail': 19}
for record in records:
    assert record['image_id'] == summary['image_id']
    assert record['implementation_sha256'] == summary['implementation_sha256']
    assert record['process_cleanup']['remaining_pids'] == []
    assert record['observations']['clock_publishers'] == 1 if record['robot'] != 'x500' else record['observations']['air_mission']['clock_publishers'] == 1
    stored = json.loads((evidence/'candidate-matrix'/f"{record['run_id']}.json").read_text())
    assert stored == record
    if record['robot'] != 'x500':
        assert len(record['observations']['cases']) == 3
        for case in record['observations']['cases']:
            assert all(key in case['observations'] for key in ['goal_error_m', 'path_length_m',
                'mission_elapsed_s', 'real_time_factor', 'unexpected_collisions',
                'max_abs_roll_rad', 'max_abs_pitch_rad', 'stalled', 'tipped'])
    else:
        mission = record['observations']['air_mission']
        assert mission['position_samples'] > 0
        if record['result'] == 'pass':
            assert mission['landed'] and len(mission['waypoints']) == 4
    paths = [record['simulation_log'], *record['ulog_files']]
    paths.append(record['observations']['trace'] if record['robot'] != 'x500' else record['observations']['air_mission']['trace'])
    for path in paths:
        assert (evidence/Path(path).relative_to('/evidence')).is_file(), path
for item in summary['images']:
    path = media/item['file']
    assert struct.unpack('>II', path.read_bytes()[16:24]) == (1920,1080)
    assert hashlib.sha256(path.read_bytes()).hexdigest() == item['sha256']
    sidecar = json.loads(path.with_suffix('.json').read_text(encoding='utf-8-sig'))
    assert sidecar['visual_review'].startswith('passed')
    assert (media/sidecar['result_record']).is_file()
    assert (media/sidecar['compressed_image']).is_file()
    linked = json.loads((media/sidecar['result_record']).read_text())
    assert sidecar['status'] == linked['result']
assert Counter(item['kind'] for item in summary['images']) == {'task':9, 'panorama':3, 'rviz':3}
installed = json.loads((evidence/'installed-source-and-geometry.json').read_text())
for item in installed['source_files']:
    path = root/Path(item['source']).relative_to('/work')
    assert hashlib.sha256(path.read_bytes()).hexdigest() == item['sha256'], path
documents = [root/'docs/simulation.md', media/'README.md', evidence/'README.md']
links = 0
for document in documents:
    for target in re.findall(r'\]\(([^)]+)\)', document.read_text()):
        if '://' in target or target.startswith('#'):
            continue
        assert (document.parent/target.split('#')[0]).exists(), (document,target)
        links += 1
readme = (root/'README.md').read_text().split('### 2026-10-07 实测',1)[1].split('## 来源与许可',1)[0]
for target in re.findall(r'\]\(([^)]+)\)', readme):
    assert (root/target).is_file(), target
    links += 1
report = {'result':'pass', 'runs':27, 'results':dict(Counter(r['result'] for r in records)),
          'reviewed_images':15, 'source_hashes':len(installed['source_files']),
          'delivery_links':links, 'image_id':summary['image_id']}
# Read-only: the preserved delivery-integrity.json records the Builder run.
print(json.dumps(report,indent=2))
