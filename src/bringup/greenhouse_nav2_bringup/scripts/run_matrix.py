#!/usr/bin/env python3
"""Run the nine-combination simulation matrix and aggregate results.

Combinations: 3 sites (maize_field, orchard, pipeline) x 3 robots
(jackal_j100, husky_a200, x500). Each combination runs three independent
resets by default. Results are written per run and aggregated into
matrix_results.json / matrix_results.csv under the output root.

Usage:
  python3 run_matrix.py --output /tmp/simulation --runs 3 \
      [--only maize_field:jackal_j100] [--timeout 600]
"""
import argparse
import csv
import json
import subprocess
import sys
from pathlib import Path

WORLDS = ('maize_field', 'orchard', 'pipeline')
ROBOTS = ('jackal_j100', 'husky_a200', 'x500')
SEED = '20261006'


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='/tmp/simulation')
    parser.add_argument('--runs', type=int, default=3)
    parser.add_argument('--timeout', type=float, default=600.0)
    parser.add_argument('--only', help='world:robot to run a single combination')
    parser.add_argument('--resume', action='store_true',
                        help='Keep completed records in this output directory and continue missing runs')
    parser.add_argument('--capture-first-run',action='store_true')
    parser.add_argument('--media-output',type=Path,default=Path('/media'))
    args = parser.parse_args()
    if args.runs < 1:
        parser.error('--runs must be positive')

    combos = [(w, r) for w in WORLDS for r in ROBOTS]
    if args.only:
        if args.only not in {f'{world}:{robot}' for world, robot in combos}:
            parser.error('--only must name a configured world:robot combination')
        world, robot = args.only.split(':', 1)
        combos = [(world, robot)]

    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    runner = Path(__file__).with_name('run_simulation_run.py')

    def checkpoint(records):
        temporary = output / 'matrix_results.json.tmp'
        temporary.write_text(json.dumps(records, indent=2))
        temporary.replace(output / 'matrix_results.json')
        with (output / 'matrix_results.csv').open('w', newline='') as handle:
            writer = csv.writer(handle)
            fields = ['run_id', 'world', 'robot', 'seed', 'localization', 'result', 'reason',
                'path_length_m','mission_elapsed_s','real_time_factor','max_abs_roll_rad','max_abs_pitch_rad',
                'unexpected_collisions','stalled','tipped','landed','max_waypoint_error_m','max_altitude_error_m',
                'max_cruise_height_loss_m','hover_max_xy_error_m','hover_max_altitude_error_m',
                'implementation_sha256','simulation_log','runtime_directory']
            writer.writerow(fields)
            for record in records:
                observations=record.get('observations',{})
                metrics=observations.get('air_mission',observations)
                row={**metrics,**record}
                if metrics.get('waypoints'):
                    row['max_waypoint_error_m']=max(item['horizontal_error'] for item in metrics['waypoints'])
                    row['max_altitude_error_m']=max(item['altitude_error'] for item in metrics['waypoints'])
                writer.writerow([row.get(field, '') for field in fields])

    records = []
    from run_simulation_run import implementation_digest
    expected_digest=implementation_digest()
    for world, robot in combos:
        for run in range(1, args.runs + 1):
            run_id = f'{world}_{robot}_r{run}'
            record_path = output / f'{run_id}.json'
            if record_path.exists():
                if not args.resume:
                    parser.error(f'{record_path} exists; use a new output directory or --resume')
                record = json.loads(record_path.read_text())
                if any(record.get(key) != value for key, value in
                       [('run_id', run_id), ('world', world), ('robot', robot), ('seed', SEED)]):
                    parser.error(f'Configuration mismatch in {record_path}')
                if record.get('implementation_sha256') != expected_digest:
                    parser.error(f'Implementation changed since {record_path}; use a new output directory')
                records.append(record)
                checkpoint(records)
                continue
            print(f'== {run_id} ==', flush=True)
            command=[sys.executable, str(runner), world, robot,
                 '--run-id', run_id, '--output', str(output),
                 '--timeout', str(args.timeout), '--seed', SEED]
            if args.capture_first_run and run==1:
                command.extend(['--capture','--media-output',str(args.media_output)])
                if robot=='jackal_j100': command.extend(['--capture-rviz','--capture-panorama'])
            completed = subprocess.run(command,
                capture_output=True, text=True)
            print(completed.stdout[-500:] or completed.stderr[-500:], flush=True)
            if record_path.is_file():
                records.append(json.loads(record_path.read_text()))
            else:
                records.append({'run_id': run_id, 'world': world, 'robot': robot,
                                'seed': SEED, 'result': 'not-run',
                                'reason': 'runner produced no record'})
            checkpoint(records)
            if records[-1].get('process_cleanup',{}).get('remaining_pids'):
                print('Matrix stopped: prior simulation processes remain; reset is not independent',flush=True)
                return 2
    summary = {}
    for record in records:
        summary.setdefault(record['result'], 0)
        summary[record['result']] += 1
    print(f'== matrix done: {summary} ==')
    return 0 if summary.get('pass', 0) == len(records) and records else 1


if __name__ == '__main__':
    raise SystemExit(main())
