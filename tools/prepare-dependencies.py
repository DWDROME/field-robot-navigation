#!/usr/bin/env python3
"""Fetch pinned dependencies into a new directory and apply their local patches."""
import argparse
from pathlib import Path
import subprocess

import yaml

ROOT = Path(__file__).resolve().parents[1]


def run(*args):
    subprocess.run(args, check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('ros', choices=('ros1', 'ros2'))
    parser.add_argument('destination', type=Path)
    parser.add_argument('packages', nargs='*', help='Default: all common and selected ROS dependencies')
    parser.add_argument('--cache', type=Path, help='Optional local Git repositories, named as in the manifests')
    args = parser.parse_args()
    repositories = {}
    for name in ('common', args.ros):
        entries = yaml.safe_load((ROOT/'third_party'/f'{name}.repos').read_text())['repositories']
        if repositories.keys() & entries.keys():
            parser.error('Duplicate dependency authority in manifests')
        repositories.update(entries)
    selected = args.packages or list(repositories)
    if set(selected) - repositories.keys():
        parser.error('Unknown dependencies: ' + ', '.join(sorted(set(selected)-repositories.keys())))
    destination = args.destination.resolve()
    if destination.exists():
        parser.error(f'Destination exists; use the existing sources or choose a new directory: {destination}')
    destination.mkdir(parents=True)
    for name in selected:
        entry = repositories[name]
        target = destination/name
        origin = str(args.cache.resolve()/name) if args.cache else entry['url']
        run('git', '-c', 'core.autocrlf=false', 'clone', '--no-checkout', origin, str(target))
        run('git', '-C', str(target), 'config', 'core.autocrlf', 'false')
        run('git', '-C', str(target), 'remote', 'set-url', 'origin', entry['url'])
        run('git', '-C', str(target), 'checkout', '--detach', entry['version'])
        run('git', '-C', str(target), 'diff', '--exit-code', 'HEAD', '--')
        run('git', '-C', str(target), 'submodule', 'update', '--init', '--recursive')
        for patch in sorted((ROOT/'third_party'/'patches'/name).glob('*.patch')):
            run('git', '-C', str(target), 'apply', '--check', str(patch))
            run('git', '-C', str(target), 'apply', str(patch))
    print(f'Prepared {len(selected)} dependencies in {destination}')


if __name__ == '__main__':
    main()
