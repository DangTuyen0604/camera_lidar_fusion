#!/usr/bin/env python3
"""Fail-fast static repository audit used by Gate 7.1 and CI."""

import argparse
import os
from pathlib import Path
import re
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
TEXT_SUFFIXES = {
    '.cpp', '.hpp', '.md', '.msg', '.py', '.sh', '.sdf', '.xml', '.xacro',
    '.yaml', '.yml'}
FORBIDDEN_PREFIXES = ('build/', 'install/', 'log/', 'data/kitti/', 'bags/')
FORBIDDEN_SUFFIXES = ('.bag', '.db3', '.mcap', '.mp4', '.onnx', '.pt')
ZERO_BYTE_ALLOWED = re.compile(
    r'(^models/\.gitkeep$|/__init__\.py$|/resource/[^/]+$)')


def git_paths(*arguments):
    output = subprocess.check_output(
        ['git', '-C', str(ROOT), 'ls-files', *arguments], text=True)
    return [Path(line) for line in output.splitlines() if line]


def repository_paths():
    paths = git_paths()
    paths += git_paths('--others', '--exclude-standard')
    return sorted({path for path in paths if (ROOT / path).is_file()})


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--maximum-file-mib', type=float, default=25.0)
    args = parser.parse_args()
    errors = []
    paths = repository_paths()
    tracked = [path for path in git_paths() if (ROOT / path).is_file()]
    debt_pattern = re.compile(
        r'\b(?:TO' r'DO|FIX' r'ME|PLACE' r'HOLDER|DUM' r'MY)\b',
        re.IGNORECASE)

    for path in tracked:
        name = path.as_posix()
        forbidden_prefix = name.startswith(FORBIDDEN_PREFIXES)
        if name == 'bags/README.md':
            forbidden_prefix = False
        if forbidden_prefix or name.endswith(FORBIDDEN_SUFFIXES):
            errors.append(f'forbidden tracked artifact: {name}')
        if name.startswith('benchmarks/results/'):
            errors.append(f'temporary benchmark result is tracked: {name}')

    limit = int(args.maximum_file_mib * 1024 * 1024)
    for path in paths:
        absolute = ROOT / path
        size = absolute.stat().st_size
        name = path.as_posix()
        if size == 0 and not ZERO_BYTE_ALLOWED.search(name):
            errors.append(f'unexpected zero-byte file: {name}')
        if size > limit:
            errors.append(f'large file ({size / 1024 / 1024:.1f} MiB): {name}')
        if absolute.suffix in ('.sh', '.py') and absolute.read_bytes().startswith(b'#!'):
            if not os.access(absolute, os.X_OK):
                errors.append(f'executable bit missing: {name}')
        if absolute.suffix not in TEXT_SUFFIXES:
            continue
        try:
            content = absolute.read_text(encoding='utf-8')
        except UnicodeDecodeError:
            errors.append(f'non-UTF-8 text file: {name}')
            continue
        if name == 'tools/audit_repository.py':
            continue
        if debt_pattern.search(content):
            errors.append(f'unresolved development marker: {name}')
        if re.search(r'(?:pytest\.mark\.skip|pytest\.skip|GTEST_SKIP|DISABLED_)', content):
            errors.append(f'skipped test: {name}')
        if re.search(r'(?:/home/[^/]+|/Users/[^/]+|[A-Za-z]:\\\\Users\\\\)', content):
            errors.append(f'hard-coded user path: {name}')

    packages = list((ROOT / 'src').glob('*/package.xml'))
    if len(packages) != 12:
        errors.append(f'expected 12 ROS packages, found {len(packages)}')
    scenario = (ROOT / 'src/warehouse_simulation/config/scenarios.yaml').read_text()
    if not re.search(r'^seed:\s*\d+', scenario, re.MULTILINE):
        errors.append('warehouse scenario has no deterministic seed')
    if 'action_timeout_sec:' not in scenario:
        errors.append('warehouse scenario has no bounded action timeout')

    if errors:
        print('\n'.join(f'ERROR: {error}' for error in errors), file=sys.stderr)
        raise SystemExit(1)
    print(f'Audit OK: {len(paths)} repository files, {len(packages)} ROS packages')


if __name__ == '__main__':
    main()
