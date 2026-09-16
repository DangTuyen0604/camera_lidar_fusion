#!/usr/bin/env python3
"""Start Ultralytics training from a YAML configuration."""

import argparse
from pathlib import Path
import shlex
import subprocess

import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('config')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    with open(args.config, encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict) or not config:
        raise SystemExit('Training config must contain a non-empty mapping')
    required = {'model', 'data', 'epochs', 'imgsz', 'project', 'name'}
    missing = sorted(required - set(config))
    if missing:
        raise SystemExit(f'Missing training config keys: {", ".join(missing)}')
    command_config = dict(config)
    command_config['project'] = str(Path(config['project']).resolve())
    command = ['yolo', 'detect', 'train']
    command.extend(
        f'{key}={value}' for key, value in command_config.items()
    )
    run_directory = Path(command_config['project']) / str(config['name'])
    run_directory.mkdir(parents=True, exist_ok=True)
    reproduction = shlex.join(command)
    (run_directory / 'reproduce_train.sh').write_text(
        '#!/usr/bin/env bash\nset -euo pipefail\n' + reproduction + '\n',
        encoding='utf-8',
    )
    print(reproduction)
    if args.dry_run:
        return
    subprocess.run(command, check=True)


if __name__ == '__main__':
    main()
