#!/usr/bin/env python3
"""Classify unique skipped tests and fail when a critical skip remains."""

from collections import Counter
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


def category_for(message):
    """Map an explicit skip reason to a release-gate category."""
    normalized = message.strip().lower().removeprefix('![cdata[').removesuffix(']]')
    for category in ('optional-data', 'optional-model', 'hardware-only'):
        if normalized.startswith(f'{category}:'):
            return category
    if 'skipped due to cppcheck' in normalized and 'performance issues' in normalized:
        return 'tooling'
    return 'critical'


def skipped_records(root):
    """Return deduplicated skips from JUnit XML files below root."""
    records = set()
    for xml_path in root.rglob('*.xml'):
        try:
            document = ET.parse(xml_path)
        except ET.ParseError:
            continue
        for case in document.iter('testcase'):
            skipped = case.find('skipped')
            if skipped is None:
                continue
            message = (
                skipped.attrib.get('message', '') or skipped.text or ''
            ).strip()
            records.add((
                case.attrib.get('classname', ''),
                case.attrib.get('name', ''),
                message,
            ))
    return sorted(records)


def main():
    """Print a categorized report and implement the critical-skip gate."""
    root = Path(sys.argv[1]) if len(sys.argv) > 1 else Path('build')
    if not root.is_dir():
        raise SystemExit(f'Test result directory does not exist: {root}')
    records = skipped_records(root)
    counts = Counter()
    for classname, name, message in records:
        category = category_for(message)
        counts[category] += 1
        print(f'[{category}] {classname}::{name}')
        if message:
            print(f'    {message}')
    print('\n===== SKIP SUMMARY =====')
    for category in (
            'critical', 'optional-data', 'optional-model', 'hardware-only', 'tooling'):
        print(f'{category:15s}: {counts[category]}')
    if counts['critical']:
        print('\nRelease gate FAILED: critical skips remain.')
        return 1
    print('\nRelease gate PASS: critical_skips=0')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
