"""Shared, deterministic contract helpers for Day-6 system tests."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import yaml


def share(package):
    """Return an installed package share directory."""
    return Path(get_package_share_directory(package))


def package_root(package):
    """Prefer the source tree so tests can inspect non-installed node code."""
    workspace = Path(__file__).resolve().parents[3]
    source = workspace / 'src' / package
    return source if source.is_dir() else share(package)


def text(package, relative):
    """Read an installed UTF-8 package asset."""
    path = package_root(package) / relative
    assert path.is_file(), f'missing installed asset: {path}'
    return path.read_text(encoding='utf-8')


def yaml_asset(package, relative):
    """Load an installed YAML asset and reject empty documents."""
    document = yaml.safe_load(text(package, relative))
    assert document is not None
    return document


def assert_contains(value, *needles):
    """Assert that every required contract token is present."""
    for needle in needles:
        assert needle in value, f'missing contract token: {needle}'
