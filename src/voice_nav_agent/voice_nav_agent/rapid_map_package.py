"""Small six-file Map Package helpers for the rapid path."""

import hashlib
import math
from pathlib import Path
import re
from typing import NamedTuple

import yaml

LOGICAL_ID = re.compile(r'[a-z][a-z0-9_-]{0,31}')
DATA_FILES = (
    'map.yaml',
    'map.pgm',
    'map.posegraph',
    'map.data',
    'named_places.yaml',
)


class MapPackage(NamedTuple):
    """Verified paths that must be loaded together in Navigation Mode."""

    map_id: str
    occupancy_yaml: Path
    named_places: Path


def load_places(path, expected_map_id=None):
    """Load and validate one immutable Named Places document."""
    document = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if not isinstance(document, dict) or document.get('schema_version') != 1:
        raise ValueError('named places schema_version must be 1')
    map_id = document.get('map_id')
    if not isinstance(map_id, str) or not LOGICAL_ID.fullmatch(map_id):
        raise ValueError('invalid named places map_id')
    if expected_map_id is not None and map_id != expected_map_id:
        raise ValueError('named places map_id mismatch')
    entries = document.get('places')
    if not isinstance(entries, list) or len(entries) > 32:
        raise ValueError('places must be a list with at most 32 items')
    places = {}
    for entry in entries:
        if (
            not isinstance(entry, dict)
            or set(entry) != {'id', 'x', 'y', 'yaw'}
        ):
            raise ValueError('invalid named place fields')
        place_id = entry['id']
        values = entry['x'], entry['y'], entry['yaw']
        if (
            not isinstance(place_id, str)
            or not LOGICAL_ID.fullmatch(place_id)
            or place_id in places
            or any(not isinstance(value, (int, float)) for value in values)
            or any(not math.isfinite(value) for value in values)
            or not -math.pi <= entry['yaw'] <= math.pi
        ):
            raise ValueError('invalid or duplicate named place')
        places[place_id] = tuple(float(value) for value in values)
    return dict(sorted(places.items()))


def finish_map_package(directory, map_id, places=()):
    """Normalize upstream output and add Named Places plus a hash manifest."""
    directory = Path(directory)
    if not LOGICAL_ID.fullmatch(map_id):
        raise ValueError('invalid map_id')
    occupancy_path = directory / 'map.yaml'
    occupancy = yaml.safe_load(occupancy_path.read_text(encoding='utf-8'))
    if not isinstance(occupancy, dict):
        raise ValueError('map.yaml must contain a mapping')
    occupancy['image'] = 'map.pgm'
    occupancy_path.write_text(
        yaml.safe_dump(occupancy, sort_keys=False), encoding='utf-8'
    )
    named_places = {
        'schema_version': 1,
        'map_id': map_id,
        'places': [
            {'id': place_id, 'x': x, 'y': y, 'yaw': yaw}
            for place_id, (x, y, yaw) in sorted(places)
        ],
    }
    (directory / 'named_places.yaml').write_text(
        yaml.safe_dump(named_places, sort_keys=False), encoding='utf-8'
    )
    for name in DATA_FILES:
        path = directory / name
        if not path.is_file() or path.stat().st_size == 0 or path.is_symlink():
            raise ValueError('Map Package file missing or invalid: ' + name)
    hashes = {
        name: hashlib.sha256((directory / name).read_bytes()).hexdigest()
        for name in DATA_FILES
    }
    manifest = {
        'schema_version': 1,
        'map_id': map_id,
        'files': {
            'occupancy_yaml': 'map.yaml',
            'occupancy_image': 'map.pgm',
            'posegraph': 'map.posegraph',
            'posegraph_data': 'map.data',
            'manifest': 'manifest.yaml',
            'named_places': 'named_places.yaml',
        },
        'versions': {
            'slam_toolbox': '2.8.5',
            'navigation2': '1.3.12',
        },
        'sha256': hashes,
    }
    (directory / 'manifest.yaml').write_text(
        yaml.safe_dump(manifest, sort_keys=False), encoding='utf-8'
    )


def verify_map_package(directory, expected_map_id=None):
    """Verify one rapid Map Package before AMCL/Nav2 consumes any file."""
    directory = Path(directory)
    if not directory.is_dir() or directory.is_symlink():
        raise ValueError('Map Package directory missing or invalid')
    directory = directory.resolve()
    manifest_path = directory / 'manifest.yaml'
    if not manifest_path.is_file() or manifest_path.is_symlink():
        raise ValueError('Map Package manifest missing or invalid')
    manifest = yaml.safe_load(manifest_path.read_text(encoding='utf-8'))
    if not isinstance(manifest, dict) or manifest.get('schema_version') != 1:
        raise ValueError('manifest schema_version must be 1')
    map_id = manifest.get('map_id')
    if not isinstance(map_id, str) or not LOGICAL_ID.fullmatch(map_id):
        raise ValueError('invalid manifest map_id')
    if expected_map_id and map_id != expected_map_id:
        raise ValueError('manifest map_id mismatch')
    expected_files = {
        'occupancy_yaml': 'map.yaml',
        'occupancy_image': 'map.pgm',
        'posegraph': 'map.posegraph',
        'posegraph_data': 'map.data',
        'manifest': 'manifest.yaml',
        'named_places': 'named_places.yaml',
    }
    if manifest.get('files') != expected_files:
        raise ValueError('manifest file mapping is invalid')
    expected_hashes = manifest.get('sha256')
    if (
        not isinstance(expected_hashes, dict)
        or set(expected_hashes) != set(DATA_FILES)
    ):
        raise ValueError('manifest hashes are incomplete')
    for name in DATA_FILES:
        path = directory / name
        if not path.is_file() or path.is_symlink() or path.stat().st_size == 0:
            raise ValueError('Map Package file missing or invalid: ' + name)
        actual = hashlib.sha256(path.read_bytes()).hexdigest()
        if expected_hashes[name] != actual:
            raise ValueError('Map Package hash mismatch: ' + name)
    occupancy = yaml.safe_load(
        (directory / 'map.yaml').read_text(encoding='utf-8')
    )
    if not isinstance(occupancy, dict) or occupancy.get('image') != 'map.pgm':
        raise ValueError('map.yaml must reference map.pgm')
    load_places(directory / 'named_places.yaml', expected_map_id=map_id)
    return MapPackage(
        map_id=map_id,
        occupancy_yaml=directory / 'map.yaml',
        named_places=directory / 'named_places.yaml',
    )
