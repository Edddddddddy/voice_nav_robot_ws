"""Focused tests for the rapid six-file Map Package."""

import hashlib

import pytest

from voice_nav_agent.rapid_map_package import (
    DATA_FILES,
    finish_map_package,
    load_places,
    verify_map_package,
)
import yaml


def test_finish_map_package_adds_manifest_and_relative_image(tmp_path):
    """A completed package is loadable until any payload is changed."""
    (tmp_path / 'map.yaml').write_text(
        'image: /temporary/staging/map.pgm\nresolution: 0.2\n',
        encoding='utf-8',
    )
    for name in ('map.pgm', 'map.posegraph', 'map.data'):
        (tmp_path / name).write_bytes(name.encode())

    finish_map_package(tmp_path, 'map_one')

    assert sorted(path.name for path in tmp_path.iterdir()) == sorted(
        (*DATA_FILES, 'manifest.yaml')
    )
    assert yaml.safe_load(
        (tmp_path / 'map.yaml').read_text(encoding='utf-8')
    )['image'] == 'map.pgm'
    manifest = yaml.safe_load(
        (tmp_path / 'manifest.yaml').read_text(encoding='utf-8')
    )
    for name in DATA_FILES:
        assert manifest['sha256'][name] == hashlib.sha256(
            (tmp_path / name).read_bytes()
        ).hexdigest()
    package = verify_map_package(tmp_path, expected_map_id='map_one')
    assert package.map_id == 'map_one'
    assert package.occupancy_yaml == tmp_path / 'map.yaml'
    assert package.named_places == tmp_path / 'named_places.yaml'

    (tmp_path / 'map.pgm').write_bytes(b'tampered')
    with pytest.raises(ValueError, match='hash mismatch: map.pgm'):
        verify_map_package(tmp_path)


def test_load_places_returns_sorted_validated_coordinates(tmp_path):
    """Named Places are validated and returned in deterministic order."""
    places_file = tmp_path / 'places.yaml'
    places_file.write_text(
        yaml.safe_dump({
            'schema_version': 1,
            'map_id': 'house_demo',
            'places': [
                {'id': 'study', 'x': -1, 'y': 0, 'yaw': 3.0},
                {'id': 'home', 'x': 0, 'y': 0, 'yaw': 0},
            ],
        }),
        encoding='utf-8',
    )

    places = load_places(places_file, expected_map_id='house_demo')

    assert list(places) == ['home', 'study']
    assert places['study'] == (-1.0, 0.0, 3.0)
