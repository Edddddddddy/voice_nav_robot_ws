"""Tests for crash-stop state observation value semantics."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import unittest


def _load_support():
    support_path = (
        Path(__file__).resolve().parents[1]
        / 'src'
        / 'voice_nav_bringup'
        / 'test'
        / 'state_observation.py'
    )
    specification = importlib.util.spec_from_file_location(
        'voice_nav_state_observation', support_path
    )
    if specification is None or specification.loader is None:
        raise RuntimeError('could not load state observation support')
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


support = _load_support()


class _AmbiguousTruth:
    def __init__(self, values):
        self.values = values

    def __bool__(self):
        raise ValueError(
            'The truth value of an array with more than one element is ambiguous'
        )


class _ArrayLike:
    def __init__(self, values):
        self.values = list(values)

    def __eq__(self, other):
        return _AmbiguousTruth(
            [left == right for left, right in zip(self.values, other.values)]
        )

    def tolist(self):
        return list(self.values)


class StateObservationTest(unittest.TestCase):
    def test_array_like_fields_compare_by_value(self):
        self.assertTrue(
            support.values_equal(_ArrayLike([1, 2]), _ArrayLike([1, 2]))
        )
        self.assertFalse(
            support.values_equal(_ArrayLike([1, 2]), _ArrayLike([1, 3]))
        )


if __name__ == '__main__':
    unittest.main()
