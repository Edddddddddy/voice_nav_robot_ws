"""Small behavioral checks for the closed simulation ScenarioSpec."""

from __future__ import annotations

import runpy
from pathlib import Path
import unittest


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
SPEC_PATH = (
    REPOSITORY_ROOT
    / "src"
    / "voice_nav_sim"
    / "python"
    / "voice_nav_sim"
    / "_scenario_spec.py"
)


def load_spec():
    return runpy.run_path(SPEC_PATH)


class SimulationContractTest(unittest.TestCase):
    def test_repository_simulation_contract_passes(self):
        spec = load_spec()
        self.assertEqual(
            spec["scenario_names"](), ("motion", "mapping", "navigation")
        )
        self.assertEqual(spec["display_names"](), ("headless", "gui"))

    def test_scenarios_share_assets_but_select_one_owner(self):
        spec = load_spec()
        resolved = [
            spec["resolve_scenario"](mode, "headless")
            for mode in spec["scenario_names"]()
        ]
        self.assertTrue(all(item.controller_owner == "/diff_drive_controller"
                            for item in resolved))
        self.assertEqual(resolved[0].map_odom_owner, None)
        self.assertEqual(resolved[1].map_odom_owner, "/slam_toolbox")
        self.assertEqual(resolved[2].map_odom_owner, "/amcl")
        self.assertEqual(
            len({item.controller_owner for item in resolved}), 1
        )
        self.assertEqual(
            len({item.map_odom_owner for item in resolved[1:]}), 2
        )

    def test_invalid_selection_is_structured_before_spawn(self):
        spec = load_spec()
        with self.assertRaises(spec["ScenarioSpecError"]) as error:
            spec["resolve_scenario"]("roundtrip", "headless")
        self.assertEqual(error.exception.code, "invalid_scenario")
        with self.assertRaises(spec["ScenarioSpecError"]) as error:
            spec["resolve_scenario"]("motion", "browser")
        self.assertEqual(error.exception.code, "invalid_display")


if __name__ == "__main__":
    unittest.main()
