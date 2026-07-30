import subprocess
import sys
import tempfile
import textwrap
import unittest
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
CHECKER = REPOSITORY_ROOT / "scripts" / "check_sdf_contract.py"


VALID_SDF = """\
<sdf version="1.11">
  <model name="voice_nav_robot">
    <link name="base_footprint">
      <collision name="base_footprint_fixed_joint_lump__caster_link_collision_1">
        <surface>
          <friction>
            <ode>
              <mu>0.001</mu>
              <mu2>0.001</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
    </link>
    <plugin name="gz::sim::systems::DiffDrive"
            filename="gz-sim-diff-drive-system">
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.4</wheel_separation>
      <wheel_radius>0.035</wheel_radius>
      <odom_publish_frequency>50</odom_publish_frequency>
      <frame_id>odom</frame_id>
      <child_frame_id>base_footprint</child_frame_id>
      <min_linear_velocity>-0.20</min_linear_velocity>
      <max_linear_velocity>0.40</max_linear_velocity>
      <min_angular_velocity>-1.20</min_angular_velocity>
      <max_angular_velocity>1.20</max_angular_velocity>
      <min_linear_acceleration>-0.50</min_linear_acceleration>
      <max_linear_acceleration>0.50</max_linear_acceleration>
      <min_angular_acceleration>-1.50</min_angular_acceleration>
      <max_angular_acceleration>1.50</max_angular_acceleration>
    </plugin>
  </model>
</sdf>
"""


class SdfContractTest(unittest.TestCase):
    def run_checker(self, sdf_text: str) -> subprocess.CompletedProcess[str]:
        with tempfile.TemporaryDirectory() as temporary_directory:
            sdf_path = Path(temporary_directory) / "model.sdf"
            sdf_path.write_text(textwrap.dedent(sdf_text), encoding="utf-8")
            return subprocess.run(
                [sys.executable, str(CHECKER), str(sdf_path)],
                check=False,
                capture_output=True,
                text=True,
            )

    def test_valid_model_passes(self) -> None:
        completed = self.run_checker(VALID_SDF)

        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn("SDF contract passed", completed.stdout)

    def test_matching_text_outside_diff_drive_plugin_does_not_pass(self) -> None:
        invalid_sdf = VALID_SDF.replace(
            '<plugin name="gz::sim::systems::DiffDrive"\n'
            '            filename="gz-sim-diff-drive-system">',
            '<plugin name="other" filename="other-system">',
        ).replace(
            "</model>",
            "</model>\n"
            '  <world name="empty">\n'
            '    <plugin name="decoy" filename="gz-sim-diff-drive-system"/>\n'
            "  </world>",
        )

        completed = self.run_checker(invalid_sdf)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("exactly one model DiffDrive plugin", completed.stderr)

    def test_wrong_controller_value_fails(self) -> None:
        completed = self.run_checker(
            VALID_SDF.replace(
                "<wheel_radius>0.035</wheel_radius>",
                "<wheel_radius>0.35</wheel_radius>",
            )
        )

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("wheel_radius must be 0.035", completed.stderr)

    def test_caster_friction_must_belong_to_caster_collision(self) -> None:
        completed = self.run_checker(
            VALID_SDF.replace(
                "base_footprint_fixed_joint_lump__caster_link_collision_1",
                "unrelated_collision",
            )
        )

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("caster collision", completed.stderr)


if __name__ == "__main__":
    unittest.main()
