import subprocess
import sys
import tempfile
import textwrap
import unittest
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
CHECKER = REPOSITORY_ROOT / "scripts" / "check_sdf_contract.py"


VALID_SENSOR = """\
      <sensor name="laser" type="gpu_lidar">
        <topic>/scan</topic>
        <gz_frame_id>laser_link</gz_frame_id>
        <update_rate>10</update_rate>
        <lidar>
          <scan>
            <horizontal>
              <samples>360</samples>
              <resolution>1</resolution>
              <min_angle>-3.141592653589793</min_angle>
              <max_angle>3.141592653589793</max_angle>
            </horizontal>
            <vertical>
              <samples>1</samples>
              <resolution>1</resolution>
              <min_angle>0</min_angle>
              <max_angle>0</max_angle>
            </vertical>
          </scan>
          <range>
            <min>0.05</min>
            <max>8.0</max>
            <resolution>0.01</resolution>
          </range>
        </lidar>
      </sensor>
"""


VALID_SDF = f"""\
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
{VALID_SENSOR}\
    </link>
    <plugin name="gz_ros2_control::GazeboSimROS2ControlPlugin"
            filename="libgz_ros2_control-system.so">
      <parameters>/tmp/controllers.yaml</parameters>
      <hold_joints>true</hold_joints>
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

    def test_matching_text_outside_control_plugin_does_not_pass(self) -> None:
        invalid_sdf = VALID_SDF.replace(
            '<plugin name="gz_ros2_control::GazeboSimROS2ControlPlugin"\n'
            '            filename="libgz_ros2_control-system.so">',
            '<plugin name="other" filename="other-system">',
        ).replace(
            "</model>",
            "</model>\n"
            '  <world name="empty">\n'
            '    <plugin name="gz_ros2_control::'
            'GazeboSimROS2ControlPlugin" '
            'filename="libgz_ros2_control-system.so"/>\n'
            "  </world>",
        )

        completed = self.run_checker(invalid_sdf)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn(
            "exactly one GazeboSimROS2ControlPlugin",
            completed.stderr,
        )

    def test_wrong_control_plugin_filename_fails(self) -> None:
        completed = self.run_checker(
            VALID_SDF.replace(
                'filename="libgz_ros2_control-system.so"',
                'filename="wrong-control-system.so"',
            )
        )

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("filename must", completed.stderr)

    def test_native_diff_drive_plugin_fails(self) -> None:
        invalid_sdf = VALID_SDF.replace(
            "</model>",
            '<plugin name="gz::sim::systems::DiffDrive" '
            'filename="gz-sim-diff-drive-system"/>\n'
            "</model>",
        )

        completed = self.run_checker(invalid_sdf)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("native Gazebo DiffDrive", completed.stderr)

    def test_generated_sdf_sensor_count_is_enforced(self) -> None:
        mutations = (
            ("missing", VALID_SDF.replace(VALID_SENSOR, "")),
            ("duplicate", VALID_SDF.replace(
                VALID_SENSOR,
                VALID_SENSOR + VALID_SENSOR.replace(
                    'name="laser"',
                    'name="rogue_lidar"',
                ),
            )),
        )
        for case, invalid_sdf in mutations:
            with self.subTest(case=case):
                completed = self.run_checker(invalid_sdf)

                self.assertNotEqual(completed.returncode, 0)
                self.assertIn(
                    "exactly one sensor",
                    completed.stderr,
                )

    def test_generated_sdf_lidar_identity_is_enforced(self) -> None:
        mutations = (
            (
                'type="gpu_lidar"',
                'type="camera"',
                "type must be gpu_lidar",
            ),
            ("<topic>/scan</topic>", "<topic>/rogue</topic>", "topic"),
            (
                "<gz_frame_id>laser_link</gz_frame_id>",
                "<gz_frame_id>base_link</gz_frame_id>",
                "gz_frame_id",
            ),
        )
        for old, new, diagnostic in mutations:
            with self.subTest(diagnostic=diagnostic):
                completed = self.run_checker(
                    VALID_SDF.replace(old, new)
                )

                self.assertNotEqual(completed.returncode, 0)
                self.assertIn(diagnostic, completed.stderr)

    def test_generated_sdf_lidar_geometry_is_enforced(self) -> None:
        mutations = (
            (
                "<update_rate>10</update_rate>",
                "<update_rate>5</update_rate>",
                "update_rate",
            ),
            (
                "<samples>360</samples>",
                "<samples>180</samples>",
                "horizontal samples",
            ),
            (
                "<samples>1</samples>",
                "<samples>2</samples>",
                "vertical samples",
            ),
            (
                "<max>8.0</max>",
                "<max>4.0</max>",
                "range max",
            ),
            (
                "</range>",
                "</range><noise><type>gaussian</type></noise>",
                "noise",
            ),
        )
        for old, new, diagnostic in mutations:
            with self.subTest(diagnostic=diagnostic):
                completed = self.run_checker(
                    VALID_SDF.replace(old, new, 1)
                )

                self.assertNotEqual(completed.returncode, 0)
                self.assertIn(diagnostic, completed.stderr)

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
