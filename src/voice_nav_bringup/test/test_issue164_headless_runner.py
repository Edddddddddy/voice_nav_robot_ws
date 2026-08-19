# Copyright 2026 Edddddddddy
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Unit and install contracts for the Issue #164 headless runner."""

import json
import importlib.util
import os
from pathlib import Path
import sys
import tempfile
import unittest

from ament_index_python.packages import get_package_prefix


_RUNNER_PATH = Path(__file__).resolve().parents[1] / (
    'voice_nav_issue164_runner.py'
)
_RUNNER_SPEC = importlib.util.spec_from_file_location(
    'voice_nav_issue164_runner', _RUNNER_PATH,
)
assert _RUNNER_SPEC is not None and _RUNNER_SPEC.loader is not None
runner = importlib.util.module_from_spec(_RUNNER_SPEC)
sys.modules[_RUNNER_SPEC.name] = runner
_RUNNER_SPEC.loader.exec_module(runner)


def _write_fake_phase_log(log_path):
    payload = 'fake phase log\n'
    if log_path.name == 'phase-D.log':
        payload += runner.NAVIGATION_STATIONARITY_PREFIX + json.dumps(
            {
                'final_stationary': True,
                'stationary_ms': 200,
                'zero_sim_ns': 0,
                'zero_receipt_ns': 1,
                'odom_receipt_ns': 2,
                'odom_stamp_ns': 3,
                'joint_receipt_ns': 3,
                'joint_stamp_ns': 4,
                'stationary_end_sim_ns': 3,
                'joint_left_velocity': 0.0,
                'joint_right_velocity': 0.0,
            },
            sort_keys=True,
            separators=(',', ':'),
        ) + '\n'
    log_path.write_text(payload, encoding='utf-8')


def _write_complete_inventory_log(command, log_path):
    package = Path(command[command.index('--test-dir') + 1]).name
    names = {
        'voice_nav_bringup': (
            'scripted_voice_demo_launch_test',
            'voice_nav_demo_stop_launch_test',
            'mapping_mvp_launch_test',
            'navigation_mvp_launch_test',
        ),
    }[package]
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_path.write_text(
        ''.join(
            f'Test #{index}: {name}\n'
            for index, name in enumerate(names, start=1)
        ),
        encoding='utf-8',
    )


class Issue164HeadlessRunnerTest(unittest.TestCase):
    def test_navigation_stationarity_parser_accepts_true_marker(self):
        marker = runner.NAVIGATION_STATIONARITY_PREFIX + json.dumps(
            {
                'final_stationary': True,
                'stationary_ms': 201,
                'zero_sim_ns': 0,
                'zero_receipt_ns': 10,
                'odom_receipt_ns': 20,
                'odom_stamp_ns': 30,
                'joint_receipt_ns': 21,
                'joint_stamp_ns': 31,
                'stationary_end_sim_ns': 30,
                'joint_left_velocity': 0.0,
                'joint_right_velocity': 0.0,
            },
            sort_keys=True,
            separators=(',', ':'),
        )

        evidence = runner.parse_navigation_stationarity_marker(
            '[navigation] ' + marker + '\n'
        )

        self.assertIsNotNone(evidence)
        self.assertIs(evidence['final_stationary'], True)
        self.assertEqual(evidence['stationary_ms'], 201)

    def test_navigation_stationarity_parser_requires_joint_evidence(self):
        marker = runner.NAVIGATION_STATIONARITY_PREFIX + json.dumps(
            {
                'final_stationary': True,
                'stationary_ms': 201,
                'zero_sim_ns': 0,
                'zero_receipt_ns': 10,
                'odom_receipt_ns': 20,
                'odom_stamp_ns': 30,
                'stationary_end_sim_ns': 30,
            },
            sort_keys=True,
            separators=(',', ':'),
        )

        with self.assertRaisesRegex(ValueError, 'joint_receipt_ns'):
            runner.parse_navigation_stationarity_marker(marker)

    def test_navigation_stationarity_parser_rejects_unbounded_joint_endpoint(
        self,
    ):
        marker = runner.NAVIGATION_STATIONARITY_PREFIX + json.dumps(
            {
                'final_stationary': True,
                'stationary_ms': 201,
                'zero_sim_ns': 0,
                'zero_receipt_ns': 10,
                'odom_receipt_ns': 20,
                'odom_stamp_ns': 30,
                'joint_receipt_ns': 21,
                'joint_stamp_ns': 31,
                'stationary_end_sim_ns': 30,
                'joint_left_velocity': 0.1,
                'joint_right_velocity': 0.0,
            },
            sort_keys=True,
            separators=(',', ':'),
        )

        with self.assertRaisesRegex(ValueError, 'joint_left_velocity'):
            runner.parse_navigation_stationarity_marker(marker)

    def test_navigation_stationarity_parser_retains_false_marker(self):
        marker = runner.NAVIGATION_STATIONARITY_PREFIX + json.dumps(
            {
                'final_stationary': False,
                'stationary_ms': 0,
                'zero_sim_ns': 0,
                'zero_receipt_ns': 10,
                'odom_receipt_ns': 20,
                'odom_stamp_ns': 30,
                'joint_receipt_ns': 21,
                'joint_stamp_ns': 31,
                'stationary_end_sim_ns': 30,
                'joint_left_velocity': 0.0,
                'joint_right_velocity': 0.0,
            },
            sort_keys=True,
            separators=(',', ':'),
        )

        evidence = runner.parse_navigation_stationarity_marker(marker)

        self.assertIsNotNone(evidence)
        self.assertIs(evidence['final_stationary'], False)

    def test_navigation_stationarity_parser_reports_missing_marker(self):
        self.assertIsNone(
            runner.parse_navigation_stationarity_marker('phase D ran\n')
        )

    def test_navigation_stationarity_parser_rejects_duplicate_marker(self):
        marker = runner.NAVIGATION_STATIONARITY_PREFIX + json.dumps(
            {
                'final_stationary': True,
                'stationary_ms': 200,
                'zero_sim_ns': 0,
                'zero_receipt_ns': 10,
                'odom_receipt_ns': 20,
                'odom_stamp_ns': 30,
                'joint_receipt_ns': 21,
                'joint_stamp_ns': 31,
                'stationary_end_sim_ns': 30,
                'joint_left_velocity': 0.0,
                'joint_right_velocity': 0.0,
            },
            sort_keys=True,
            separators=(',', ':'),
        )

        with self.assertRaisesRegex(ValueError, 'exactly once'):
            runner.parse_navigation_stationarity_marker(marker + '\n' + marker)

    def test_phase_d_requires_true_stationarity_evidence(self):
        false_marker = runner.NAVIGATION_STATIONARITY_PREFIX + json.dumps(
            {
                'final_stationary': False,
                'stationary_ms': 0,
                'zero_sim_ns': 0,
                'zero_receipt_ns': 10,
                'odom_receipt_ns': 20,
                'odom_stamp_ns': 30,
                'joint_receipt_ns': 21,
                'joint_stamp_ns': 31,
                'stationary_end_sim_ns': 30,
                'joint_left_velocity': 0.0,
                'joint_right_velocity': 0.0,
            },
            sort_keys=True,
            separators=(',', ':'),
        )
        for marker, expected_stationary in (
            (None, None),
            (false_marker, False),
        ):
            with self.subTest(marker=marker):
                def fake_runner(command, *, cwd, env, timeout_s, log_path):
                    del cwd, env, timeout_s
                    log_path.parent.mkdir(parents=True, exist_ok=True)
                    if '-N' in command:
                        _write_complete_inventory_log(command, log_path)
                        return {
                            'returncode': 0,
                            'duration_ms': 1,
                            'cleanup_stage': 'exited',
                            'owned_processes_remaining': 0,
                        }
                    content = 'fake phase log\n'
                    if log_path.name == 'phase-D.log' and marker is not None:
                        content += marker + '\n'
                    log_path.write_text(content, encoding='utf-8')
                    return {
                        'returncode': 0,
                        'duration_ms': 1,
                        'cleanup_stage': 'exited',
                        'owned_processes_remaining': 0,
                    }

                with tempfile.TemporaryDirectory() as directory:
                    output = Path(directory) / 'result.json'
                    result = runner.run_pipeline(
                        workspace_root=Path(directory),
                        output_path=output,
                        task_id='stationarity-contract',
                        exact_head='f' * 40,
                        command_runner=fake_runner,
                    )
                    document = json.loads(
                        output.read_text(encoding='utf-8')
                    )

                self.assertEqual(result, 1)
                self.assertEqual(document['phases'][2]['status'], 'failed')
                self.assertEqual(
                    document['final_stationary'], expected_stationary
                )

    def test_pipeline_json_has_exact_three_non_audio_phase_contract(self):
        calls = []

        def fake_runner(command, *, cwd, env, timeout_s, log_path):
            calls.append((tuple(command), cwd, env, timeout_s))
            log_path.parent.mkdir(parents=True, exist_ok=True)
            if '-N' in command:
                _write_complete_inventory_log(command, log_path)
            elif log_path.name == 'phase-D.log' and '--verbose' not in command:
                log_path.write_text('ctest summary\n', encoding='utf-8')
            else:
                _write_fake_phase_log(log_path)
            return {
                'returncode': 0,
                'duration_ms': 7,
                'cleanup_stage': 'exited',
                'owned_processes_remaining': 0,
                'pid': 100 + len(calls),
            }

        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'result.json'
            result = runner.run_pipeline(
                workspace_root=Path(directory),
                output_path=output,
                task_id='unit-red',
                exact_head='a' * 40,
                command_runner=fake_runner,
            )

            self.assertEqual(result, 0)
            document = json.loads(output.read_text(encoding='utf-8'))

        self.assertEqual(document['schema_version'], 1)
        self.assertEqual(document['schema'], 'voice_nav.issue164.headless')
        self.assertEqual(document['exact_head'], 'a' * 40)
        self.assertEqual(document['status'], 'passed')
        self.assertEqual(document['phase_count'], 3)
        self.assertEqual(
            document['inventory_preflight']['status'], 'passed'
        )
        self.assertEqual(
            [
                check['status']
                for check in document['inventory_preflight']['checks']
            ],
            ['passed', 'passed', 'passed'],
        )
        self.assertEqual(document['owned_processes_remaining'], 0)
        self.assertEqual(
            [phase['id'] for phase in document['phases']],
            ['B', 'C', 'D'],
        )
        self.assertEqual(
            [phase['status'] for phase in document['phases']],
            ['passed', 'passed', 'passed'],
        )
        self.assertEqual(len(calls), 6)
        inventory_calls = [call for call in calls if '-N' in call[0]]
        phase_calls = [call for call in calls if '-N' not in call[0]]
        self.assertEqual(len(inventory_calls), 3)
        self.assertEqual(len(phase_calls), 3)
        self.assertEqual(
            [
                'inventory' if '-N' in call[0] else 'phase'
                for call in calls
            ],
            [
                'inventory', 'inventory', 'inventory',
                'phase', 'phase', 'phase',
            ],
        )
        self.assertEqual(
            [call[0][call[0].index('-R') + 1] for call in phase_calls],
            [
                '^(scripted_voice_demo_launch_test|voice_nav_demo_stop_launch_test)$',
                '^mapping_mvp_launch_test$',
                '^navigation_mvp_launch_test$',
            ],
        )
        self.assertIn('MOVE', document['phases'][0]['proves'])
        self.assertIn('STOP', document['phases'][0]['proves'])
        self.assertIn('final_zero', document['phases'][0]['proves'])
        self.assertIn('stationary>=200ms', document['phases'][0]['proves'])
        self.assertTrue(document['final_zero'])
        self.assertTrue(document['final_stationary'])
        self.assertFalse(document['tf_owner_overlap_observed'])
        self.assertTrue(
            all('--no-tests=error' in call[0] for call in phase_calls)
        )
        self.assertNotIn('--verbose', phase_calls[0][0])
        self.assertNotIn('--verbose', phase_calls[1][0])
        self.assertIn('--verbose', phase_calls[2][0])
        self.assertTrue(all('cleanup_stage' in phase for phase in document['phases']))
        self.assertTrue(all('log' in phase for phase in document['phases']))
        self.assertTrue(all(call[3] > 0 for call in calls))

    def test_missing_product_inventory_blocks_all_product_phases(self):
        calls = []

        def fake_runner(command, *, cwd, env, timeout_s, log_path):
            del cwd, env, timeout_s
            calls.append((
                'inventory' if '-N' in command else 'phase',
                tuple(command),
            ))
            log_path.parent.mkdir(parents=True, exist_ok=True)
            if '-N' in command:
                package = Path(
                    command[command.index('--test-dir') + 1]
                ).name
                names = {
                    'voice_nav_bringup': (
                        'voice_nav_demo_stop_launch_test',
                        'mapping_mvp_launch_test',
                        'navigation_mvp_launch_test',
                    ),
                }[package]
                log_path.write_text(
                    ''.join(
                        f'Test #{index}: {name}\n'
                        for index, name in enumerate(names, start=1)
                    ),
                    encoding='utf-8',
                )
            else:
                _write_fake_phase_log(log_path)
            return {
                'returncode': 0,
                'duration_ms': 1,
                'cleanup_stage': 'exited',
                'owned_processes_remaining': 0,
            }

        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'missing-inventory.json'
            result = runner.run_pipeline(
                workspace_root=Path(directory),
                output_path=output,
                task_id='missing-product-inventory',
                exact_head='a' * 40,
                command_runner=fake_runner,
            )
            document = json.loads(output.read_text(encoding='utf-8'))

        self.assertEqual(result, 1)
        self.assertEqual(document['status'], 'failed')
        self.assertEqual(document['phase_count'], 0)
        self.assertEqual(document['completed_phase_count'], 0)
        self.assertEqual(document['phases'], [])
        self.assertEqual(
            document['inventory_preflight']['status'], 'failed'
        )
        failures = document['inventory_preflight']['failures']
        self.assertEqual(len(failures), 1)
        self.assertEqual(failures[0]['phase'], 'B')
        self.assertEqual(
            failures[0]['test'], 'scripted_voice_demo_launch_test'
        )
        self.assertEqual(failures[0]['failure_kind'], 'build_contract')
        self.assertEqual(failures[0]['reason'], 'missing_required_test')
        self.assertEqual([kind for kind, command in calls], ['inventory'] * 3)

    def test_duplicate_required_inventory_test_blocks_all_product_phases(self):
        calls = []

        def fake_runner(command, *, cwd, env, timeout_s, log_path):
            del cwd, env, timeout_s
            calls.append(
                ('inventory' if '-N' in command else 'phase', command)
            )
            log_path.parent.mkdir(parents=True, exist_ok=True)
            if '-N' in command:
                package = Path(
                    command[command.index('--test-dir') + 1]
                ).name
                names = {
                    'voice_nav_bringup': (
                        'scripted_voice_demo_launch_test',
                        'scripted_voice_demo_launch_test',
                        'voice_nav_demo_stop_launch_test',
                        'mapping_mvp_launch_test',
                        'navigation_mvp_launch_test',
                    ),
                }[package]
                log_path.write_text(
                    ''.join(
                        f'Test #{index}: {name}\n'
                        for index, name in enumerate(names, start=1)
                    ),
                    encoding='utf-8',
                )
            else:
                _write_fake_phase_log(log_path)
            return {
                'returncode': 0,
                'duration_ms': 1,
                'cleanup_stage': 'exited',
                'owned_processes_remaining': 0,
            }

        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'duplicate-inventory.json'
            result = runner.run_pipeline(
                workspace_root=Path(directory),
                output_path=output,
                task_id='duplicate-inventory',
                exact_head='b' * 40,
                command_runner=fake_runner,
            )
            document = json.loads(output.read_text(encoding='utf-8'))

        self.assertEqual(result, 1)
        self.assertEqual(document['phase_count'], 0)
        self.assertEqual(document['phases'], [])
        failures = document['inventory_preflight']['failures']
        self.assertEqual(len(failures), 1)
        self.assertEqual(failures[0]['phase'], 'B')
        self.assertEqual(
            failures[0]['test'], 'scripted_voice_demo_launch_test'
        )
        self.assertEqual(failures[0]['failure_kind'], 'build_contract')
        self.assertEqual(failures[0]['reason'], 'duplicate_required_test')
        self.assertEqual([kind for kind, command in calls], ['inventory'] * 3)

    def test_exact_head_is_injected_into_every_phase_environment(self):
        exact_head = 'e' * 40
        environments = []

        def fake_runner(command, *, cwd, env, timeout_s, log_path):
            del cwd, timeout_s
            if '-N' in command:
                _write_complete_inventory_log(command, log_path)
                return {
                    'returncode': 0,
                    'duration_ms': 1,
                    'cleanup_stage': 'exited',
                    'owned_processes_remaining': 0,
                }
            environments.append(dict(env))
            log_path.parent.mkdir(parents=True, exist_ok=True)
            _write_fake_phase_log(log_path)
            return {
                'returncode': 0,
                'duration_ms': 1,
                'cleanup_stage': 'exited',
                'owned_processes_remaining': 0,
                'pid': len(environments),
            }

        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'result.json'
            self.assertEqual(
                runner.run_pipeline(
                    workspace_root=Path(directory),
                    output_path=output,
                    task_id='exact-head-env',
                    exact_head=exact_head,
                    command_runner=fake_runner,
                ),
                0,
            )

        self.assertEqual(len(environments), 3)
        self.assertEqual(
            [environment['VOICE_NAV_EXACT_HEAD'] for environment in environments],
            [exact_head] * 3,
        )

    def test_failure_is_recorded_and_later_phases_are_skipped(self):
        calls = []
        phase_calls = []

        def fake_runner(command, *, cwd, env, timeout_s, log_path):
            calls.append(tuple(command))
            log_path.parent.mkdir(parents=True, exist_ok=True)
            if '-N' in command:
                _write_complete_inventory_log(command, log_path)
                return {
                    'returncode': 0,
                    'duration_ms': 3,
                    'cleanup_stage': 'exited',
                    'owned_processes_remaining': 0,
                    'pid': 200 + len(calls),
                }
            phase_calls.append(tuple(command))
            _write_fake_phase_log(log_path)
            return {
                'returncode': 1 if len(phase_calls) == 3 else 0,
                'duration_ms': 3,
                'cleanup_stage': 'exited',
                'owned_processes_remaining': 0,
                'pid': 200 + len(calls),
            }

        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'failed.json'
            result = runner.run_pipeline(
                workspace_root=Path(directory),
                output_path=output,
                task_id='unit-failure',
                exact_head='b' * 40,
                command_runner=fake_runner,
            )
            document = json.loads(output.read_text(encoding='utf-8'))

        self.assertEqual(result, 1)
        self.assertEqual(document['status'], 'failed')
        self.assertEqual(
            [phase['status'] for phase in document['phases']],
            ['passed', 'passed', 'failed'],
        )
        self.assertEqual(len(calls), 6)
        self.assertEqual(len(phase_calls), 3)
        self.assertEqual(document['phases'][2]['returncode'], 1)

    def test_result_path_is_no_replace(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'existing.json'
            output.write_text('{"old":true}\n', encoding='utf-8')

            with self.assertRaises(FileExistsError):
                runner.write_result_no_replace(output, {'new': True})

            self.assertEqual(
                output.read_text(encoding='utf-8'), '{"old":true}\n'
            )

    def test_preflight_rejects_any_task_evidence_before_product_phases(self):
        evidence_names = (
            'result.json',
            'inventory-B.log',
            'inventory-C.log',
            'inventory-D.log',
            'phase-B.log',
            'phase-C.log',
            'phase-D.log',
        )
        for evidence_name in evidence_names:
            with self.subTest(evidence_name=evidence_name):
                calls = []

                def fake_runner(command, **kwargs):
                    calls.append(command)
                    return {
                        'returncode': 0,
                        'duration_ms': 1,
                        'cleanup_stage': 'exited',
                        'owned_processes_remaining': 0,
                    }

                with tempfile.TemporaryDirectory() as directory:
                    output = Path(directory) / 'result.json'
                    output.parent.joinpath(evidence_name).write_text(
                        'old\n', encoding='utf-8'
                    )
                    with self.assertRaises(FileExistsError):
                        runner.run_pipeline(
                            workspace_root=Path(directory),
                            output_path=output,
                            task_id='preflight',
                            exact_head='d' * 40,
                            command_runner=fake_runner,
                        )

                self.assertEqual(calls, [])

    def test_exact_head_requires_forty_hex_digits(self):
        with self.assertRaises(ValueError):
            runner.validate_exact_head('not-a-head')
        self.assertEqual(runner.validate_exact_head('C' * 40), 'C' * 40)

    @unittest.skipUnless(os.name == 'posix', 'Linux process groups are the MVP path')
    def test_owned_group_cleanup_catches_descendant_after_parent_exit(self):
        child_code = 'import time; time.sleep(30)'
        parent_code = (
            'import subprocess,sys; '
            'subprocess.Popen([sys.executable, "-c", sys.argv[1]], '
            'start_new_session=False);'
        )
        with tempfile.TemporaryDirectory() as directory:
            log_path = Path(directory) / 'phase-B.log'
            result = runner._run_owned_command(
                (sys.executable, '-c', parent_code, child_code),
                cwd=Path(directory),
                env=os.environ,
                timeout_s=5.0,
                log_path=log_path,
            )

        self.assertEqual(result['returncode'], 0)
        self.assertEqual(result['owned_processes_remaining'], 0)
        self.assertIn(
            result['cleanup_stage'], {'graceful', 'terminated', 'killed'}
        )

    def test_install_contract_names_the_extensionless_entrypoint(self):
        package = Path(__file__).resolve().parents[1]
        cmake = (package / 'CMakeLists.txt').read_text(encoding='utf-8')
        self.assertIn('voice_nav_issue164_runner.py', cmake)
        self.assertIn('RENAME voice_nav_issue164_runner', cmake)
        self.assertIn('DESTINATION lib/${PROJECT_NAME}', cmake)

        executable = (
            Path(get_package_prefix('voice_nav_bringup'))
            / 'lib' / 'voice_nav_bringup' / 'voice_nav_issue164_runner'
        )
        self.assertTrue(executable.is_file())
        self.assertTrue(os.access(executable, os.X_OK))


if __name__ == '__main__':
    unittest.main()
