cmake_minimum_required(VERSION 3.22)
project(voice_nav_bringup)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ament_lint_auto_find_test_dependencies()

  ament_add_pytest_test(
    motion_gate_open_convergence_test
    test/test_motion_gate_open_convergence.py
    TIMEOUT 30
  )
  ament_add_pytest_test(
    scripted_voice_demo_install_contract
    test/test_scripted_voice_demo_install_contract.py
    TIMEOUT 30
  )
  ament_add_pytest_test(
    scripted_voice_demo_route_replay
    test/test_scripted_voice_demo_route_replay.py
    TIMEOUT 30
  )
  ament_add_pytest_test(
    voice_nav_demo_command_entry
    test/test_voice_nav_demo_command_entry.py
    TIMEOUT 30
  )
  ament_add_pytest_test(
    voice_nav_demo_command_replay
    test/test_voice_nav_demo_command_replay.py
    TIMEOUT 30
  )
  ament_add_pytest_test(
    voice_nav_demo_stop_replay
    test/test_voice_nav_demo_stop_replay.py
    TIMEOUT 30
  )
  ament_add_pytest_test(
    voice_nav_session_boundary
    test/test_voice_nav_session_boundary.py
    TIMEOUT 30
  )
  ament_add_pytest_test(
    voice_nav_session_install_contract
    test/test_voice_nav_session_install_contract.py
    TIMEOUT 30
  )
  ament_add_pytest_test(
    voice_nav_console
    test/test_voice_nav_console.py
    TIMEOUT 30
  )
  ament_add_pytest_test(
    voice_nav_console_install_contract
    test/test_voice_nav_console_install_contract.py
    TIMEOUT 30
  )
  ament_add_pytest_test(
    voice_nav_app
    test/test_voice_nav_app.py
    TIMEOUT 30
  )
  ament_add_pytest_test(
    voice_nav_app_install_contract
    test/test_voice_nav_app_install_contract.py
    TIMEOUT 30
  )
  ament_add_pytest_test(
    canonical_session_driver
    test/test_canonical_session_driver.py
    TIMEOUT 30
  )
  ament_add_pytest_test(
    mapping_mvp_contract
    test/test_mapping_mvp_contract.py
    TIMEOUT 30
  )
  ament_add_pytest_test(
    navigation_mvp_contract
    test/test_navigation_mvp_contract.py
    TIMEOUT 30
  )
  ament_add_pytest_test(
    issue164_headless_runner
    test/test_issue164_headless_runner.py
    TIMEOUT 30
  )

  add_launch_test(
    test/test_motion_gate_product.py
    TIMEOUT 180
    RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py"
  )
  add_launch_test(
    test/test_mission_runtime_crash_stop.py
    TARGET mission_runtime_crash_stop
    TIMEOUT 300
    RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py"
  )
  add_launch_test(
    test/test_motion_gate_consumer_deadman.py
    TARGET motion_gate_consumer_deadman
    TIMEOUT 300
    RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py"
  )
  add_launch_test(
    test/test_relative_motion_product.py
    TIMEOUT 300
    RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py"
  )
  add_launch_test(
    test/test_scripted_voice_demo_launch.py
    TARGET scripted_voice_demo_launch_test
    TIMEOUT 300
    RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py"
  )
  add_launch_test(
    test/test_voice_nav_demo_stop_launch.py
    TARGET voice_nav_demo_stop_launch_test
    TIMEOUT 300
    RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py"
  )
  add_launch_test(
    test/test_mapping_mvp_launch.py
    TARGET mapping_mvp_launch_test
    TIMEOUT 300
    RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py"
  )
  add_launch_test(
    test/test_navigation_mvp_launch.py
    TARGET navigation_mvp_launch_test
    TIMEOUT 420
    RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py"
  )
  set_tests_properties(
    test_test_motion_gate_product.py
    mission_runtime_crash_stop
    motion_gate_consumer_deadman
    test_test_relative_motion_product.py
    scripted_voice_demo_launch_test
    voice_nav_demo_stop_launch_test
    mapping_mvp_launch_test
    navigation_mvp_launch_test
    PROPERTIES
      ENVIRONMENT
        "RMW_IMPLEMENTATION=rmw_fastrtps_cpp;ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST"
      RUN_SERIAL TRUE
  )
  set_tests_properties(
    test_test_motion_gate_product.py
    mission_runtime_crash_stop
    motion_gate_consumer_deadman
    test_test_relative_motion_product.py
    scripted_voice_demo_launch_test
    voice_nav_demo_stop_launch_test
    mapping_mvp_launch_test
    navigation_mvp_launch_test
    PROPERTIES
      ENVIRONMENT_MODIFICATION
        "ROS_DOMAIN_ID=unset:;DISABLE_ROS_ISOLATION=unset:"
  )
  # run_test.py is also invoked directly by the headless product gate.  Keep
  # the workspace overlay discoverable even when the caller sourced only the
  # system Jazzy setup file.
  set_tests_properties(
    navigation_mvp_launch_test
    PROPERTIES
      ENVIRONMENT
        "RMW_IMPLEMENTATION=rmw_fastrtps_cpp;ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST;AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}:$ENV{AMENT_PREFIX_PATH};PYTHONPATH=${CMAKE_INSTALL_PREFIX}/voice_nav_interfaces/lib/python3.12/site-packages:${CMAKE_INSTALL_PREFIX}/voice_nav_mission/lib/python3.12/site-packages:$ENV{PYTHONPATH};LD_LIBRARY_PATH=${CMAKE_INSTALL_PREFIX}/voice_nav_interfaces/lib:${CMAKE_INSTALL_PREFIX}/voice_nav_mission/lib:${CMAKE_INSTALL_PREFIX}/voice_nav_sim/lib:$ENV{LD_LIBRARY_PATH}"
  )
endif()

install(
  PROGRAMS
    voice_nav_console.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME voice_nav_console
)

install(
  FILES
    _mode_readiness.py
    _sensevoice_input.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    voice_nav_app.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME voice_nav_app
)

install(
  PROGRAMS
    voice_nav_issue164_runner.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME voice_nav_issue164_runner
)

install(
  DIRECTORY
    config
    launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

ament_package()
