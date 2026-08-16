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
    canonical_session_driver
    test/test_canonical_session_driver.py
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
  set_tests_properties(
    test_test_motion_gate_product.py
    mission_runtime_crash_stop
    motion_gate_consumer_deadman
    test_test_relative_motion_product.py
    scripted_voice_demo_launch_test
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
    PROPERTIES
      ENVIRONMENT_MODIFICATION
        "ROS_DOMAIN_ID=unset:;DISABLE_ROS_ISOLATION=unset:"
  )
endif()

install(
  PROGRAMS
    voice_nav_console.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME voice_nav_console
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
