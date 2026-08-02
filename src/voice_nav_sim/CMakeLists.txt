cmake_minimum_required(VERSION 3.22)
project(voice_nav_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_ros REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(gz_sim_vendor REQUIRED)
  find_package(gz-sim8 REQUIRED)
  find_package(gz_ros2_control REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(pluginlib REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_lifecycle REQUIRED)
  find_package(rmw REQUIRED)
  find_package(tf2_msgs REQUIRED)

  add_library(
    voice_nav_sim_journaled_gazebo_sim_system_adapter SHARED
    test_support/journaled_gazebo_sim_system_adapter.cpp
  )
  target_compile_features(
    voice_nav_sim_journaled_gazebo_sim_system_adapter
    PRIVATE cxx_std_17
  )
  ament_target_dependencies(
    voice_nav_sim_journaled_gazebo_sim_system_adapter
    gz_ros2_control
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
  )
  target_link_libraries(
    voice_nav_sim_journaled_gazebo_sim_system_adapter
    gz-sim8::gz-sim8
  )
  pluginlib_export_plugin_description_file(
    gz_ros2_control
    test_support/journaled_gazebo_sim_system_adapter_plugins.xml
  )

  add_library(
    voice_nav_sim_hardware_write_ledger STATIC
    test_support/hardware_write_ledger.cpp
  )
  set_target_properties(
    voice_nav_sim_hardware_write_ledger
    PROPERTIES POSITION_INDEPENDENT_CODE ON
  )
  target_compile_features(
    voice_nav_sim_hardware_write_ledger
    PRIVATE cxx_std_17
  )
  target_include_directories(
    voice_nav_sim_hardware_write_ledger
    PUBLIC test_support
  )

  add_executable(
    hardware_write_ledger_c_abi_test
    test/hardware_write_ledger_c_abi_test.c
  )
  target_compile_features(
    hardware_write_ledger_c_abi_test
    PRIVATE c_std_11
  )
  target_include_directories(
    hardware_write_ledger_c_abi_test
    PRIVATE test_support
  )
  add_test(
    NAME hardware_write_ledger_c_abi_test
    COMMAND $<TARGET_FILE:hardware_write_ledger_c_abi_test>
  )

  add_executable(
    hardware_write_ledger_attach_probe
    test/hardware_write_ledger_attach_probe.cpp
  )
  add_library(
    voice_nav_sim_hardware_write_ledger_posix STATIC
    test_support/attached_hardware_write_ledger.cpp
  )
  set_target_properties(
    voice_nav_sim_hardware_write_ledger_posix
    PROPERTIES POSITION_INDEPENDENT_CODE ON
  )
  target_compile_features(
    voice_nav_sim_hardware_write_ledger_posix
    PRIVATE cxx_std_17
  )
  target_include_directories(
    voice_nav_sim_hardware_write_ledger_posix
    PUBLIC test_support
  )
  target_link_libraries(
    voice_nav_sim_hardware_write_ledger_posix
    rt
  )
  target_compile_features(
    hardware_write_ledger_attach_probe
    PRIVATE cxx_std_17
  )
  target_include_directories(
    hardware_write_ledger_attach_probe
    PRIVATE test_support
  )
  target_link_libraries(
    hardware_write_ledger_attach_probe
    voice_nav_sim_hardware_write_ledger_posix
  )
  add_test(
    NAME hardware_write_ledger_cross_process_test
    COMMAND
      "${Python3_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/test/test_hardware_write_ledger_cross_process.py"
      "$<TARGET_FILE:hardware_write_ledger_attach_probe>"
  )
  set_tests_properties(
    hardware_write_ledger_cross_process_test
    PROPERTIES
      ENVIRONMENT "PYTHONDONTWRITEBYTECODE=1"
      LABELS cross_process
      RUN_SERIAL TRUE
      TIMEOUT 15
  )

  install(
    TARGETS voice_nav_sim_journaled_gazebo_sim_system_adapter
    DESTINATION lib
  )

  add_executable(
    tf_ownership_auditor
    test/tf_ownership_auditor.cpp
  )
  target_compile_features(tf_ownership_auditor PUBLIC cxx_std_17)
  ament_target_dependencies(
    tf_ownership_auditor
    rclcpp
    rmw
    tf2_msgs
  )

  install(
    TARGETS tf_ownership_auditor
    DESTINATION lib/${PROJECT_NAME}
  )

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(
    tf_ownership_evaluator_test
    test/tf_ownership_evaluator_test.cpp
  )

  ament_add_gtest(
    journaled_gazebo_sim_system_adapter_test
    test/journaled_gazebo_sim_system_adapter_test.cpp
  )
  target_compile_features(
    journaled_gazebo_sim_system_adapter_test
    PRIVATE cxx_std_17
  )
  ament_target_dependencies(
    journaled_gazebo_sim_system_adapter_test
    gz_ros2_control
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
  )
  target_link_libraries(
    journaled_gazebo_sim_system_adapter_test
    voice_nav_sim_journaled_gazebo_sim_system_adapter
    gz-sim8::gz-sim8
  )
  target_include_directories(
    journaled_gazebo_sim_system_adapter_test
    PRIVATE test_support
  )

  ament_add_gtest(
    hardware_write_ledger_test
    test/hardware_write_ledger_test.cpp
  )
  target_compile_features(
    hardware_write_ledger_test
    PRIVATE cxx_std_17
  )
  target_include_directories(
    hardware_write_ledger_test
    PRIVATE test_support
  )
  target_link_libraries(
    hardware_write_ledger_test
    voice_nav_sim_hardware_write_ledger
  )

  ament_add_pytest_test(
    crash_evidence_test
    test/test_crash_evidence.py
    TIMEOUT 10
  )
  ament_add_pytest_test(
    crash_robot_description_test
    test/test_crash_robot_description.py
    TIMEOUT 10
  )
  target_include_directories(
    tf_ownership_evaluator_test
    PRIVATE test
  )

  add_launch_test(
    test/test_simulation_control.py
    TIMEOUT 120
    RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py"
  )
  add_launch_test(
    test/test_simulation_interfaces.py
    TIMEOUT 120
    RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py"
  )
  add_launch_test(
    test/test_tf_ownership_conflict.py
    TIMEOUT 30
    RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py"
  )

  set_tests_properties(
    test_test_simulation_control.py
    test_test_simulation_interfaces.py
    test_test_tf_ownership_conflict.py
    PROPERTIES
      ENVIRONMENT
        "RMW_IMPLEMENTATION=rmw_fastrtps_cpp;ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST"
      RUN_SERIAL TRUE
  )
  set_tests_properties(
    test_test_simulation_control.py
    test_test_simulation_interfaces.py
    test_test_tf_ownership_conflict.py
    PROPERTIES
      ENVIRONMENT_MODIFICATION
        "ROS_DOMAIN_ID=unset:;DISABLE_ROS_ISOLATION=unset:"
  )
endif()

install(
  DIRECTORY
    config
    launch
    test_support
    urdf
    worlds
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

ament_package()
