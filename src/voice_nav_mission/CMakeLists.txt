cmake_minimum_required(VERSION 3.22)
project(voice_nav_mission)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/InternalMotionGateState.msg"
  "srv/InternalMotionGateControl.srv"
)

add_library(
  gate_event_journal STATIC
  src/gate_event_journal.cpp
)
target_compile_features(gate_event_journal PUBLIC cxx_std_17)
target_include_directories(
  gate_event_journal
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>"
)

add_library(
  attached_gate_event_journal STATIC
  src/attached_gate_event_journal.cpp
)
target_compile_features(attached_gate_event_journal PUBLIC cxx_std_17)
target_include_directories(
  attached_gate_event_journal
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>"
)
target_link_libraries(
  attached_gate_event_journal
  PUBLIC gate_event_journal
  PRIVATE rt
)

add_library(
  motion_gate_process_runtime STATIC
  src/motion_gate_process_runtime.cpp
)
target_compile_features(motion_gate_process_runtime PUBLIC cxx_std_17)
target_include_directories(
  motion_gate_process_runtime
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>"
)

add_library(
  motion_gate_core STATIC
  src/motion_gate_core.cpp
)
target_compile_features(motion_gate_core PUBLIC cxx_std_17)
target_include_directories(
  motion_gate_core
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
)
target_link_libraries(motion_gate_core PRIVATE gate_event_journal)
target_link_libraries(
  motion_gate_process_runtime
  PUBLIC
    attached_gate_event_journal
    motion_gate_core
)

add_executable(
  motion_gate_node
  src/motion_gate_node.cpp
  src/writer_observation.cpp
)
target_compile_features(motion_gate_node PUBLIC cxx_std_17)
target_include_directories(
  motion_gate_node
  PRIVATE include
)
ament_target_dependencies(
  motion_gate_node
  geometry_msgs
  rcl_interfaces
  rclcpp
  rmw
)

rosidl_get_typesupport_target(
  motion_gate_typesupport
  ${PROJECT_NAME}
  rosidl_typesupport_cpp
)
target_link_libraries(
  motion_gate_node
  motion_gate_core
  "${motion_gate_typesupport}"
)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_ros REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(
    motion_gate_core_test
    test/motion_gate_core_test.cpp
  )
  target_compile_features(motion_gate_core_test PUBLIC cxx_std_17)
  target_link_libraries(
    motion_gate_core_test
    motion_gate_core
  )

  ament_add_gtest(
    gate_event_journal_test
    test/gate_event_journal_test.cpp
  )
  target_compile_features(gate_event_journal_test PUBLIC cxx_std_17)
  target_link_libraries(
    gate_event_journal_test
    gate_event_journal
  )

  ament_add_gtest(
    motion_gate_journal_test
    test/motion_gate_journal_test.cpp
  )
  target_compile_features(motion_gate_journal_test PUBLIC cxx_std_17)
  target_include_directories(
    motion_gate_journal_test
    PRIVATE src
  )
  target_link_libraries(
    motion_gate_journal_test
    motion_gate_core
    gate_event_journal
  )

  ament_add_gtest(
    attached_gate_event_journal_test
    test/attached_gate_event_journal_test.cpp
  )
  target_compile_features(
    attached_gate_event_journal_test PUBLIC cxx_std_17
  )
  target_include_directories(
    attached_gate_event_journal_test
    PRIVATE src
  )
  target_link_libraries(
    attached_gate_event_journal_test
    attached_gate_event_journal
    rt
  )

  ament_add_gtest(
    motion_gate_process_runtime_test
    test/motion_gate_process_runtime_test.cpp
  )
  target_compile_features(
    motion_gate_process_runtime_test PUBLIC cxx_std_17
  )
  target_include_directories(
    motion_gate_process_runtime_test
    PRIVATE src
  )
  target_link_libraries(
    motion_gate_process_runtime_test
    motion_gate_process_runtime
  )

  add_executable(
    gate_event_journal_c_abi_test
    test/gate_event_journal_c_abi_test.c
  )
  target_compile_features(gate_event_journal_c_abi_test PUBLIC c_std_11)
  target_include_directories(
    gate_event_journal_c_abi_test
    PRIVATE src
  )
  add_test(
    NAME gate_event_journal_c_abi_test
    COMMAND gate_event_journal_c_abi_test
  )
  set_tests_properties(
    gate_event_journal_c_abi_test
    PROPERTIES LABELS "c_abi"
  )

  add_executable(
    gate_event_journal_attach_probe
    test/gate_event_journal_attach_probe.cpp
  )
  target_compile_features(
    gate_event_journal_attach_probe PUBLIC cxx_std_17
  )
  target_include_directories(
    gate_event_journal_attach_probe
    PRIVATE src
  )
  target_link_libraries(
    gate_event_journal_attach_probe
    attached_gate_event_journal
    rt
  )

  add_test(
    NAME gate_event_journal_cross_process_test
    COMMAND
      "${Python3_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/test/test_gate_event_journal_cross_process.py"
      "$<TARGET_FILE:gate_event_journal_attach_probe>"
  )
  set_tests_properties(
    gate_event_journal_cross_process_test
    PROPERTIES
      ENVIRONMENT "PYTHONDONTWRITEBYTECODE=1"
      LABELS "cross_process"
      RUN_SERIAL TRUE
      TIMEOUT 15
  )

  ament_add_gtest(
    writer_observation_test
    test/writer_observation_test.cpp
    src/writer_observation.cpp
  )
  target_compile_features(writer_observation_test PUBLIC cxx_std_17)
  target_include_directories(
    writer_observation_test
    PRIVATE include src
  )
  ament_target_dependencies(writer_observation_test rmw)
  target_link_libraries(
    writer_observation_test
    motion_gate_core
  )

  add_launch_test(
    test/test_motion_gate_node.py
    TIMEOUT 60
    RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py"
  )
  set_tests_properties(
    test_test_motion_gate_node.py
    PROPERTIES
      ENVIRONMENT
        "RMW_IMPLEMENTATION=rmw_fastrtps_cpp;ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST"
      RUN_SERIAL TRUE
  )
  set_tests_properties(
    test_test_motion_gate_node.py
    PROPERTIES
      ENVIRONMENT_MODIFICATION
        "ROS_DOMAIN_ID=unset:;DISABLE_ROS_ISOLATION=unset:"
  )
endif()

install(
  TARGETS motion_gate_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
