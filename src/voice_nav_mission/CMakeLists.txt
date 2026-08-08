cmake_minimum_required(VERSION 3.22)
project(voice_nav_mission)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(composition_interfaces REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rmw REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(voice_nav_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/InternalMotionGateState.msg"
  "srv/InternalMotionGateControl.srv"
)

rosidl_get_typesupport_target(
  motion_gate_typesupport
  ${PROJECT_NAME}
  rosidl_typesupport_cpp
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

add_library(
  mission_runtime_core STATIC
  src/mission_runtime_core.cpp
)
target_compile_features(mission_runtime_core PUBLIC cxx_std_17)
target_include_directories(
  mission_runtime_core
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
)

add_library(
  relative_motion_controller STATIC
  src/relative_motion_controller.cpp
)
target_compile_features(relative_motion_controller PUBLIC cxx_std_17)
target_include_directories(
  relative_motion_controller
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
)

add_library(
  motion_conditioning_pipeline STATIC
  src/motion_authority_ros_adapter.cpp
  src/motion_conditioning_pipeline.cpp
  src/motion_source_freshness.cpp
)
target_compile_features(motion_conditioning_pipeline PUBLIC cxx_std_17)
target_include_directories(
  motion_conditioning_pipeline
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
)
ament_target_dependencies(
  motion_conditioning_pipeline
  composition_interfaces
  controller_manager_msgs
  geometry_msgs
  lifecycle_msgs
  nav_msgs
  nav2_msgs
  rcl_interfaces
  rclcpp
  rosgraph_msgs
  rmw
  sensor_msgs
)
target_link_libraries(
  motion_conditioning_pipeline
  "${motion_gate_typesupport}"
)

add_library(
  relative_motion_ros_adapter STATIC
  src/relative_motion_ros_adapter.cpp
)
target_compile_features(relative_motion_ros_adapter PUBLIC cxx_std_17)
target_include_directories(
  relative_motion_ros_adapter
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
)
ament_target_dependencies(
  relative_motion_ros_adapter
  geometry_msgs
  nav_msgs
  rclcpp
  rosgraph_msgs
  sensor_msgs
)
target_link_libraries(
  relative_motion_ros_adapter
  motion_conditioning_pipeline
  relative_motion_controller
)

add_executable(
  mission_runtime_node
  src/mission_runtime_node.cpp
)
target_compile_features(mission_runtime_node PUBLIC cxx_std_17)
target_include_directories(
  mission_runtime_node
  PRIVATE include
)
ament_target_dependencies(
  mission_runtime_node
  rcl_interfaces
  rclcpp
  rclcpp_action
  voice_nav_interfaces
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
target_link_libraries(
  mission_runtime_node
  mission_runtime_core
  relative_motion_ros_adapter
  "${motion_gate_typesupport}"
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_ros REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(rclcpp_components REQUIRED)
  find_package(rosgraph_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(tf2_msgs REQUIRED)

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
    mission_runtime_core_test
    test/mission_runtime_core_test.cpp
  )
  target_compile_features(mission_runtime_core_test PUBLIC cxx_std_17)
  target_include_directories(
    mission_runtime_core_test
    PRIVATE include
  )
  target_link_libraries(
    mission_runtime_core_test
    mission_runtime_core
  )

  ament_add_gtest(
    runtime_event_queue_test
    test/runtime_event_queue_test.cpp
  )
  target_compile_features(runtime_event_queue_test PUBLIC cxx_std_17)
  target_include_directories(
    runtime_event_queue_test
    PRIVATE include
  )
  target_link_libraries(
    runtime_event_queue_test
    mission_runtime_core
  )

  ament_add_gtest(
    runtime_completion_registry_test
    test/runtime_completion_registry_test.cpp
  )
  target_compile_features(runtime_completion_registry_test PUBLIC cxx_std_17)
  target_include_directories(
    runtime_completion_registry_test
    PRIVATE include
  )
  ament_target_dependencies(
    runtime_completion_registry_test
    rclcpp
  )
  target_link_libraries(
    runtime_completion_registry_test
    mission_runtime_core
  )

  ament_add_gtest(
    relative_motion_ros_adapter_test
    test/relative_motion_ros_adapter_test.cpp
  )
  target_compile_features(relative_motion_ros_adapter_test PUBLIC cxx_std_17)
  target_include_directories(
    relative_motion_ros_adapter_test
    PRIVATE include
  )
  ament_target_dependencies(
    relative_motion_ros_adapter_test
    geometry_msgs
    nav_msgs
    rclcpp
    rosgraph_msgs
    sensor_msgs
  )
  target_link_libraries(
    relative_motion_ros_adapter_test
    relative_motion_ros_adapter
  )

  ament_add_gtest(
    relative_motion_controller_test
    test/relative_motion_controller_test.cpp
  )
  target_compile_features(relative_motion_controller_test PUBLIC cxx_std_17)
  target_include_directories(
    relative_motion_controller_test
    PRIVATE include
  )
  target_link_libraries(
    relative_motion_controller_test
    relative_motion_controller
  )

  ament_add_gtest(
    motion_source_freshness_test
    test/motion_source_freshness_test.cpp
  )
  target_compile_features(motion_source_freshness_test PUBLIC cxx_std_17)
  target_include_directories(
    motion_source_freshness_test
    PRIVATE include
  )
  target_link_libraries(
    motion_source_freshness_test
    motion_conditioning_pipeline
  )

  ament_add_gtest(
    mission_action_result_router_test
    test/mission_action_result_router_test.cpp
  )
  target_compile_features(mission_action_result_router_test PUBLIC cxx_std_17)
  target_include_directories(
    mission_action_result_router_test
    PRIVATE include
  )

  ament_add_gtest(
    mission_authority_convergence_test
    test/mission_authority_convergence_test.cpp
  )
  target_compile_features(mission_authority_convergence_test PUBLIC cxx_std_17)
  target_include_directories(
    mission_authority_convergence_test
    PRIVATE include
  )
  target_link_libraries(
    mission_authority_convergence_test
    motion_gate_core
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

  ament_add_gtest(
    motion_conditioning_pipeline_test
    test/motion_conditioning_pipeline_test.cpp
  )
  target_compile_features(motion_conditioning_pipeline_test PUBLIC cxx_std_17)
  target_include_directories(
    motion_conditioning_pipeline_test
    PRIVATE include
  )
  ament_target_dependencies(
    motion_conditioning_pipeline_test
    composition_interfaces
    controller_manager_msgs
    lifecycle_msgs
    nav_msgs
    rclcpp
    rosgraph_msgs
    sensor_msgs
  )
  target_link_libraries(
    motion_conditioning_pipeline_test
    motion_conditioning_pipeline
  )

  ament_add_gtest(
    motion_conditioning_pipeline_integration_test
    test/motion_conditioning_pipeline_integration_test.cpp
    src/writer_observation.cpp
  )
  target_compile_features(
    motion_conditioning_pipeline_integration_test PUBLIC cxx_std_17)
  target_include_directories(
    motion_conditioning_pipeline_integration_test
    PRIVATE include src
  )
  ament_target_dependencies(
    motion_conditioning_pipeline_integration_test
    composition_interfaces
    controller_manager_msgs
    geometry_msgs
    lifecycle_msgs
    nav_msgs
    rclcpp
    rclcpp_components
    rosgraph_msgs
    sensor_msgs
    tf2_msgs
  )
  target_link_libraries(
    motion_conditioning_pipeline_integration_test
    motion_conditioning_pipeline
    motion_gate_core
    "${motion_gate_typesupport}"
  )
  set_tests_properties(
    motion_conditioning_pipeline_integration_test
    PROPERTIES
      ENVIRONMENT
        "RMW_IMPLEMENTATION=rmw_fastrtps_cpp;ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST"
      RUN_SERIAL TRUE
  )

  add_launch_test(
    test/test_motion_gate_node.py
    TIMEOUT 60
    RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py"
  )
  add_launch_test(
    test/test_mission_runtime_node.py
    TIMEOUT 60
    RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py"
  )
  add_launch_test(
    test/test_mission_runtime_admission_barrier.py
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
    test_test_mission_runtime_node.py
    PROPERTIES
      ENVIRONMENT
        "RMW_IMPLEMENTATION=rmw_fastrtps_cpp;ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST"
      RUN_SERIAL TRUE
  )
  set_tests_properties(
    test_test_mission_runtime_admission_barrier.py
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
  set_tests_properties(
    test_test_mission_runtime_node.py
    PROPERTIES
      ENVIRONMENT_MODIFICATION
        "ROS_DOMAIN_ID=unset:;DISABLE_ROS_ISOLATION=unset:"
  )
  set_tests_properties(
    test_test_mission_runtime_admission_barrier.py
    PROPERTIES
      ENVIRONMENT_MODIFICATION
        "ROS_DOMAIN_ID=unset:;DISABLE_ROS_ISOLATION=unset:"
  )
endif()

install(
  TARGETS motion_gate_node mission_runtime_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
