cmake_minimum_required(VERSION 3.22)
project(voice_nav_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_ros REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rmw REQUIRED)
  find_package(tf2_msgs REQUIRED)

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
  target_include_directories(
    tf_ownership_evaluator_test
    PRIVATE test
  )

  ament_add_pytest_test(
    scenario_spec
    test/test_scenario_spec.py
    TIMEOUT 30
  )
  add_launch_test(
    test/test_tf_ownership_conflict.py
    TIMEOUT 30
    RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py"
  )

  set_tests_properties(
    test_test_tf_ownership_conflict.py
    PROPERTIES
      ENVIRONMENT
        "RMW_IMPLEMENTATION=rmw_fastrtps_cpp;ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST"
      RUN_SERIAL TRUE
  )
  set_tests_properties(
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

ament_python_install_package(
  voice_nav_sim
  PACKAGE_DIR python/voice_nav_sim
)

ament_package()
