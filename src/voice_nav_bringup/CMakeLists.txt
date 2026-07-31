cmake_minimum_required(VERSION 3.8)
project(voice_nav_bringup)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ament_lint_auto_find_test_dependencies()

  ament_add_pytest_test(
    motion_gate_open_convergence_test
    test/test_motion_gate_open_convergence.py
    TIMEOUT 30
  )

  add_launch_test(
    test/test_motion_gate_product.py
    TIMEOUT 180
  )
  set_tests_properties(
    test_test_motion_gate_product.py
    PROPERTIES
      ENVIRONMENT
        "RMW_IMPLEMENTATION=rmw_fastrtps_cpp;ROS_DOMAIN_ID=92;ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST;GZ_PARTITION=voice_nav_l0009_product_test"
      RUN_SERIAL TRUE
  )
endif()

install(
  DIRECTORY
    config
    launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

ament_package()
