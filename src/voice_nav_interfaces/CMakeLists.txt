cmake_minimum_required(VERSION 3.8)
project(voice_nav_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MissionStep.msg"
  "msg/MissionState.msg"
  "msg/VoiceTurn.msg"
  "action/ExecuteMission.action"
  "srv/StopMission.srv"
)


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(rosidl_typesupport_introspection_cpp REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_pytest_test(
    mission_interface_python_contract
    test/mission_interface_python_contract.py
  )

  ament_add_gtest(
    mission_interface_cpp_contract
    test/mission_interface_cpp_contract.cpp
  )
  target_compile_features(mission_interface_cpp_contract PUBLIC cxx_std_17)
  rosidl_get_typesupport_target(
    mission_interface_typesupport_target
    ${PROJECT_NAME}
    rosidl_typesupport_cpp
  )
  target_link_libraries(
    mission_interface_cpp_contract
    "${mission_interface_typesupport_target}"
  )

  ament_add_pytest_test(
    voice_interface_python_contract
    test/voice_interface_python_contract.py
  )

  ament_add_gtest(
    voice_interface_cpp_contract
    test/voice_interface_cpp_contract.cpp
  )
  target_compile_features(voice_interface_cpp_contract PUBLIC cxx_std_17)
  rosidl_get_typesupport_target(
    voice_interface_typesupport_target
    ${PROJECT_NAME}
    rosidl_typesupport_introspection_cpp
  )
  target_link_libraries(
    voice_interface_cpp_contract
    "${voice_interface_typesupport_target}"
  )
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
