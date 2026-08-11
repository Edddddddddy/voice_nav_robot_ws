cmake_minimum_required(VERSION 3.22)
project(voice_nav_audio)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Threads REQUIRED)
pkg_check_modules(PORTAUDIO REQUIRED IMPORTED_TARGET portaudio-2.0)
pkg_check_modules(
  WEBRTC_APM REQUIRED IMPORTED_TARGET webrtc-audio-processing
)

add_executable(audio_engine_node src/audio_engine_node.cpp)
target_compile_features(audio_engine_node PUBLIC cxx_std_17)
target_include_directories(audio_engine_node PUBLIC include)
ament_target_dependencies(audio_engine_node rclcpp)
target_link_libraries(
  audio_engine_node
  PkgConfig::PORTAUDIO
  PkgConfig::WEBRTC_APM
  Threads::Threads
)
target_compile_definitions(
  audio_engine_node PRIVATE
  "VOICE_NAV_WEBRTC_APM_VERSION=\"${WEBRTC_APM_VERSION}\""
)

install(TARGETS audio_engine_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(spsc_audio_ring_test test/spsc_audio_ring_test.cpp)
  target_include_directories(spsc_audio_ring_test PRIVATE include)
endif()

ament_package()
