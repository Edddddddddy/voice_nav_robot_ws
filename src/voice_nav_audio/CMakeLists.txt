cmake_minimum_required(VERSION 3.8)
project(voice_nav_audio)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(voice_nav_interfaces REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(audio_engine_callback OBJECT src/audio_engine_callback.cpp)
add_library(audio_engine_core OBJECT src/audio_engine.cpp)
add_library(dsp_pipeline STATIC src/dsp_pipeline.cpp)
add_library(speech_input_core STATIC src/speech_input_core.cpp)
add_library(sensevoice_provider STATIC src/sensevoice_provider.cpp)
add_library(speech_input_node STATIC src/speech_input_node.cpp)
add_library(speech_output_core STATIC src/speech_output_core.cpp)
add_library(speech_output_node STATIC src/speech_output_node.cpp)
add_library(voice_pipeline_coordination STATIC src/voice_pipeline_coordination.cpp)
add_library(voice_pipeline STATIC src/voice_pipeline.cpp)
add_library(ros_stop_mission_port STATIC src/ros_stop_mission_port.cpp)
add_library(chaowen_tts_adapter STATIC src/chaowen_tts_adapter.cpp)
add_library(
  audio_engine STATIC
  $<TARGET_OBJECTS:audio_engine_core>
  $<TARGET_OBJECTS:audio_engine_callback>
)
add_library(portaudio_adapter_callback OBJECT src/portaudio_adapter_callback.cpp)
add_library(
  portaudio_adapter STATIC
  src/portaudio_adapter.cpp
  $<TARGET_OBJECTS:portaudio_adapter_callback>
)
foreach(target
    audio_engine
    audio_engine_core
    audio_engine_callback
    dsp_pipeline
    speech_input_core
    sensevoice_provider
    speech_input_node
    speech_output_core
    speech_output_node
    voice_pipeline_coordination
    voice_pipeline
    ros_stop_mission_port
    chaowen_tts_adapter
    portaudio_adapter
    portaudio_adapter_callback)
  target_compile_features(${target} PUBLIC cxx_std_17)
  target_include_directories(
    ${target}
    PUBLIC
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  )
endforeach()
target_link_libraries(sensevoice_provider speech_input_core)
ament_target_dependencies(speech_input_node rclcpp voice_nav_interfaces)
target_link_libraries(speech_input_node speech_input_core)
target_link_libraries(speech_output_core audio_engine)
ament_target_dependencies(speech_output_node rclcpp rclcpp_action voice_nav_interfaces)
target_link_libraries(speech_output_node speech_output_core)
target_link_libraries(voice_pipeline_coordination speech_input_core speech_output_core)
ament_target_dependencies(voice_pipeline rclcpp rclcpp_action voice_nav_interfaces)
ament_target_dependencies(ros_stop_mission_port rclcpp voice_nav_interfaces)
target_link_libraries(ros_stop_mission_port voice_pipeline_coordination)
target_link_libraries(
  voice_pipeline
  speech_input_node
  speech_output_node
  voice_pipeline_coordination
  ros_stop_mission_port
  portaudio_adapter
)
target_link_libraries(chaowen_tts_adapter speech_output_core)
target_link_libraries(portaudio_adapter PUBLIC audio_engine)

# This executable is deliberately limited to the installed, simulation-only
# scripted demo.  It composes the existing private VoicePipeline rather than
# creating a second voice or motion control path.
add_executable(scripted_voice_demo src/scripted_voice_demo.cpp)
target_compile_features(scripted_voice_demo PRIVATE cxx_std_17)
target_include_directories(scripted_voice_demo PRIVATE include src)
ament_target_dependencies(
  scripted_voice_demo
  action_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  rcl_interfaces
  rosgraph_msgs
  voice_nav_interfaces
)
target_link_libraries(scripted_voice_demo voice_pipeline)

install(TARGETS scripted_voice_demo DESTINATION lib/${PROJECT_NAME})

option(
  VOICE_NAV_AUDIO_WITH_WEBRTC_APM
  "Build the WebRTC APM adapter from the explicitly provisioned #53 dependency"
  OFF
)
if(VOICE_NAV_AUDIO_WITH_WEBRTC_APM)
  set(
    VOICE_NAV_WEBRTC_APM_PREFIX "" CACHE PATH
    "Canonical #53 WebRTC APM and Abseil provenance receipt prefix"
  )
  include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/webrtc_apm_prefix_validation.cmake")
  voice_nav_validate_webrtc_apm_prefix(
    "${VOICE_NAV_WEBRTC_APM_PREFIX}" WEBRTC_APM_INCLUDE_DIR WEBRTC_APM_LIBRARY)
  find_package(Threads REQUIRED)
  add_library(webrtc_apm_adapter STATIC src/webrtc_apm_adapter.cpp)
  target_compile_features(webrtc_apm_adapter PUBLIC cxx_std_17)
  target_include_directories(
    webrtc_apm_adapter
    PRIVATE include src ${WEBRTC_APM_INCLUDE_DIR}
  )
  target_link_libraries(
    webrtc_apm_adapter
    PUBLIC "${WEBRTC_APM_LIBRARY}" Threads::Threads rt
  )
endif()

option(
  VOICE_NAV_AUDIO_WITH_PORTAUDIO
  "Build the PortAudio device backend from an explicitly provisioned dependency"
  OFF
)
if(VOICE_NAV_AUDIO_WITH_PORTAUDIO)
  set(
    VOICE_NAV_PORTAUDIO_PREFIX "" CACHE PATH
    "Canonical #53 PortAudio provenance receipt prefix"
  )
  include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/portaudio_prefix_validation.cmake")
  voice_nav_validate_portaudio_prefix(
    "${VOICE_NAV_PORTAUDIO_PREFIX}" PORTAUDIO_INCLUDE_DIR PORTAUDIO_LIBRARY)
  target_compile_definitions(portaudio_adapter PRIVATE VOICE_NAV_AUDIO_WITH_PORTAUDIO=1)
  target_include_directories(portaudio_adapter PRIVATE "${PORTAUDIO_INCLUDE_DIR}")
  target_link_libraries(portaudio_adapter PRIVATE "${PORTAUDIO_LIBRARY}")
endif()

option(
  VOICE_NAV_AUDIO_WITH_SHERPA_ONNX
  "Build the package-private SenseVoice/Silero adapter from the exact sherpa-onnx prefix"
  OFF
)
if(VOICE_NAV_AUDIO_WITH_SHERPA_ONNX)
  set(
    VOICE_NAV_SHERPA_ONNX_PREFIX "" CACHE PATH
    "Canonical sherpa-onnx v1.13.4 shared-ORT prefix and provenance receipt"
  )
  include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/sherpa_onnx_prefix_validation.cmake")
  voice_nav_validate_sherpa_onnx_prefix(
    "${VOICE_NAV_SHERPA_ONNX_PREFIX}"
    SHERPA_ONNX_INCLUDE_DIR
    SHERPA_ONNX_LIBRARIES
    SHERPA_ONNX_RUNTIME_RPATH)
  add_library(sensevoice_sherpa_adapter STATIC src/sensevoice_sherpa_adapter.cpp)
  target_compile_features(sensevoice_sherpa_adapter PUBLIC cxx_std_17)
  target_compile_definitions(
    sensevoice_sherpa_adapter
    PRIVATE VOICE_NAV_AUDIO_WITH_SHERPA_ONNX=1)
  target_include_directories(
    sensevoice_sherpa_adapter
    PRIVATE include src "${SHERPA_ONNX_INCLUDE_DIR}")
  target_link_libraries(
    sensevoice_sherpa_adapter
    PUBLIC sensevoice_provider ${SHERPA_ONNX_LIBRARIES})
  target_link_options(
    sensevoice_sherpa_adapter
    INTERFACE
      "-Wl,--disable-new-dtags,-rpath,${SHERPA_ONNX_RUNTIME_RPATH}")
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # This tracer bullet launches only test code around the installed Agent.
  # The scripted driver remains a non-installed target below.
  add_launch_test(
    test/test_scripted_voice_agent_launch.py
    TARGET scripted_voice_agent_launch_test
    TIMEOUT 60
  )
  add_launch_test(
    test/test_scripted_voice_agent_launch.py
    TARGET scripted_voice_agent_llm_launch_test
    TIMEOUT 60
  )
  ament_add_gtest(
    audio_engine_test
    test/audio_engine_test.cpp
  )
  target_compile_features(audio_engine_test PUBLIC cxx_std_17)
  target_include_directories(
    audio_engine_test
    PRIVATE include
  )
  target_link_libraries(
    audio_engine_test
    audio_engine
  )

  ament_add_gtest(
    dsp_pipeline_test
    test/dsp_pipeline_test.cpp
  )
  target_compile_features(dsp_pipeline_test PUBLIC cxx_std_17)
  target_include_directories(dsp_pipeline_test PRIVATE include src)
  target_link_libraries(dsp_pipeline_test dsp_pipeline)

  ament_add_gtest(
    speech_input_core_test
    test/speech_input_core_test.cpp
  )
  add_executable(speech_input_identity_probe test/speech_input_identity_probe.cpp)
  target_compile_features(speech_input_identity_probe PRIVATE cxx_std_17)
  target_include_directories(speech_input_identity_probe PRIVATE include src)
  target_link_libraries(speech_input_identity_probe speech_input_core)
  target_compile_features(speech_input_core_test PUBLIC cxx_std_17)
  target_include_directories(speech_input_core_test PRIVATE include src)
  target_link_libraries(speech_input_core_test speech_input_core)
  target_compile_definitions(
    speech_input_core_test PRIVATE
    "SPEECH_INPUT_IDENTITY_PROBE=\"$<TARGET_FILE:speech_input_identity_probe>\"")
  add_dependencies(speech_input_core_test speech_input_identity_probe)

  ament_add_gtest(
    sensevoice_provider_test
    test/sensevoice_provider_test.cpp
  )
  target_compile_features(sensevoice_provider_test PUBLIC cxx_std_17)
  target_include_directories(sensevoice_provider_test PRIVATE include src)
  target_link_libraries(sensevoice_provider_test sensevoice_provider)

  if(VOICE_NAV_AUDIO_WITH_SHERPA_ONNX)
    set(_voice_nav_real_asset_root
      "${CMAKE_CURRENT_SOURCE_DIR}/../../models/weights/voice-assets")
    ament_add_gtest(
      sensevoice_sherpa_adapter_test
      test/sensevoice_sherpa_adapter_test.cpp
    )
    target_compile_features(sensevoice_sherpa_adapter_test PUBLIC cxx_std_17)
    target_include_directories(sensevoice_sherpa_adapter_test PRIVATE include src)
    target_link_libraries(sensevoice_sherpa_adapter_test sensevoice_sherpa_adapter)
    target_link_options(
      sensevoice_sherpa_adapter_test
      PRIVATE
        "-Wl,--disable-new-dtags,-rpath,${SHERPA_ONNX_RUNTIME_RPATH}")
    set_tests_properties(
      sensevoice_sherpa_adapter_test
      PROPERTIES
        ENVIRONMENT
          "VOICE_NAV_SENSEVOICE_VAD_MODEL=${_voice_nav_real_asset_root}/vad/silero_vad.int8.onnx;VOICE_NAV_SENSEVOICE_MODEL=${_voice_nav_real_asset_root}/asr/sensevoice-small-int8-2024-07-17/model.int8.onnx;VOICE_NAV_SENSEVOICE_TOKENS=${_voice_nav_real_asset_root}/asr/sensevoice-small-int8-2024-07-17/tokens.txt")

    # Test-only longitudinal gate: locked WAV -> actual sherpa adapters ->
    # existing SpeechInputNode. It is deliberately not installed or exposed
    # as a ROS executable.
    add_executable(
      sensevoice_real_model_gate
      test/sensevoice_real_model_gate.cpp
    )
    target_compile_features(sensevoice_real_model_gate PRIVATE cxx_std_17)
    target_include_directories(
      sensevoice_real_model_gate
      PRIVATE include src "${SHERPA_ONNX_INCLUDE_DIR}"
    )
    ament_target_dependencies(
      sensevoice_real_model_gate
      rclcpp
      voice_nav_interfaces
    )
    target_link_libraries(
      sensevoice_real_model_gate
      speech_input_node sensevoice_sherpa_adapter
    )
    target_link_options(
      sensevoice_real_model_gate
      PRIVATE
        "-Wl,--disable-new-dtags,-rpath,${SHERPA_ONNX_RUNTIME_RPATH}")

    add_launch_test(
      test/test_real_sensevoice_agent_launch.py
      TARGET real_sensevoice_agent_launch_test
      TIMEOUT 120
    )
    set(_voice_nav_real_report "$ENV{VOICE_NAV_REAL_GATE_REPORT}")
    if(_voice_nav_real_report STREQUAL "")
      set(_voice_nav_real_report
        "${CMAKE_CURRENT_BINARY_DIR}/voice_nav_real_model_gate.json")
    endif()
    set(_voice_nav_real_head "$ENV{VOICE_NAV_REAL_GATE_HEAD}")
    if(_voice_nav_real_head STREQUAL "")
      set(_voice_nav_real_head "unknown")
    endif()
    set_tests_properties(
      real_sensevoice_agent_launch_test
      PROPERTIES
        ENVIRONMENT
          "VOICE_NAV_REAL_MODEL_GATE=$<TARGET_FILE:sensevoice_real_model_gate>;VOICE_NAV_REAL_GATE_REPORT=${_voice_nav_real_report};VOICE_NAV_REAL_GATE_HEAD=${_voice_nav_real_head};VOICE_NAV_SENSEVOICE_VAD_MODEL=${_voice_nav_real_asset_root}/vad/silero_vad.int8.onnx;VOICE_NAV_SENSEVOICE_MODEL=${_voice_nav_real_asset_root}/asr/sensevoice-small-int8-2024-07-17/model.int8.onnx;VOICE_NAV_SENSEVOICE_TOKENS=${_voice_nav_real_asset_root}/asr/sensevoice-small-int8-2024-07-17/tokens.txt;VOICE_NAV_SENSEVOICE_WAV=${_voice_nav_real_asset_root}/asr/sensevoice-small-int8-2024-07-17/test_wavs/zh.wav")
  endif()

  ament_add_gtest(
    speech_output_core_test
    test/speech_output_core_test.cpp
  )
  target_compile_features(speech_output_core_test PUBLIC cxx_std_17)
  target_include_directories(speech_output_core_test PRIVATE include src)
  target_link_libraries(speech_output_core_test speech_output_core)

  ament_add_gtest(
    voice_pipeline_coordination_test
    test/voice_pipeline_coordination_test.cpp
  )
  target_compile_features(voice_pipeline_coordination_test PUBLIC cxx_std_17)
  target_include_directories(voice_pipeline_coordination_test PRIVATE include src)
  target_link_libraries(
    voice_pipeline_coordination_test
    voice_pipeline_coordination
    speech_input_core
    speech_output_core
  )

  ament_add_gtest(
    ros_stop_mission_port_test
    test/ros_stop_mission_port_test.cpp
  )
  target_compile_features(ros_stop_mission_port_test PUBLIC cxx_std_17)
  target_include_directories(ros_stop_mission_port_test PRIVATE include src)
  ament_target_dependencies(ros_stop_mission_port_test rclcpp voice_nav_interfaces)
  target_link_libraries(ros_stop_mission_port_test ros_stop_mission_port)

  ament_add_gtest(
    speech_output_node_test
    test/speech_output_node_test.cpp
  )
  target_compile_features(speech_output_node_test PUBLIC cxx_std_17)
  target_include_directories(speech_output_node_test PRIVATE include src)
  ament_target_dependencies(speech_output_node_test rclcpp rclcpp_action voice_nav_interfaces)
  target_link_libraries(speech_output_node_test speech_output_node)

  ament_add_gtest(
    speech_input_node_test
    test/speech_input_node_test.cpp
  )
  target_compile_features(speech_input_node_test PUBLIC cxx_std_17)
  target_include_directories(speech_input_node_test PRIVATE include src)
  ament_target_dependencies(speech_input_node_test rclcpp voice_nav_interfaces)
  target_link_libraries(speech_input_node_test speech_input_node sensevoice_provider)

  ament_add_gtest(
    voice_pipeline_test
    test/voice_pipeline_test.cpp
  )
  target_compile_features(voice_pipeline_test PUBLIC cxx_std_17)
  target_include_directories(voice_pipeline_test PRIVATE include src)
  ament_target_dependencies(voice_pipeline_test rclcpp rclcpp_action voice_nav_interfaces)
  target_link_libraries(
    voice_pipeline_test
    voice_pipeline
  )

  ament_add_gtest(
    voice_command_gateway_readiness_test
    test/voice_command_gateway_readiness_test.cpp
  )
  target_compile_features(voice_command_gateway_readiness_test PUBLIC cxx_std_17)
  target_include_directories(
    voice_command_gateway_readiness_test
    PRIVATE include src
  )
  ament_target_dependencies(
    voice_command_gateway_readiness_test
    action_msgs
    geometry_msgs
    nav_msgs
    rclcpp
    rcl_interfaces
    rosgraph_msgs
    voice_nav_interfaces
  )
  target_link_libraries(
    voice_command_gateway_readiness_test
    voice_pipeline
  )

  # This executable is intentionally test-only: it links the package-private
  # composition seam and is never installed or registered as a ROS component.
  add_executable(
    scripted_speech_agent_driver
    test/scripted_speech_agent_driver.cpp
  )
  target_compile_features(scripted_speech_agent_driver PRIVATE cxx_std_17)
  target_include_directories(scripted_speech_agent_driver PRIVATE include src)
  ament_target_dependencies(
    scripted_speech_agent_driver
    rclcpp
    voice_nav_interfaces
  )
  target_link_libraries(scripted_speech_agent_driver speech_input_node)

  set_tests_properties(
    scripted_voice_agent_launch_test
    PROPERTIES
      ENVIRONMENT
        "VOICE_NAV_SCRIPTED_SPEECH_DRIVER=$<TARGET_FILE:scripted_speech_agent_driver>"
  )
  set_tests_properties(
    scripted_voice_agent_llm_launch_test
    PROPERTIES
      ENVIRONMENT
        "VOICE_NAV_SCRIPTED_SPEECH_DRIVER=$<TARGET_FILE:scripted_speech_agent_driver>;VOICE_NAV_SCRIPTED_SCENARIO=llm"
  )
  add_test(
    NAME speech_input_install_contract
    COMMAND "${CMAKE_COMMAND}"
      "-DPACKAGE_PREFIX=${CMAKE_INSTALL_PREFIX}"
      -P "${CMAKE_CURRENT_SOURCE_DIR}/test/speech_input_install_contract.cmake"
  )

  if(VOICE_NAV_AUDIO_WITH_WEBRTC_APM)
    add_executable(dsp_webrtc_fixture_runner test/dsp_webrtc_fixture_runner.cpp)
    target_compile_features(dsp_webrtc_fixture_runner PRIVATE cxx_std_17)
    target_include_directories(
      dsp_webrtc_fixture_runner
      PRIVATE include src ${WEBRTC_APM_INCLUDE_DIR}
    )
    target_link_libraries(
      dsp_webrtc_fixture_runner
      PRIVATE dsp_pipeline webrtc_apm_adapter
    )
    add_test(
      NAME dsp_webrtc_fixture_runner
      COMMAND dsp_webrtc_fixture_runner
        "${CMAKE_CURRENT_BINARY_DIR}/dsp_webrtc_fixture_report.json"
    )
    add_test(
      NAME dsp_webrtc_fixture_runner_fails_when_report_unwritable
      COMMAND dsp_webrtc_fixture_runner
        "${CMAKE_CURRENT_BINARY_DIR}/fixture-report-unwritable/missing/report.json"
    )
    set_tests_properties(
      dsp_webrtc_fixture_runner_fails_when_report_unwritable
      PROPERTIES WILL_FAIL TRUE
    )
  endif()

  ament_add_gtest(
    portaudio_adapter_test
    test/portaudio_adapter_test.cpp
  )
  target_compile_features(portaudio_adapter_test PUBLIC cxx_std_17)
  target_include_directories(
    portaudio_adapter_test
    PRIVATE include src
  )
  add_library(audio_engine_callback_test_support OBJECT src/audio_engine_callback.cpp)
  target_compile_definitions(
    audio_engine_callback_test_support
    PRIVATE VOICE_NAV_AUDIO_TEST_CALLBACK_BOUNDARY=1
  )
  add_library(
    audio_engine_test_support STATIC
    src/audio_engine.cpp
    src/audio_engine_callback_test_support.cpp
    $<TARGET_OBJECTS:audio_engine_callback_test_support>
  )
  add_library(
    portaudio_adapter_test_support STATIC
    src/portaudio_adapter.cpp
    $<TARGET_OBJECTS:portaudio_adapter_callback>
  )
  foreach(target
      audio_engine_callback_test_support
      audio_engine_test_support
      portaudio_adapter_test_support)
    target_compile_features(${target} PUBLIC cxx_std_17)
    target_include_directories(${target} PRIVATE include src)
  endforeach()
  target_link_libraries(portaudio_adapter_test_support PUBLIC audio_engine_test_support)
  target_link_libraries(portaudio_adapter_test portaudio_adapter_test_support)

  ament_add_gtest(
    audio_callback_contract_test
    test/audio_callback_contract_test.cpp
  )
  target_compile_features(audio_callback_contract_test PUBLIC cxx_std_17)
  target_include_directories(audio_callback_contract_test PRIVATE include)
  target_link_libraries(audio_callback_contract_test portaudio_adapter)

  add_library(
    audio_callback_link_mutation_helper STATIC
    test/audio_callback_link_mutation_helper.cpp
  )
  add_library(
    audio_callback_link_guard_mutation_helper STATIC
    test/audio_callback_link_guard_mutation_helper.cpp
  )
  add_library(
    audio_callback_link_direct_object_mutation OBJECT
    test/audio_callback_link_provider_mutation.cpp
  )
  add_library(
    audio_callback_link_external_archive_mutation STATIC
    test/audio_callback_link_provider_mutation.cpp
  )
  set_target_properties(
    audio_callback_link_external_archive_mutation
    PROPERTIES
      ARCHIVE_OUTPUT_DIRECTORY
        "${CMAKE_CURRENT_BINARY_DIR}/../audio_callback_link_external_provider"
  )
  add_library(
    portaudio_adapter_callback_mutation OBJECT
    src/portaudio_adapter_callback.cpp
  )
  add_library(
    portaudio_adapter_callback_guard_mutation OBJECT
    src/portaudio_adapter_callback.cpp
  )
  add_library(
    portaudio_adapter_mutation STATIC
    src/portaudio_adapter.cpp
    $<TARGET_OBJECTS:portaudio_adapter_callback_mutation>
  )
  add_library(
    portaudio_adapter_guard_mutation STATIC
    src/portaudio_adapter.cpp
    $<TARGET_OBJECTS:portaudio_adapter_callback_guard_mutation>
  )
  add_library(
    portaudio_adapter_external_archive_mutation STATIC
    src/portaudio_adapter.cpp
    $<TARGET_OBJECTS:portaudio_adapter_callback>
  )
  foreach(target
      audio_callback_link_mutation_helper
      audio_callback_link_guard_mutation_helper
      audio_callback_link_direct_object_mutation
      audio_callback_link_external_archive_mutation
      portaudio_adapter_callback_mutation
      portaudio_adapter_callback_guard_mutation
      portaudio_adapter_mutation
      portaudio_adapter_guard_mutation
      portaudio_adapter_external_archive_mutation)
    target_compile_features(${target} PUBLIC cxx_std_17)
    target_include_directories(${target} PRIVATE include src)
  endforeach()
  target_compile_definitions(
    portaudio_adapter_callback_mutation
    PRIVATE VOICE_NAV_AUDIO_TEST_CALLBACK_LINK_MUTATION=1
  )
  target_compile_definitions(
    portaudio_adapter_callback_guard_mutation
    PRIVATE VOICE_NAV_AUDIO_TEST_CALLBACK_LINK_GUARD_MUTATION=1
  )
  target_link_libraries(
    portaudio_adapter_mutation
    PUBLIC audio_engine audio_callback_link_mutation_helper
  )
  target_link_libraries(
    portaudio_adapter_guard_mutation
    PUBLIC audio_engine audio_callback_link_guard_mutation_helper
  )
  target_link_libraries(
    portaudio_adapter_external_archive_mutation
    PUBLIC audio_engine audio_callback_link_external_archive_mutation
  )

  function(add_callback_link_surface_artifact target adapter_target link_map)
    add_executable(${target} test/audio_callback_link_surface.cpp)
    target_compile_features(${target} PRIVATE cxx_std_17)
    target_include_directories(${target} PRIVATE include src)
    target_link_libraries(${target} PRIVATE ${adapter_target})
    target_link_options(${target} PRIVATE "-Wl,-Map,${link_map}")
  endfunction()

  add_callback_link_surface_artifact(
    audio_callback_link_surface_artifact
    portaudio_adapter
    "${CMAKE_CURRENT_BINARY_DIR}/audio_callback_link_surface.map"
  )
  add_callback_link_surface_artifact(
    audio_callback_link_surface_mutation
    portaudio_adapter_mutation
    "${CMAKE_CURRENT_BINARY_DIR}/audio_callback_link_surface_mutation.map"
  )
  add_callback_link_surface_artifact(
    audio_callback_link_surface_guard_mutation
    portaudio_adapter_guard_mutation
    "${CMAKE_CURRENT_BINARY_DIR}/audio_callback_link_surface_guard_mutation.map"
  )
  add_callback_link_surface_artifact(
    audio_callback_link_surface_direct_object_mutation
    portaudio_adapter
    "${CMAKE_CURRENT_BINARY_DIR}/audio_callback_link_surface_direct_object_mutation.map"
  )
  target_sources(
    audio_callback_link_surface_direct_object_mutation
    PRIVATE $<TARGET_OBJECTS:audio_callback_link_direct_object_mutation>
  )
  add_callback_link_surface_artifact(
    audio_callback_link_surface_external_archive_mutation
    portaudio_adapter_external_archive_mutation
    "${CMAKE_CURRENT_BINARY_DIR}/audio_callback_link_surface_external_archive_mutation.map"
  )

  add_test(
    NAME portaudio_requires_verified_prefix
    COMMAND "${CMAKE_COMMAND}"
      "-DSOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}"
      "-DBINARY_DIR=${CMAKE_CURRENT_BINARY_DIR}/portaudio-prefix-contract"
      -DTEST_MODE=prefix
      -P "${CMAKE_CURRENT_SOURCE_DIR}/test/portaudio_prefix_contract.cmake"
  )
  add_test(
    NAME portaudio_adapter_builds_from_verified_prefix
    COMMAND "${CMAKE_COMMAND}"
      "-DSOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}"
      "-DBINARY_DIR=${CMAKE_CURRENT_BINARY_DIR}/portaudio-backend-contract"
      -DTEST_MODE=backend
      -P "${CMAKE_CURRENT_SOURCE_DIR}/test/portaudio_prefix_contract.cmake"
  )
  add_test(
    NAME audio_callback_link_surface
    COMMAND "${CMAKE_COMMAND}"
      "-DNM_EXECUTABLE=${CMAKE_NM}"
      "-DCALLBACK_LINK_MAP=${CMAKE_CURRENT_BINARY_DIR}/audio_callback_link_surface.map"
      "-DCALLBACK_LINK_DIRECTORY=${CMAKE_CURRENT_BINARY_DIR}"
      "-DCALLBACK_NATIVE_ROOT=_ZN15voice_nav_audio25native_portaudio_callbackEPKvPvmPK24PaStreamCallbackTimeInfomS2_"
      "-DCALLBACK_ADAPTER_ROOT=_ZN15voice_nav_audio16PortAudioAdapter8callbackEPvPKsPsmNS_14CallbackStatusE"
      "-DCALLBACK_ENGINE_ROOT=_ZN15voice_nav_audio11AudioEngine16process_callbackEPKsPsmNS_14CallbackStatusE"
      -P "${CMAKE_CURRENT_SOURCE_DIR}/test/audio_callback_link_contract.cmake"
  )
  add_test(
    NAME audio_callback_link_mutations
    COMMAND "${CMAKE_COMMAND}"
      "-DCONTRACT_SCRIPT=${CMAKE_CURRENT_SOURCE_DIR}/test/audio_callback_link_contract.cmake"
      "-DFIXTURE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/test/callback_link_fixtures"
      "-DNM_EXECUTABLE=${CMAKE_NM}"
      "-DMUTATION_CALLBACK_ARTIFACT=$<TARGET_FILE:audio_callback_link_surface_mutation>"
      "-DMUTATION_CALLBACK_LINK_MAP=${CMAKE_CURRENT_BINARY_DIR}/audio_callback_link_surface_mutation.map"
      "-DMUTATION_CALLBACK_LINK_DIRECTORY=${CMAKE_CURRENT_BINARY_DIR}"
      "-DGUARD_MUTATION_CALLBACK_ARTIFACT=$<TARGET_FILE:audio_callback_link_surface_guard_mutation>"
      "-DGUARD_MUTATION_CALLBACK_LINK_MAP=${CMAKE_CURRENT_BINARY_DIR}/audio_callback_link_surface_guard_mutation.map"
      "-DGUARD_MUTATION_CALLBACK_LINK_DIRECTORY=${CMAKE_CURRENT_BINARY_DIR}"
      "-DGUARD_MUTATION_CALLBACK_PROVIDER=$<TARGET_FILE:audio_callback_link_guard_mutation_helper>"
      "-DDIRECT_OBJECT_MUTATION_CALLBACK_ARTIFACT=$<TARGET_FILE:audio_callback_link_surface_direct_object_mutation>"
      "-DDIRECT_OBJECT_MUTATION_CALLBACK_LINK_MAP=${CMAKE_CURRENT_BINARY_DIR}/audio_callback_link_surface_direct_object_mutation.map"
      "-DDIRECT_OBJECT_MUTATION_CALLBACK_LINK_DIRECTORY=${CMAKE_CURRENT_BINARY_DIR}"
      "-DDIRECT_OBJECT_MUTATION_CALLBACK_PROVIDER=$<TARGET_OBJECTS:audio_callback_link_direct_object_mutation>"
      "-DEXTERNAL_ARCHIVE_MUTATION_CALLBACK_ARTIFACT=$<TARGET_FILE:audio_callback_link_surface_external_archive_mutation>"
      "-DEXTERNAL_ARCHIVE_MUTATION_CALLBACK_LINK_MAP=${CMAKE_CURRENT_BINARY_DIR}/audio_callback_link_surface_external_archive_mutation.map"
      "-DEXTERNAL_ARCHIVE_MUTATION_CALLBACK_LINK_DIRECTORY=${CMAKE_CURRENT_BINARY_DIR}"
      "-DEXTERNAL_ARCHIVE_MUTATION_CALLBACK_PROVIDER=$<TARGET_FILE:audio_callback_link_external_archive_mutation>"
      "-DCALLBACK_NATIVE_ROOT=_ZN15voice_nav_audio25native_portaudio_callbackEPKvPvmPK24PaStreamCallbackTimeInfomS2_"
      "-DCALLBACK_ADAPTER_ROOT=_ZN15voice_nav_audio16PortAudioAdapter8callbackEPvPKsPsmNS_14CallbackStatusE"
      "-DCALLBACK_ENGINE_ROOT=_ZN15voice_nav_audio11AudioEngine16process_callbackEPKsPsmNS_14CallbackStatusE"
      -P "${CMAKE_CURRENT_SOURCE_DIR}/test/audio_callback_link_mutation_test.cmake"
  )
endif()

ament_package()
