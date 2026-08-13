cmake_minimum_required(VERSION 3.8)
project(voice_nav_audio)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(audio_engine_callback OBJECT src/audio_engine_callback.cpp)
add_library(audio_engine_core OBJECT src/audio_engine.cpp)
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
    portaudio_adapter
    portaudio_adapter_callback)
  target_compile_features(${target} PUBLIC cxx_std_17)
  target_include_directories(
    ${target}
    PUBLIC
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  )
endforeach()
target_link_libraries(portaudio_adapter PUBLIC audio_engine)

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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

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
