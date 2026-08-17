# #53 canonical receipt contract.  Each fixture configures the real package;
# it does not inspect CMake source or depend on a system PortAudio install.
set(portaudio_lock
  "revision=147dd722548358763a8b649b3e4b41dfffbcfbb6\nsource_sha256=95457b809ce60d4d4790f84bb692e271f644e59d8adf96feb988c89ab52a506a\nshared=OFF\n")
set(canonical_receipt
  "schema_version=1\nid=portaudio\nversion=v19.7.0\nrevision=147dd722548358763a8b649b3e4b41dfffbcfbb6\nsource_sha256=95457b809ce60d4d4790f84bb692e271f644e59d8adf96feb988c89ab52a506a\nbuild_system=CMake\nPA_BUILD_SHARED=OFF\nPA_BUILD_TESTS=OFF\ntarget=portaudio\ninclude_dir=include\nlibrary=lib/libportaudio.a\n")

file(REMOVE_RECURSE "${BINARY_DIR}")
file(MAKE_DIRECTORY "${BINARY_DIR}")
find_program(portaudio_fixture_compiler NAMES cc gcc clang)
find_program(portaudio_fixture_archiver NAMES ar llvm-ar gcc-ar)
if(NOT portaudio_fixture_compiler OR NOT portaudio_fixture_archiver)
  message(FATAL_ERROR "PortAudio prefix contract needs a C compiler and static archiver")
endif()
set(fixture_source_dir "${BINARY_DIR}/fixture-source")
file(MAKE_DIRECTORY "${fixture_source_dir}")
if(TEST_MODE STREQUAL "backend")
file(WRITE "${fixture_source_dir}/CMakeLists.txt" [=[
cmake_minimum_required(VERSION 3.8)
project(portaudio_prefix_fixture C CXX)
include("${VOICE_NAV_AUDIO_SOURCE_DIR}/cmake/portaudio_prefix_validation.cmake")
voice_nav_validate_portaudio_prefix(
  "${VOICE_NAV_PORTAUDIO_PREFIX}" PORTAUDIO_INCLUDE_DIR PORTAUDIO_LIBRARY)
add_library(audio_engine_callback OBJECT
  "${VOICE_NAV_AUDIO_SOURCE_DIR}/src/audio_engine_callback.cpp")
add_library(audio_engine STATIC
  "${VOICE_NAV_AUDIO_SOURCE_DIR}/src/audio_engine.cpp"
  $<TARGET_OBJECTS:audio_engine_callback>)
add_library(portaudio_adapter_callback OBJECT
  "${VOICE_NAV_AUDIO_SOURCE_DIR}/src/portaudio_adapter_callback.cpp")
add_library(portaudio_adapter STATIC
  "${VOICE_NAV_AUDIO_SOURCE_DIR}/src/portaudio_adapter.cpp"
  $<TARGET_OBJECTS:portaudio_adapter_callback>)
foreach(target
    audio_engine
    audio_engine_callback
    portaudio_adapter
    portaudio_adapter_callback)
  target_compile_features(${target} PUBLIC cxx_std_17)
  target_include_directories(${target} PRIVATE
    "${VOICE_NAV_AUDIO_SOURCE_DIR}/include")
endforeach()
target_compile_definitions(portaudio_adapter PRIVATE VOICE_NAV_AUDIO_WITH_PORTAUDIO=1)
target_include_directories(portaudio_adapter PRIVATE "${PORTAUDIO_INCLUDE_DIR}")
target_link_libraries(portaudio_adapter PRIVATE "${PORTAUDIO_LIBRARY}")
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  target_link_libraries(portaudio_adapter PRIVATE m)
endif()
target_link_libraries(portaudio_adapter PUBLIC audio_engine)
add_executable(portaudio_adapter_backend_fixture portaudio_adapter_backend_fixture.cpp)
target_compile_features(portaudio_adapter_backend_fixture PRIVATE cxx_std_17)
target_include_directories(portaudio_adapter_backend_fixture PRIVATE
  "${VOICE_NAV_AUDIO_SOURCE_DIR}/include"
  "${PORTAUDIO_INCLUDE_DIR}")
target_link_libraries(portaudio_adapter_backend_fixture PRIVATE
  portaudio_adapter
  "${PORTAUDIO_LIBRARY}")
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  target_link_libraries(portaudio_adapter_backend_fixture PRIVATE m)
endif()
]=])
else()
file(WRITE "${fixture_source_dir}/CMakeLists.txt" [=[
cmake_minimum_required(VERSION 3.8)
project(portaudio_prefix_fixture C)
include("${VOICE_NAV_AUDIO_SOURCE_DIR}/cmake/portaudio_prefix_validation.cmake")
voice_nav_validate_portaudio_prefix(
  "${VOICE_NAV_PORTAUDIO_PREFIX}" PORTAUDIO_INCLUDE_DIR PORTAUDIO_LIBRARY)
add_library(portaudio_adapter STATIC portaudio_adapter_probe.c)
target_include_directories(portaudio_adapter PRIVATE "${PORTAUDIO_INCLUDE_DIR}")
target_link_libraries(portaudio_adapter PRIVATE "${PORTAUDIO_LIBRARY}")
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  target_link_libraries(portaudio_adapter PRIVATE m)
endif()
]=])
file(WRITE "${fixture_source_dir}/portaudio_adapter_probe.c" [=[
#include <portaudio.h>

int portaudio_adapter_probe(void)
{
  return Pa_GetVersion();
}
]=])
endif()

if(TEST_MODE STREQUAL "backend")
file(WRITE "${fixture_source_dir}/portaudio_adapter_backend_fixture.cpp" [=[
#include <algorithm>

#include <portaudio.h>

#include "voice_nav_audio/portaudio_adapter.hpp"

extern "C" int voice_nav_portaudio_fixture_open_arguments_are_exact(void);
extern "C" int voice_nav_portaudio_fixture_fire_callback(PaStreamCallbackFlags status_flags);
extern "C" int voice_nav_portaudio_fixture_output_sample(unsigned long index);
extern "C" int voice_nav_portaudio_fixture_lifecycle_is_complete(void);

int main()
{
  voice_nav_audio::AudioEngine engine;
  voice_nav_audio::PortAudioAdapter adapter(engine);
  if(adapter.start() != voice_nav_audio::AdapterStartResult::Started || !adapter.running()) {
    return 1;
  }
  if(!voice_nav_portaudio_fixture_open_arguments_are_exact()) {
    return 2;
  }
  if(voice_nav_portaudio_fixture_fire_callback(0U) != paContinue) {
    return 3;
  }

  voice_nav_audio::AudioFrame capture{};
  voice_nav_audio::AudioFrame reference{};
  if(!engine.try_pop_capture(capture) || !engine.try_pop_reference(reference)) {
    return 4;
  }
  for(std::size_t index = 0U; index < capture.samples.size(); ++index) {
    if(capture.samples[index] != static_cast<voice_nav_audio::Sample>(index + 1U)) {
      return 5;
    }
    if(voice_nav_portaudio_fixture_output_sample(static_cast<unsigned long>(index)) !=
      reference.samples[index])
    {
      return 6;
    }
  }
  if(!std::all_of(reference.samples.begin(), reference.samples.end(),
      [](const voice_nav_audio::Sample sample) {return sample == 0;}))
  {
    return 7;
  }
  if(capture.generation != engine.generation() || reference.generation != engine.generation()) {
    return 8;
  }

  const auto generation_before_xrun = engine.generation();
  if(voice_nav_portaudio_fixture_fire_callback(paOutputUnderflow) != paContinue) {
    return 9;
  }
  if(engine.metrics().xruns != 1U || engine.generation() <= generation_before_xrun) {
    return 10;
  }
  for(std::size_t index = 0U; index < reference.samples.size(); ++index) {
    if(voice_nav_portaudio_fixture_output_sample(static_cast<unsigned long>(index)) != 0) {
      return 11;
    }
  }

  adapter.stop();
  if(adapter.running() || !voice_nav_portaudio_fixture_lifecycle_is_complete()) {
    return 12;
  }
  return 0;
}
]=])
endif()

function(write_prefix prefix receipt)
  file(MAKE_DIRECTORY
    "${prefix}/include"
    "${prefix}/lib"
    "${prefix}/share/voice_nav")
  file(WRITE "${prefix}/include/portaudio.h"
    "#ifndef PORTAUDIO_H\n#define PORTAUDIO_H\n#ifdef __cplusplus\nextern \"C\" {\n#endif\ntypedef struct PaStream PaStream;\ntypedef unsigned long PaStreamCallbackFlags;\ntypedef struct PaStreamCallbackTimeInfo { double input_buffer_adc_time; double current_time; double output_buffer_dac_time; } PaStreamCallbackTimeInfo;\ntypedef int PaStreamCallback(const void *, void *, unsigned long, const PaStreamCallbackTimeInfo *, PaStreamCallbackFlags, void *);\nenum { paNoError = 0, paContinue = 0, paInt16 = 8, paInputUnderflow = 0x00000001, paInputOverflow = 0x00000002, paOutputUnderflow = 0x00000004, paOutputOverflow = 0x00000008 };\nint Pa_GetVersion(void);\nint Pa_Initialize(void);\nint Pa_Terminate(void);\nint Pa_OpenDefaultStream(PaStream **, int, int, unsigned long, double, unsigned long, PaStreamCallback *, void *);\nint Pa_StartStream(PaStream *);\nint Pa_StopStream(PaStream *);\nint Pa_CloseStream(PaStream *);\n#ifdef __cplusplus\n}\n#endif\n#endif\n")
  set(archive_source "${prefix}/portaudio_fixture.c")
  set(archive_object "${prefix}/portaudio_fixture.o")
  file(WRITE "${archive_source}" [=[
#include <math.h>
#include <portaudio.h>

struct PaStream
{
  int open;
};

static struct PaStream fixture_stream;
static PaStreamCallback * fixture_callback;
static void * fixture_callback_context;
static int initialize_calls;
static int open_calls;
static int start_calls;
static int stop_calls;
static int close_calls;
static int terminate_calls;
static int stream_started;
static int opened_input_channels;
static int opened_output_channels;
static unsigned long opened_sample_format;
static double opened_sample_rate;
static unsigned long opened_frames_per_buffer;
static short callback_output[480];

int Pa_GetVersion(void)
{
  volatile double fixture_version = 1.25;
  return (int)floor(fixture_version);
}

int Pa_Initialize(void)
{
  ++initialize_calls;
  return paNoError;
}

int Pa_Terminate(void)
{
  ++terminate_calls;
  return paNoError;
}

int Pa_OpenDefaultStream(
  PaStream ** stream, int input_channels, int output_channels, unsigned long sample_format,
  double sample_rate, unsigned long frames_per_buffer,
  PaStreamCallback * callback, void * context)
{
  ++open_calls;
  opened_input_channels = input_channels;
  opened_output_channels = output_channels;
  opened_sample_format = sample_format;
  opened_sample_rate = sample_rate;
  opened_frames_per_buffer = frames_per_buffer;
  fixture_stream.open = 1;
  fixture_callback = callback;
  fixture_callback_context = context;
  *stream = &fixture_stream;
  return paNoError;
}

int Pa_StartStream(PaStream * stream)
{
  if(stream != &fixture_stream) {
    return -1;
  }
  ++start_calls;
  stream_started = 1;
  return paNoError;
}

int Pa_StopStream(PaStream * stream)
{
  if(stream != &fixture_stream) {
    return -1;
  }
  ++stop_calls;
  stream_started = 0;
  return paNoError;
}

int Pa_CloseStream(PaStream * stream)
{
  if(stream != &fixture_stream) {
    return -1;
  }
  ++close_calls;
  fixture_stream.open = 0;
  return paNoError;
}

int voice_nav_portaudio_fixture_open_arguments_are_exact(void)
{
  return opened_input_channels == 1 && opened_output_channels == 1 &&
    opened_sample_format == paInt16 && opened_sample_rate == 48000.0 &&
    opened_frames_per_buffer == 480U;
}

int voice_nav_portaudio_fixture_fire_callback(PaStreamCallbackFlags status_flags)
{
  short input[480];
  short output[480];
  unsigned long index;
  if(fixture_callback == 0 || !stream_started) {
    return -1;
  }
  for(index = 0U; index < 480U; ++index) {
    input[index] = (short)(index + 1U);
    output[index] = 123;
  }
  {
    int result = fixture_callback(input, output, 480U, 0, status_flags, fixture_callback_context);
    for(index = 0U; index < 480U; ++index) {
      callback_output[index] = output[index];
    }
    return result;
  }
}

int voice_nav_portaudio_fixture_output_sample(unsigned long index)
{
  if(index >= 480U) {
    return -32769;
  }
  return callback_output[index];
}

int voice_nav_portaudio_fixture_lifecycle_is_complete(void)
{
  return initialize_calls == 1 && open_calls == 1 && start_calls == 1 && stop_calls == 1 && close_calls == 1 && terminate_calls == 1;
}
]=])
  execute_process(
    COMMAND "${portaudio_fixture_compiler}" -I "${prefix}/include" -c "${archive_source}"
      -o "${archive_object}"
    RESULT_VARIABLE compile_result
    OUTPUT_VARIABLE compile_output
    ERROR_VARIABLE compile_error)
  if(NOT compile_result EQUAL 0)
    message(FATAL_ERROR "could not compile PortAudio archive fixture:\n${compile_output}\n${compile_error}")
  endif()
  execute_process(
    COMMAND "${portaudio_fixture_archiver}" rcs "${prefix}/lib/libportaudio.a" "${archive_object}"
    RESULT_VARIABLE archive_result
    OUTPUT_VARIABLE archive_output
    ERROR_VARIABLE archive_error)
  if(NOT archive_result EQUAL 0)
    message(FATAL_ERROR "could not create PortAudio archive fixture:\n${archive_output}\n${archive_error}")
  endif()
  file(WRITE "${prefix}/share/voice_nav/portaudio.lock" "${portaudio_lock}")
  file(WRITE "${prefix}/share/voice_nav/portaudio-provenance.receipt" "${receipt}")
endfunction()

function(configure_fixture label prefix result output error)
  set(extra_arguments ${ARGN})
  execute_process(
    COMMAND "${CMAKE_COMMAND}"
      -S "${fixture_source_dir}"
      -B "${BINARY_DIR}/${label}"
      "-DVOICE_NAV_AUDIO_SOURCE_DIR=${SOURCE_DIR}"
      "-DVOICE_NAV_PORTAUDIO_PREFIX=${prefix}"
      ${extra_arguments}
    RESULT_VARIABLE configure_result
    OUTPUT_VARIABLE configure_output
    ERROR_VARIABLE configure_error
  )
  set(${result} "${configure_result}" PARENT_SCOPE)
  set(${output} "${configure_output}" PARENT_SCOPE)
  set(${error} "${configure_error}" PARENT_SCOPE)
endfunction()

function(expect_accepted label prefix)
  configure_fixture("${label}" "${prefix}" result output error ${ARGN})
  if(NOT result EQUAL 0)
    message(FATAL_ERROR "${label} was rejected:\n${output}\n${error}")
  endif()
  if(TEST_MODE STREQUAL "backend")
    execute_process(
      COMMAND "${CMAKE_COMMAND}" --build "${BINARY_DIR}/${label}"
        --target portaudio_adapter_backend_fixture
      RESULT_VARIABLE build_result
      OUTPUT_VARIABLE build_output
      ERROR_VARIABLE build_error)
    if(NOT build_result EQUAL 0)
      message(FATAL_ERROR "${label} could not link the PortAudio backend fixture:\n${build_output}\n${build_error}")
    endif()
    execute_process(
      COMMAND "${BINARY_DIR}/${label}/portaudio_adapter_backend_fixture"
      RESULT_VARIABLE run_result
      OUTPUT_VARIABLE run_output
      ERROR_VARIABLE run_error)
    if(NOT run_result EQUAL 0)
      message(FATAL_ERROR "${label} PortAudio backend fixture failed:\n${run_output}\n${run_error}")
    endif()
  endif()
endfunction()

function(expect_rejected label prefix reason)
  configure_fixture("${label}" "${prefix}" result output error ${ARGN})
  if(result EQUAL 0)
    message(FATAL_ERROR "${label} was accepted")
  endif()
  set(configure_log "${output}\n${error}")
  if(NOT configure_log MATCHES "${reason}")
    message(FATAL_ERROR "${label} failed for an unexpected reason:\n${configure_log}")
  endif()
endfunction()

set(positive_prefix "${BINARY_DIR}/positive-prefix")
write_prefix("${positive_prefix}" "${canonical_receipt}")
expect_accepted("positive" "${positive_prefix}")
if(TEST_MODE STREQUAL "backend")
  return()
endif()

set(text_archive_prefix "${BINARY_DIR}/text-archive-prefix")
write_prefix("${text_archive_prefix}" "${canonical_receipt}")
file(WRITE "${text_archive_prefix}/lib/libportaudio.a" "fixture static archive\n")
expect_rejected("text-archive" "${text_archive_prefix}" "static archive")

set(outside_prefix "${BINARY_DIR}/outside-prefix")
write_prefix("${outside_prefix}" "${canonical_receipt}")
set(forged_prefix "${BINARY_DIR}/forged-prefix")
write_prefix("${forged_prefix}" "${canonical_receipt}")
file(REMOVE "${forged_prefix}/lib/libportaudio.a")
file(CREATE_LINK
  "${outside_prefix}/lib/libportaudio.a"
  "${forged_prefix}/lib/libportaudio.a"
  SYMBOLIC)
expect_rejected("forged" "${forged_prefix}" "must resolve inside")

set(header_forged_prefix "${BINARY_DIR}/header-forged-prefix")
write_prefix("${header_forged_prefix}" "${canonical_receipt}")
file(REMOVE "${header_forged_prefix}/include/portaudio.h")
file(CREATE_LINK
  "${outside_prefix}/include/portaudio.h"
  "${header_forged_prefix}/include/portaudio.h"
  SYMBOLIC)
expect_rejected("header-forged" "${header_forged_prefix}" "must resolve inside")

set(cached_prefix "${BINARY_DIR}/cached-prefix")
write_prefix("${cached_prefix}" "${canonical_receipt}")
expect_rejected(
  "cached" "${cached_prefix}" "cache injection"
  "-DPORTAUDIO_INCLUDE_DIR=${outside_prefix}/include"
  "-DPORTAUDIO_LIBRARY=${outside_prefix}/lib/libportaudio.a")

string(REPLACE "target=portaudio" "target=portaudio-forged" wrong_identity_receipt "${canonical_receipt}")
set(wrong_identity_prefix "${BINARY_DIR}/wrong-identity-prefix")
write_prefix("${wrong_identity_prefix}" "${wrong_identity_receipt}")
expect_rejected("wrong-identity" "${wrong_identity_prefix}" "canonical provenance receipt")

set(wrong_lock_prefix "${BINARY_DIR}/wrong-lock-prefix")
write_prefix("${wrong_lock_prefix}" "${canonical_receipt}")
file(WRITE "${wrong_lock_prefix}/share/voice_nav/portaudio.lock"
  "revision=forged\nsource_sha256=95457b809ce60d4d4790f84bb692e271f644e59d8adf96feb988c89ab52a506a\nshared=OFF\n")
expect_rejected("wrong-lock" "${wrong_lock_prefix}" "source lock metadata")
