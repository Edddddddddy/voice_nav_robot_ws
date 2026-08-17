function(voice_nav_validate_portaudio_prefix prefix include_variable library_variable)
  if(NOT IS_ABSOLUTE "${prefix}")
    message(FATAL_ERROR
      "VOICE_NAV_PORTAUDIO_PREFIX must be an absolute canonical provisioned prefix")
  endif()
  if(DEFINED CACHE{PORTAUDIO_INCLUDE_DIR} OR DEFINED CACHE{PORTAUDIO_LIBRARY})
    message(FATAL_ERROR
      "PortAudio cache injection is not accepted; use only VOICE_NAV_PORTAUDIO_PREFIX")
  endif()

  set(portaudio_lock_file "${prefix}/share/voice_nav/portaudio.lock")
  if(NOT EXISTS "${portaudio_lock_file}")
    message(FATAL_ERROR "canonical PortAudio source lock is missing from the prefix")
  endif()
  file(READ "${portaudio_lock_file}" portaudio_source_lock)
  set(portaudio_expected_source_lock
    "revision=147dd722548358763a8b649b3e4b41dfffbcfbb6\nsource_sha256=95457b809ce60d4d4790f84bb692e271f644e59d8adf96feb988c89ab52a506a\nshared=OFF\n")
  if(NOT portaudio_source_lock STREQUAL portaudio_expected_source_lock)
    message(FATAL_ERROR "PortAudio prefix lacks the canonical source lock metadata")
  endif()

  set(portaudio_receipt_file
    "${prefix}/share/voice_nav/portaudio-provenance.receipt")
  if(NOT EXISTS "${portaudio_receipt_file}")
    message(FATAL_ERROR "canonical PortAudio provenance receipt is missing from the prefix")
  endif()
  file(READ "${portaudio_receipt_file}" portaudio_receipt)
  set(portaudio_expected_receipt
    "schema_version=1\nid=portaudio\nversion=v19.7.0\nrevision=147dd722548358763a8b649b3e4b41dfffbcfbb6\nsource_sha256=95457b809ce60d4d4790f84bb692e271f644e59d8adf96feb988c89ab52a506a\nbuild_system=CMake\nPA_BUILD_SHARED=OFF\nPA_BUILD_TESTS=OFF\ntarget=portaudio\ninclude_dir=include\nlibrary=lib/libportaudio.a\n")
  if(NOT portaudio_receipt STREQUAL portaudio_expected_receipt)
    message(FATAL_ERROR "PortAudio prefix lacks the canonical provenance receipt identity")
  endif()

  file(REAL_PATH "${prefix}" portaudio_real_prefix)
  set(portaudio_include_dir "${portaudio_real_prefix}/include")
  set(portaudio_header "${portaudio_include_dir}/portaudio.h")
  set(portaudio_library "${portaudio_real_prefix}/lib/libportaudio.a")
  if(NOT EXISTS "${portaudio_header}" OR NOT EXISTS "${portaudio_library}")
    message(FATAL_ERROR "canonical PortAudio prefix lacks its receipt target files")
  endif()
  file(REAL_PATH "${portaudio_header}" portaudio_real_header)
  file(REAL_PATH "${portaudio_library}" portaudio_real_library)
  foreach(portaudio_real_path portaudio_real_header portaudio_real_library)
    file(RELATIVE_PATH portaudio_relative_path
      "${portaudio_real_prefix}" "${${portaudio_real_path}}")
    if(IS_ABSOLUTE "${portaudio_relative_path}" OR
      portaudio_relative_path MATCHES "^\\.\\.(/|$)")
      message(FATAL_ERROR
        "canonical PortAudio receipt target must resolve inside VOICE_NAV_PORTAUDIO_PREFIX")
    endif()
  endforeach()

  execute_process(
    COMMAND "${CMAKE_AR}" t "${portaudio_real_library}"
    RESULT_VARIABLE portaudio_archive_result
    OUTPUT_VARIABLE portaudio_archive_contents
    ERROR_VARIABLE portaudio_archive_error)
  if(NOT portaudio_archive_result EQUAL 0)
    message(FATAL_ERROR "canonical PortAudio library must be a readable static archive")
  endif()
  include(CheckCSourceCompiles)
  set(CMAKE_REQUIRED_INCLUDES "${portaudio_include_dir}")
  set(portaudio_probe_libraries "${portaudio_real_library}")
  if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    list(APPEND portaudio_probe_libraries m)
  endif()
  set(CMAKE_REQUIRED_LIBRARIES "${portaudio_probe_libraries}")
  unset(VOICE_NAV_PORTAUDIO_ARCHIVE_LINKS CACHE)
  check_c_source_compiles(
    "#include <portaudio.h>\nint main(void) { return Pa_GetVersion(); }"
    VOICE_NAV_PORTAUDIO_ARCHIVE_LINKS)
  unset(CMAKE_REQUIRED_INCLUDES)
  unset(CMAKE_REQUIRED_LIBRARIES)
  if(NOT VOICE_NAV_PORTAUDIO_ARCHIVE_LINKS)
    message(FATAL_ERROR "canonical PortAudio static archive must link with its header")
  endif()

  set(${include_variable} "${portaudio_include_dir}" PARENT_SCOPE)
  set(${library_variable} "${portaudio_real_library}" PARENT_SCOPE)
endfunction()
