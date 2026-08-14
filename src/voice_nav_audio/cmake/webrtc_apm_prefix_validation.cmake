function(voice_nav_validate_webrtc_apm_prefix prefix include_variable library_variable)
  if(NOT IS_ABSOLUTE "${prefix}")
    message(FATAL_ERROR
      "VOICE_NAV_WEBRTC_APM_PREFIX must be an absolute canonical provisioned prefix")
  endif()
  if(DEFINED CACHE{WEBRTC_APM_INCLUDE_DIR} OR DEFINED CACHE{WEBRTC_APM_LIBRARY})
    message(FATAL_ERROR
      "WebRTC cache injection is not accepted; use only VOICE_NAV_WEBRTC_APM_PREFIX")
  endif()

  set(webrtc_lock_file "${prefix}/share/voice_nav/webrtc-apm.lock")
  if(NOT EXISTS "${webrtc_lock_file}")
    message(FATAL_ERROR "canonical WebRTC APM source lock is missing from the prefix")
  endif()
  file(READ "${webrtc_lock_file}" webrtc_source_lock)
  set(webrtc_expected_source_lock
    "revision=2.1\nsource_sha256=ae9302824b2038d394f10213cab05312c564a038434269f11dbf68f511f9f9fe\nabseil_revision=20240722.0\nabseil_source_sha256=f50e5ac311a81382da7fa75b97310e4b9006474f9560ac46f54a9967f07d4ae3\nabseil_patch_revision=20240722.0-3\nabseil_patch_sha256=12dd8df1488a314c53e3751abd2750cf233b830651d168b6a9f15e7d0cf71f7b\ndefault_library=static\ntests=disabled\n")
  if(NOT webrtc_source_lock STREQUAL webrtc_expected_source_lock)
    message(FATAL_ERROR "WebRTC APM prefix lacks the canonical source lock metadata")
  endif()

  set(webrtc_receipt_file "${prefix}/share/voice_nav/webrtc-apm-provenance.receipt")
  if(NOT EXISTS "${webrtc_receipt_file}")
    message(FATAL_ERROR "canonical WebRTC APM provenance receipt is missing from the prefix")
  endif()
  file(READ "${webrtc_receipt_file}" webrtc_receipt)
  set(webrtc_expected_receipt
    "schema_version=1\nid=webrtc-audio-processing\nversion=2.1\nrevision=2.1\nsource_sha256=ae9302824b2038d394f10213cab05312c564a038434269f11dbf68f511f9f9fe\nabseil_version=20240722.0\nabseil_revision=20240722.0\nabseil_source_sha256=f50e5ac311a81382da7fa75b97310e4b9006474f9560ac46f54a9967f07d4ae3\nabseil_patch_version=20240722.0-3\nabseil_patch_revision=20240722.0-3\nabseil_patch_sha256=12dd8df1488a314c53e3751abd2750cf233b830651d168b6a9f15e7d0cf71f7b\nbuild_system=Meson\ndefault_library=static\ntests=disabled\ntarget=webrtc-audio-processing-2\ninclude_dir=include/webrtc-audio-processing-2\nlibrary=lib/x86_64-linux-gnu/libwebrtc-audio-processing-2.a\n")
  if(NOT webrtc_receipt STREQUAL webrtc_expected_receipt)
    message(FATAL_ERROR "WebRTC APM prefix lacks the canonical provenance receipt identity")
  endif()

  file(REAL_PATH "${prefix}" webrtc_real_prefix)
  set(webrtc_include_dir "${webrtc_real_prefix}/include/webrtc-audio-processing-2")
  set(webrtc_header "${webrtc_include_dir}/modules/audio_processing/include/audio_processing.h")
  set(webrtc_library "${webrtc_real_prefix}/lib/x86_64-linux-gnu/libwebrtc-audio-processing-2.a")
  if(NOT EXISTS "${webrtc_header}" OR NOT EXISTS "${webrtc_library}")
    message(FATAL_ERROR "canonical WebRTC APM prefix lacks its receipt target files")
  endif()
  file(REAL_PATH "${webrtc_header}" webrtc_real_header)
  file(REAL_PATH "${webrtc_library}" webrtc_real_library)
  foreach(webrtc_real_path webrtc_real_header webrtc_real_library)
    file(RELATIVE_PATH webrtc_relative_path
      "${webrtc_real_prefix}" "${${webrtc_real_path}}")
    if(IS_ABSOLUTE "${webrtc_relative_path}" OR
      webrtc_relative_path MATCHES "^\\.\\.(/|$)")
      message(FATAL_ERROR
        "canonical WebRTC APM receipt target must resolve inside VOICE_NAV_WEBRTC_APM_PREFIX")
    endif()
  endforeach()

  execute_process(
    COMMAND "${CMAKE_AR}" t "${webrtc_real_library}"
    RESULT_VARIABLE webrtc_archive_result
    OUTPUT_VARIABLE webrtc_archive_contents
    ERROR_VARIABLE webrtc_archive_error)
  if(NOT webrtc_archive_result EQUAL 0)
    message(FATAL_ERROR "canonical WebRTC APM library must be a readable static archive")
  endif()
  include(CheckCXXSourceCompiles)
  set(webrtc_include_dirs "${webrtc_include_dir};${webrtc_real_prefix}/include")
  set(CMAKE_REQUIRED_INCLUDES "${webrtc_include_dirs}")
  set(CMAKE_REQUIRED_LIBRARIES "${webrtc_real_library};rt;pthread")
  unset(VOICE_NAV_WEBRTC_APM_ARCHIVE_LINKS CACHE)
  check_cxx_source_compiles(
    "#include <modules/audio_processing/include/audio_processing.h>\nint main() { auto apm = webrtc::AudioProcessingBuilder().Create(); return apm ? 0 : 1; }"
    VOICE_NAV_WEBRTC_APM_ARCHIVE_LINKS)
  unset(CMAKE_REQUIRED_INCLUDES)
  unset(CMAKE_REQUIRED_LIBRARIES)
  if(NOT VOICE_NAV_WEBRTC_APM_ARCHIVE_LINKS)
    message(FATAL_ERROR "canonical WebRTC APM static archive must link with its header")
  endif()

  set(${include_variable} "${webrtc_include_dirs}" PARENT_SCOPE)
  set(${library_variable} "${webrtc_real_library}" PARENT_SCOPE)
endfunction()
