function(voice_nav_validate_sherpa_onnx_prefix
    prefix include_variable libraries_variable rpath_variable)
  if(NOT IS_ABSOLUTE "${prefix}")
    message(FATAL_ERROR
      "VOICE_NAV_SHERPA_ONNX_PREFIX must be an absolute canonical provisioned prefix")
  endif()
  if(DEFINED CACHE{SHERPA_ONNX_INCLUDE_DIR} OR DEFINED CACHE{SHERPA_ONNX_LIBRARY})
    message(FATAL_ERROR
      "sherpa-onnx cache injection is not accepted; use only VOICE_NAV_SHERPA_ONNX_PREFIX")
  endif()
  if(NOT EXISTS "${prefix}" OR IS_SYMLINK "${prefix}")
    message(FATAL_ERROR
      "VOICE_NAV_SHERPA_ONNX_PREFIX must name an existing non-symlink directory")
  endif()

  file(REAL_PATH "${prefix}" sherpa_real_prefix)
  set(sherpa_receipt_file
    "${sherpa_real_prefix}/share/voice_nav/sherpa-onnx-provenance.receipt")
  if(NOT EXISTS "${sherpa_receipt_file}")
    message(FATAL_ERROR "canonical sherpa-onnx provenance receipt is missing from the prefix")
  endif()
  file(READ "${sherpa_receipt_file}" sherpa_receipt)
  set(sherpa_expected_receipt
    "schema_version=2\nid=sherpa-onnx\nversion=v1.13.4\nrevision=142807252687d81b40d6315f23470a1512a00de3\nsource_sha256=f0dc7c9b41b8691313daee671e826eb23946fa1320559a8d37e84f8774af76b2\nonnxruntime_mode=shared\nonnxruntime_version=1.27.0\nonnxruntime_url=https://github.com/csukuangfj/onnxruntime-libs/releases/download/v1.27.0/onnxruntime-linux-x64-glibc2_17-Release-1.27.0.zip\nonnxruntime_zip_size=8509524\nonnxruntime_zip_sha256=9f0c0a6998f1b94c399eeddcb443beb4a922c9a4fd431fdc9cd6de67a1935d00\nonnxruntime_git_commit=8f0278c77bf44b0cc83c098c6c722b92a36ac4b5\nonnxruntime_license=MIT\nonnxruntime_soname=libonnxruntime.so\nonnxruntime_library=lib/libonnxruntime.so\nonnxruntime_library_size=26403889\nonnxruntime_library_sha256=026c7d5c609323fb16506dbc3cce801bcdffdd7566fdba49a50727e2e1e881ca\nbuild_system=CMake\ncxx_compiler=GNU 13.3.0\nBUILD_SHARED_LIBS=OFF\nSHERPA_ONNX_ENABLE_C_API=ON\nSHERPA_ONNX_ENABLE_TESTS=OFF\nSHERPA_ONNX_ENABLE_PORTAUDIO=OFF\nSHERPA_ONNX_ENABLE_WEBSOCKET=OFF\nSHERPA_ONNX_ENABLE_TTS=ON\nSHERPA_ONNX_ENABLE_SPEAKER_DIARIZATION=OFF\nSHERPA_ONNX_ENABLE_BINARY=OFF\nSHERPA_ONNX_USE_PRE_INSTALLED_ONNXRUNTIME_IF_AVAILABLE=ON\nc_api_header=include/sherpa-onnx/c-api/c-api.h\nc_api_library=lib/libsherpa-onnx-c-api.a\ncore_library=lib/libsherpa-onnx-core.a\n")
  if(NOT sherpa_receipt STREQUAL sherpa_expected_receipt)
    message(FATAL_ERROR "canonical sherpa-onnx prefix lacks the frozen shared-ORT receipt identity")
  endif()

  set(sherpa_include_dir "${sherpa_real_prefix}/include")
  set(sherpa_header "${sherpa_include_dir}/sherpa-onnx/c-api/c-api.h")
  set(sherpa_c_api_library "${sherpa_real_prefix}/lib/libsherpa-onnx-c-api.a")
  set(sherpa_core_library "${sherpa_real_prefix}/lib/libsherpa-onnx-core.a")
  set(sherpa_ort_library "${sherpa_real_prefix}/lib/libonnxruntime.so")
  if(NOT EXISTS "${sherpa_header}" OR
    NOT EXISTS "${sherpa_c_api_library}" OR
    NOT EXISTS "${sherpa_core_library}" OR
    NOT EXISTS "${sherpa_ort_library}")
    message(FATAL_ERROR
      "canonical shared-ORT prefix lacks its receipt target files")
  endif()
  if(IS_SYMLINK "${sherpa_ort_library}")
    message(FATAL_ERROR "canonical ONNX Runtime library must be the receipt-owned regular file")
  endif()

  file(REAL_PATH "${sherpa_header}" sherpa_real_header)
  file(REAL_PATH "${sherpa_c_api_library}" sherpa_real_c_api_library)
  file(REAL_PATH "${sherpa_core_library}" sherpa_real_core_library)
  file(REAL_PATH "${sherpa_ort_library}" sherpa_real_ort_library)
  foreach(sherpa_real_path
      sherpa_real_header sherpa_real_c_api_library sherpa_real_core_library sherpa_real_ort_library)
    file(RELATIVE_PATH sherpa_relative_path
      "${sherpa_real_prefix}" "${${sherpa_real_path}}")
    if(IS_ABSOLUTE "${sherpa_relative_path}" OR
      sherpa_relative_path MATCHES "^\.\.(/|$)")
      message(FATAL_ERROR
        "canonical sherpa-onnx receipt target must resolve inside the prefix")
    endif()
  endforeach()

  file(SIZE "${sherpa_real_ort_library}" sherpa_ort_size)
  file(SHA256 "${sherpa_real_ort_library}" sherpa_ort_sha256)
  if(NOT sherpa_ort_size STREQUAL "26403889" OR
    NOT sherpa_ort_sha256 STREQUAL
      "026c7d5c609323fb16506dbc3cce801bcdffdd7566fdba49a50727e2e1e881ca")
    message(FATAL_ERROR "canonical ONNX Runtime .so does not match the frozen artifact identity")
  endif()

  find_program(sherpa_readelf NAMES readelf)
  if(NOT sherpa_readelf)
    message(FATAL_ERROR "readelf is required to verify the canonical ONNX Runtime SONAME")
  endif()
  execute_process(
    COMMAND "${sherpa_readelf}" -d "${sherpa_real_ort_library}"
    RESULT_VARIABLE sherpa_readelf_result
    OUTPUT_VARIABLE sherpa_readelf_output
    ERROR_VARIABLE sherpa_readelf_error)
  if(NOT sherpa_readelf_result EQUAL 0 OR
    NOT sherpa_readelf_output MATCHES "SONAME.*libonnxruntime\\.so")
    message(FATAL_ERROR "canonical ONNX Runtime .so must declare SONAME libonnxruntime.so")
  endif()

  execute_process(
    COMMAND "${CMAKE_AR}" t "${sherpa_real_c_api_library}"
    RESULT_VARIABLE sherpa_c_api_archive_result
    OUTPUT_VARIABLE sherpa_c_api_archive_contents
    ERROR_VARIABLE sherpa_c_api_archive_error)
  execute_process(
    COMMAND "${CMAKE_AR}" t "${sherpa_real_core_library}"
    RESULT_VARIABLE sherpa_core_archive_result
    OUTPUT_VARIABLE sherpa_core_archive_contents
    ERROR_VARIABLE sherpa_core_archive_error)
  if(NOT sherpa_c_api_archive_result EQUAL 0 OR
    NOT sherpa_core_archive_result EQUAL 0)
    message(FATAL_ERROR "canonical sherpa-onnx target libraries must be readable static archives")
  endif()

  file(GLOB sherpa_archive_files LIST_DIRECTORIES false
    "${sherpa_real_prefix}/lib/*.a")
  if(NOT sherpa_archive_files)
    message(FATAL_ERROR "canonical sherpa-onnx prefix has no static sherpa link set")
  endif()
  foreach(sherpa_archive IN LISTS sherpa_archive_files)
    get_filename_component(sherpa_archive_name "${sherpa_archive}" NAME)
    if(sherpa_archive_name STREQUAL "libonnxruntime.a")
      message(FATAL_ERROR "legacy static libonnxruntime.a is forbidden by the shared-ORT contract")
    endif()
    file(REAL_PATH "${sherpa_archive}" sherpa_real_archive)
    file(RELATIVE_PATH sherpa_archive_relative
      "${sherpa_real_prefix}" "${sherpa_real_archive}")
    if(IS_ABSOLUTE "${sherpa_archive_relative}" OR
      sherpa_archive_relative MATCHES "^\.\.(/|$)")
      message(FATAL_ERROR "sherpa-onnx static link set must stay inside the prefix")
    endif()
  endforeach()

  set(sherpa_libraries "-Wl,--start-group")
  list(APPEND sherpa_libraries ${sherpa_archive_files})
  list(APPEND sherpa_libraries "-Wl,--end-group" "${sherpa_real_ort_library}" "m" "dl" "pthread")

  set(${include_variable} "${sherpa_include_dir}" PARENT_SCOPE)
  set(${libraries_variable} "${sherpa_libraries}" PARENT_SCOPE)
  set(${rpath_variable} "${sherpa_real_prefix}/lib" PARENT_SCOPE)
endfunction()
