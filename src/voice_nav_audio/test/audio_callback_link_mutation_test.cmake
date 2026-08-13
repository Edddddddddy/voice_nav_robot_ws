function(expect_fixture fixture expected_result)
  execute_process(
    COMMAND "${CMAKE_COMMAND}"
      "-DSYMBOL_FIXTURE=${FIXTURE_DIR}/${fixture}"
      -P "${CONTRACT_SCRIPT}"
    RESULT_VARIABLE result
    OUTPUT_VARIABLE output
    ERROR_VARIABLE error
  )
  if(expected_result STREQUAL "pass" AND NOT result EQUAL 0)
    message(FATAL_ERROR "allowed fixture was rejected: ${fixture}\n${output}\n${error}")
  endif()
  if(expected_result STREQUAL "fail" AND result EQUAL 0)
    message(FATAL_ERROR "forbidden mutation was accepted: ${fixture}")
  endif()
endfunction()

function(expect_hop_fixture adapter_fixture engine_fixture expected_result)
  execute_process(
    COMMAND "${CMAKE_COMMAND}"
      "-DADAPTER_HOP_FIXTURE=${FIXTURE_DIR}/${adapter_fixture}"
      "-DENGINE_HOP_FIXTURE=${FIXTURE_DIR}/${engine_fixture}"
      -P "${CONTRACT_SCRIPT}"
    RESULT_VARIABLE result
    OUTPUT_VARIABLE output
    ERROR_VARIABLE error
  )
  if(expected_result STREQUAL "pass" AND NOT result EQUAL 0)
    message(FATAL_ERROR "allowed callback chain was rejected: ${adapter_fixture}/${engine_fixture}\n${output}\n${error}")
  endif()
  if(expected_result STREQUAL "fail" AND result EQUAL 0)
    message(FATAL_ERROR "broken callback chain was accepted: ${adapter_fixture}/${engine_fixture}")
  endif()
endfunction()

expect_fixture("allowed.txt" "pass")
foreach(fixture
    "new.txt"
    "new-array.txt"
    "delete.txt"
    "delete-array.txt"
    "aligned-new.txt"
    "aligned-new-array.txt"
    "aligned-delete.txt"
    "aligned-delete-array.txt"
    "nothrow-new.txt"
    "nothrow-new-array.txt"
    "malloc.txt"
    "calloc.txt"
    "realloc.txt"
    "free.txt"
    "aligned-alloc.txt"
    "posix-memalign.txt"
    "mutex.txt"
    "condition-wait.txt"
    "nanosleep.txt"
    "cerr.txt"
    "log.txt"
    "file.txt"
    "network.txt"
    "ros.txt"
    "rmw.txt"
    "inference.txt")
  expect_fixture("${fixture}" "fail")
endforeach()

expect_hop_fixture("callback-hops-adapter.txt" "callback-hops-engine.txt" "pass")
expect_hop_fixture("callback-hops-missing-native.txt" "callback-hops-engine.txt" "fail")
expect_hop_fixture("callback-hops-missing-adapter.txt" "callback-hops-engine.txt" "fail")
expect_hop_fixture("callback-hops-adapter.txt" "callback-hops-missing-engine.txt" "fail")

if(NOT EXISTS "${MUTATION_CALLBACK_ARTIFACT}")
  message(FATAL_ERROR "actual callback mutation artifact was not linked")
endif()
if(NOT EXISTS "${MUTATION_CALLBACK_LINK_MAP}")
  message(FATAL_ERROR "actual callback mutation link map was not generated")
endif()

execute_process(
  COMMAND "${CMAKE_COMMAND}"
    "-DNM_EXECUTABLE=${NM_EXECUTABLE}"
    "-DCALLBACK_LINK_MAP=${MUTATION_CALLBACK_LINK_MAP}"
    "-DCALLBACK_LINK_DIRECTORY=${MUTATION_CALLBACK_LINK_DIRECTORY}"
    "-DCALLBACK_NATIVE_ROOT=${CALLBACK_NATIVE_ROOT}"
    "-DCALLBACK_ADAPTER_ROOT=${CALLBACK_ADAPTER_ROOT}"
    "-DCALLBACK_ENGINE_ROOT=${CALLBACK_ENGINE_ROOT}"
    -P "${CONTRACT_SCRIPT}"
  RESULT_VARIABLE mutation_result
  OUTPUT_VARIABLE mutation_output
  ERROR_VARIABLE mutation_error
)
if(mutation_result EQUAL 0)
  message(FATAL_ERROR
    "actual callback link surface accepted a reachable external malloc helper:\n"
    "${mutation_output}${mutation_error}")
endif()
if(NOT "${mutation_output}${mutation_error}" MATCHES "forbidden reachable dependency" OR
  NOT "${mutation_output}${mutation_error}" MATCHES "malloc")
  message(FATAL_ERROR
    "actual callback link surface failed for a reason other than the external malloc helper:\n"
    "${mutation_output}${mutation_error}")
endif()

if(NOT EXISTS "${GUARD_MUTATION_CALLBACK_ARTIFACT}")
  message(FATAL_ERROR "actual callback guard mutation artifact was not linked")
endif()
if(NOT EXISTS "${GUARD_MUTATION_CALLBACK_LINK_MAP}")
  message(FATAL_ERROR "actual callback guard mutation link map was not generated")
endif()
if(NOT EXISTS "${GUARD_MUTATION_CALLBACK_PROVIDER}")
  message(FATAL_ERROR "actual callback guard mutation provider was not built")
endif()

file(READ "${GUARD_MUTATION_CALLBACK_LINK_MAP}" guard_mutation_link_map)
if(NOT "${guard_mutation_link_map}" MATCHES "audio_callback_link_guard_mutation_helper")
  message(FATAL_ERROR "actual callback guard mutation provider was not selected by the GNU link map")
endif()

execute_process(
  COMMAND "${NM_EXECUTABLE}" -A --format=posix -u "${GUARD_MUTATION_CALLBACK_PROVIDER}"
  RESULT_VARIABLE guard_provider_nm_result
  OUTPUT_VARIABLE guard_provider_undefined_symbols
  ERROR_VARIABLE guard_provider_nm_error
)
if(NOT guard_provider_nm_result EQUAL 0)
  message(FATAL_ERROR
    "could not inspect the actual callback guard mutation provider:\n${guard_provider_nm_error}")
endif()
if(NOT "${guard_provider_undefined_symbols}" MATCHES "__cxa_guard_acquire")
  message(FATAL_ERROR
    "actual callback guard mutation provider lacks __cxa_guard_acquire:\n"
    "${guard_provider_undefined_symbols}")
endif()

execute_process(
  COMMAND "${CMAKE_COMMAND}"
    "-DNM_EXECUTABLE=${NM_EXECUTABLE}"
    "-DCALLBACK_LINK_MAP=${GUARD_MUTATION_CALLBACK_LINK_MAP}"
    "-DCALLBACK_LINK_DIRECTORY=${GUARD_MUTATION_CALLBACK_LINK_DIRECTORY}"
    "-DCALLBACK_NATIVE_ROOT=${CALLBACK_NATIVE_ROOT}"
    "-DCALLBACK_ADAPTER_ROOT=${CALLBACK_ADAPTER_ROOT}"
    "-DCALLBACK_ENGINE_ROOT=${CALLBACK_ENGINE_ROOT}"
    -P "${CONTRACT_SCRIPT}"
  RESULT_VARIABLE guard_mutation_result
  OUTPUT_VARIABLE guard_mutation_output
  ERROR_VARIABLE guard_mutation_error
)
if(guard_mutation_result EQUAL 0)
  message(FATAL_ERROR
    "actual callback link surface accepted the reachable blocking symbol __cxa_guard_acquire:\n"
    "${guard_mutation_output}${guard_mutation_error}")
endif()
if(NOT "${guard_mutation_output}${guard_mutation_error}" MATCHES "__cxa_guard_acquire")
  message(FATAL_ERROR
    "actual callback guard mutation failed for a reason other than __cxa_guard_acquire:\n"
    "${guard_mutation_output}${guard_mutation_error}")
endif()

function(expect_actual_provider_mutation_rejected
    artifact link_map link_directory provider provider_name provider_type)
  if(NOT EXISTS "${artifact}")
    message(FATAL_ERROR "actual callback ${provider_type} mutation artifact was not linked")
  endif()
  if(NOT EXISTS "${link_map}")
    message(FATAL_ERROR "actual callback ${provider_type} mutation link map was not generated")
  endif()
  if(NOT EXISTS "${provider}")
    message(FATAL_ERROR "actual callback ${provider_type} mutation provider was not built")
  endif()

  file(READ "${link_map}" provider_mutation_link_map)
  if(NOT "${provider_mutation_link_map}" MATCHES "${provider_name}")
    message(FATAL_ERROR
      "actual callback ${provider_type} mutation provider was not selected by the GNU link map")
  endif()

  execute_process(
    COMMAND "${NM_EXECUTABLE}" -A --format=posix -u "${provider}"
    RESULT_VARIABLE provider_nm_result
    OUTPUT_VARIABLE provider_undefined_symbols
    ERROR_VARIABLE provider_nm_error
  )
  if(NOT provider_nm_result EQUAL 0)
    message(FATAL_ERROR
      "could not inspect the actual callback ${provider_type} mutation provider:\n"
      "${provider_nm_error}")
  endif()
  if(NOT "${provider_undefined_symbols}" MATCHES "malloc")
    message(FATAL_ERROR
      "actual callback ${provider_type} mutation provider lacks malloc:\n"
      "${provider_undefined_symbols}")
  endif()

  execute_process(
    COMMAND "${CMAKE_COMMAND}"
      "-DNM_EXECUTABLE=${NM_EXECUTABLE}"
      "-DCALLBACK_LINK_MAP=${link_map}"
      "-DCALLBACK_LINK_DIRECTORY=${link_directory}"
      "-DCALLBACK_NATIVE_ROOT=${CALLBACK_NATIVE_ROOT}"
      "-DCALLBACK_ADAPTER_ROOT=${CALLBACK_ADAPTER_ROOT}"
      "-DCALLBACK_ENGINE_ROOT=${CALLBACK_ENGINE_ROOT}"
      -P "${CONTRACT_SCRIPT}"
    RESULT_VARIABLE provider_mutation_result
    OUTPUT_VARIABLE provider_mutation_output
    ERROR_VARIABLE provider_mutation_error
  )
  if(provider_mutation_result EQUAL 0)
    message(FATAL_ERROR
      "actual callback link surface accepted a reachable ${provider_type} provider's malloc:\n"
      "${provider_mutation_output}${provider_mutation_error}")
  endif()
  if(NOT "${provider_mutation_output}${provider_mutation_error}" MATCHES
      "forbidden reachable dependency" OR
    NOT "${provider_mutation_output}${provider_mutation_error}" MATCHES "malloc")
    message(FATAL_ERROR
      "actual callback ${provider_type} mutation failed for a reason other than malloc:\n"
      "${provider_mutation_output}${provider_mutation_error}")
  endif()
endfunction()

expect_actual_provider_mutation_rejected(
  "${DIRECT_OBJECT_MUTATION_CALLBACK_ARTIFACT}"
  "${DIRECT_OBJECT_MUTATION_CALLBACK_LINK_MAP}"
  "${DIRECT_OBJECT_MUTATION_CALLBACK_LINK_DIRECTORY}"
  "${DIRECT_OBJECT_MUTATION_CALLBACK_PROVIDER}"
  "audio_callback_link_direct_object_mutation"
  "direct object"
)
expect_actual_provider_mutation_rejected(
  "${EXTERNAL_ARCHIVE_MUTATION_CALLBACK_ARTIFACT}"
  "${EXTERNAL_ARCHIVE_MUTATION_CALLBACK_LINK_MAP}"
  "${EXTERNAL_ARCHIVE_MUTATION_CALLBACK_LINK_DIRECTORY}"
  "${EXTERNAL_ARCHIVE_MUTATION_CALLBACK_PROVIDER}"
  "audio_callback_link_external_archive_mutation"
  "external archive"
)
