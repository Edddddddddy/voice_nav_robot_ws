# Package-private link-surface contract for the actual native PortAudio entry
# thunk -> PortAudioAdapter::callback -> AudioEngine::process_callback
# closure.  The link map is the source of truth for archive members selected
# into the callback surface; no hand-maintained object list is accepted.
function(assert_callback_surface undefined_symbols)
  set(forbidden "(operator (new|delete)|malloc|calloc|realloc|free|aligned_alloc|posix_memalign|")
  set(forbidden "${forbidden}pthread_(mutex|cond)|std::(mutex|recursive_mutex|timed_mutex|condition_variable|")
  set(forbidden "${forbidden}unique_lock|lock_guard|this_thread)|futex|sem_|waitpid|")
  set(forbidden "${forbidden}nanosleep|clock_nanosleep|usleep|sleep|")
  set(forbidden "${forbidden}std::(cerr|clog|cout)|syslog|openlog|closelog|spdlog|glog|")
  set(forbidden "${forbidden}fopen|fdopen|freopen|open|openat|close|read|write|printf|fprintf|puts|")
  set(forbidden "${forbidden}std::basic_(ifstream|ofstream|fstream)|")
  set(forbidden "${forbidden}socket|connect|accept|send|recv|getaddrinfo|curl|")
  set(forbidden "${forbidden}rcl|rmw|rcutils|rosidl|")
  set(forbidden "${forbidden}onnx|Ort[A-Z]|sherpa|tensorflow|tflite|openvino|whisper|piper|kws|vad|asr|tts)")
  if("${undefined_symbols}" MATCHES "${forbidden}")
    message(FATAL_ERROR
      "real-time callback links a forbidden reachable dependency:\n${undefined_symbols}")
  endif()
endfunction()

function(assert_callback_hops adapter_symbols engine_symbols)
  if(NOT "${adapter_symbols}" MATCHES
      "voice_nav_audio::native_portaudio_callback\\(")
    message(FATAL_ERROR "real-time callback is missing the native PortAudio thunk")
  endif()
  if(NOT "${adapter_symbols}" MATCHES
      "voice_nav_audio::PortAudioAdapter::callback\\(")
    message(FATAL_ERROR "real-time callback is missing the PortAudioAdapter hop")
  endif()
  if(NOT "${engine_symbols}" MATCHES
      "voice_nav_audio::AudioEngine::process_callback\\(")
    message(FATAL_ERROR "real-time callback is missing the AudioEngine hop")
  endif()
endfunction()

function(resolve_callback_link_map_path map_path link_directory output)
  if(IS_ABSOLUTE "${map_path}")
    set(provider_path "${map_path}")
  else()
    get_filename_component(provider_path "${link_directory}/${map_path}" ABSOLUTE)
  endif()
  if(NOT EXISTS "${provider_path}")
    message(FATAL_ERROR "actual callback link map provider does not exist: ${provider_path}")
  endif()
  file(REAL_PATH "${provider_path}" resolved_provider_path)
  set("${output}" "${resolved_provider_path}" PARENT_SCOPE)
endfunction()

function(read_selected_link_providers link_map link_directory output)
  if(NOT EXISTS "${link_map}")
    message(FATAL_ERROR "actual callback link map is missing: ${link_map}")
  endif()
  file(REAL_PATH "${link_directory}" normalized_link_directory)
  file(STRINGS "${link_map}" link_map_lines)
  set(selected_providers "")
  foreach(link_map_line IN LISTS link_map_lines)
    string(REGEX MATCH "([^ ]+\\.a)\\(([^()]+)\\)" archive_member_match "${link_map_line}")
    if(NOT archive_member_match STREQUAL "")
      resolve_callback_link_map_path(
        "${CMAKE_MATCH_1}" "${normalized_link_directory}" archive_path)
      list(APPEND selected_providers "archive|${archive_path}|${CMAKE_MATCH_2}")
    endif()

    string(REGEX MATCH "^LOAD[ ]+(.+\\.o)$" direct_object_match "${link_map_line}")
    if(NOT direct_object_match STREQUAL "")
      resolve_callback_link_map_path(
        "${CMAKE_MATCH_1}" "${normalized_link_directory}" direct_object_path)
      list(APPEND selected_providers "object|${direct_object_path}|_")
    endif()

    string(REGEX MATCH "^LOAD[ ]+(.+\\.so(\\.[0-9]+)?)$" shared_library_match "${link_map_line}")
    if(NOT shared_library_match STREQUAL "")
      resolve_callback_link_map_path(
        "${CMAKE_MATCH_1}" "${normalized_link_directory}" shared_library_path)
      file(READ "${shared_library_path}" shared_library_magic OFFSET 0 LIMIT 4 HEX)
      string(TOLOWER "${shared_library_magic}" shared_library_magic)
      if(shared_library_magic STREQUAL "7f454c46")
        list(APPEND selected_providers "shared|${shared_library_path}|_")
      endif()
    endif()
  endforeach()
  list(REMOVE_DUPLICATES selected_providers)
  if(selected_providers STREQUAL "")
    message(FATAL_ERROR "actual callback link map has no inspectable providers")
  endif()
  set("${output}" "${selected_providers}" PARENT_SCOPE)
endfunction()

function(get_callback_provider_type provider output)
  string(REPLACE "|" ";" provider_fields "${provider}")
  list(GET provider_fields 0 provider_type)
  set("${output}" "${provider_type}" PARENT_SCOPE)
endfunction()

function(read_provider_symbols provider selector output)
  string(MD5 cache_key "${provider}|${selector}")
  get_property(symbols_cached GLOBAL PROPERTY "voice_nav_callback_symbols_${cache_key}" SET)
  if(symbols_cached)
    get_property(cached_symbols GLOBAL PROPERTY "voice_nav_callback_symbols_${cache_key}")
    set("${output}" "${cached_symbols}" PARENT_SCOPE)
    return()
  endif()

  string(REPLACE "|" ";" provider_fields "${provider}")
  list(GET provider_fields 0 provider_type)
  list(GET provider_fields 1 provider_path)
  if(provider_type STREQUAL "archive")
    list(GET provider_fields 2 archive_member)
  elseif(provider_type STREQUAL "object")
    set(archive_member "")
  elseif(provider_type STREQUAL "shared")
    set(archive_member "")
    set(nm_provider_options --dynamic)
  else()
    message(FATAL_ERROR "actual callback link map has an unsupported provider type: ${provider}")
  endif()
  execute_process(
    COMMAND "${NM_EXECUTABLE}" -A --format=posix ${nm_provider_options}
      "${selector}" "${provider_path}"
    RESULT_VARIABLE nm_result
    OUTPUT_VARIABLE provider_symbols
    ERROR_VARIABLE nm_error
  )
  if(NOT nm_result EQUAL 0)
    message(FATAL_ERROR "nm failed for ${provider_path}: ${nm_error}")
  endif()

  string(REPLACE "\n" ";" symbol_lines "${provider_symbols}")
  set(provider_symbol_names "")
  foreach(symbol_line IN LISTS symbol_lines)
    if(provider_type STREQUAL "archive")
      string(REGEX MATCH "^.*\\[([^]]+)\\]:[ ]+([^ ]+)[ ]+[A-Za-z]" symbol_match "${symbol_line}")
      if(NOT symbol_match STREQUAL "" AND CMAKE_MATCH_1 STREQUAL "${archive_member}")
        set(provider_symbol "${CMAKE_MATCH_2}")
      else()
        set(provider_symbol "")
      endif()
    else()
      string(REGEX MATCH "^.*:[ ]+([^ ]+)[ ]+[A-Za-z]" symbol_match "${symbol_line}")
      if(NOT symbol_match STREQUAL "")
        set(provider_symbol "${CMAKE_MATCH_1}")
      else()
        set(provider_symbol "")
      endif()
    endif()
    if(NOT provider_symbol STREQUAL "")
      string(REGEX REPLACE "@.*$" "" provider_symbol "${provider_symbol}")
      list(APPEND provider_symbol_names "${provider_symbol}")
    endif()
  endforeach()
  list(REMOVE_DUPLICATES provider_symbol_names)
  set_property(
    GLOBAL PROPERTY "voice_nav_callback_symbols_${cache_key}" "${provider_symbol_names}")
  set("${output}" "${provider_symbol_names}" PARENT_SCOPE)
endfunction()

function(assert_known_system_shared_provider provider)
  string(REPLACE "|" ";" provider_fields "${provider}")
  list(GET provider_fields 1 provider_path)
  get_filename_component(provider_name "${provider_path}" NAME)
  if(NOT provider_name MATCHES "^libc\\.so\\.[0-9]+$" AND
    NOT provider_name MATCHES "^libstdc\\+\\+\\.so\\.[0-9]+(\\.[0-9]+)*$" AND
    NOT provider_name MATCHES "^libgcc_s\\.so\\.[0-9]+$")
    message(FATAL_ERROR
      "real-time callback has an unapproved shared-library provider: ${provider_path}")
  endif()
endfunction()

function(find_selected_providers symbol selected_providers output)
  set(local_providers "")
  set(shared_providers "")
  foreach(selected_provider IN LISTS selected_providers)
    read_provider_symbols("${selected_provider}" --defined-only provider_definitions)
    list(FIND provider_definitions "${symbol}" definition_index)
    if(NOT definition_index EQUAL -1)
      get_callback_provider_type("${selected_provider}" provider_type)
      if(provider_type STREQUAL "shared")
        list(APPEND shared_providers "${selected_provider}")
      else()
        list(APPEND local_providers "${selected_provider}")
      endif()
    endif()
  endforeach()
  list(REMOVE_DUPLICATES local_providers)
  list(REMOVE_DUPLICATES shared_providers)
  if(NOT local_providers STREQUAL "")
    set("${output}" "${local_providers}" PARENT_SCOPE)
  else()
    set("${output}" "${shared_providers}" PARENT_SCOPE)
  endif()
endfunction()

function(assert_explicitly_forbidden_system_symbol symbol)
  set(forbidden_callback_system_symbols
    "__cxa_guard_acquire"
    "__cxa_guard_release"
    "__cxa_guard_abort"
    "__cxa_allocate_exception"
    "__cxa_free_exception"
    "__cxa_throw"
    "__cxa_rethrow"
    "__atomic_always_lock_free"
    "__atomic_is_lock_free"
    "__atomic_load"
    "__atomic_store"
    "__atomic_exchange"
    "__atomic_compare_exchange"
    "__atomic_test_and_set"
    "__atomic_clear"
  )
  foreach(atomic_width IN ITEMS 1 2 4 8 16)
    foreach(atomic_operation IN ITEMS
        load store exchange compare_exchange
        fetch_add fetch_sub fetch_and fetch_or fetch_xor fetch_nand)
      list(APPEND forbidden_callback_system_symbols
        "__atomic_${atomic_operation}_${atomic_width}")
    endforeach()
  endforeach()
  list(FIND forbidden_callback_system_symbols "${symbol}" forbidden_index)
  if(NOT forbidden_index EQUAL -1)
    message(FATAL_ERROR
      "real-time callback has an explicitly forbidden system/toolchain symbol: ${symbol}")
  endif()
endfunction()

function(assert_closed_system_allowlist symbol)
  assert_explicitly_forbidden_system_symbol("${symbol}")
  set(allowed
    "__cxa_begin_catch"
    "__cxa_end_catch"
    "__gxx_personality_v0"
    "__stack_chk_fail"
    "memmove"
  )
  list(FIND allowed "${symbol}" allowed_index)
  if(allowed_index EQUAL -1)
    message(FATAL_ERROR
      "real-time callback has an unresolved symbol outside the closed system/toolchain allowlist: ${symbol}")
  endif()
endfunction()

function(assert_reachable_selected_provider selected_provider selected_providers)
  get_property(visited GLOBAL PROPERTY voice_nav_callback_visited)
  list(FIND visited "${selected_provider}" visited_index)
  if(NOT visited_index EQUAL -1)
    return()
  endif()
  set_property(GLOBAL APPEND PROPERTY voice_nav_callback_visited "${selected_provider}")

  get_callback_provider_type("${selected_provider}" provider_type)
  if(provider_type STREQUAL "shared")
    assert_known_system_shared_provider("${selected_provider}")
    return()
  endif()

  read_provider_symbols("${selected_provider}" -u undefined_symbols)
  foreach(undefined_symbol IN LISTS undefined_symbols)
    assert_callback_surface("${undefined_symbol}")
    find_selected_providers("${undefined_symbol}" "${selected_providers}" providers)
    list(LENGTH providers provider_count)
    if(provider_count EQUAL 1)
      list(GET providers 0 provider)
      get_callback_provider_type("${provider}" provider_type)
      if(provider_type STREQUAL "shared")
        assert_closed_system_allowlist("${undefined_symbol}")
        assert_known_system_shared_provider("${provider}")
      else()
        assert_reachable_selected_provider("${provider}" "${selected_providers}")
      endif()
    elseif(provider_count GREATER 1)
      message(FATAL_ERROR
        "real-time callback has multiple exact symbol providers for ${undefined_symbol}: ${providers}")
    elseif("${undefined_symbol}" MATCHES "^_ZN(K)?15voice_nav_audio")
      message(FATAL_ERROR
        "real-time callback project symbol has no provider in the actual link closure: ${undefined_symbol}")
    else()
      message(FATAL_ERROR
        "real-time callback symbol has no provider in the actual GNU link map: ${undefined_symbol}")
    endif()
  endforeach()
endfunction()

function(assert_actual_callback_closure)
  foreach(required_variable
      CALLBACK_LINK_MAP
      CALLBACK_LINK_DIRECTORY
      CALLBACK_NATIVE_ROOT
      CALLBACK_ADAPTER_ROOT
      CALLBACK_ENGINE_ROOT)
    if(NOT DEFINED ${required_variable} OR "${${required_variable}}" STREQUAL "")
      message(FATAL_ERROR "actual callback link contract is missing ${required_variable}")
    endif()
  endforeach()

  read_selected_link_providers(
    "${CALLBACK_LINK_MAP}" "${CALLBACK_LINK_DIRECTORY}" selected_providers)
  set_property(GLOBAL PROPERTY voice_nav_callback_visited "")
  foreach(root_symbol
      "${CALLBACK_NATIVE_ROOT}"
      "${CALLBACK_ADAPTER_ROOT}"
      "${CALLBACK_ENGINE_ROOT}")
    find_selected_providers("${root_symbol}" "${selected_providers}" root_providers)
    list(LENGTH root_providers root_provider_count)
    if(NOT root_provider_count EQUAL 1)
      message(FATAL_ERROR
        "real-time callback root requires one exact mangled-symbol provider for ${root_symbol}: ${root_providers}")
    endif()
    list(GET root_providers 0 root_provider)
    assert_reachable_selected_provider("${root_provider}" "${selected_providers}")
  endforeach()
endfunction()

if(DEFINED SYMBOL_FIXTURE)
  file(READ "${SYMBOL_FIXTURE}" undefined_symbols)
  assert_callback_surface("${undefined_symbols}")
  return()
endif()

if(DEFINED ADAPTER_HOP_FIXTURE AND DEFINED ENGINE_HOP_FIXTURE)
  file(READ "${ADAPTER_HOP_FIXTURE}" adapter_symbols)
  file(READ "${ENGINE_HOP_FIXTURE}" engine_symbols)
  assert_callback_hops("${adapter_symbols}" "${engine_symbols}")
  return()
endif()

assert_actual_callback_closure()
