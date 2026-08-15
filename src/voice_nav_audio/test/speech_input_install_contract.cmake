if(EXISTS "${PACKAGE_PREFIX}/lib/voice_nav_audio")
  file(GLOB installed_executables "${PACKAGE_PREFIX}/lib/voice_nav_audio/*")
  set(expected_demo "${PACKAGE_PREFIX}/lib/voice_nav_audio/scripted_voice_demo")
  list(LENGTH installed_executables executable_count)
  if(executable_count GREATER 1 OR
    (executable_count EQUAL 1 AND NOT "${installed_executables}" STREQUAL "${expected_demo}"))
    message(FATAL_ERROR
      "Only the explicit scripted simulation demo may be installed in voice_nav_audio")
  endif()
endif()
