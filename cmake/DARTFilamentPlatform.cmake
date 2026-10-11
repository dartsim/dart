# Filament disables RTTI. Restrict that requirement to the SDK subclass;
# DART's shape extraction and public implementation retain normal RTTI.
function(dart_gui_filament_platform_source output)
  set(_source "${CMAKE_CURRENT_SOURCE_DIR}/detail/headless_vulkan_platform.cpp")
  if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set_source_files_properties(
      "${_source}"
      PROPERTIES COMPILE_OPTIONS -fno-rtti
    )
  endif()
  set(${output} "${_source}" PARENT_SCOPE)
endfunction()
