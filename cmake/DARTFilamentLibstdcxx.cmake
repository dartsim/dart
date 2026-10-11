# GNU's std::mutex lacks the capability annotations upstream Clang analysis
# assumes. Disable only those diagnostics after SDK warning options are set.
function(dart_filament_stdmutex_diagnostics)
  foreach(_target IN ITEMS filament backend)
    target_compile_options(
      ${_target}
      PRIVATE -Wno-thread-safety-attributes -Wno-thread-safety-analysis
    )
  endforeach()
endfunction()
if(CMAKE_CURRENT_SOURCE_DIR MATCHES "/filament$")
  cmake_language(DEFER CALL dart_filament_stdmutex_diagnostics)
endif()
