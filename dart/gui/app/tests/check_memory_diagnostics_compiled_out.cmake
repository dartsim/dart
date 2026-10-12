# Copyright (c) 2011, The DART development contributors
# All rights reserved.
#
# This file is provided under the "BSD-style" License.

if(NOT DEFINED DART_DEMOS_EXECUTABLE OR DART_DEMOS_EXECUTABLE STREQUAL "")
  message(FATAL_ERROR "DART_DEMOS_EXECUTABLE is required")
endif()

if(NOT EXISTS "${DART_DEMOS_EXECUTABLE}")
  message(
    FATAL_ERROR
    "Demo executable does not exist: ${DART_DEMOS_EXECUTABLE}"
  )
endif()

# Check both UI/environment literals and C++ symbol fragments. This remains
# useful for stripped binaries because the opt-in environment variable and UI
# copy must still be present whenever the feature is linked.
set(
  memory_diagnostics_binary_markers
  "DART_DEMOS_MEMORY_DIAGNOSTICS"
  "dart.memory-diagnostics.v2"
  "DiagnosticSession"
  "collectMemoryDiagnostics"
  "renderMemoryDiagnostics"
  "Memory diagnostics are off by default"
)

foreach(marker IN LISTS memory_diagnostics_binary_markers)
  file(
    STRINGS "${DART_DEMOS_EXECUTABLE}"
    marker_matches
    REGEX "${marker}"
    LIMIT_COUNT 1
  )
  if(marker_matches)
    message(
      FATAL_ERROR
      "Memory diagnostics marker '${marker}' is present in the default-OFF "
      "demo executable: ${DART_DEMOS_EXECUTABLE}"
    )
  endif()
endforeach()
