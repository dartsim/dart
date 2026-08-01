# Copyright (c) 2011, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License.

if(
  NOT DEFINED DART_DEMOS_EXECUTABLE
  OR DART_DEMOS_EXECUTABLE STREQUAL ""
  OR NOT EXISTS "${DART_DEMOS_EXECUTABLE}"
)
  message(FATAL_ERROR "dart-demos executable is required")
endif()
if(NOT DEFINED DART_EXPECT_MEMORY_DIAGNOSTICS)
  message(FATAL_ERROR "DART_EXPECT_MEMORY_DIAGNOSTICS is required")
endif()

# Exact UI/schema strings remain in the executable whenever the static
# diagnostics implementation is linked, even if a linker strips symbol names.
file(STRINGS "${DART_DEMOS_EXECUTABLE}" panel_title REGEX "Memory Diagnostics")
file(
  STRINGS "${DART_DEMOS_EXECUTABLE}"
  schema
  REGEX "dart\\.memory-diagnostics\\.v2"
)
if(DART_EXPECT_MEMORY_DIAGNOSTICS)
  if(NOT panel_title OR NOT schema)
    message(
      FATAL_ERROR
      "Memory diagnostics strings are missing from enabled dart-demos"
    )
  endif()
elseif(panel_title OR schema)
  message(FATAL_ERROR "Memory diagnostics strings found in OFF dart-demos")
endif()

# On platforms with an nm-compatible tool, also check the demo collector,
# panel, and session symbols directly. The string checks above remain the
# portable artifact oracle.
if(
  DEFINED DART_NM_EXECUTABLE
  AND NOT DART_NM_EXECUTABLE STREQUAL ""
  AND NOT WIN32
)
  execute_process(
    COMMAND "${DART_NM_EXECUTABLE}" -C "${DART_DEMOS_EXECUTABLE}"
    RESULT_VARIABLE nm_result
    OUTPUT_VARIABLE symbols
    ERROR_QUIET
  )
  if(nm_result EQUAL 0)
    string(
      REGEX MATCH
      "DiagnosticSession|collectMemoryDiagnostics|createMemoryDiagnosticsPanel"
      diagnostic_symbol
      "${symbols}"
    )
    if(DART_EXPECT_MEMORY_DIAGNOSTICS AND NOT diagnostic_symbol)
      message(
        FATAL_ERROR
        "Memory diagnostics symbols are missing from enabled dart-demos"
      )
    elseif(NOT DART_EXPECT_MEMORY_DIAGNOSTICS AND diagnostic_symbol)
      message(
        FATAL_ERROR
        "Memory diagnostics symbol found in OFF dart-demos: "
        "${diagnostic_symbol}"
      )
    endif()
  endif()
endif()
