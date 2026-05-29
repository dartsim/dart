if(NOT DEFINED INPUT)
  message(FATAL_ERROR "INPUT is required")
endif()
if(NOT DEFINED OUTPUT)
  message(FATAL_ERROR "OUTPUT is required")
endif()
if(NOT DEFINED SYMBOL)
  message(FATAL_ERROR "SYMBOL is required")
endif()
if(NOT DEFINED NAMESPACE)
  set(NAMESPACE "dart::examples")
endif()

file(READ "${INPUT}" _hex HEX)

set(
  _content
  "#pragma once\n\n#include <cstddef>\n#include <cstdint>\n\nnamespace ${NAMESPACE} {\n\ninline constexpr std::uint8_t ${SYMBOL}[] = {\n"
)
string(LENGTH "${_hex}" _hex_length)
set(_hex_offset 0)
set(_hex_chars_per_line 512)
while(_hex_offset LESS _hex_length)
  string(
    SUBSTRING "${_hex}"
    "${_hex_offset}"
    "${_hex_chars_per_line}"
    _line_hex
  )
  string(
    REGEX REPLACE
    "([0-9A-Fa-f][0-9A-Fa-f])"
    "0x\\1, "
    _line
    "${_line_hex}"
  )
  string(APPEND _content "  ${_line}\n")
  math(EXPR _hex_offset "${_hex_offset} + ${_hex_chars_per_line}")
endwhile()
string(
  APPEND _content
  "};\ninline constexpr std::size_t ${SYMBOL}Size = sizeof(${SYMBOL});\n\n} // namespace ${NAMESPACE}\n"
)

file(WRITE "${OUTPUT}" "${_content}")
