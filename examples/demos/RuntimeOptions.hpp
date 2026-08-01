/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * This file is provided under the "BSD-style" License.
 */

#ifndef DART_EXAMPLES_DEMOS_RUNTIMEOPTIONS_HPP_
#define DART_EXAMPLES_DEMOS_RUNTIMEOPTIONS_HPP_

#include <algorithm>
#include <string_view>

#include <cctype>
#include <cerrno>
#include <cstddef>
#include <cstdlib>

namespace dart_demos {

// Keep command-line and programmatic startup requests within the same bound as
// the runtime UI. Zero remains the hardware-concurrency sentinel.
inline constexpr std::size_t kMaxSimulationThreads = 256u;

namespace detail {

//==============================================================================
/// Parses a decimal worker count without accepting signs, trailing characters,
/// range overflow, or values above the demo's supported runtime limit.
inline bool parseSimulationThreadCount(
    const char* value, std::size_t& threadCount)
{
  if (value == nullptr || value[0] == '\0')
    return false;

  const std::string_view text(value);
  if (!std::all_of(text.begin(), text.end(), [](unsigned char c) {
        return std::isdigit(c);
      })) {
    return false;
  }

  errno = 0;
  char* end = nullptr;
  const auto parsed = std::strtoull(value, &end, 10);
  if (errno == ERANGE || end == value || *end != '\0'
      || parsed > kMaxSimulationThreads) {
    return false;
  }

  threadCount = static_cast<std::size_t>(parsed);
  return true;
}

} // namespace detail

} // namespace dart_demos

#endif // DART_EXAMPLES_DEMOS_RUNTIMEOPTIONS_HPP_
