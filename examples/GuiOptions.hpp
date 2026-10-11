/* Copyright (c) 2026, The DART development contributors.
 * All rights reserved. See LICENSE for the BSD-style license.
 */
#ifndef DART_EXAMPLES_GUIOPTIONS_HPP_
#define DART_EXAMPLES_GUIOPTIONS_HPP_

#include <algorithm>
#include <optional>
#include <ostream>
#include <string>

#include <cmath>
#include <cstdlib>

namespace dart_examples {

inline double sanitizeGuiScale(double scale)
{
  return std::isfinite(scale) ? std::clamp(scale, 0.5, 4.0) : 1.0;
}

inline double parseGuiScale(
    const std::string& value, double fallback, std::ostream* errorStream)
{
  char* end = nullptr;
  const double parsed = std::strtod(value.c_str(), &end);
  if (end == value.c_str() || *end != '\0' || !std::isfinite(parsed)
      || parsed <= 0) {
    if (errorStream)
      *errorStream << "Invalid --gui-scale value '" << value
                   << "'. Falling back to " << fallback << ".\n";
    return fallback;
  }
  const double clamped = sanitizeGuiScale(parsed);
  if (clamped != parsed && errorStream)
    *errorStream << "--gui-scale " << value
                 << " is outside the supported range [0.5, 4]; clamped to "
                 << clamped << ".\n";
  return clamped;
}

inline int scaleWindowExtent(int extent, double scale)
{
  return std::max(1, static_cast<int>(extent * sanitizeGuiScale(scale) + 0.5));
}

struct GuiScaleOptions
{
  std::optional<double> scale;
  bool showHelp = false;
};

inline GuiScaleOptions parseGuiScaleOptions(
    int argc, char* argv[], std::ostream* errorStream)
{
  GuiScaleOptions options;
  if (const char* value = std::getenv("DART_GUI_SCALE"))
    options.scale = parseGuiScale(value, 1.0, errorStream);
  const std::string prefix = "--gui-scale=";
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "-h" || arg == "--help")
      options.showHelp = true;
    else if (arg == "--gui-scale") {
      if (i + 1 < argc)
        options.scale = parseGuiScale(
            argv[++i], options.scale.value_or(1.0), errorStream);
      else if (errorStream)
        *errorStream << "--gui-scale requires a value.\n";
    } else if (arg.compare(0, prefix.size(), prefix) == 0)
      options.scale = parseGuiScale(
          arg.substr(prefix.size()), options.scale.value_or(1.0), errorStream);
    else if (errorStream)
      *errorStream << "Ignoring unknown option '" << arg << "'.\n";
  }
  return options;
}

inline void printGuiScaleUsage(std::ostream& stream, const char* executable)
{
  stream << "Usage: " << executable << " [--gui-scale SCALE]\n"
         << "  --gui-scale SCALE   Override automatic DPI scaling.\n"
         << "  --gui-scale=SCALE   Same as --gui-scale SCALE.\n"
         << "  -h, --help          Show this help message.\n";
}

} // namespace dart_examples
#endif // DART_EXAMPLES_GUIOPTIONS_HPP_
