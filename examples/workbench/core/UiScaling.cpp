#include "UiScaling.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>

#if defined(__linux__) && !defined(__ANDROID__)
#  include <X11/Xlib.h>
#endif

namespace workbench {
namespace {

float clampScale(float scale)
{
  if (!std::isfinite(scale) || scale <= 0.0f)
    return 1.0f;

  return std::clamp(scale, 0.5f, 3.0f);
}

float parseEnvScale(const char* name)
{
  const char* value = std::getenv(name);
  if (!value || *value == '\0')
    return 0.0f;

  char* end = nullptr;
  const float parsed = std::strtof(value, &end);
  if (end == value)
    return 0.0f;

  return parsed;
}

float queryX11DpiScale()
{
#if defined(__linux__) && !defined(__ANDROID__)
  Display* display = XOpenDisplay(nullptr);
  if (!display)
    return 0.0f;

  const int screen = DefaultScreen(display);
  const double widthPx = static_cast<double>(DisplayWidth(display, screen));
  const double widthMm = static_cast<double>(DisplayWidthMM(display, screen));
  XCloseDisplay(display);

  if (widthPx <= 0.0 || widthMm <= 0.0)
    return 0.0f;

  const double dpi = widthPx * 25.4 / widthMm;
  if (!std::isfinite(dpi) || dpi <= 0.0)
    return 0.0f;

  return static_cast<float>(dpi / 96.0);
#else
  return 0.0f;
#endif
}

} // namespace

float detectUiScale()
{
  // Highest priority: explicit override
  const std::array<const char*, 3> overrideNames{
      "DART_WORKBENCH_UI_SCALE", "DART_UI_SCALE", "IMGUI_UI_SCALE"};
  for (const char* name : overrideNames) {
    const float value = parseEnvScale(name);
    if (value > 0.0f)
      return clampScale(value);
  }

  // Next: honour GTK/QT conventions when present.
  float gtkScale = parseEnvScale("GDK_SCALE");
  float gtkDpiScale = parseEnvScale("GDK_DPI_SCALE");
  float qtScale = parseEnvScale("QT_SCALE_FACTOR");

  float combinedGtkScale = 1.0f;
  bool haveGtkHint = false;

  if (gtkScale > 0.0f) {
    combinedGtkScale *= gtkScale;
    haveGtkHint = true;
  }
  if (gtkDpiScale > 0.0f) {
    combinedGtkScale *= gtkDpiScale;
    haveGtkHint = true;
  }
  if (haveGtkHint)
    return clampScale(combinedGtkScale);

  if (qtScale > 0.0f)
    return clampScale(qtScale);

  // Fallback: approximate from monitor DPI.
  const float dpiScale = queryX11DpiScale();
  if (dpiScale > 0.0f)
    return clampScale(dpiScale);

  return 1.0f;
}

} // namespace workbench
