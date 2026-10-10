/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License.
 */

#include "DisplaySettings.hpp"

#include <dart/gui/osg/Utils.hpp>

#include <osgViewer/GraphicsWindow>

#include <algorithm>
#include <limits>

#include <cmath>
#include <cstdint>

#if defined(DART_DEMOS_HAVE_X11)
  #include <X11/Xatom.h>
  #include <X11/Xresource.h>
  #include <osgViewer/api/X11/GraphicsWindowX11>
  #if defined(DART_DEMOS_HAVE_XRANDR)
    #include <X11/extensions/Xrandr.h>
  #endif

  #include <string>
  #include <vector>

  #include <cctype>
  #include <cstdlib>
#elif defined(_WIN32)
  #if !defined(NOMINMAX)
    #define NOMINMAX
    #define DART_DEMOS_UNDEFINE_NOMINMAX
  #endif
  #include <osgViewer/api/Win32/GraphicsWindowWin32>
#endif

namespace dart_demos {
namespace {

//==============================================================================
int displayCoordinate(std::int64_t coordinate)
{
  return static_cast<int>(std::clamp<std::int64_t>(
      coordinate,
      std::numeric_limits<int>::min(),
      std::numeric_limits<int>::max()));
}

#if defined(DART_DEMOS_HAVE_X11)

//==============================================================================
WindowRectangle displayIntersection(
    const WindowRectangle& first, const WindowRectangle& second)
{
  const std::int64_t x = std::max(first.x, second.x);
  const std::int64_t y = std::max(first.y, second.y);
  const auto right = std::min(
      std::int64_t{first.x} + first.width,
      std::int64_t{second.x} + second.width);
  const auto bottom = std::min(
      std::int64_t{first.y} + first.height,
      std::int64_t{second.y} + second.height);
  return {
      displayCoordinate(x),
      displayCoordinate(y),
      displayCoordinate(std::max<std::int64_t>(0, right - x)),
      displayCoordinate(std::max<std::int64_t>(0, bottom - y))};
}

//==============================================================================
std::vector<unsigned long> displayCardinals(
    Display* display, Window root, const char* name, long maxItems)
{
  const Atom property = XInternAtom(display, name, True);
  if (property == None)
    return {};

  Atom type = None;
  int format = 0;
  unsigned long count = 0;
  unsigned long remaining = 0;
  unsigned char* data = nullptr;
  const int status = XGetWindowProperty(
      display,
      root,
      property,
      0,
      maxItems,
      False,
      XA_CARDINAL,
      &type,
      &format,
      &count,
      &remaining,
      &data);
  std::vector<unsigned long> values;
  if (status == Success && type == XA_CARDINAL && format == 32 && remaining == 0
      && count <= static_cast<unsigned long>(maxItems) && data != nullptr) {
    const auto* cardinals = reinterpret_cast<unsigned long*>(data);
    values.assign(cardinals, cardinals + count);
  }
  if (data != nullptr)
    XFree(data);
  return values;
}

//==============================================================================
std::optional<double> displayXftScale(Display* display, Window root)
{
  Atom type = None;
  int format = 0;
  unsigned long count = 0;
  unsigned long remaining = 0;
  unsigned char* data = nullptr;
  // XResourceManagerString caches the resource database opened with Display.
  const int status = XGetWindowProperty(
      display,
      root,
      XA_RESOURCE_MANAGER,
      0,
      16384,
      False,
      XA_STRING,
      &type,
      &format,
      &count,
      &remaining,
      &data);
  std::string resources;
  if (status == Success && type == XA_STRING && format == 8 && remaining == 0
      && count <= 65536 && data != nullptr)
    resources.assign(reinterpret_cast<char*>(data), count);
  if (data != nullptr)
    XFree(data);
  if (resources.empty())
    return {};

  XrmInitialize();
  XrmDatabase database = XrmGetStringDatabase(resources.c_str());
  if (database == nullptr)
    return {};

  char* resourceType = nullptr;
  XrmValue value{};
  std::string dpi;
  if (XrmGetResource(database, "Xft.dpi", "Xft.Dpi", &resourceType, &value)
      && value.addr != nullptr && value.size > 0 && value.size <= 128)
    dpi.assign(value.addr, value.size);
  XrmDestroyDatabase(database);
  if (dpi.empty())
    return {};

  char* end = nullptr;
  const double parsed = std::strtod(dpi.c_str(), &end);
  if (end == dpi.c_str())
    return {};
  while (std::isspace(static_cast<unsigned char>(*end)))
    ++end;
  return *end == '\0' ? guiScaleFromDpi(parsed) : std::nullopt;
}

//==============================================================================
std::optional<WindowRectangle> displayX11Monitor(
    Display* display, Window root, const WindowRectangle& current)
{
  std::optional<WindowRectangle> monitor;
  #if defined(DART_DEMOS_HAVE_XRANDR)
  int major = 0;
  int minor = 0;
  if (XRRQueryVersion(display, &major, &minor)
      && (major > 1 || (major == 1 && minor >= 2))) {
    XRRScreenResources* resources
        = major > 1 || minor >= 3 ? XRRGetScreenResourcesCurrent(display, root)
                                  : XRRGetScreenResources(display, root);
    if (resources != nullptr) {
      std::int64_t largestIntersection = -1;
      double closestCenter = std::numeric_limits<double>::infinity();
      for (int index = 0; index < resources->ncrtc; ++index) {
        XRRCrtcInfo* info
            = XRRGetCrtcInfo(display, resources, resources->crtcs[index]);
        if (info == nullptr)
          continue;
        if (info->width > 0 && info->height > 0
            && info->width <= std::numeric_limits<int>::max()
            && info->height <= std::numeric_limits<int>::max()) {
          const WindowRectangle candidate{
              info->x,
              info->y,
              static_cast<int>(info->width),
              static_cast<int>(info->height)};
          const auto intersection = displayIntersection(candidate, current);
          const auto area
              = std::int64_t{intersection.width} * intersection.height;
          const double dx = candidate.x + candidate.width * 0.5
                            - (current.x + current.width * 0.5);
          const double dy = candidate.y + candidate.height * 0.5
                            - (current.y + current.height * 0.5);
          const double distance = dx * dx + dy * dy;
          if (area > largestIntersection
              || (area == largestIntersection && distance < closestCenter)) {
            monitor = candidate;
            largestIntersection = area;
            closestCenter = distance;
          }
        }
        XRRFreeCrtcInfo(info);
      }
      XRRFreeScreenResources(resources);
    }
  }
  #else
  (void)display;
  (void)root;
  (void)current;
  #endif
  return monitor;
}

//==============================================================================
DisplayMetrics displayX11Metrics(::osgViewer::GraphicsWindow& window)
{
  auto* native = dynamic_cast<::osgViewer::GraphicsWindowX11*>(&window);
  if (native == nullptr || native->getDisplay() == nullptr
      || native->getWindow() == None)
    return {};

  Display* display = native->getDisplay();
  XWindowAttributes attributes{};
  if (!XGetWindowAttributes(display, native->getWindow(), &attributes))
    return {};
  const Window root = attributes.root;
  DisplayMetrics metrics;
  metrics.guiScale = displayXftScale(display, root);

  WindowRectangle current;
  window.getWindowRectangle(
      current.x, current.y, current.width, current.height);
  int rootX = current.x;
  int rootY = current.y;
  Window child = None;
  XTranslateCoordinates(
      display, native->getWindow(), root, 0, 0, &rootX, &rootY, &child);
  current.x = rootX;
  current.y = rootY;
  auto monitor = displayX11Monitor(display, root, current);
  if (!monitor) {
    auto* interface = ::osg::GraphicsContext::getWindowingSystemInterface();
    const auto* traits = window.getTraits();
    if (interface != nullptr && traits != nullptr) {
      ::osg::GraphicsContext::ScreenSettings settings;
      interface->getScreenSettings(*traits, settings);
      if (settings.width > 0 && settings.height > 0)
        monitor = WindowRectangle{0, 0, settings.width, settings.height};
    }
  }
  if (!monitor)
    return metrics;

  const auto desktop
      = displayCardinals(display, root, "_NET_CURRENT_DESKTOP", 1);
  const auto workAreas = displayCardinals(display, root, "_NET_WORKAREA", 1024);
  if (desktop.size() == 1 && workAreas.size() % 4 == 0
      && desktop[0] < workAreas.size() / 4) {
    const auto offset = desktop[0] * 4;
    if (workAreas[offset + 2] > 0 && workAreas[offset + 3] > 0
        && workAreas[offset + 2] <= std::numeric_limits<int>::max()
        && workAreas[offset + 3] <= std::numeric_limits<int>::max()) {
      const WindowRectangle workArea{
          static_cast<std::int32_t>(workAreas[offset]),
          static_cast<std::int32_t>(workAreas[offset + 1]),
          static_cast<int>(workAreas[offset + 2]),
          static_cast<int>(workAreas[offset + 3])};
      const auto intersection = displayIntersection(*monitor, workArea);
      if (intersection.width > 0 && intersection.height > 0)
        monitor = intersection;
    }
  }
  metrics.workArea = monitor;
  return metrics;
}

#elif defined(_WIN32)

//==============================================================================
DisplayMetrics displayWin32Metrics(::osgViewer::GraphicsWindow& window)
{
  auto* native = dynamic_cast<::osgViewer::GraphicsWindowWin32*>(&window);
  if (native == nullptr || native->getHWND() == nullptr)
    return {};

  const HWND handle = native->getHWND();
  DisplayMetrics metrics;
  using DisplayGetDpiForWindow = UINT(WINAPI*)(HWND);
  const HMODULE user32 = GetModuleHandleW(L"user32.dll");
  if (user32 != nullptr) {
    const auto getDpi = reinterpret_cast<DisplayGetDpiForWindow>(
        GetProcAddress(user32, "GetDpiForWindow"));
    if (getDpi != nullptr)
      metrics.guiScale = guiScaleFromDpi(getDpi(handle));
  }

  MONITORINFO info{};
  info.cbSize = sizeof(info);
  const HMONITOR monitor = MonitorFromWindow(handle, MONITOR_DEFAULTTONEAREST);
  POINT clientOrigin{};
  RECT frame{};
  RECT client{};
  if (monitor == nullptr || !GetMonitorInfoW(monitor, &info)
      || !ClientToScreen(handle, &clientOrigin)
      || !GetWindowRect(handle, &frame) || !GetClientRect(handle, &client))
    return metrics;

  WindowRectangle current;
  window.getWindowRectangle(
      current.x, current.y, current.width, current.height);
  const auto left = std::int64_t{clientOrigin.x} - frame.left;
  const auto top = std::int64_t{clientOrigin.y} - frame.top;
  const auto right = std::int64_t{frame.right} - clientOrigin.x
                     - (client.right - client.left);
  const auto bottom = std::int64_t{frame.bottom} - clientOrigin.y
                      - (client.bottom - client.top);
  WindowRectangle workArea{
      displayCoordinate(
          std::int64_t{info.rcWork.left} + left - (clientOrigin.x - current.x)),
      displayCoordinate(
          std::int64_t{info.rcWork.top} + top - (clientOrigin.y - current.y)),
      displayCoordinate(
          std::int64_t{info.rcWork.right} - info.rcWork.left - left - right),
      displayCoordinate(
          std::int64_t{info.rcWork.bottom} - info.rcWork.top - top - bottom)};
  if (workArea.width > 0 && workArea.height > 0)
    metrics.workArea = workArea;
  return metrics;
}

#endif

} // namespace

#if defined(__APPLE__)
DisplayMetrics queryCocoaDisplayMetrics(::osgViewer::GraphicsWindow& window);
#endif

//==============================================================================
std::optional<double> guiScaleFromDpi(double dpi)
{
  if (!std::isfinite(dpi) || dpi <= 0)
    return {};
  return dart::gui::osg::sanitizeGuiScale(dpi / 96.0);
}

//==============================================================================
DisplayMetrics queryDisplayMetrics(::osgViewer::GraphicsWindow& window)
{
#if defined(DART_DEMOS_HAVE_X11)
  return displayX11Metrics(window);
#elif defined(_WIN32)
  return displayWin32Metrics(window);
#elif defined(__APPLE__)
  return queryCocoaDisplayMetrics(window);
#else
  (void)window;
  return {};
#endif
}

//==============================================================================
WindowRectangle fitInitialWindow(
    const WindowRectangle& current,
    const DisplayMetrics& metrics,
    double guiScale,
    std::optional<int> width,
    std::optional<int> height)
{
  WindowRectangle result{
      current.x,
      current.y,
      width.value_or(dart::gui::osg::scaleWindowExtent(1600, guiScale)),
      height.value_or(dart::gui::osg::scaleWindowExtent(1000, guiScale))};
  if (!metrics.workArea || metrics.workArea->width <= 0
      || metrics.workArea->height <= 0)
    return result;

  const auto& area = *metrics.workArea;
  const int maxWidth = std::max(1, static_cast<int>(area.width * 0.9));
  const int maxHeight = std::max(1, static_cast<int>(area.height * 0.9));
  if (!width && !height) {
    const double fit = std::min(
        {1.0,
         static_cast<double>(maxWidth) / result.width,
         static_cast<double>(maxHeight) / result.height});
    result.width
        = std::max(1, static_cast<int>(std::lround(result.width * fit)));
    result.height
        = std::max(1, static_cast<int>(std::lround(result.height * fit)));
  } else {
    if (!width)
      result.width = std::min(result.width, maxWidth);
    if (!height)
      result.height = std::min(result.height, maxHeight);
  }
  result.x = displayCoordinate(
      std::int64_t{area.x} + (std::int64_t{area.width} - result.width) / 2);
  result.y = displayCoordinate(
      std::int64_t{area.y} + (std::int64_t{area.height} - result.height) / 2);
  return result;
}

} // namespace dart_demos

#if defined(DART_DEMOS_HAVE_X11)
  #undef None
  #undef Bool
  #undef Status
  #undef Success
  #undef Always
  #undef Complex
  #undef GLX_GLXEXT_PROTOTYPES
#elif defined(_WIN32)
  #if defined(DART_DEMOS_UNDEFINE_NOMINMAX)
    #undef NOMINMAX
    #undef DART_DEMOS_UNDEFINE_NOMINMAX
  #endif
#endif
