/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License.
 */

#ifndef DART_EXAMPLES_DEMOS_DISPLAYSETTINGS_HPP_
#define DART_EXAMPLES_DEMOS_DISPLAYSETTINGS_HPP_

#include <optional>

namespace osgViewer {
class GraphicsWindow;
}

namespace dart_demos {

struct WindowRectangle
{
  int x = 0;
  int y = 0;
  int width = 0;
  int height = 0;
};

struct DisplayMetrics
{
  std::optional<double> guiScale;
  std::optional<WindowRectangle> workArea;
};

std::optional<double> guiScaleFromDpi(double dpi);

DisplayMetrics queryDisplayMetrics(::osgViewer::GraphicsWindow& window);

WindowRectangle fitInitialWindow(
    const WindowRectangle& current,
    const DisplayMetrics& metrics,
    double guiScale,
    std::optional<int> width = {},
    std::optional<int> height = {});

} // namespace dart_demos

#endif // DART_EXAMPLES_DEMOS_DISPLAYSETTINGS_HPP_
