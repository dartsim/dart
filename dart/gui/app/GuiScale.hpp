/* Copyright (c) 2026, The DART development contributors.
 * All rights reserved. See LICENSE for the BSD-style license.
 */
#ifndef DART_GUI_APP_GUI_SCALE_HPP_
#define DART_GUI_APP_GUI_SCALE_HPP_

#include <dart/external/imgui/imgui.h>

#include <cmath>

namespace dart::gui::app {

/// ImGui layout uses logical window coordinates.
inline double guiLayoutScale(double scale)
{
  if (ImGui::GetCurrentContext()) {
    const float framebufferScale = ImGui::GetIO().DisplayFramebufferScale.y;
    if (framebufferScale > 0)
      return scale / framebufferScale;
  }
  return scale;
}

inline void applyGuiScale(double scale)
{
  const float sanitized
      = static_cast<float>(std::isfinite(scale) && scale > 0 ? scale : 1.0);
  ImGui::GetStyle().ScaleAllSizes(sanitized);
#if IMGUI_VERSION_NUM >= 19200
  ImGui::GetStyle().FontScaleMain = sanitized;
#else
  ImGui::GetIO().FontGlobalScale = sanitized;
#endif
}

} // namespace dart::gui::app

#endif // DART_GUI_APP_GUI_SCALE_HPP_
