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

#include <dart/common/Deprecated.hpp>

#include <osgViewer/api/Cocoa/GraphicsWindowCocoa>

#import <Cocoa/Cocoa.h>

#include <cmath>

namespace dart_demos {

//==============================================================================
DisplayMetrics queryCocoaDisplayMetrics(::osgViewer::GraphicsWindow& window)
{
  @autoreleasepool {
    auto* native = dynamic_cast<::osgViewer::GraphicsWindowCocoa*>(&window);
    if (native == nullptr || native->getWindow() == nullptr)
      return {};

    NSWindow* nativeWindow = reinterpret_cast<NSWindow*>(native->getWindow());
    NSView* view = [nativeWindow contentView];
    DART_SUPPRESS_DEPRECATED_BEGIN
    if ([view isKindOfClass:[NSOpenGLView class]]) {
      auto* glView = reinterpret_cast<NSOpenGLView*>(view);
      // OSG reports viewport sizes and input in logical points.
      if ([glView wantsBestResolutionOpenGLSurface])
        [glView setWantsBestResolutionOpenGLSurface:NO];
    }
    DART_SUPPRESS_DEPRECATED_END

    DisplayMetrics metrics;
    metrics.guiScale = 1.0;
    NSScreen* screen = [nativeWindow screen];
    if (screen == nil)
      return metrics;

    const NSRect frame = [nativeWindow frame];
    const NSRect client = [nativeWindow contentRectForFrameRect:frame];
    const NSRect visible = [screen visibleFrame];
    const CGFloat left = NSMinX(client) - NSMinX(frame);
    const CGFloat top = NSMaxY(frame) - NSMaxY(client);
    const CGFloat right = NSMaxX(frame) - NSMaxX(client);
    const CGFloat bottom = NSMinY(client) - NSMinY(frame);

    WindowRectangle current;
    window.getWindowRectangle(
        current.x, current.y, current.width, current.height);
    const WindowRectangle workArea{
        static_cast<int>(
            std::ceil(NSMinX(visible) + left - (NSMinX(client) - current.x))),
        static_cast<int>(
            std::ceil(current.y + NSMaxY(client) - NSMaxY(visible) + top)),
        static_cast<int>(std::floor(NSWidth(visible) - left - right)),
        static_cast<int>(std::floor(NSHeight(visible) - top - bottom))};
    if (workArea.width > 0 && workArea.height > 0)
      metrics.workArea = workArea;
    return metrics;
  }
}

} // namespace dart_demos
