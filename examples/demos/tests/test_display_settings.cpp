/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License.
 */

#include "../DisplaySettings.hpp"

#include <gtest/gtest.h>

#include <limits>

#if defined(DART_DEMOS_HAVE_X11)
  #include <osgViewer/api/X11/GraphicsWindowX11>

  #include <cstdlib>
#endif

namespace dart_demos {
namespace {

//==============================================================================
TEST(DisplaySettings, DpiUsesDesktopScalingAndSharedLimits)
{
  EXPECT_EQ(guiScaleFromDpi(96), 1.0);
  EXPECT_EQ(guiScaleFromDpi(120), 1.25);
  EXPECT_EQ(guiScaleFromDpi(144), 1.5);
  EXPECT_EQ(guiScaleFromDpi(192), 2.0);
  EXPECT_EQ(guiScaleFromDpi(24), 0.5);
  EXPECT_EQ(guiScaleFromDpi(768), 4.0);
  EXPECT_FALSE(guiScaleFromDpi(0));
  EXPECT_FALSE(guiScaleFromDpi(-96));
  EXPECT_FALSE(guiScaleFromDpi(std::numeric_limits<double>::infinity()));
  EXPECT_FALSE(guiScaleFromDpi(-std::numeric_limits<double>::infinity()));
  EXPECT_FALSE(guiScaleFromDpi(std::numeric_limits<double>::quiet_NaN()));
}

//==============================================================================
TEST(DisplaySettings, MissingWorkAreaPreservesPositionAndScalesDefaults)
{
  const WindowRectangle current{-100, 35, 50, 60};
  const auto result = fitInitialWindow(current, {}, 1.25);
  EXPECT_EQ(result.x, -100);
  EXPECT_EQ(result.y, 35);
  EXPECT_EQ(result.width, 2000);
  EXPECT_EQ(result.height, 1250);
  EXPECT_EQ(fitInitialWindow(current, {}, 0).width, 800);
  EXPECT_EQ(fitInitialWindow(current, {}, 10).width, 6400);
  EXPECT_EQ(
      fitInitialWindow(current, {}, std::numeric_limits<double>::quiet_NaN())
          .height,
      1000);
}

//==============================================================================
TEST(DisplaySettings, DefaultsFitProportionallyAndCenterOnLargeMonitor)
{
  const DisplayMetrics display{{}, WindowRectangle{0, 0, 3840, 2160}};
  const auto result = fitInitialWindow({}, display, 2.0);
  EXPECT_EQ(result.width, 3110);
  EXPECT_EQ(result.height, 1944);
  EXPECT_EQ(result.x, 365);
  EXPECT_EQ(result.y, 108);
  EXPECT_NEAR(static_cast<double>(result.width) / result.height, 1.6, 0.001);
}

//==============================================================================
TEST(DisplaySettings, DefaultsFitSmallMonitorAndKeepAspectRatio)
{
  const DisplayMetrics display{{}, WindowRectangle{0, 0, 640, 480}};
  const auto result = fitInitialWindow({}, display, 2.0);
  EXPECT_EQ(result.width, 576);
  EXPECT_EQ(result.height, 360);
  EXPECT_EQ(result.x, 32);
  EXPECT_EQ(result.y, 60);
}

//==============================================================================
TEST(DisplaySettings, MonitorOriginsCanBeNegative)
{
  const DisplayMetrics display{{}, WindowRectangle{-1920, -200, 1920, 1080}};
  const auto result = fitInitialWindow({}, display, 1.0);
  EXPECT_EQ(result.width, 1555);
  EXPECT_EQ(result.height, 972);
  EXPECT_EQ(result.x, -1738);
  EXPECT_EQ(result.y, -146);
}

//==============================================================================
TEST(DisplaySettings, ExplicitDimensionsRemainExactEvenOutsideWorkArea)
{
  const DisplayMetrics display{{}, WindowRectangle{0, 0, 1920, 1080}};
  const auto result = fitInitialWindow({}, display, 2.0, 2000, 1500);
  EXPECT_EQ(result.width, 2000);
  EXPECT_EQ(result.height, 1500);
  EXPECT_EQ(result.x, -40);
  EXPECT_EQ(result.y, -210);
  const auto unavailable = fitInitialWindow({20, 40, 0, 0}, {}, 2.0, 800, 600);
  EXPECT_EQ(unavailable.width, 800);
  EXPECT_EQ(unavailable.height, 600);
  EXPECT_EQ(unavailable.x, 20);
  EXPECT_EQ(unavailable.y, 40);
}

//==============================================================================
TEST(DisplaySettings, OneExplicitDimensionOnlyFitsTheUnspecifiedDimension)
{
  const DisplayMetrics display{{}, WindowRectangle{0, 0, 1920, 1080}};
  const auto explicitWidth = fitInitialWindow({}, display, 2.0, 1000);
  EXPECT_EQ(explicitWidth.width, 1000);
  EXPECT_EQ(explicitWidth.height, 972);
  EXPECT_EQ(explicitWidth.x, 460);
  EXPECT_EQ(explicitWidth.y, 54);
  const auto explicitHeight = fitInitialWindow({}, display, 2.0, {}, 700);
  EXPECT_EQ(explicitHeight.width, 1728);
  EXPECT_EQ(explicitHeight.height, 700);
  EXPECT_EQ(explicitHeight.x, 96);
  EXPECT_EQ(explicitHeight.y, 190);
}

//==============================================================================
TEST(DisplaySettings, DegenerateWorkAreasDoNotCreateZeroSizedWindows)
{
  const auto tiny
      = fitInitialWindow({}, {{}, WindowRectangle{0, 0, 1, 1}}, 2.0);
  EXPECT_EQ(tiny.width, 1);
  EXPECT_EQ(tiny.height, 1);
  const auto invalid = fitInitialWindow(
      {20, 40, 0, 0}, {{}, WindowRectangle{0, 0, 0, 1000}}, 2.0);
  EXPECT_EQ(invalid.width, 3200);
  EXPECT_EQ(invalid.height, 2000);
  EXPECT_EQ(invalid.x, 20);
  EXPECT_EQ(invalid.y, 40);
}

#if defined(DART_DEMOS_HAVE_X11)

//==============================================================================
TEST(DisplaySettings, NativeX11QueryDoesNotMoveOrResizeTheWindow)
{
  Display* display = XOpenDisplay(nullptr);
  if (display == nullptr)
    GTEST_SKIP() << "No X11 display is available";
  XCloseDisplay(display);

  ::osg::ref_ptr<::osg::GraphicsContext::Traits> traits
      = new ::osg::GraphicsContext::Traits;
  traits->readDISPLAY();
  traits->setUndefinedScreenDetailsToDefaultScreen();
  traits->x = 20;
  traits->y = 20;
  traits->width = 64;
  traits->height = 64;
  traits->windowDecoration = false;
  traits->doubleBuffer = true;
  ::osg::ref_ptr<::osg::GraphicsContext> context
      = ::osg::GraphicsContext::createGraphicsContext(traits);
  ASSERT_TRUE(context.valid());
  ASSERT_TRUE(context->realize());
  auto* window = dynamic_cast<::osgViewer::GraphicsWindowX11*>(context.get());
  ASSERT_NE(window, nullptr);

  WindowRectangle before;
  window->getWindowRectangle(before.x, before.y, before.width, before.height);
  const auto first = queryDisplayMetrics(*window);
  const auto second = queryDisplayMetrics(*window);
  ASSERT_TRUE(first.workArea);
  EXPECT_GT(first.workArea->width, 0);
  EXPECT_GT(first.workArea->height, 0);
  EXPECT_EQ(second.guiScale, first.guiScale);
  if (first.guiScale) {
    EXPECT_GE(*first.guiScale, 0.5);
    EXPECT_LE(*first.guiScale, 4.0);
  }
  // Native placement may be newer than OSG's cached window position.
  traits->x += 70;
  traits->y += 40;
  const auto stalePosition = queryDisplayMetrics(*window);
  ASSERT_TRUE(stalePosition.workArea);
  EXPECT_EQ(stalePosition.workArea->x, first.workArea->x);
  EXPECT_EQ(stalePosition.workArea->y, first.workArea->y);
  traits->x = before.x;
  traits->y = before.y;
  WindowRectangle after;
  window->getWindowRectangle(after.x, after.y, after.width, after.height);
  EXPECT_EQ(after.x, before.x);
  EXPECT_EQ(after.y, before.y);
  EXPECT_EQ(after.width, before.width);
  EXPECT_EQ(after.height, before.height);
  context->close();
}

#endif

} // namespace
} // namespace dart_demos

#if defined(DART_DEMOS_HAVE_X11)
  #undef None
  #undef Bool
  #undef Status
  #undef Success
  #undef Always
  #undef Complex
  #undef GLX_GLXEXT_PROTOTYPES
#endif
