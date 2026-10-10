/* Copyright (c) 2026, The DART development contributors.
 * All rights reserved. See LICENSE for the BSD-style license.
 */

#include "Theme.hpp"

#include <dart/gui/osg/ImGuiHandler.hpp>
#include <dart/gui/osg/Utils.hpp>

#include <gtest/gtest.h>

#include <memory>

#include <cmath>

namespace dart_demos {
namespace {

class GuiScaleThemeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    mHandler = new dart::gui::osg::ImGuiHandler();
    auto& io = ImGui::GetIO();
    io.IniFilename = nullptr;
    io.DisplaySize = ImVec2(1600, 1000);
    io.DeltaTime = 1.0f / 60.0f;
#ifndef IMGUI_HAS_TEXTURES
    unsigned char* pixels;
    int width, height;
    io.Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);
#endif
    applyModernDarkColors();
    applyModernDarkMetrics();
    mBaseline = ImGui::GetStyle();
    mTheme = std::make_unique<GuiScaleTheme>(*mHandler);
  }

  void TearDown() override
  {
    mTheme.reset();
    ImGui_ImplOpenGL2_Shutdown();
    ImGui::DestroyContext();
    mHandler = nullptr;
  }

  void beginFrame(double scale)
  {
    mHandler->setGuiScale(scale);
    dart::gui::osg::applyImGuiScale(scale, &mPreviousScale);
    ImGui::NewFrame();
    ImGui::Begin("About");
  }

  void endFrame()
  {
    ImGui::End();
    ImGui::Render();
  }

  ::osg::ref_ptr<dart::gui::osg::ImGuiHandler> mHandler;
  std::unique_ptr<GuiScaleTheme> mTheme;
  ImGuiStyle mBaseline;
  double mPreviousScale = 1.0;
};

TEST_F(GuiScaleThemeTest, FractionalChangesRestoreMetricsAndFirstFrameFonts)
{
  beginFrame(1.0);
  const float baseFontSize = ImGui::GetFontSize();
  endFrame();

  for (int cycle = 0; cycle < 5; ++cycle) {
    for (const double scale : {1.25, 1.5, 2.0, 1.0}) {
      beginFrame(scale);
      const auto& style = ImGui::GetStyle();
      EXPECT_FLOAT_EQ(
          style.WindowPadding.x,
          std::floor(mBaseline.WindowPadding.x * static_cast<float>(scale)));
      EXPECT_FLOAT_EQ(
          style.FramePadding.y,
          std::floor(mBaseline.FramePadding.y * static_cast<float>(scale)));
      EXPECT_FLOAT_EQ(
          style.ScrollbarSize,
          std::floor(mBaseline.ScrollbarSize * static_cast<float>(scale)));
#if IMGUI_VERSION_NUM >= 18991
      EXPECT_FLOAT_EQ(ImGui::GetFontSize(), std::round(baseFontSize * scale));
#else
      EXPECT_FLOAT_EQ(ImGui::GetFontSize(), baseFontSize * scale);
#endif
      ImGui::TextUnformatted("Readable at the new scale on its first frame");
      endFrame();
    }
  }
}

TEST_F(GuiScaleThemeTest, RemovingThemeStopsItsFrameHook)
{
  beginFrame(2.0);
  endFrame();
  mTheme.reset();
  ImGui::GetStyle().WindowPadding.x = 17.0f;
  beginFrame(1.0);
  EXPECT_FLOAT_EQ(ImGui::GetStyle().WindowPadding.x, 8.0f);
  endFrame();
}

} // namespace
} // namespace dart_demos
