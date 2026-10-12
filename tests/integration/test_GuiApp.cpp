/* Copyright (c) 2026, The DART development contributors.
 * All rights reserved. See LICENSE for the BSD-style license.
 */

#include "GuiOptions.hpp"

#include <dart/gui/app/App.hpp>
#include <dart/gui/app/Theme.hpp>

#include <gtest/gtest.h>

#include <limits>
#include <memory>
#include <sstream>

#include <cmath>

namespace dart::gui::app {
namespace {

class GuiScaleThemeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    mPreviousContext = ImGui::GetCurrentContext();
    mContext = ImGui::CreateContext();
    mViewer = std::make_unique<dart::gui::Viewer>();
    auto& io = ImGui::GetIO();
    io.IniFilename = nullptr;
    io.DisplaySize = ImVec2(1600, 1000);
    io.DeltaTime = 1.0f / 60.0f;
#if IMGUI_VERSION_NUM >= 19200
    io.BackendFlags |= ImGuiBackendFlags_RendererHasTextures;
    io.Fonts->AddFontDefault();
#else
    unsigned char* pixels;
    int width, height;
    io.Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);
#endif
    applyModernDarkColors();
    applyModernDarkMetrics();
    mBaseline = ImGui::GetStyle();
    mTheme = std::make_unique<GuiScaleTheme>(*mViewer);
  }

  void TearDown() override
  {
    mTheme.reset();
    mViewer.reset();
    ImGui::DestroyContext(mContext);
    ImGui::SetCurrentContext(mPreviousContext);
  }

  void beginFrame(double scale)
  {
    mViewer->setGuiScale(scale);
    ImGui::NewFrame();
    ImGui::Begin("About");
  }

  void endFrame()
  {
    ImGui::End();
    ImGui::Render();
  }

  std::unique_ptr<dart::gui::Viewer> mViewer;
  ImGuiContext* mContext = nullptr;
  ImGuiContext* mPreviousContext = nullptr;
  std::unique_ptr<GuiScaleTheme> mTheme;
  ImGuiStyle mBaseline;
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
  EXPECT_FLOAT_EQ(ImGui::GetStyle().WindowPadding.x, 17.0f);
  endFrame();
}

TEST_F(GuiScaleThemeTest, RetinaFramebufferKeepsLogicalMetrics)
{
  ImGui::GetIO().DisplayFramebufferScale = ImVec2(2.0f, 2.0f);
  beginFrame(2.0);
  EXPECT_FLOAT_EQ(ImGui::GetStyle().WindowPadding.x, mBaseline.WindowPadding.x);
  endFrame();
  beginFrame(3.0);
  EXPECT_FLOAT_EQ(
      ImGui::GetStyle().WindowPadding.x,
      std::floor(mBaseline.WindowPadding.x * 1.5f));
  endFrame();
}

TEST(SceneCycle, MissingAssetsAreSkippedAndPreviousSceneIsPreserved)
{
  Scene missing;
  missing.id = "missing";
  missing.factory = []() -> SceneSetup {
    throw AssetsNotPrefetched("Prefetch test assets");
  };
  Scene valid;
  valid.id = "valid";
  int activations = 0;
  int teardowns = 0;
  valid.factory = [&] {
    SceneSetup setup;
    setup.world = dart::simulation::World::create();
    setup.onActivate = [&](SceneContext& context) {
      ++activations;
      context.addTeardown([&] { ++teardowns; });
    };
    return setup;
  };
  {
    // The first miss installs a fallback; the second preserves the valid scene.
    App host({missing, valid}, std::nullopt);
    testing::internal::CaptureStdout();
    EXPECT_EQ(host.cycleScenes(0), 0);
    const auto output = testing::internal::GetCapturedStdout();
    EXPECT_NE(
        output.find(
            "[cycle-scenes] demo 'missing' skipped: Prefetch test assets"),
        std::string::npos);
    EXPECT_NE(output.find("2 skipped. OK"), std::string::npos);
    EXPECT_EQ(host.getActiveWorldNodeCount(), 1u);
    EXPECT_EQ(activations, 2);
    EXPECT_EQ(teardowns, 1);
  }
  EXPECT_EQ(teardowns, 2);
}

TEST(SceneCycle, OtherFactoryFailuresStillFail)
{
  Scene missing;
  missing.id = "missing";
  missing.factory = []() -> SceneSetup {
    throw AssetsNotPrefetched("Prefetch test assets");
  };
  for (int failure = 0; failure < 3; ++failure) {
    Scene broken;
    broken.id = "broken";
    broken.factory = [failure]() -> SceneSetup {
      if (failure == 0)
        throw std::runtime_error("Corrupt cached model");
      if (failure == 1)
        throw 42;
      return {};
    };
    App host({missing, broken}, std::nullopt);
    EXPECT_EQ(host.cycleScenes(0), 1);
    EXPECT_EQ(host.getActiveWorldNodeCount(), 1u);
  }
}

TEST(GuiScaleOptions, InvalidAndOutOfRangeOverridesStayUsable)
{
  for (const auto* value : {"", "0", "-1", "nan", "inf", "1.5trailing"})
    EXPECT_EQ(dart_examples::parseGuiScale(value, 1.25, nullptr), 1.25);
  EXPECT_EQ(dart_examples::parseGuiScale("1.5", 1.0, nullptr), 1.5);
  EXPECT_EQ(dart_examples::parseGuiScale("0.1", 1.0, nullptr), 0.5);
  EXPECT_EQ(dart_examples::parseGuiScale("100", 1.0, nullptr), 4.0);
  EXPECT_EQ(
      dart_examples::sanitizeGuiScale(std::numeric_limits<double>::quiet_NaN()),
      1.0);
  EXPECT_EQ(dart_examples::scaleWindowExtent(640, 1.5), 960);
}

TEST(GuiScaleOptions, EqualsAndSeparateArgumentsSelectManualScaling)
{
  char executable[] = "hello_world";
  char separate[] = "--gui-scale";
  char scale[] = "1.25";
  char equals[] = "--gui-scale=2";
  char help[] = "--help";
  char* arguments[] = {executable, separate, scale, equals, help};
  const auto options
      = dart_examples::parseGuiScaleOptions(5, arguments, nullptr);
  ASSERT_TRUE(options.scale);
  EXPECT_EQ(*options.scale, 2.0);
  EXPECT_TRUE(options.showHelp);
}

} // namespace
} // namespace dart::gui::app
