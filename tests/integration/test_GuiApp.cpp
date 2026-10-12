/* Copyright (c) 2026, The DART development contributors.
 * All rights reserved. See LICENSE for the BSD-style license.
 */

#include "GuiOptions.hpp"

#include <dart/gui/app/App.hpp>
#include <dart/gui/app/Theme.hpp>

#include <dart/constraint/ConstraintSolver.hpp>

#include <dart/collision/CollisionFilter.hpp>

#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/FreeJoint.hpp>

#include <gtest/gtest.h>

#include <filesystem>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <cmath>
#include <cstdio>

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
    // The cycle tears its last scene down too.
    EXPECT_EQ(host.getActiveWorldNodeCount(), 0u);
    EXPECT_EQ(activations, 2);
    EXPECT_EQ(teardowns, 2);
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
    EXPECT_EQ(host.getActiveWorldNodeCount(), 0u);
  }
}

//==============================================================================
// Control: the declarative scene panel.
//==============================================================================

/// Owns an ImGui context with a built font atlas so Control::render() can run
/// a full frame without a window or renderer.
class ScopedImGuiFrameContext
{
public:
  ScopedImGuiFrameContext() : mPrevious(ImGui::GetCurrentContext())
  {
    mContext = ImGui::CreateContext();
    ImGui::SetCurrentContext(mContext);
    ImGuiIO& io = ImGui::GetIO();
    io.DisplaySize = ImVec2(800.0f, 600.0f);
    io.DeltaTime = 1.0f / 60.0f;
    io.IniFilename = nullptr;
    // The test never uploads textures; claiming the capability lets ImGui
    // manage the font atlas itself.
    io.BackendFlags |= ImGuiBackendFlags_RendererHasTextures;
    io.Fonts->AddFontDefault();
  }

  ~ScopedImGuiFrameContext()
  {
    ImGui::DestroyContext(mContext);
    ImGui::SetCurrentContext(mPrevious);
  }

  template <typename Draw>
  void frame(Draw&& draw)
  {
    ImGui::NewFrame();
    ImGui::Begin("panel");
    draw();
    ImGui::End();
    ImGui::Render();
  }

private:
  ImGuiContext* mPrevious;
  ImGuiContext* mContext;
};

//==============================================================================
TEST(GuiAppControl, BuildersValidateAndSanitizeValues)
{
  auto panel = Control::makePanel();
  EXPECT_EQ(panel->getKind(), Control::Kind::Group);

  auto slider = panel->addSlider("gain", 5.0, 0.0, 2.0, 2);
  EXPECT_DOUBLE_EQ(slider->getValue().x(), 2.0);
  slider->setValue(Eigen::Vector4d(-1.0, 0.0, 0.0, 0.0));
  EXPECT_DOUBLE_EQ(slider->getValue().x(), 0.0);
  EXPECT_THROW(
      slider->setValue(Eigen::Vector4d(std::nan(""), 0.0, 0.0, 0.0)),
      std::invalid_argument);
  EXPECT_DOUBLE_EQ(slider->getValue().x(), 0.0);
  EXPECT_THROW(slider->setRange(1.0, 0.0), std::invalid_argument);
  constexpr double infinity = std::numeric_limits<double>::infinity();
  EXPECT_THROW(
      slider->setRange(0.0, std::numeric_limits<double>::infinity()),
      std::invalid_argument);
  EXPECT_THROW(
      panel->addSlider("wide", 0.0, -1e300, 1e300, 3, true),
      std::invalid_argument);
  EXPECT_THROW(
      panel->addSlider("digits", 0.0, 0.0, 1.0, 16), std::invalid_argument);

  auto count = panel->addInput("count", 2.6, 1.0, 0, true, 0.0, 10.0);
  EXPECT_DOUBLE_EQ(count->getValue().x(), 3.0);
  // Integer ranges shrink to whole bounds and must contain an integer.
  auto whole = panel->addInput("whole", 2.6, 1.0, 0, true, 0.2, 2.8);
  EXPECT_DOUBLE_EQ(whole->getMin(), 1.0);
  EXPECT_DOUBLE_EQ(whole->getMax(), 2.0);
  EXPECT_DOUBLE_EQ(whole->getValue().x(), 2.0);
  EXPECT_THROW(whole->setRange(0.2, 0.8), std::invalid_argument);
  auto unbounded = panel->addInput("n", 1e12, 1.0, 0, true);
  EXPECT_DOUBLE_EQ(unbounded->getValue().x(), std::numeric_limits<int>::max());

  auto checkbox = panel->addCheckbox("on", true);
  checkbox->setValue(Eigen::Vector4d(0.25, 0.0, 0.0, 0.0));
  EXPECT_DOUBLE_EQ(checkbox->getValue().x(), 1.0);

  auto choice = panel->addChoice("mode", {"a", "b", "c"}, 1);
  choice->setValue(Eigen::Vector4d(7.0, 0.0, 0.0, 0.0));
  EXPECT_DOUBLE_EQ(choice->getValue().x(), 2.0);
  EXPECT_THROW(panel->addChoice("empty", {}), std::invalid_argument);
  EXPECT_THROW(panel->addChoice("index", {"a"}, 1), std::invalid_argument);

  auto color
      = panel->addColor("tint", Eigen::Vector4d(2.0, -1.0, 0.5, 0.5), false);
  EXPECT_TRUE(color->getValue().isApprox(Eigen::Vector4d(1.0, 0.0, 0.5, 1.0)));

  auto vec = panel->addVec3(
      "target", Eigen::Vector3d(5.0, -5.0, 0.5), 0.1, -1.0, 1.0);
  EXPECT_TRUE(
      vec->getValue().head<3>().isApprox(Eigen::Vector3d(1.0, -1.0, 0.5)));
  // A range must admit a finite value; unbounded ones stay valid.
  EXPECT_THROW(vec->setRange(infinity, infinity), std::invalid_argument);
  EXPECT_THROW(vec->setRange(-infinity, -infinity), std::invalid_argument);
  EXPECT_NO_THROW(count->setRange(-infinity, infinity));

  auto plot = panel->addPlot("error", 3, -1.0, 1.0);
  for (double sample : {0.1, 0.2, 0.3, 0.4})
    plot->push(sample);
  // Samples are stored as float, which ImGui plots.
  EXPECT_EQ(plot->getSamples(), (std::vector<double>{0.2f, 0.3f, 0.4f}));
  EXPECT_THROW(
      plot->push(std::numeric_limits<double>::infinity()),
      std::invalid_argument);
  // Samples and bounds must fit the float storage ImGui plots.
  EXPECT_THROW(plot->push(1e40), std::invalid_argument);
  EXPECT_THROW(panel->addPlot("wide", 4, -1e39, 1e39), std::invalid_argument);
  // Each bound fits, but their span overflows float.
  EXPECT_THROW(panel->addPlot("span", 4, -2e38, 2e38), std::invalid_argument);
  // Distinct doubles that round to the same float leave no span.
  EXPECT_THROW(
      panel->addPlot("flat", 4, 1.0, std::nextafter(1.0, 2.0)),
      std::invalid_argument);
  plot->clear();
  EXPECT_TRUE(plot->getSamples().empty());
  EXPECT_THROW(panel->addPlot("bad", 0, 0.0, 1.0), std::invalid_argument);
  EXPECT_THROW(panel->addPlot("bad", 4, 1.0, 1.0), std::invalid_argument);

  auto text = panel->addText("hello");
  EXPECT_THROW(text->addButton("child"), std::logic_error);
  EXPECT_THROW(text->setRange(0.0, 1.0), std::logic_error);
  EXPECT_THROW(text->push(1.0), std::logic_error);
}

//==============================================================================
TEST(GuiAppControl, OnlyEditsQueueCallbacksAndFailuresDisableTheControl)
{
  auto panel = Control::makePanel();
  auto group = panel->addGroup("Tuning");
  auto slider = group->addSlider("gain", 0.5, 0.0, 1.0);
  auto button = group->addButton("Go");
  int sliderCalls = 0;
  slider->setOnChange([&] { ++sliderCalls; });
  button->setOnChange([] { throw std::runtime_error("boom"); });

  std::vector<std::string> errors;
  const auto onError = [&](Control& control, const std::string& reason) {
    errors.push_back(control.getLabel() + ": " + reason);
  };

  // Programmatic writes never run callbacks.
  slider->setValue(Eigen::Vector4d(0.75, 0.0, 0.0, 0.0));
  EXPECT_EQ(panel->dispatch(onError), 0u);
  EXPECT_EQ(sliderCalls, 0);

  // A user edit queues exactly one call; an unchanged or non-finite edit none.
  slider->edit(Eigen::Vector4d(0.25, 0.0, 0.0, 0.0));
  slider->edit(Eigen::Vector4d(0.25, 0.0, 0.0, 0.0));
  slider->edit(Eigen::Vector4d(std::nan(""), 0.0, 0.0, 0.0));
  EXPECT_DOUBLE_EQ(slider->getValue().x(), 0.25);
  EXPECT_EQ(panel->dispatch(onError), 0u);
  EXPECT_EQ(sliderCalls, 1);
  EXPECT_EQ(panel->dispatch(onError), 0u);
  EXPECT_EQ(sliderCalls, 1);

  // A failing callback is reported once, then the control is disabled.
  button->markChanged();
  EXPECT_EQ(panel->dispatch(onError), 1u);
  ASSERT_EQ(errors.size(), 1u);
  EXPECT_EQ(errors.front(), "Go: boom");
  EXPECT_FALSE(button->isEnabled());
  button->markChanged();
  EXPECT_EQ(panel->dispatch(onError), 0u);
  EXPECT_EQ(errors.size(), 1u);

  // A callback may clear itself, or every callback, while it runs.
  auto calls = std::make_shared<int>(0);
  auto self = group->addButton("Self");
  self->setOnChange([control = self.get(), calls] {
    control->setOnChange(nullptr);
    ++*calls;
  });
  self->markChanged();
  EXPECT_EQ(panel->dispatch(onError), 0u);
  auto clearing = group->addButton("Clear all");
  clearing->setOnChange([root = panel.get(), calls] {
    root->clearCallbacks();
    ++*calls;
  });
  clearing->markChanged();
  EXPECT_EQ(panel->dispatch(onError), 0u);
  EXPECT_EQ(*calls, 2);

  // A callback keeps the state it owns from one call to the next.
  std::vector<int> counts;
  auto counter = group->addButton("Count");
  counter->setOnChange(
      [count = 0, &counts]() mutable { counts.push_back(++count); });
  for (int click = 0; click < 3; ++click) {
    counter->markChanged();
    EXPECT_EQ(panel->dispatch(onError), 0u);
  }
  EXPECT_EQ(counts, (std::vector<int>{1, 2, 3}));
  slider->setOnChange([&] { ++sliderCalls; });

  // Teardown drops every callback, including pending ones.
  slider->edit(Eigen::Vector4d(0.5, 0.0, 0.0, 0.0));
  panel->clearCallbacks();
  EXPECT_EQ(panel->dispatch(onError), 0u);
  EXPECT_EQ(sliderCalls, 1);
}

//==============================================================================
TEST(GuiAppControl, RendersEveryKindInOneFrame)
{
  auto panel = Control::makePanel();
  panel->addText("plain");
  auto tinted = panel->addText("tinted", true);
  tinted->setColor(Eigen::Vector4d(1.0, 0.0, 0.0, 1.0));
  auto group = panel->addGroup("Group", true);
  group->addButton("Button")->setTooltip("tip");
  group->addCheckbox("Checkbox", true)->setSameLine(true);
  group->addSlider("Slider", 0.5, 0.0, 1.0);
  group->addSlider("Integer slider", 2.0, 0.0, 5.0, 0, true, true);
  group->addInput("Input", 1.0, 0.1);
  group->addInput("Integer input", 1.0, 1.0, 0, true);
  group->addInput("Integer input with a huge step", 1.0, 1e12, 0, true);
  group->addVec3("Vec3", Eigen::Vector3d::Zero());
  group->addChoice("Radio", {"a", "b"});
  group->addChoice("Combo", {"a", "b"}, 1, true);
  group->addChoice("Repeated", {"same", "same"});
  group->addChoice("Repeated combo", {"same", "same"}, 1, true);
  group->addColor("Color", Eigen::Vector4d(0.1, 0.2, 0.3, 1.0));
  group->addSwatch(Eigen::Vector4d(0.1, 0.2, 0.3, 1.0));
  auto plot = group->addPlot("Plot", 4, -1.0, 1.0);
  plot->push(0.5);
  group->addSeparator();
  auto hidden = panel->addButton("Hidden");
  hidden->setVisible(false);
  auto disabled = panel->addGroup("Disabled", false);
  disabled->setEnabled(false);
  disabled->addButton("Inside");

  ScopedImGuiFrameContext context;
  for (int i = 0; i < 2; ++i)
    context.frame([&] { panel->render(); });
  // A disabled root disables everything inside it.
  panel->setEnabled(false);
  context.frame([&] { panel->render(); });
  EXPECT_EQ(panel->dispatch({}), 0u);
}

//==============================================================================
TEST(GuiAppControl, ReleaseNotifiesOnlyAcceptedEdits)
{
  auto panel = Control::makePanel();
  int calls = 0;
  auto slider = panel->addSlider("gain", 0.5, 0.0, 1.0, 3, false, true);
  slider->setOnChange([&] { ++calls; });

  // A drag whose every input was rejected changes nothing on release.
  slider->edit(Eigen::Vector4d(std::nan(""), 0.0, 0.0, 0.0), false);
  slider->release();
  EXPECT_EQ(panel->dispatch({}), 0u);
  EXPECT_EQ(calls, 0);

  // An accepted drag waits for its release, then notifies once.
  slider->edit(Eigen::Vector4d(0.7, 0.0, 0.0, 0.0), false);
  EXPECT_EQ(panel->dispatch({}), 0u);
  EXPECT_EQ(calls, 0);
  slider->release();
  slider->release();
  EXPECT_EQ(panel->dispatch({}), 0u);
  EXPECT_EQ(calls, 1);
}

//==============================================================================
TEST(GuiAppControl, IntegerTextEntryClampsInsteadOfWrapping)
{
  auto panel = Control::makePanel();
  auto slider = panel->addSlider("count", 5.0, 0.0, 100.0, 0, true);
  int calls = 0;
  slider->setOnChange([&] { ++calls; });

  ScopedImGuiFrameContext context;
  ImVec2 center;
  context.frame([&] {
    panel->render();
    const ImVec2 min = ImGui::GetItemRectMin();
    const ImVec2 max = ImGui::GetItemRectMax();
    center = ImVec2(0.5f * (min.x + max.x), 0.5f * (min.y + max.y));
  });
  // Ctrl+click opens the slider's text entry; type a number past int32.
  ImGuiIO& io = ImGui::GetIO();
  const std::vector<std::function<void()>> inputs = {
      [&] { io.AddMousePosEvent(center.x, center.y); },
      [&] {
        io.AddKeyEvent(ImGuiMod_Ctrl, true);
        io.AddMouseButtonEvent(0, true);
      },
      [&] { io.AddMouseButtonEvent(0, false); },
      [&] { io.AddKeyEvent(ImGuiMod_Ctrl, false); },
      [&] { io.AddInputCharactersUTF8("4294967297"); },
      [&] { io.AddKeyEvent(ImGuiKey_Enter, true); },
      [&] { io.AddKeyEvent(ImGuiKey_Enter, false); },
      [] {},
  };
  for (const auto& input : inputs) {
    input();
    context.frame([&] { panel->render(); });
  }
  EXPECT_EQ(panel->dispatch({}), 0u);
  EXPECT_EQ(calls, 1);
  EXPECT_DOUBLE_EQ(slider->getValue().x(), 100.0);
}

//==============================================================================
TEST(GuiAppControl, RenamingKeepsWidgetIds)
{
  auto panel = Control::makePanel();
  auto group = panel->addGroup("Group");
  auto button = panel->addButton("Button");
  auto checkbox = panel->addCheckbox("Checkbox");

  ScopedImGuiFrameContext context;
  for (const auto& control : {group, button, checkbox}) {
    const auto id = [&] {
      ImGuiID item = 0;
      context.frame([&] {
        control->render();
        item = ImGui::GetItemID();
      });
      return item;
    };
    const ImGuiID before = id();
    control->setLabel("Renamed");
    EXPECT_EQ(id(), before) << "a rename changed a widget's ID";
  }
}

//==============================================================================
TEST(GuiAppControl, RadioChoicesWrapInsideNarrowPanels)
{
  auto panel = Control::makePanel();
  panel->addChoice(
      "Friction", {"Frictionless", "Coulomb friction", "Cone friction"});

  ScopedImGuiFrameContext context;
  float right = 0.0f;
  float lastRight = 0.0f;
  for (int i = 0; i < 2; ++i) {
    context.frame([&] {
      ImGui::SetWindowSize(ImVec2(250.0f, 300.0f));
      right = ImGui::GetWindowPos().x + ImGui::GetWindowWidth();
      panel->render();
      lastRight = ImGui::GetItemRectMax().x;
    });
  }
  EXPECT_LE(lastRight, right);
}

//==============================================================================
// The scene contract.
//==============================================================================

dart::gui::app::SceneSetup emptySetup()
{
  SceneSetup setup;
  setup.world = dart::simulation::World::create();
  return setup;
}

Scene makeScene(std::string id, std::function<SceneSetup()> factory)
{
  Scene scene;
  scene.id = id;
  scene.title = std::move(id);
  scene.factory = std::move(factory);
  return scene;
}

//==============================================================================
TEST(SceneContract, CameraHomeRejectsPosesTheViewerRejects)
{
  const Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  EXPECT_NO_THROW(CameraHome(Eigen::Vector3d(1.0, 0.0, 0.0), origin));
  EXPECT_THROW(CameraHome(origin, origin), std::invalid_argument);
  EXPECT_THROW(
      CameraHome(Eigen::Vector3d(0.0, 0.0, 2.0), origin),
      std::invalid_argument);
  EXPECT_THROW(
      CameraHome(Eigen::Vector3d(std::nan(""), 0.0, 0.0), origin),
      std::invalid_argument);
  EXPECT_THROW(
      CameraHome(Eigen::Vector3d(1.0, 0.0, 0.0), origin, origin),
      std::invalid_argument);
}

//==============================================================================
TEST(SceneContract, SavedContextExpiresAfterSwitchAndShutdown)
{
  std::optional<SceneContext> fromFirst;
  std::optional<SceneContext> fromLast;
  int teardownsWithLiveContext = 0;
  bool liveWhileActive = false;
  const Scene first = makeScene("first", [&] {
    auto setup = emptySetup();
    setup.onActivate = [&](SceneContext& ctx) {
      fromFirst.emplace(ctx);
      ctx.addTeardown([&] {
        // Teardowns run before the context expires.
        fromFirst->log("tearing down");
        ++teardownsWithLiveContext;
      });
    };
    return setup;
  });
  const Scene last = makeScene("last", [&] {
    auto setup = emptySetup();
    setup.onActivate = [&](SceneContext& ctx) {
      fromLast.emplace(ctx);
    };
    setup.preStep = [&] {
      liveWhileActive
          = fromLast->viewer() != nullptr
            && fromLast->viewerPtr().get() == fromLast->viewer()
            && fromLast->worldNodePtr().get() == fromLast->worldNode();
    };
    return setup;
  });
  {
    App host({first, last}, std::nullopt);
    testing::internal::CaptureStdout();
    EXPECT_EQ(host.cycleScenes(1), 0);
    testing::internal::GetCapturedStdout();
    EXPECT_EQ(teardownsWithLiveContext, 2);
    EXPECT_TRUE(liveWhileActive);
    EXPECT_THROW(fromFirst->viewer(), std::logic_error);
    EXPECT_THROW(fromFirst->addTeardown([] {}), std::logic_error);
    // The cycle tore the last scene down as well.
    EXPECT_THROW(fromLast->log("after the cycle"), std::logic_error);
  }
  EXPECT_THROW(fromLast->worldNodePtr(), std::logic_error);
}

//==============================================================================
TEST(SceneContract, FailingSceneHookIsDisabledAloneAndFailsTheCycle)
{
  int preSteps = 0;
  int postSteps = 0;
  int preRefreshes = 0;
  const Scene scene = makeScene("throwing", [&] {
    auto setup = emptySetup();
    setup.preStep = [&] {
      ++preSteps;
      throw std::runtime_error("preStep boom");
    };
    setup.postStep = [&] {
      ++postSteps;
    };
    setup.preRefresh = [&] {
      ++preRefreshes;
    };
    return setup;
  });
  App host({scene}, std::nullopt);
  testing::internal::CaptureStdout();
  testing::internal::CaptureStderr();
  EXPECT_EQ(host.cycleScenes(3), 1);
  const std::string errors = testing::internal::GetCapturedStderr();
  testing::internal::GetCapturedStdout();
  EXPECT_NE(errors.find("preStep failed: preStep boom"), std::string::npos);
  EXPECT_NE(errors.find("disabled 1 callback(s)"), std::string::npos);
  // Each activation's preStep fails once and is disabled; the other hooks,
  // and the display-free cycle's per-frame preRefresh, keep running.
  EXPECT_EQ(preSteps, 2);
  EXPECT_EQ(postSteps, 6);
  EXPECT_EQ(preRefreshes, 6);
  EXPECT_EQ(host.getCallbackFailureCount(), 2u);
}

//==============================================================================
TEST(SceneContract, DestroyedTeardownsCannotAddTeardowns)
{
  // A teardown capture whose destructor tries to add a teardown, as a
  // binding's finalizer might.
  struct AddsOnDestruction
  {
    App* host = nullptr;
    bool* rejected = nullptr;
    ~AddsOnDestruction()
    {
      if (!host)
        return;
      try {
        host->addTeardown([] {});
      } catch (const std::logic_error&) {
        *rejected = true;
      }
    }
  };
  bool rejected = false;
  App* hostPtr = nullptr;
  const Scene scene = makeScene("leaky", [&] {
    auto setup = emptySetup();
    setup.onActivate = [&](SceneContext& ctx) {
      auto capture = std::make_shared<AddsOnDestruction>();
      capture->host = hostPtr;
      capture->rejected = &rejected;
      ctx.addTeardown([capture] {});
    };
    return setup;
  });
  App host({scene}, std::nullopt);
  hostPtr = &host;
  testing::internal::CaptureStdout();
  EXPECT_EQ(host.cycleScenes(1), 0);
  testing::internal::GetCapturedStdout();
  EXPECT_TRUE(rejected);
}

//==============================================================================
TEST(GuiAppControl, ClearingCallbacksSurvivesCapturesThatAddControls)
{
  auto panel = Control::makePanel();
  auto group = panel->addGroup("Group");
  int lateCapturesDestroyed = 0;
  struct Counts
  {
    explicit Counts(int* destroyed) : destroyed(destroyed) {}
    ~Counts()
    {
      ++*destroyed;
    }
    int* destroyed;
  };
  // Each capture appends a control with a callback of its own when it is
  // destroyed, which also reallocates the group's children while they are
  // being cleared.
  struct Appends
  {
    Appends(Control* group, int* destroyed) : group(group), destroyed(destroyed)
    {
    }
    ~Appends()
    {
      auto counts = std::make_shared<Counts>(destroyed);
      group->addButton("late")->setOnChange([counts] {});
    }
    Control* group;
    int* destroyed;
  };
  for (int i = 0; i < 8; ++i) {
    auto capture
        = std::make_shared<Appends>(group.get(), &lateCapturesDestroyed);
    group->addButton("b")->setOnChange([capture] {});
  }
  panel->clearCallbacks();
  EXPECT_EQ(group->getChildren().size(), 16u);
  // The appended controls' callbacks were cleared too.
  EXPECT_EQ(lateCapturesDestroyed, 8);

  // Clearing a later group may add a callback to an earlier one.
  auto first = panel->addGroup("First");
  auto second = panel->addGroup("Second");
  {
    auto capture
        = std::make_shared<Appends>(first.get(), &lateCapturesDestroyed);
    second->addButton("b")->setOnChange([capture] {});
  }
  panel->clearCallbacks();
  EXPECT_EQ(first->getChildren().size(), 1u);
  EXPECT_EQ(lateCapturesDestroyed, 9);
}

//==============================================================================
TEST(SceneContract, PanelOutlivesTheSceneCallables)
{
  // Once destroyed, each capture adds a control whose callback keeps the
  // panel alive.
  struct Appends
  {
    explicit Appends(std::weak_ptr<Control> panel) : panel(std::move(panel)) {}
    ~Appends()
    {
      if (auto owner = panel.lock())
        owner->addButton("late")->setOnChange([owner] {});
    }
    std::weak_ptr<Control> panel;
  };
  std::vector<std::weak_ptr<Control>> panels;
  const Scene scene = makeScene("appends", [&] {
    auto setup = emptySetup();
    setup.panel = Control::makePanel();
    panels.push_back(setup.panel);
    auto key = std::make_shared<Appends>(setup.panel);
    setup.keyActions.push_back({'k', "Key", [key] {
                                }});
    setup.onActivate = [panel = panels.back()](SceneContext& ctx) {
      auto teardown = std::make_shared<Appends>(panel);
      ctx.addTeardown([teardown] {});
    };
    return setup;
  });
  {
    App host({scene}, std::nullopt);
    testing::internal::CaptureStdout();
    EXPECT_EQ(host.cycleScenes(1), 0);
    testing::internal::GetCapturedStdout();
  }
  ASSERT_FALSE(panels.empty());
  for (const auto& panel : panels)
    EXPECT_TRUE(panel.expired());
}

//==============================================================================
TEST(SceneContract, TeardownsCannotAddTeardowns)
{
  const Scene scene = makeScene("nested", [] {
    auto setup = emptySetup();
    setup.onActivate = [](SceneContext& ctx) {
      ctx.addTeardown([ctx]() mutable { ctx.addTeardown([] {}); });
    };
    return setup;
  });
  App host({scene, makeScene("other", emptySetup)}, std::nullopt);
  testing::internal::CaptureStdout();
  testing::internal::CaptureStderr();
  EXPECT_EQ(host.cycleScenes(0), 1);
  const std::string errors = testing::internal::GetCapturedStderr();
  testing::internal::GetCapturedStdout();
  EXPECT_NE(errors.find("teardown failed"), std::string::npos);
  EXPECT_EQ(host.getCallbackFailureCount(), 2u);
}

//==============================================================================
TEST(SceneContract, CycleCountsTheFinalTeardown)
{
  int activations = 0;
  const Scene scene = makeScene("last", [&] {
    auto setup = emptySetup();
    setup.onActivate = [&](SceneContext& ctx) {
      // Only the second activation, which the cycle ends with, fails.
      if (++activations == 2)
        ctx.addTeardown([] { throw std::runtime_error("cleanup failed"); });
    };
    return setup;
  });
  App host({scene}, std::nullopt);
  testing::internal::CaptureStdout();
  testing::internal::CaptureStderr();
  EXPECT_EQ(host.cycleScenes(0), 1);
  const std::string errors = testing::internal::GetCapturedStderr();
  testing::internal::GetCapturedStdout();
  EXPECT_NE(
      errors.find("the final teardown disabled 1 callback(s)"),
      std::string::npos);
}

//==============================================================================
TEST(SceneContract, CycleDispatchesQueuedPanelEvents)
{
  int clicks = 0;
  const Scene scene = makeScene("clicked", [&] {
    auto setup = emptySetup();
    setup.panel = Control::makePanel();
    auto button = setup.panel->addButton("Go");
    button->setOnChange([&] { ++clicks; });
    button->markChanged();
    auto failing = setup.panel->addButton("Fail");
    failing->setOnChange([] { throw std::runtime_error("click failed"); });
    failing->markChanged();
    return setup;
  });
  App host({scene}, std::nullopt);
  testing::internal::CaptureStdout();
  testing::internal::CaptureStderr();
  EXPECT_EQ(host.cycleScenes(1), 1);
  const std::string errors = testing::internal::GetCapturedStderr();
  testing::internal::GetCapturedStdout();
  EXPECT_EQ(clicks, 2);
  EXPECT_NE(errors.find("click failed"), std::string::npos);
}

//==============================================================================
TEST(SceneContract, KinematicScenesDisablePlayUntilTornDown)
{
  std::vector<bool> allowed;
  const auto record = [&](bool allowSimulation) {
    return [&allowed, allowSimulation] {
      auto setup = emptySetup();
      setup.allowSimulation = allowSimulation;
      setup.onActivate = [&allowed](SceneContext& ctx) {
        allowed.push_back(ctx.viewer()->isAllowingSimulation());
      };
      return setup;
    };
  };
  App host(
      {makeScene("kinematic", record(false)),
       makeScene("dynamic", record(true))},
      std::nullopt);
  testing::internal::CaptureStdout();
  EXPECT_EQ(host.cycleScenes(1), 0);
  testing::internal::GetCapturedStdout();
  EXPECT_EQ(allowed, (std::vector<bool>{false, true, false, true}));

  // A viewer whose owner disallowed simulation stays that way.
  allowed.clear();
  App owned({makeScene("kinematic", record(false))}, std::nullopt);
  owned.getViewer()->allowSimulation(false);
  testing::internal::CaptureStdout();
  EXPECT_EQ(owned.cycleScenes(1), 0);
  testing::internal::GetCapturedStdout();
  EXPECT_FALSE(owned.getViewer()->isAllowingSimulation());
}

//==============================================================================
TEST(SceneContract, RetainedViewerNoLongerReachesTheHost)
{
  std::shared_ptr<dart::gui::Viewer> viewer;
  const Scene scene = makeScene("kept", [&] {
    auto setup = emptySetup();
    setup.onActivate = [&](SceneContext& ctx) {
      viewer = ctx.viewerPtr();
    };
    return setup;
  });
  const std::string shot = (std::filesystem::temp_directory_path()
                            / "dart-gui-app-retained-viewer.png")
                               .string();
  {
    App host({scene}, 1.0);
    // Stops after configuring the viewer, before any graphics device starts.
    host.setStopRequested([] { return true; });
    testing::internal::CaptureStdout();
    testing::internal::CaptureStderr();
    EXPECT_EQ(
        host.runHeadlessShot(shot, 0, "kept", 32, 24), App::kStoppedExitCode);
    testing::internal::GetCapturedStderr();
    testing::internal::GetCapturedStdout();
    ASSERT_NE(viewer, nullptr);
    EXPECT_FALSE(
        viewer->getDefaultEventHandler()->getMouseEventHandlers().empty());
    EXPECT_FALSE(viewer->getAttachments().empty());
  }
  std::remove(shot.c_str());
  EXPECT_TRUE(
      viewer->getDefaultEventHandler()->getMouseEventHandlers().empty());
  EXPECT_TRUE(viewer->getAttachments().empty());
  // The host's key handler is gone too, so this never reaches freed memory.
  viewer->handleEvent(
      dart::gui::GUIEventAdapter(dart::gui::GUIEventAdapter::KEYDOWN, 'k'));
}

//==============================================================================
TEST(SceneContract, StopRequestEndsRuns)
{
  int built = 0;
  const Scene scene = makeScene("counted", [&] {
    ++built;
    return emptySetup();
  });
  {
    App host({scene, scene}, std::nullopt);
    host.setStopRequested([&] { return built >= 1; });
    EXPECT_EQ(host.cycleScenes(5), App::kStoppedExitCode);
    EXPECT_EQ(built, 1);
    // A stopped run tears its scene down before returning.
    EXPECT_EQ(host.getActiveWorldNodeCount(), 0u);
  }
  {
    // A capture with no settle steps still honors the request.
    App host({scene}, 1.0);
    host.setStopRequested([] { return true; });
    EXPECT_EQ(
        host.runHeadlessShot("unused.png", 0, "counted", 32, 24),
        App::kStoppedExitCode);
  }
  testing::internal::CaptureStdout();
  EXPECT_EQ(
      runCommandLine(
          {scene},
          {"app", "--cycle-scenes", "--frames", "1"},
          [] { return true; }),
      App::kStoppedExitCode);
  testing::internal::GetCapturedStdout();
  {
    // A predicate that reports a stop only once still ends the run, even
    // when the world node's per-step poll is the one that sees it.
    int polls = 0;
    App host({scene}, std::nullopt);
    host.setStopRequested([&] { return ++polls == 3; });
    testing::internal::CaptureStdout();
    EXPECT_EQ(host.cycleScenes(2), App::kStoppedExitCode);
    testing::internal::GetCapturedStdout();
  }
  {
    // A stop requested by the cycle's very last step still counts.
    int steps = 0;
    const Scene last = makeScene("last", [&] {
      auto setup = emptySetup();
      setup.preStep = [&] {
        ++steps;
      };
      return setup;
    });
    App host({last}, std::nullopt);
    host.setStopRequested([&] { return steps >= 2; });
    testing::internal::CaptureStdout();
    EXPECT_EQ(host.cycleScenes(1), App::kStoppedExitCode);
    testing::internal::GetCapturedStdout();
    EXPECT_EQ(steps, 2);
  }
}

//==============================================================================
TEST(SceneContract, StopRequestEndsRealTimeCatchUp)
{
  auto world = dart::simulation::World::create();
  auto node = std::make_shared<AppWorldNode>(world);
  bool stop = false;
  int preSteps = 0;
  node->setHooks(
      [&] {
        ++preSteps;
        stop = true;
      },
      {});
  node->setStopRequested([&] { return stop; });
  // A huge real-time factor gives refresh() a budget of many steps.
  node->setTargetRealTimeFactor(1e9);
  node->simulate(true);
  node->refresh();
  node->refresh();
  EXPECT_EQ(preSteps, 1);
  EXPECT_EQ(world->getSimFrames(), 0);
  EXPECT_FALSE(node->isSimulating());
  // stepOnce() honors the request too.
  node->stepOnce();
  EXPECT_EQ(world->getSimFrames(), 0);
}

//==============================================================================
TEST(SceneContract, StopExceptionsEndCyclesWithoutFailing)
{
  // A collision filter that throws mid-step stands in for a binding's
  // viewer override that raised an interrupt.
  struct Interrupt : std::runtime_error
  {
    using std::runtime_error::runtime_error;
  };
  struct ThrowingFilter : dart::collision::CollisionFilter
  {
    bool ignoresCollision(
        const dart::collision::CollisionObject*,
        const dart::collision::CollisionObject*) const override
    {
      throw Interrupt("interrupted");
    }
  };
  const Scene scene = makeScene("throwing", [] {
    auto setup = emptySetup();
    for (int i = 0; i < 2; ++i) {
      auto skeleton
          = dart::dynamics::Skeleton::create("box" + std::to_string(i));
      skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>()
          .second->createShapeNodeWith<dart::dynamics::CollisionAspect>(
              std::make_shared<dart::dynamics::BoxShape>(
                  Eigen::Vector3d::Ones()));
      setup.world->addSkeleton(skeleton);
    }
    setup.world->getConstraintSolver()->getCollisionOption().collisionFilter
        = std::make_shared<ThrowingFilter>();
    return setup;
  });

  const auto cycle = [&](App& host, int expected) {
    testing::internal::CaptureStderr();
    testing::internal::CaptureStdout();
    EXPECT_EQ(host.cycleScenes(1), expected);
    testing::internal::GetCapturedStdout();
    return testing::internal::GetCapturedStderr();
  };
  {
    // Unclassified, the exception fails the scene.
    App host({scene}, std::nullopt);
    EXPECT_NE(cycle(host, 1).find("failed to render"), std::string::npos);
  }
  {
    bool stop = false;
    App host({scene}, std::nullopt);
    host.setStopRequested([&] { return stop; });
    host.setStopException([&](const std::exception& error) {
      stop = dynamic_cast<const Interrupt*>(&error) != nullptr;
      return stop;
    });
    EXPECT_EQ(
        cycle(host, App::kStoppedExitCode).find("failed to render"),
        std::string::npos);
  }
}

//==============================================================================
TEST(SceneContract, CommandLineRejectsInvalidNumbersAndNestedRuns)
{
  const std::vector<std::vector<std::string>> invalid = {
      {"app", "--frames", "abc"},
      {"app", "--frames", "-1"},
      {"app", "--frames", "1000001"},
      {"app", "--steps", "1e3"},
      {"app", "--width", "0"},
      {"app", "--height", "16385"},
      {"app", "--threads", "257"},
      {"app", "--threads", "4x"},
      {"app", "--frames"},
  };
  for (const auto& args : invalid) {
    testing::internal::CaptureStderr();
    EXPECT_EQ(runCommandLine({}, args), 1) << args[1] << " " << args.back();
    testing::internal::GetCapturedStderr();
  }

  bool nestedThrew = false;
  const Scene scene = makeScene("nested", [&] {
    try {
      runCommandLine({}, {"app", "--list-scenes"});
    } catch (const std::logic_error&) {
      nestedThrew = true;
    }
    return emptySetup();
  });
  testing::internal::CaptureStdout();
  EXPECT_EQ(
      runCommandLine(
          {scene},
          {"app",
           "--cycle-scenes",
           "--frames",
           "1",
           "--threads",
           "0",
           "--width",
           "16384"}),
      0);
  testing::internal::GetCapturedStdout();
  EXPECT_TRUE(nestedThrew);
}

//==============================================================================
TEST(LogCapture, SinkWritesAreForwardedButNotCapturedAgain)
{
  int lines = 0;
  testing::internal::CaptureStderr();
  {
    LogCapture capture([&](CapturedLogLevel, const std::string& line) {
      // Bounded, so a regression fails instead of recursing without end.
      if (++lines < 5)
        std::cerr << "echo: " << line << std::endl;
    });
    std::cerr << "\033[1;31mError\033[0m first" << std::endl;
  }
  const std::string output = testing::internal::GetCapturedStderr();
  EXPECT_EQ(lines, 1);
  EXPECT_NE(output.find("echo: Error first"), std::string::npos);
}

//==============================================================================
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
