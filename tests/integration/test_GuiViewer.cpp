/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <dart/gui/DefaultEventHandler.hpp>
#include <dart/gui/DragAndDrop.hpp>
#include <dart/gui/InteractiveFrame.hpp>
#include <dart/gui/RealTimeWorldNode.hpp>
#include <dart/gui/Viewer.hpp>
#include <dart/gui/ViewerAttachment.hpp>
#include <dart/gui/WorldNode.hpp>
#include <dart/gui/detail/native_window.hpp>

#include <dart/simulation/World.hpp>

#include <dart/dynamics/SimpleFrame.hpp>
#include <dart/dynamics/SphereShape.hpp>

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
class GuiCountingNode : public dart::gui::WorldNode
{
public:
  using WorldNode::WorldNode;
  std::vector<std::string> events;
  void customPreRefresh() override
  {
    events.emplace_back("pre-refresh");
  }
  void customPostRefresh() override
  {
    events.emplace_back("post-refresh");
  }
  void customPreStep() override
  {
    events.emplace_back("pre-step");
  }
  void customPostStep() override
  {
    events.emplace_back("post-step");
  }
  void setupViewer() override
  {
    events.emplace_back("setup");
  }
};
class GuiCountingAttachment : public dart::gui::ViewerAttachment
{
public:
  int attaches = 0, detaches = 0;
  bool throwOnDetach = false;
  bool throwOnAttach = false;
  void refresh() override {}

protected:
  void customAttach(dart::gui::Viewer* viewer) override
  {
    if (viewer) {
      ++attaches;
      if (throwOnAttach)
        throw std::runtime_error("fixture attach failure");
    } else {
      ++detaches;
      if (throwOnDetach)
        throw std::runtime_error("fixture detach failure");
    }
  }
};
} // namespace

TEST(GuiViewer, ConstructionDoesNotInitializeGraphics)
{
  auto world = dart::simulation::World::create();
  auto node = std::make_shared<GuiCountingNode>(world);
  dart::gui::Viewer viewer;
  viewer.addWorldNode(node);
  EXPECT_TRUE(viewer.getRenderBackend().empty());
  EXPECT_TRUE(viewer.getRenderDevice().empty());
  EXPECT_EQ(viewer.getWorldNode(world), node);
  EXPECT_EQ(viewer.getNumWorldNodes(), 1u);
  EXPECT_EQ(node->events, std::vector<std::string>({"setup"}));
}

TEST(GuiViewer, DefaultWindowFitsProportionallyAtDesktopDpi)
{
  namespace detail = dart::gui::detail;
  const auto large
      = detail::fitDefaultWindow({0, 0, 3840, 2160}, 2.0f, 0.0, false);
  EXPECT_EQ(large.width, 3110);
  EXPECT_EQ(large.height, 1944);
  EXPECT_EQ(large.x, 365);
  EXPECT_EQ(large.y, 108);
  EXPECT_NEAR(static_cast<double>(large.width) / large.height, 1.6, 0.001);
  const auto smallWindow
      = detail::fitDefaultWindow({0, 0, 640, 480}, 2.0f, 0.0, false);
  EXPECT_EQ(smallWindow.width, 576);
  EXPECT_EQ(smallWindow.height, 360);
  EXPECT_EQ(smallWindow.x, 32);
  EXPECT_EQ(smallWindow.y, 60);
}

TEST(GuiViewer, DefaultWindowRespectsManualScaleAndNegativeMonitorOrigin)
{
  namespace detail = dart::gui::detail;
  const auto manual
      = detail::fitDefaultWindow({0, 0, 3840, 2160}, 1.0f, 2.0, false);
  EXPECT_EQ(manual.width, 3110);
  EXPECT_EQ(manual.height, 1944);
  const auto negative
      = detail::fitDefaultWindow({-1920, -200, 1920, 1080}, 1.0f, 0.0, false);
  EXPECT_EQ(negative.width, 1555);
  EXPECT_EQ(negative.height, 972);
  EXPECT_EQ(negative.x, -1738);
  EXPECT_EQ(negative.y, -146);
}

TEST(GuiViewer, DefaultWindowUsesLogicalRetinaCoordinates)
{
  namespace detail = dart::gui::detail;
  const detail::NativeWindowRectangle area{0, 0, 3840, 2160};
  const auto automatic = detail::fitDefaultWindow(area, 2.0f, 0.0, true);
  EXPECT_EQ(automatic.width, 1600);
  EXPECT_EQ(automatic.height, 1000);
  const auto manual = detail::fitDefaultWindow(area, 2.0f, 3.0, true);
  EXPECT_EQ(manual.width, 2400);
  EXPECT_EQ(manual.height, 1500);
}

TEST(GuiViewer, DefaultWindowHandlesInvalidWorkAreasScalesAndExtremeOrigins)
{
  namespace detail = dart::gui::detail;
  const auto unavailable
      = detail::fitDefaultWindow({20, 40, 0, 1000}, 1.0f, 2.0, false);
  EXPECT_EQ(unavailable.width, 3200);
  EXPECT_EQ(unavailable.height, 2000);
  EXPECT_EQ(unavailable.x, 20);
  EXPECT_EQ(unavailable.y, 40);
  const auto tiny = detail::fitDefaultWindow({0, 0, 1, 1}, 2.0f, 0.0, false);
  EXPECT_EQ(tiny.width, 1);
  EXPECT_EQ(tiny.height, 1);
  const auto invalidScale = detail::fitDefaultWindow(
      {0, 0, 0, 0},
      std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<double>::infinity(),
      false);
  EXPECT_EQ(invalidScale.width, 1600);
  EXPECT_EQ(invalidScale.height, 1000);
  const auto extreme = detail::fitDefaultWindow(
      {std::numeric_limits<int>::max(),
       std::numeric_limits<int>::min(),
       3840,
       2160},
      1.0f,
      1.0,
      false);
  EXPECT_EQ(extreme.x, std::numeric_limits<int>::max());
  EXPECT_GT(extreme.y, std::numeric_limits<int>::min());
}

TEST(GuiViewer, AutomaticScaleRestoresDetectedDpi)
{
  dart::gui::Viewer viewer;
  const double detected = viewer.getGuiScale();
  viewer.setGuiScale(2.5);
  EXPECT_DOUBLE_EQ(viewer.getGuiScale(), 2.5);
  viewer.setAutomaticGuiScale();
  EXPECT_DOUBLE_EQ(viewer.getGuiScale(), detected);
  viewer.setAutomaticGuiScale(false);
  EXPECT_DOUBLE_EQ(viewer.getGuiScale(), detected);
  EXPECT_TRUE(viewer.getRenderBackend().empty());
}

TEST(GuiViewer, RefreshAndStepCallbacksFollowSimulation)
{
  auto world = dart::simulation::World::create();
  world->setTimeStep(0.001);
  GuiCountingNode node(world);
  node.refresh();
  EXPECT_EQ(
      node.events, std::vector<std::string>({"pre-refresh", "post-refresh"}));
  EXPECT_DOUBLE_EQ(world->getTime(), 0.0);
  node.events.clear();
  node.simulate(true);
  node.setNumStepsPerCycle(2);
  node.refresh();
  EXPECT_EQ(
      node.events,
      std::vector<std::string>(
          {"pre-refresh",
           "pre-step",
           "post-step",
           "pre-step",
           "post-step",
           "post-refresh"}));
  EXPECT_DOUBLE_EQ(world->getTime(), 0.002);
  node.setWorld(nullptr);
  EXPECT_NO_THROW(node.refresh());
}

TEST(GuiViewer, RetainsAndReleasesWorldNodes)
{
  auto world = dart::simulation::World::create();
  auto node = std::make_shared<dart::gui::WorldNode>(world);
  std::weak_ptr<dart::gui::WorldNode> weak = node;
  dart::gui::Viewer viewer;
  viewer.addWorldNode(node);
  viewer.addWorldNode(node);
  EXPECT_EQ(viewer.getNumWorldNodes(), 1u);
  node.reset();
  EXPECT_FALSE(weak.expired());
  viewer.removeWorldNode(world);
  EXPECT_TRUE(weak.expired());
  EXPECT_NO_THROW(viewer.removeWorldNode(world));
  EXPECT_THROW(viewer.addWorldNode(nullptr), std::invalid_argument);
}

TEST(GuiViewer, SimulationPermissionAndActiveNodes)
{
  dart::gui::Viewer viewer;
  auto node = std::make_shared<dart::gui::WorldNode>(
      dart::simulation::World::create());
  viewer.addWorldNode(node);
  viewer.simulate(true);
  EXPECT_TRUE(node->isSimulating());
  viewer.allowSimulation(false);
  EXPECT_FALSE(viewer.isSimulating());
  viewer.simulate(true);
  EXPECT_FALSE(node->isSimulating());
  viewer.allowSimulation(true);
  viewer.setWorldNodeActive(node, false);
  viewer.simulate(true);
  EXPECT_FALSE(node->isSimulating());
  viewer.setWorldNodeActive(node->getWorld(), true);
  EXPECT_TRUE(node->isSimulating());
}

TEST(GuiViewer, AttachmentOwnershipAndFallibleTeardown)
{
  auto attachment = std::make_shared<GuiCountingAttachment>();
  {
    dart::gui::Viewer first, second;
    first.addAttachment(attachment);
    ASSERT_EQ(first.getAttachments().size(), 1u);
    EXPECT_EQ(first.getAttachments().front(), attachment);
    EXPECT_EQ(attachment->getViewer(), &first);
    EXPECT_THROW(second.addAttachment(attachment), std::invalid_argument);
    first.removeAttachment(attachment);
    EXPECT_TRUE(first.getAttachments().empty());
    EXPECT_EQ(attachment->getViewer(), nullptr);
    second.addAttachment(attachment);
    attachment->throwOnDetach = true;
  }
  EXPECT_EQ(attachment->attaches, 2);
  EXPECT_EQ(attachment->detaches, 2);
  EXPECT_EQ(attachment->getViewer(), nullptr);
}

TEST(GuiViewer, RealTimeConfigurationRejectsInvalidBudgets)
{
  const auto world = dart::simulation::World::create();
  dart::gui::RealTimeWorldNode node(world, 60.0, 0.0);
  node.simulate(true);
  node.refresh();
  EXPECT_DOUBLE_EQ(world->getTime(), 0.0);
  EXPECT_DOUBLE_EQ(node.getTargetFrequency(), 60.0);
  EXPECT_THROW(node.setTargetFrequency(0.0), std::invalid_argument);
  EXPECT_THROW(
      node.setTargetFrequency(std::numeric_limits<double>::infinity()),
      std::invalid_argument);
  EXPECT_THROW(node.setTargetRealTimeFactor(-1.0), std::invalid_argument);
  EXPECT_THROW(
      node.setTargetRealTimeFactor(std::numeric_limits<double>::quiet_NaN()),
      std::invalid_argument);
}

TEST(GuiViewer, FailedCallbacksRollBackRegistration)
{
  class Node : public dart::gui::WorldNode
  {
  public:
    bool fail = true;
    void setupViewer() override
    {
      if (fail)
        throw std::runtime_error("fixture setup failure");
    }
  };
  dart::gui::Viewer viewer;
  const auto node = std::make_shared<Node>();
  EXPECT_THROW(viewer.addWorldNode(node), std::runtime_error);
  EXPECT_EQ(viewer.getNumWorldNodes(), 0u);
  EXPECT_EQ(node->getViewer(), nullptr);
  node->fail = false;
  EXPECT_NO_THROW(viewer.addWorldNode(node));
  const auto attachment = std::make_shared<GuiCountingAttachment>();
  attachment->throwOnAttach = true;
  EXPECT_THROW(viewer.addAttachment(attachment), std::runtime_error);
  EXPECT_EQ(attachment->getViewer(), nullptr);
  attachment->throwOnAttach = false;
  EXPECT_NO_THROW(viewer.addAttachment(attachment));
  EXPECT_EQ(attachment->attaches, 2);
}

TEST(GuiViewer, ShapeDragIsSafeAfterShapeReplacement)
{
  dart::gui::Viewer viewer;
  auto frame = std::make_shared<dart::dynamics::SimpleFrame>(
      dart::dynamics::Frame::World());
  frame->setShape(std::make_shared<dart::dynamics::SphereShape>(0.5));
  auto* drag = viewer.enableDragAndDrop(frame.get(), frame->getShape().get());
  auto retained = drag->shared_from_this();
  frame->setShape(std::make_shared<dart::dynamics::SphereShape>(1.0));
  EXPECT_EQ(drag->getShape(), nullptr);
  EXPECT_EQ(drag->getEntity(), nullptr);
  EXPECT_NO_THROW(drag->update());
  EXPECT_FALSE(viewer.disableDragAndDrop(drag));
}

TEST(GuiViewer, CloseEventReachesCustomHandlers)
{
  class Handler : public dart::gui::GUIEventHandler
  {
  public:
    int closes = 0;
    bool handle(
        const dart::gui::GUIEventAdapter& event,
        dart::gui::GUIActionAdapter&) override
    {
      if (event.getEventType() == dart::gui::GUIEventAdapter::CLOSE_WINDOW)
        ++closes;
      return false;
    }
  };
  dart::gui::Viewer viewer;
  const auto handler = std::make_shared<Handler>();
  viewer.addEventHandler(handler);
  viewer.handleEvent(
      dart::gui::GUIEventAdapter(dart::gui::GUIEventAdapter::CLOSE_WINDOW));
  EXPECT_EQ(handler->closes, 1);
  EXPECT_TRUE(viewer.done());
}

TEST(GuiViewer, ValidatesCameraAndCaptureSetupWithoutGraphics)
{
  dart::gui::Viewer viewer;
  EXPECT_THROW(viewer.setUpViewForCapture(0, 480), std::invalid_argument);
  EXPECT_THROW(viewer.setVerticalFieldOfView(180.0), std::invalid_argument);
  EXPECT_THROW(
      viewer.setCameraHome(
          Eigen::Vector3d::Zero(),
          Eigen::Vector3d::Zero(),
          Eigen::Vector3d::UnitZ()),
      std::invalid_argument);
  EXPECT_THROW(viewer.setGuiScale(0.0), std::invalid_argument);
}

TEST(GuiViewer, PicksVisualFramesWithoutCollisionShapes)
{
  auto world = dart::simulation::World::create();
  auto frame = std::make_shared<dart::dynamics::SimpleFrame>(
      dart::dynamics::Frame::World());
  frame->setShape(std::make_shared<dart::dynamics::SphereShape>(0.5));
  frame->createVisualAspect();
  world->addSimpleFrame(frame);
  dart::gui::Viewer viewer;
  viewer.addWorldNode(std::make_shared<dart::gui::WorldNode>(world));
  viewer.setCameraHome({0, 0, 3}, {0, 0, 0}, {0, 1, 0});
  std::vector<dart::gui::PickInfo> picks;
  viewer.computeIntersections(
      dart::gui::GUIEventAdapter(dart::gui::GUIEventAdapter::PUSH, 0, 800, 500),
      picks);
  ASSERT_EQ(picks.size(), 1u);
  EXPECT_EQ(picks.front().frame, frame.get());
  EXPECT_TRUE(
      picks.front().position.isApprox(Eigen::Vector3d(0, 0, 0.5), 1e-9));
  EXPECT_TRUE(picks.front().normal.isApprox(Eigen::Vector3d::UnitZ(), 1e-9));
  frame->getVisualAspect()->hide();
  viewer.computeIntersections(
      dart::gui::GUIEventAdapter(dart::gui::GUIEventAdapter::PUSH, 0, 800, 500),
      picks);
  EXPECT_TRUE(picks.empty());
  EXPECT_TRUE(viewer.getRenderBackend().empty());
}

TEST(GuiViewer, DragUsesWorldCoordinatesAndCancelsWhenFrameIsDestroyed)
{
  using Event = dart::gui::GUIEventAdapter;
  auto world = dart::simulation::World::create();
  auto parent = std::make_shared<dart::dynamics::SimpleFrame>(
      dart::dynamics::Frame::World());
  parent->setRelativeRotation(
      Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitY()).toRotationMatrix());
  auto frame = std::make_shared<dart::dynamics::SimpleFrame>(parent.get());
  frame->setShape(std::make_shared<dart::dynamics::SphereShape>(0.5));
  frame->createVisualAspect();
  world->addSimpleFrame(parent);
  dart::gui::Viewer viewer;
  viewer.addWorldNode(std::make_shared<dart::gui::WorldNode>(world));
  viewer.setCameraHome({0, 0, 3}, {0, 0, 0}, {0, 1, 0});
  auto* drag = viewer.enableDragAndDrop(frame.get());
  EXPECT_EQ(viewer.enableDragAndDrop(frame.get()), drag);
  viewer.handleEvent(Event(Event::PUSH, 0, 800, 500, Event::LEFT_MOUSE_BUTTON));
  ASSERT_TRUE(drag->isMoving());
  viewer.handleEvent(Event(Event::DRAG, 0, 900, 500, Event::LEFT_MOUSE_BUTTON));
  EXPECT_GT(frame->getWorldTransform().translation().x(), 0.1);
  EXPECT_NEAR(frame->getWorldTransform().translation().y(), 0, 1e-9);
  EXPECT_NEAR(frame->getWorldTransform().translation().z(), 0, 1e-9);
  viewer.handleEvent(Event(Event::RELEASE, 0, 900, 500));
  EXPECT_FALSE(drag->isMoving());
  const auto retained = drag->shared_from_this();
  frame.reset();
  EXPECT_EQ(retained->getEntity(), nullptr);
  EXPECT_FALSE(retained->isMoving());
  EXPECT_NO_THROW(retained->move());
  EXPECT_NO_THROW(retained->saveState());
  EXPECT_FALSE(viewer.disableDragAndDrop(drag));
  EXPECT_NO_THROW(viewer.updateDragAndDrops());
}

TEST(GuiViewer, EventCallbacksMayRemoveThemselvesAndConsumeDefaultKeys)
{
  class Handler : public dart::gui::GUIEventHandler
  {
  public:
    std::weak_ptr<Handler> self;
    int calls = 0;
    bool handle(
        const dart::gui::GUIEventAdapter&,
        dart::gui::GUIActionAdapter& action) override
    {
      ++calls;
      action.getViewer().removeEventHandler(self.lock());
      return true;
    }
  };
  dart::gui::Viewer viewer;
  auto node = std::make_shared<dart::gui::WorldNode>(
      dart::simulation::World::create());
  viewer.addWorldNode(node);
  auto handler = std::make_shared<Handler>();
  handler->self = handler;
  viewer.addEventHandler(handler);
  viewer.handleEvent(
      dart::gui::GUIEventAdapter(dart::gui::GUIEventAdapter::KEYDOWN, ' '));
  EXPECT_FALSE(node->isSimulating());
  viewer.handleEvent(
      dart::gui::GUIEventAdapter(dart::gui::GUIEventAdapter::KEYDOWN, ' '));
  EXPECT_TRUE(node->isSimulating());
  EXPECT_EQ(handler->calls, 1);
}

TEST(GuiViewer, StepCallbackMayClearItsWorld)
{
  class Node : public dart::gui::WorldNode
  {
  public:
    using WorldNode::WorldNode;
    void customPreStep() override
    {
      setWorld(nullptr);
    }
  };
  const auto world = dart::simulation::World::create();
  Node node(world);
  node.simulate(true);
  EXPECT_NO_THROW(node.refresh());
  EXPECT_EQ(world->getTime(), 0.0);
}

TEST(GuiViewer, InteractiveToolsRejectInvalidScalesAndIndices)
{
  dart::gui::InteractiveFrame frame(dart::dynamics::Frame::World());
  ASSERT_NE(frame.getTool(dart::gui::InteractiveTool::LINEAR, 0), nullptr);
  EXPECT_FALSE(frame.getTool(dart::gui::InteractiveTool::LINEAR, 0)
                   ->getShapeFrames()
                   .empty());
  EXPECT_EQ(
      frame.getTool(static_cast<dart::gui::InteractiveTool::Type>(-1), 0),
      nullptr);
  EXPECT_THROW(frame.resizeStandardVisuals(0), std::invalid_argument);
}

TEST(GuiViewer, NewAndActivatedNodesInheritRequestedSimulation)
{
  dart::gui::Viewer viewer;
  viewer.simulate(true);
  EXPECT_TRUE(viewer.isSimulating());
  auto active
      = std::make_shared<GuiCountingNode>(dart::simulation::World::create());
  auto inactive
      = std::make_shared<GuiCountingNode>(dart::simulation::World::create());
  viewer.addWorldNode(active);
  viewer.addWorldNode(inactive, false);
  EXPECT_TRUE(active->isSimulating());
  EXPECT_FALSE(inactive->isSimulating());
  viewer.setWorldNodeActive(inactive, true);
  EXPECT_TRUE(inactive->isSimulating());
  viewer.allowSimulation(false);
  EXPECT_FALSE(viewer.isSimulating());
  EXPECT_FALSE(active->isSimulating());
  EXPECT_FALSE(inactive->isSimulating());
}

TEST(GuiViewer, DefaultHandlerMayBeDisabledAndReenabled)
{
  using Event = dart::gui::GUIEventAdapter;
  dart::gui::Viewer viewer;
  auto* handler = viewer.getDefaultEventHandler();
  viewer.switchDefaultEventHandler(false);
  viewer.handleEvent(Event(Event::KEYDOWN, ' '));
  viewer.handleEvent(Event(Event::KEYDOWN, 'h', 0, 0, 0, Event::MODKEY_CTRL));
  EXPECT_FALSE(viewer.isSimulating());
  EXPECT_TRUE(viewer.checkHeadlights());
  viewer.switchDefaultEventHandler(true);
  viewer.handleEvent(Event(Event::KEYDOWN, ' '));
  viewer.handleEvent(Event(Event::KEYDOWN, 'h', 0, 0, 0, Event::MODKEY_CTRL));
  EXPECT_TRUE(viewer.isSimulating());
  EXPECT_FALSE(viewer.checkHeadlights());
  EXPECT_EQ(viewer.getDefaultEventHandler(), handler);
  EXPECT_THROW(dart::gui::DefaultEventHandler(nullptr), std::invalid_argument);
}

TEST(GuiViewer, ScrollCanBeConsumedAndValidatesCoordinates)
{
  using Event = dart::gui::GUIEventAdapter;
  class Handler : public dart::gui::GUIEventHandler
  {
  public:
    bool handle(const Event& event, dart::gui::GUIActionAdapter&) override
    {
      return event.getEventType() == Event::SCROLL;
    }
  };
  dart::gui::Viewer viewer;
  Eigen::Vector3d nearBefore, farBefore, nearAfter, farAfter;
  viewer.getRayUnderCursor(800, 500, nearBefore, farBefore);
  auto handler = std::make_shared<Handler>();
  viewer.addEventHandler(handler);
  const Event scroll(Event::SCROLL, 0, 800, 500, 0, 0, 0, 1);
  EXPECT_EQ(scroll.getScrollingMotion(), Event::SCROLL_UP);
  EXPECT_TRUE(viewer.handleEvent(scroll));
  viewer.getRayUnderCursor(800, 500, nearAfter, farAfter);
  EXPECT_TRUE(nearAfter.isApprox(nearBefore));
  viewer.removeEventHandler(handler);
  EXPECT_TRUE(viewer.handleEvent(scroll));
  viewer.getRayUnderCursor(800, 500, nearAfter, farAfter);
  EXPECT_FALSE(nearAfter.isApprox(nearBefore));
  EXPECT_THROW(
      viewer.handleEvent(Event(
          Event::SCROLL,
          0,
          0,
          0,
          0,
          0,
          0,
          std::numeric_limits<double>::infinity())),
      std::invalid_argument);
}

TEST(GuiViewer, PickingRespectsCameraClipPlanesAndRetainsWorldFrames)
{
  auto world = dart::simulation::World::create();
  const auto makeSphere = [&](double z, double radius) {
    auto frame = std::make_shared<dart::dynamics::SimpleFrame>(
        dart::dynamics::Frame::World());
    frame->setRelativeTranslation({0, 0, z});
    frame->setShape(std::make_shared<dart::dynamics::SphereShape>(radius));
    frame->createVisualAspect();
    world->addSimpleFrame(frame);
    return frame;
  };
  auto beforeNear = makeSphere(-0.002, 0.001);
  auto afterFar = makeSphere(-1002, 0.5);
  auto visible = makeSphere(-2, 0.5);
  dart::gui::Viewer viewer;
  viewer.addWorldNode(std::make_shared<dart::gui::WorldNode>(world));
  viewer.setCameraHome({0, 0, 0}, {0, 0, -1}, {0, 1, 0});
  std::vector<dart::gui::PickInfo> picks;
  viewer.computeIntersections(
      dart::gui::GUIEventAdapter(dart::gui::GUIEventAdapter::PUSH, 0, 800, 500),
      picks);
  ASSERT_EQ(picks.size(), 1u);
  EXPECT_EQ(picks.front().frame, visible.get());
  std::weak_ptr<dart::dynamics::SimpleFrame> retained = visible;
  world->removeSimpleFrame(visible);
  visible.reset();
  EXPECT_FALSE(retained.expired());
  EXPECT_EQ(picks.front().frame->getShape(), picks.front().shape);
  picks.clear();
  EXPECT_TRUE(retained.expired());
}

TEST(GuiViewer, SharedInteractiveChildrenRemainValidAfterRemoval)
{
  auto frame = std::make_shared<dart::gui::InteractiveFrame>(
      dart::dynamics::Frame::World());
  auto child = frame->addShapeFrameShared(
      std::make_shared<dart::dynamics::SphereShape>(0.1));
  auto tool = frame->getToolShared(dart::gui::InteractiveTool::LINEAR, 0);
  auto toolChild = tool->getShapeFrameOwners().front();
  frame->resizeStandardVisuals(2);
  EXPECT_EQ(child->getParentFrame(), dart::dynamics::Frame::World());
  EXPECT_EQ(toolChild->getParentFrame(), dart::dynamics::Frame::World());
  frame.reset();
  EXPECT_EQ(tool->getParentFrame(), dart::dynamics::Frame::World());
  EXPECT_EQ(tool->getInteractiveFrame(), nullptr);
  EXPECT_TRUE(child->getWorldTransform().matrix().allFinite());
  EXPECT_TRUE(toolChild->getWorldTransform().matrix().allFinite());
}

TEST(GuiViewer, AttachmentTeardownMayRemoveOtherAttachments)
{
  class Attachment : public dart::gui::ViewerAttachment
  {
  public:
    dart::gui::Viewer* savedViewer = nullptr;
    std::shared_ptr<Attachment> other;
    int detaches = 0;
    void refresh() override {}

  protected:
    void customAttach(dart::gui::Viewer* viewer) override
    {
      if (viewer) {
        savedViewer = viewer;
        return;
      }
      ++detaches;
      EXPECT_FALSE(savedViewer->handleEvent(
          dart::gui::GUIEventAdapter(dart::gui::GUIEventAdapter::FRAME)));
      EXPECT_NO_THROW(savedViewer->frame());
      EXPECT_TRUE(savedViewer->getRenderBackend().empty());
      if (other)
        savedViewer->removeAttachment(other);
      EXPECT_THROW(
          savedViewer->addAttachment(std::make_shared<Attachment>()),
          std::logic_error);
      EXPECT_THROW(
          savedViewer->addWorldNode(std::make_shared<dart::gui::WorldNode>(
              dart::simulation::World::create())),
          std::logic_error);
    }
  };
  auto first = std::make_shared<Attachment>();
  auto second = std::make_shared<Attachment>();
  first->other = second;
  {
    dart::gui::Viewer viewer;
    viewer.addAttachment(first);
    viewer.addAttachment(second);
  }
  EXPECT_EQ(first->detaches, 1);
  EXPECT_EQ(second->detaches, 1);
  EXPECT_EQ(first->getViewer(), nullptr);
  EXPECT_EQ(second->getViewer(), nullptr);
}

TEST(GuiViewer, DefaultViewerRefreshUpdatesDragTools)
{
  auto frame = std::make_shared<dart::gui::InteractiveFrame>(
      dart::dynamics::Frame::World());
  auto* tool = frame->getTool(dart::gui::InteractiveTool::LINEAR, 0);
  auto* shape = tool->getShapeFrames().front();
  dart::gui::Viewer viewer;
  viewer.enableDragAndDrop(frame.get());
  shape->getVisualAspect()->hide();
  viewer.updateViewer();
  EXPECT_FALSE(shape->getVisualAspect()->isHidden());
}

TEST(GuiViewer, UpwardsDirectionReorientsLightsWithoutChangingCamera)
{
  dart::gui::Viewer viewer;
  viewer.setCameraHome({0, 0, 3}, {0, 0, 0}, {0, 1, 0});
  Eigen::Vector3d nearBefore, farBefore, nearAfter, farAfter;
  viewer.getRayUnderCursor(750, 520, nearBefore, farBefore);
  viewer.setUpwardsDirection(Eigen::Vector3d::UnitY());
  EXPECT_NO_THROW(viewer.setUpwardsDirection(Eigen::Vector3d::UnitZ()));
  viewer.home();
  viewer.getRayUnderCursor(750, 520, nearAfter, farAfter);
  EXPECT_TRUE(nearAfter.isApprox(nearBefore));
  EXPECT_TRUE(farAfter.isApprox(farBefore));
  EXPECT_NO_THROW(viewer.setUpwardsDirection(Eigen::Vector3d::Zero()));
}

TEST(GuiViewer, FrameEventsPreserveHeldDragModifiers)
{
  using Event = dart::gui::GUIEventAdapter;
  dart::gui::Viewer viewer;
  viewer.handleEvent(Event(
      Event::KEYDOWN,
      Event::KEY_Control_L,
      0,
      0,
      0,
      Event::MODKEY_CTRL | Event::MODKEY_SHIFT | Event::MODKEY_SUPER));
  auto* handler = viewer.getDefaultEventHandler();
  viewer.handleEvent(Event(Event::FRAME));
  EXPECT_EQ(
      handler->getModKeyMask(),
      Event::MODKEY_CTRL | Event::MODKEY_SHIFT | Event::MODKEY_SUPER);
  viewer.handleEvent(Event(Event::KEYUP, Event::KEY_Control_L));
  EXPECT_EQ(handler->getModKeyMask(), 0);
}

TEST(GuiViewer, DirectDragsDetachBeforeViewerTeardownCallbacks)
{
  class Attachment : public dart::gui::ViewerAttachment
  {
  public:
    dart::gui::SimpleFrameDnD* simple = nullptr;
    dart::gui::SimpleFrameShapeDnD* shape = nullptr;
    dart::gui::InteractiveFrameDnD* interactive = nullptr;
    bool detached = false;
    void refresh() override {}
    void customAttach(dart::gui::Viewer* viewer) override
    {
      if (viewer)
        return;
      detached = true;
      EXPECT_EQ(simple->getEntity(), nullptr);
      EXPECT_EQ(shape->getEntity(), nullptr);
      EXPECT_EQ(interactive->getEntity(), nullptr);
      EXPECT_NO_THROW(simple->update());
      EXPECT_NO_THROW(simple->move());
      EXPECT_NO_THROW(shape->update());
      EXPECT_NO_THROW(interactive->update());
      EXPECT_TRUE(simple->getConstrainedDx().isZero());
      EXPECT_EQ(simple->getConstrainedRotation().angle(), 0);
    }
  };
  dart::dynamics::SimpleFrame frame;
  auto shape = std::make_shared<dart::dynamics::SphereShape>(0.5);
  frame.setShape(shape);
  dart::gui::InteractiveFrame interactive(dart::dynamics::Frame::World());
  auto viewer = std::make_unique<dart::gui::Viewer>();
  dart::gui::SimpleFrameDnD simpleDrag(viewer.get(), &frame);
  dart::gui::SimpleFrameShapeDnD shapeDrag(viewer.get(), &frame, shape.get());
  dart::gui::InteractiveFrameDnD interactiveDrag(viewer.get(), &interactive);
  auto attachment = std::make_shared<Attachment>();
  attachment->simple = &simpleDrag;
  attachment->shape = &shapeDrag;
  attachment->interactive = &interactiveDrag;
  viewer->addAttachment(attachment);
  EXPECT_NO_THROW(viewer.reset());
  EXPECT_TRUE(attachment->detached);
  EXPECT_EQ(simpleDrag.getEntity(), nullptr);
  EXPECT_EQ(shapeDrag.getShape(), nullptr);
  EXPECT_EQ(interactiveDrag.getFrame(), nullptr);
}

TEST(GuiViewer, DirectDragsIgnoreEventsDuringEarlierObserverNotification)
{
  struct OrderedDrags
  {
    class Observer : public dart::common::Observer
    {
    public:
      explicit Observer(dart::gui::Viewer* viewer)
      {
        addSubject(viewer);
      }
      dart::gui::SimpleFrameDnD* simple = nullptr;
      dart::gui::SimpleFrameShapeDnD* shape = nullptr;
      dart::gui::InteractiveFrameDnD* interactive = nullptr;
      bool notified = false;

    protected:
      void handleDestructionNotification(const dart::common::Subject*) override
      {
        notified = true;
        ASSERT_NE(simple->getEntity(), nullptr);
        EXPECT_NO_THROW(simple->update());
        EXPECT_NO_THROW(simple->move());
        EXPECT_NO_THROW(shape->update());
        EXPECT_NO_THROW(interactive->update());
        EXPECT_TRUE(simple->getConstrainedDx().isZero());
        EXPECT_EQ(simple->getConstrainedRotation().angle(), 0);
      }
    } observer;
    dart::gui::SimpleFrameDnD simple;
    dart::gui::SimpleFrameShapeDnD shape;
    dart::gui::InteractiveFrameDnD interactive;
    OrderedDrags(
        dart::gui::Viewer* viewer,
        dart::dynamics::SimpleFrame* frame,
        dart::gui::InteractiveFrame* interactiveFrame)
      : observer(viewer),
        simple(viewer, frame),
        shape(viewer, frame, frame->getShape().get()),
        interactive(viewer, interactiveFrame)
    {
      observer.simple = &simple;
      observer.shape = &shape;
      observer.interactive = &interactive;
    }
  };
  dart::dynamics::SimpleFrame frame;
  frame.setShape(std::make_shared<dart::dynamics::SphereShape>(0.5));
  dart::gui::InteractiveFrame interactive(dart::dynamics::Frame::World());
  auto viewer = std::make_unique<dart::gui::Viewer>();
  OrderedDrags drags(viewer.get(), &frame, &interactive);
  ASSERT_TRUE(
      std::less<dart::common::Observer*>()(&drags.observer, &drags.simple));
  ASSERT_TRUE(
      std::less<dart::common::Observer*>()(&drags.observer, &drags.shape));
  ASSERT_TRUE(std::less<dart::common::Observer*>()(
      &drags.observer, &drags.interactive));
  EXPECT_NO_THROW(viewer.reset());
  EXPECT_TRUE(drags.observer.notified);
  EXPECT_EQ(drags.simple.getEntity(), nullptr);
  EXPECT_EQ(drags.shape.getEntity(), nullptr);
  EXPECT_EQ(drags.interactive.getEntity(), nullptr);
}

TEST(GuiViewer, InteractiveDragCanBeDisabledByTransformCallback)
{
  using Event = dart::gui::GUIEventAdapter;
  dart::gui::InteractiveFrame frame(dart::dynamics::Frame::World());
  dart::gui::Viewer viewer;
  auto* drag = viewer.enableDragAndDrop(&frame);
  const auto retained = drag->shared_from_this();
  auto* handler = viewer.getDefaultEventHandler();
  dart::gui::GUIActionAdapter action(viewer);
  handler->handle(
      Event(Event::PUSH, 0, 400, 300, Event::LEFT_MOUSE_BUTTON), action);
  auto* toolFrame = frame.getTool(dart::gui::InteractiveTool::LINEAR, 0)
                        ->getShapeFrames()
                        .front();
  auto& picks = const_cast<std::vector<dart::gui::PickInfo>&>(
      handler->getButtonPicks(dart::gui::LEFT_MOUSE, dart::gui::BUTTON_PUSH));
  picks.emplace_back(
      toolFrame,
      toolFrame->getShape(),
      toolFrame->getWorldTransform().translation(),
      Eigen::Vector3d::UnitZ());
  viewer.updateDragAndDrops();
  ASSERT_TRUE(drag->isMoving());
  int notifications = 0;
  auto connection
      = frame.onTransformUpdated.connect([&](const dart::dynamics::Entity*) {
          ++notifications;
          EXPECT_TRUE(viewer.disableDragAndDrop(drag));
        });
  EXPECT_NO_THROW(viewer.updateDragAndDrops());
  EXPECT_EQ(notifications, 1);
  EXPECT_EQ(retained->getEntity(), nullptr);
  EXPECT_FALSE(retained->isMoving());
}

TEST(GuiViewer, InvalidMouseEnumsCannotIndexPickArrays)
{
  dart::gui::Viewer viewer;
  auto* handler = viewer.getDefaultEventHandler();
  for (int button :
       {-1, static_cast<int>(dart::gui::NUM_MOUSE_BUTTONS), 1000000}) {
    const auto invalid = static_cast<dart::gui::MouseButton>(button);
    EXPECT_THROW(handler->getButtonEvent(invalid), std::invalid_argument);
    EXPECT_THROW(
        handler->getButtonPicks(invalid, dart::gui::BUTTON_PUSH),
        std::invalid_argument);
    EXPECT_THROW(
        handler->suppressButtonPicks(invalid, dart::gui::BUTTON_PUSH),
        std::invalid_argument);
    EXPECT_THROW(
        handler->activateButtonPicks(invalid, dart::gui::BUTTON_NOTHING),
        std::invalid_argument);
  }
  for (int event :
       {-1, static_cast<int>(dart::gui::BUTTON_NOTHING) + 1, 1000000}) {
    const auto invalid = static_cast<dart::gui::MouseButtonEvent>(event);
    EXPECT_THROW(
        handler->getButtonPicks(dart::gui::LEFT_MOUSE, invalid),
        std::invalid_argument);
    EXPECT_THROW(
        handler->suppressButtonPicks(dart::gui::LEFT_MOUSE, invalid),
        std::invalid_argument);
    EXPECT_THROW(
        handler->activateButtonPicks(dart::gui::LEFT_MOUSE, invalid),
        std::invalid_argument);
  }
  EXPECT_NO_THROW(handler->suppressButtonPicks(
      dart::gui::LEFT_MOUSE, dart::gui::BUTTON_NOTHING));
  EXPECT_NO_THROW(handler->activateButtonPicks(
      dart::gui::LEFT_MOUSE, dart::gui::BUTTON_NOTHING));
}

TEST(GuiViewer, FailedAttachCallbackPreservesNewViewerOwnership)
{
  class Attachment : public dart::gui::ViewerAttachment,
                     public std::enable_shared_from_this<Attachment>
  {
  public:
    dart::gui::Viewer* destination = nullptr;
    void refresh() override {}

  protected:
    void customAttach(dart::gui::Viewer* viewer) override
    {
      if (!viewer || viewer == destination)
        return;
      viewer->removeAttachment(shared_from_this());
      destination->addAttachment(shared_from_this());
      throw std::runtime_error("fixture migration failure");
    }
  };
  dart::gui::Viewer first, second;
  const auto attachment = std::make_shared<Attachment>();
  attachment->destination = &second;
  EXPECT_THROW(first.addAttachment(attachment), std::runtime_error);
  EXPECT_TRUE(first.getAttachments().empty());
  ASSERT_EQ(second.getAttachments().size(), 1u);
  EXPECT_EQ(attachment->getViewer(), &second);
  second.removeAttachment(attachment);
  EXPECT_EQ(attachment->getViewer(), nullptr);
}

TEST(GuiViewer, RemovingAttachmentThroughPublicListRetainsTheCorrectOwner)
{
  dart::gui::Viewer viewer;
  const auto first = std::make_shared<GuiCountingAttachment>();
  const auto second = std::make_shared<GuiCountingAttachment>();
  viewer.addAttachment(first);
  viewer.addAttachment(second);
  viewer.removeAttachment(viewer.getAttachments().front());
  EXPECT_EQ(first->getViewer(), nullptr);
  EXPECT_EQ(first->detaches, 1);
  EXPECT_EQ(second->getViewer(), &viewer);
  EXPECT_EQ(second->detaches, 0);
  viewer.removeAttachment(viewer.getAttachments().front());
  EXPECT_EQ(second->getViewer(), nullptr);
  EXPECT_EQ(second->detaches, 1);
  EXPECT_TRUE(viewer.getAttachments().empty());
}

TEST(GuiViewer, NativeDragConstructorsRejectNullArguments)
{
  dart::gui::Viewer viewer;
  dart::dynamics::SimpleFrame frame(dart::dynamics::Frame::World());
  dart::gui::InteractiveFrame interactive(dart::dynamics::Frame::World());
  EXPECT_THROW(
      dart::gui::SimpleFrameDnD(nullptr, &frame), std::invalid_argument);
  EXPECT_THROW(
      dart::gui::SimpleFrameDnD(&viewer, nullptr), std::invalid_argument);
  EXPECT_THROW(
      dart::gui::InteractiveFrameDnD(nullptr, &interactive),
      std::invalid_argument);
  EXPECT_THROW(
      dart::gui::InteractiveFrameDnD(&viewer, nullptr), std::invalid_argument);
  EXPECT_THROW(dart::gui::BodyNodeDnD(&viewer, nullptr), std::invalid_argument);
  EXPECT_THROW(
      dart::gui::SimpleFrameShapeDnD(&viewer, &frame, nullptr),
      std::invalid_argument);
}

TEST(GuiViewer, ReleasePicksTrackOnlyNewlyReleasedButtons)
{
  using Event = dart::gui::GUIEventAdapter;
  auto world = dart::simulation::World::create();
  auto frame = std::make_shared<dart::dynamics::SimpleFrame>(
      dart::dynamics::Frame::World());
  frame->setShape(std::make_shared<dart::dynamics::SphereShape>(0.5));
  frame->createVisualAspect();
  world->addSimpleFrame(frame);
  dart::gui::Viewer viewer;
  viewer.addWorldNode(std::make_shared<dart::gui::WorldNode>(world));
  viewer.setCameraHome({0, 0, 3}, {0, 0, 0}, {0, 1, 0});
  auto* handler = viewer.getDefaultEventHandler();
  const auto& left = handler->getButtonPicks(
      dart::gui::LEFT_MOUSE, dart::gui::BUTTON_RELEASE);
  const auto& right = handler->getButtonPicks(
      dart::gui::RIGHT_MOUSE, dart::gui::BUTTON_RELEASE);

  viewer.handleEvent(Event(Event::PUSH, 0, 800, 500, Event::LEFT_MOUSE_BUTTON));
  viewer.handleEvent(Event(Event::RELEASE, 0, 800, 500));
  ASSERT_EQ(left.size(), 1u);
  EXPECT_EQ(left.front().frame, frame.get());
  viewer.handleEvent(Event(Event::PUSH, 0, 800, 500, Event::LEFT_MOUSE_BUTTON));
  viewer.handleEvent(Event(Event::RELEASE, 0, 0, 0));
  EXPECT_TRUE(left.empty());

  const int chord = Event::LEFT_MOUSE_BUTTON | Event::RIGHT_MOUSE_BUTTON;
  viewer.handleEvent(Event(Event::PUSH, 0, 800, 500, chord));
  viewer.handleEvent(Event(Event::DRAG, 0, 800, 500, chord));
  viewer.handleEvent(
      Event(Event::RELEASE, 0, 800, 500, Event::RIGHT_MOUSE_BUTTON));
  ASSERT_EQ(left.size(), 1u);
  EXPECT_EQ(left.front().frame, frame.get());
  EXPECT_TRUE(right.empty());
  EXPECT_EQ(
      handler->getButtonEvent(dart::gui::LEFT_MOUSE),
      dart::gui::BUTTON_RELEASE);
  EXPECT_EQ(
      handler->getButtonEvent(dart::gui::RIGHT_MOUSE), dart::gui::BUTTON_DRAG);
  viewer.handleEvent(Event(Event::RELEASE, 0, 0, 0, Event::RIGHT_MOUSE_BUTTON));
  EXPECT_EQ(left.size(), 1u);
  viewer.handleEvent(Event(Event::RELEASE, 0, 800, 500));
  ASSERT_EQ(right.size(), 1u);
  EXPECT_EQ(right.front().frame, frame.get());

  handler->suppressButtonPicks(
      dart::gui::LEFT_MOUSE, dart::gui::BUTTON_RELEASE);
  viewer.handleEvent(Event(Event::PUSH, 0, 800, 500, chord));
  viewer.handleEvent(Event(Event::RELEASE, 0, 0, 0));
  EXPECT_EQ(left.size(), 1u);
  EXPECT_TRUE(right.empty());
}
