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

#include <dart/gui/osg/osg.hpp>

#include <dart/utils/utils.hpp>

#include <dart/dart.hpp>

#include <osg/GraphicsContext>
#include <osg/Viewport>
#include <osgViewer/Viewer>

#include <algorithm>
#include <array>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <cstdlib>
#include <cstring>

using namespace dart::dynamics;

struct SoftBodyDisplayOptions
{
  float softMeshAlpha = 0.5f;
  bool showEmbeddedVisuals = true;
};

class RecordingWorld : public dart::gui::osg::RealTimeWorldNode
{
public:
  RecordingWorld(const dart::simulation::WorldPtr& world)
    : dart::gui::osg::RealTimeWorldNode(world)
  {
    grabTimeSlice();
    mCurrentIndex = 0;
  }

  void grabTimeSlice()
  {
    TimeSlice slice;
    slice.reserve(mWorld->getNumSkeletons());

    for (std::size_t i = 0; i < mWorld->getNumSkeletons(); ++i) {
      const SkeletonPtr& skeleton = mWorld->getSkeleton(i);
      State state;
      state.mConfig = skeleton->getConfiguration();
      state.mAspectStates.reserve(skeleton->getNumBodyNodes());

      for (std::size_t j = 0; j < skeleton->getNumBodyNodes(); ++j) {
        BodyNode* bn = skeleton->getBodyNode(j);
        state.mAspectStates.push_back(bn->getCompositeState());
      }

      slice.push_back(state);
    }

    mHistory.push_back(slice);
  }

  void customPostStep() override
  {
    if (mCurrentIndex < mHistory.size() - 1)
      mHistory.resize(mCurrentIndex + 1);

    grabTimeSlice();
    ++mCurrentIndex;
  }

  void moveTo(std::size_t index)
  {
    mViewer->simulate(false);

    if (mHistory.empty())
      return;

    if (index >= mHistory.size())
      index = mHistory.size() - 1;

    std::cout << "Moving to time step #" << index << std::endl;

    const TimeSlice& slice = mHistory[index];
    for (std::size_t i = 0; i < slice.size(); ++i) {
      const State& state = slice[i];
      const SkeletonPtr& skeleton = mWorld->getSkeleton(i);

      skeleton->setConfiguration(state.mConfig);

      for (std::size_t j = 0; j < skeleton->getNumBodyNodes(); ++j) {
        BodyNode* bn = skeleton->getBodyNode(j);
        bn->setCompositeState(state.mAspectStates[j]);
      }
    }

    mCurrentIndex = index;
  }

  void moveForward(int delta)
  {
    moveTo(mCurrentIndex + delta);
  }

  void moveBackward(int delta)
  {
    if (mCurrentIndex > 0)
      moveTo(mCurrentIndex - delta);
  }

  void restart()
  {
    moveTo(0);
  }

  void moveToEnd()
  {
    moveTo(mHistory.size() - 1);
  }

  struct State
  {
    Skeleton::Configuration mConfig;
    std::vector<dart::common::Composite::State> mAspectStates;
  };

  using TimeSlice = std::vector<State>;
  using History = std::vector<TimeSlice>;

  History mHistory;

  std::size_t mCurrentIndex;
};

class RecordingEventHandler : public osgGA::GUIEventHandler
{
public:
  RecordingEventHandler(RecordingWorld* rec) : mRecWorld(rec)
  {
    // Do nothing
  }

  virtual bool handle(
      const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&) override
  {
    if (!mRecWorld)
      return false;

    if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN) {
      if (ea.getKey() == '[') {
        mRecWorld->moveBackward(1);
        return true;
      }

      if (ea.getKey() == ']') {
        mRecWorld->moveForward(1);
        return true;
      }

      if (ea.getKey() == '{') {
        mRecWorld->moveBackward(10);
        return true;
      }

      if (ea.getKey() == '}') {
        mRecWorld->moveForward(10);
        return true;
      }

      if (ea.getKey() == 'r') {
        mRecWorld->restart();
        return true;
      }

      if (ea.getKey() == '\\') {
        mRecWorld->moveToEnd();
        return true;
      }
    }

    return false;
  }

  RecordingWorld* mRecWorld;
};

struct Options
{
  double scale = 1.0;
  bool showHelp = false;
  bool headless = false;
  std::string shotPath = "soft_bodies.png";
  int steps = 120;
  int width = 640;
  int height = 480;
  SoftBodyDisplayOptions display;
};

void printUsage(const char* executable)
{
  dart::gui::osg::printGuiScaleUsage(std::cout, executable);
  std::cout << "\nAdditional options:\n"
            << "  --headless       Render off-screen to a PNG and exit.\n"
            << "  --shot PATH      Output PNG path for --headless (default "
               "soft_bodies.png).\n"
            << "  --steps N        Simulation steps before a headless capture "
               "(default 120).\n"
            << "  --width W        Headless capture width before --gui-scale "
               "(default 640).\n"
            << "  --height H       Headless capture height before --gui-scale "
               "(default 480).\n"
            << "  --soft-alpha A   Soft mesh alpha in [0, 1] (default 0.5).\n"
            << "  --hide-embedded  Hide embedded rigid visualization shapes.\n";
}

Options parseOptions(int argc, char* argv[])
{
  Options options;

  std::vector<char*> forwarded;
  forwarded.push_back(argv[0]);
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--headless") == 0) {
      options.headless = true;
    } else if (std::strcmp(argv[i], "--shot") == 0 && i + 1 < argc) {
      options.shotPath = argv[++i];
    } else if (std::strcmp(argv[i], "--steps") == 0 && i + 1 < argc) {
      options.steps = std::atoi(argv[++i]);
    } else if (std::strcmp(argv[i], "--width") == 0 && i + 1 < argc) {
      options.width = std::atoi(argv[++i]);
    } else if (std::strcmp(argv[i], "--height") == 0 && i + 1 < argc) {
      options.height = std::atoi(argv[++i]);
    } else if (std::strcmp(argv[i], "--soft-alpha") == 0 && i + 1 < argc) {
      options.display.softMeshAlpha
          = static_cast<float>(std::clamp(std::atof(argv[++i]), 0.0, 1.0));
    } else if (std::strcmp(argv[i], "--hide-embedded") == 0) {
      options.display.showEmbeddedVisuals = false;
    } else {
      forwarded.push_back(argv[i]);
    }
  }

  const dart::gui::osg::GuiScaleOptions gui
      = dart::gui::osg::parseGuiScaleOptions(
          static_cast<int>(forwarded.size()), forwarded.data(), &std::cerr);
  options.scale = gui.scale;
  options.showHelp = gui.showHelp;
  return options;
}

void styleSoftBodyVisuals(
    const dart::simulation::WorldPtr& world,
    const SoftBodyDisplayOptions& display)
{
  const double alpha = std::clamp<double>(display.softMeshAlpha, 0.0, 1.0);
  const std::array<Eigen::Vector4d, 5> softBodyColors = {
      Eigen::Vector4d(0.35, 0.48, 1.00, alpha),
      Eigen::Vector4d(0.95, 0.45, 0.30, alpha),
      Eigen::Vector4d(0.30, 0.72, 0.48, alpha),
      Eigen::Vector4d(0.95, 0.76, 0.28, alpha),
      Eigen::Vector4d(0.70, 0.45, 0.90, alpha),
  };

  std::size_t softBodyIndex = 0;
  for (std::size_t i = 0; i < world->getNumSkeletons(); ++i) {
    const SkeletonPtr& skeleton = world->getSkeleton(i);
    for (std::size_t j = 0; j < skeleton->getNumBodyNodes(); ++j) {
      auto* softBodyNode = skeleton->getBodyNode(j)->asSoftBodyNode();
      if (!softBodyNode)
        continue;

      const Eigen::Vector4d& color
          = softBodyColors[softBodyIndex % softBodyColors.size()];
      ++softBodyIndex;

      for (std::size_t k = 0; k < softBodyNode->getNumShapeNodes(); ++k) {
        auto* shapeNode = softBodyNode->getShapeNode(k);
        auto* visualAspect = shapeNode->getVisualAspect(false);
        if (!visualAspect)
          continue;

        const auto shape = shapeNode->getShape();
        if (shape && shape->getType() == SoftMeshShape::getStaticType()) {
          visualAspect->setRGBA(color);
          visualAspect->show();
        } else if (display.showEmbeddedVisuals) {
          visualAspect->show();
        } else {
          visualAspect->hide();
        }
      }
    }
  }
}

void applySoftBodiesCameraPose(dart::gui::osg::Viewer& viewer);

class SoftBodiesWidget : public dart::gui::osg::ImGuiWidget
{
public:
  SoftBodiesWidget(
      dart::gui::osg::ImGuiViewer* viewer,
      dart::simulation::WorldPtr world,
      SoftBodyDisplayOptions* display)
    : mViewer(viewer), mWorld(std::move(world)), mDisplay(display)
  {
  }

  void render() override
  {
    const float guiScale
        = static_cast<float>(mViewer->getImGuiHandler()->getGuiScale());
    const float margin = 12.0f * guiScale;
    ImGui::SetNextWindowPos(ImVec2(margin, margin), ImGuiCond_Once);
    ImGui::SetNextWindowSize(
        ImVec2(300.0f * guiScale, 190.0f * guiScale), ImGuiCond_Once);
    ImGui::SetNextWindowBgAlpha(0.92f);
    if (!ImGui::Begin(
            "Soft Bodies",
            nullptr,
            ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings)) {
      ImGui::End();
      return;
    }

    bool changed = false;
    bool simulating = mViewer->isSimulating();
    if (ImGui::Checkbox("Run simulation", &simulating))
      mViewer->simulate(simulating);

    changed |= ImGui::SliderFloat(
        "Soft mesh alpha", &mDisplay->softMeshAlpha, 0.0f, 1.0f, "%.2f");
    changed
        |= ImGui::Checkbox("Embedded visuals", &mDisplay->showEmbeddedVisuals);

    if (ImGui::Button("Reset view"))
      applySoftBodiesCameraPose(*mViewer);

    ImGui::Separator();
    ImGui::Text("Time %.3f s", mWorld->getTime());
    ImGui::SameLine();
    ImGui::Text("FPS %.0f", ImGui::GetIO().Framerate);

    if (changed)
      styleSoftBodyVisuals(mWorld, *mDisplay);

    ImGui::End();
  }

private:
  dart::gui::osg::ImGuiViewer* mViewer;
  dart::simulation::WorldPtr mWorld;
  SoftBodyDisplayOptions* mDisplay;
};

void applySoftBodiesCameraPose(dart::gui::osg::Viewer& viewer)
{
  const ::osg::Vec3 eye(3.0f, 1.5f, 3.0f);
  const ::osg::Vec3 center(0.0f, 0.0f, 0.0f);
  const ::osg::Vec3 up(0.0f, 1.0f, 0.0f);

  viewer.setUpwardsDirection(up);
  viewer.getCameraManipulator()->setHomePosition(eye, center, up);
  viewer.setCameraManipulator(viewer.getCameraManipulator());
  viewer.getCamera()->setViewMatrixAsLookAt(eye, center, up);
}

int runHeadless(
    dart::gui::osg::Viewer& viewer,
    RecordingWorld* node,
    const dart::simulation::WorldPtr& world,
    const Options& options)
{
  const int width = dart::gui::osg::scaleWindowExtent(
      std::max(1, options.width), options.scale);
  const int height = dart::gui::osg::scaleWindowExtent(
      std::max(1, options.height), options.scale);

  ::osg::ref_ptr<::osg::GraphicsContext::Traits> traits
      = new ::osg::GraphicsContext::Traits;
  traits->readDISPLAY();
  traits->setUndefinedScreenDetailsToDefaultScreen();
  traits->x = 0;
  traits->y = 0;
  traits->width = width;
  traits->height = height;
  traits->red = traits->green = traits->blue = 8;
  traits->alpha = 8;
  traits->depth = 24;
  traits->windowDecoration = false;
  traits->pbuffer = true;
  traits->doubleBuffer = true;

  ::osg::ref_ptr<::osg::GraphicsContext> gc
      = ::osg::GraphicsContext::createGraphicsContext(traits.get());
  if (!gc) {
    std::cerr << "[headless] Failed to create an off-screen GL context "
                 "(no usable DISPLAY?).\n";
    return 1;
  }

  auto* camera = viewer.getCamera();
  camera->setGraphicsContext(gc.get());
  camera->setViewport(new ::osg::Viewport(0, 0, width, height));
  camera->setProjectionMatrixAsPerspective(
      30.0, static_cast<double>(width) / height, 0.1, 1000.0);
  const GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;
  camera->setDrawBuffer(buffer);
  camera->setReadBuffer(buffer);

  viewer.setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
  viewer.simulate(false);
  node->simulate(false);
  applySoftBodiesCameraPose(viewer);

  viewer.realize();
  if (!viewer.isRealized()) {
    std::cerr << "[headless] Viewer failed to realize off-screen.\n";
    return 1;
  }
  if (auto* queue = viewer.getEventQueue()) {
    queue->windowResize(0, 0, width, height);
    queue->setMouseInputRange(0.0f, 0.0f, width, height);
  }

  for (int i = 0; i < std::max(0, options.steps); ++i) {
    world->step();
    node->grabTimeSlice();
  }

  applySoftBodiesCameraPose(viewer);
  viewer.frame();
  viewer.frame();
  viewer.captureScreen(options.shotPath);
  viewer.frame();

  std::cout << "[headless] wrote " << options.shotPath << " (" << width << "x"
            << height << ", steps " << std::max(0, options.steps) << ")\n";
  return 0;
}

int main(int argc, char* argv[])
{
  Options options = parseOptions(argc, argv);
  if (options.showHelp) {
    printUsage(argv[0]);
    return 0;
  }

  using namespace dart::dynamics;

  dart::simulation::WorldPtr world = dart::utils::SkelParser::readWorld(
      "dart://sample/skel/softBodies.skel");
  if (!world) {
    std::cerr << "Failed to load dart://sample/skel/softBodies.skel\n";
    return 1;
  }

  styleSoftBodyVisuals(world, options.display);

  osg::ref_ptr<RecordingWorld> node = new RecordingWorld(world);

  node->simulate(true);

  osg::ref_ptr<dart::gui::osg::ImGuiViewer> viewer
      = new dart::gui::osg::ImGuiViewer();
  viewer->addWorldNode(node);
  viewer->addEventHandler(new RecordingEventHandler(node));
  viewer->getImGuiHandler()->setGuiScale(options.scale);
  applySoftBodiesCameraPose(*viewer);

  std::cout << viewer->getInstructions() << std::endl;

  if (options.headless)
    return runHeadless(*viewer, node.get(), world, options);

  viewer->getImGuiHandler()->addWidget(std::make_shared<SoftBodiesWidget>(
      viewer.get(), world, &options.display));

  viewer->setUpViewInWindow(
      0,
      0,
      dart::gui::osg::scaleWindowExtent(640, options.scale),
      dart::gui::osg::scaleWindowExtent(480, options.scale));

  viewer->run();
}
