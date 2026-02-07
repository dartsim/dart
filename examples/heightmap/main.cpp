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

#include "dart/common/macros.hpp"

#include <dart/config.hpp>

#include <dart/gui/all.hpp>
#include <dart/gui/im_gui_handler.hpp>
#include <dart/gui/include_im_gui.hpp>

#include <dart/utils/All.hpp>
#include <dart/utils/urdf/All.hpp>

#if DART_HAVE_ODE
  #include <dart/collision/ode/ode_collision_detector.hpp>
#endif

#include <dart/all.hpp>

#include <CLI/CLI.hpp>

#include <string>
#include <vector>

#include <cmath>

using namespace dart;
using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::math;

struct HeightmapAlignmentDemoConfig
{
  Eigen::Vector3d heightmapOrigin = Eigen::Vector3d::Zero();
  std::size_t heightmapXResolution = 2u;
  std::size_t heightmapYResolution = 2u;
  float heightmapScale = 2.0f;
  float heightmapZMin = 0.0f;
  float heightmapZMax = 0.0f;
  Eigen::Vector3d boxSize = Eigen::Vector3d(2.0, 2.0, 2.0);
  Eigen::Vector3d boxOffset = Eigen::Vector3d(3.0, 0.0, -1.0);
  std::size_t ballGridCount = 5u;
  double ballRadius = 0.08;
  double ballMass = 0.1;
  double ballDropHeight = 1.0;
};

template <typename S>
typename HeightmapShape<S>::HeightField generateHeightField(
    std::size_t xResolution, std::size_t yResolution, S zMin, S zMax)
{
  typename HeightmapShape<S>::HeightField data(yResolution, xResolution);
  for (auto i = 0u; i < yResolution; ++i) {
    for (auto j = 0u; j < xResolution; ++j) {
      data(i, j) = math::Random::uniform(zMin, zMax);
    }
  }
  return data;
}

template <typename S>
dynamics::ShapePtr createHeightmapShape(
    std::size_t xResolution = 20u,
    std::size_t yResolution = 20u,
    S xSize = S(2),
    S ySize = S(2),
    S zMin = S(0.0),
    S zMax = S(0.1))
{
  using Vector3 = Eigen::Matrix<S, 3, 1>;

  auto data = generateHeightField<S>(xResolution, yResolution, zMin, zMax);
  const auto xStride = xResolution > 1 ? xResolution - 1 : 1u;
  const auto yStride = yResolution > 1 ? yResolution - 1 : 1u;
  auto scale = Vector3(
      xSize / static_cast<S>(xStride), ySize / static_cast<S>(yStride), 1);

  auto terrainShape = std::make_shared<HeightmapShape<S>>();
  terrainShape->setScale(scale);
  terrainShape->setHeightField(data);
  terrainShape->setDataVariance(dynamics::Shape::DYNAMIC);

  return terrainShape;
}

template <typename S>
dynamics::SimpleFramePtr createHeightmapFrame(
    std::size_t xResolution = 20u,
    std::size_t yResolution = 20u,
    S xSize = S(2),
    S ySize = S(2),
    S zMin = S(0.0),
    S zMax = S(0.1))
{
  auto terrainFrame = SimpleFrame::createShared(Frame::World());

  terrainFrame->createVisualAspect();

  auto terrainShape = createHeightmapShape(
      xResolution, yResolution, xSize, ySize, zMin, zMax);
  terrainFrame->setShape(terrainShape);

  return terrainFrame;
}

SkeletonPtr createAlignmentHeightmap(const HeightmapAlignmentDemoConfig& config)
{
  auto heightmap = Skeleton::create("heightmap");

  auto [joint, body]
      = heightmap->createJointAndBodyNodePair<WeldJoint>(nullptr);
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = config.heightmapOrigin;
  joint->setTransformFromParentBodyNode(tf);

  std::vector<float> heights(
      config.heightmapXResolution * config.heightmapYResolution, 0.0f);
  for (auto& height : heights) {
    height = math::Random::uniform(config.heightmapZMin, config.heightmapZMax);
  }

  auto shape = std::make_shared<HeightmapShape<float>>();
  shape->setHeightField(
      config.heightmapXResolution, config.heightmapYResolution, heights);
  shape->setScale(
      Eigen::Vector3f(config.heightmapScale, config.heightmapScale, 1.0f));

  auto shapeNode = body->createShapeNodeWith<
      CollisionAspect,
      DynamicsAspect,
      VisualAspect>(shape);
  shapeNode->getVisualAspect()->setColor(Eigen::Vector3d(0.2, 0.4, 0.9));

  return heightmap;
}

SkeletonPtr createAlignmentReferenceBox(
    const HeightmapAlignmentDemoConfig& config)
{
  auto box = Skeleton::create("reference_box");

  auto [joint, body] = box->createJointAndBodyNodePair<WeldJoint>(nullptr);
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = config.boxOffset;
  joint->setTransformFromParentBodyNode(tf);

  auto shape = std::make_shared<BoxShape>(config.boxSize);
  auto shapeNode = body->createShapeNodeWith<
      CollisionAspect,
      DynamicsAspect,
      VisualAspect>(shape);
  shapeNode->getVisualAspect()->setColor(Eigen::Vector3d(0.2, 0.7, 0.2));

  return box;
}

SkeletonPtr createAlignmentBall(
    const std::string& name,
    const Eigen::Vector3d& position,
    double radius,
    double mass,
    const Eigen::Vector3d& color)
{
  auto ball = Skeleton::create(name);

  auto [joint, body] = ball->createJointAndBodyNodePair<FreeJoint>(nullptr);
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = position;
  joint->setTransformFromParentBodyNode(tf);

  auto shape = std::make_shared<SphereShape>(radius);
  auto shapeNode = body->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(shape);
  shapeNode->getVisualAspect()->setColor(color);

  dart::dynamics::Inertia inertia;
  inertia.setMass(mass);
  inertia.setMoment(shape->computeInertia(mass));
  body->setInertia(inertia);

  return ball;
}

void addAlignmentBallGrid(
    const simulation::WorldPtr& world,
    const std::string& prefix,
    const Eigen::Vector3d& center,
    double halfExtent,
    std::size_t count,
    double dropHeight,
    double radius,
    double mass,
    const Eigen::Vector3d& color)
{
  if (count == 0u) {
    return;
  }

  const double step = (count == 1u) ? 0.0 : (2.0 * halfExtent) / (count - 1u);
  std::size_t index = 0u;
  for (std::size_t row = 0u; row < count; ++row) {
    for (std::size_t col = 0u; col < count; ++col) {
      const double x = center.x() - halfExtent + step * row;
      const double y = center.y() - halfExtent + step * col;
      const double z = dropHeight;
      const Eigen::Vector3d position(x, y, z);
      world->addSkeleton(createAlignmentBall(
          prefix + std::to_string(index++), position, radius, mass, color));
    }
  }
}

void setupAlignmentDemo(const simulation::WorldPtr& world)
{
  HeightmapAlignmentDemoConfig config;

  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world->setTimeStep(0.001);
#if DART_HAVE_ODE
  world->getConstraintSolver()->setCollisionDetector(
      collision::OdeCollisionDetector::create());
#else
  DART_WARN(
      "Heightmap alignment demo requires ODE; using default collision "
      "detector.");
#endif

  world->addSkeleton(createAlignmentHeightmap(config));

  // Offset the reference box in X to keep collisions from overlapping while
  // preserving the vertical offset described in the report.
  world->addSkeleton(createAlignmentReferenceBox(config));

  const double halfExtent = 1.0 - config.ballRadius * 1.1;
  const Eigen::Vector3d ballColor(0.9, 0.7, 0.3);
  addAlignmentBallGrid(
      world,
      "heightmap_ball_",
      config.heightmapOrigin,
      halfExtent,
      config.ballGridCount,
      config.ballDropHeight,
      config.ballRadius,
      config.ballMass,
      ballColor);
  addAlignmentBallGrid(
      world,
      "box_ball_",
      config.heightmapOrigin + config.boxOffset,
      halfExtent,
      config.ballGridCount,
      config.ballDropHeight,
      config.ballRadius,
      config.ballMass,
      ballColor);
}

class HeightmapWorld : public gui::WorldNode
{
public:
  explicit HeightmapWorld(simulation::WorldPtr world)
    : gui::WorldNode(std::move(world))
  {
    // Do nothing
  }

  // Triggered at the beginning of each simulation step
  void customPreStep() override
  {
    // Do nothing
  }

protected:
};

template <typename S>
class HeightmapWidget : public dart::gui::ImGuiWidget
{
public:
  HeightmapWidget(
      dart::gui::ImGuiViewer* viewer,
      HeightmapWorld* node,
      dynamics::SimpleFramePtr terrain,
      gui::GridVisual* grid,
      std::size_t xResolution = 20u,
      std::size_t yResolution = 20u,
      S xSize = S(2),
      S ySize = S(2),
      S zMin = S(0.0),
      S zMax = S(0.1))
    : mViewer(viewer),
      mNode(node),
      mTerrain(std::move(terrain)),
      mGrid(grid),
      mHeightmapShape(
          std::dynamic_pointer_cast<HeightmapShape<S>>(mTerrain->getShape()))
  {
    mXResolution = xResolution;
    mYResolution = yResolution;
    mXSize = xSize;
    mYSize = ySize;
    mZMin = zMin;
    mZMax = zMax;

    updateHeightmapShape();
  }

  void updateHeightmapShape()
  {
    if (!mHeightmapShape) {
      mTerrain->setShape(createHeightmapShape(
          mXResolution, mYResolution, mXSize, mYSize, mZMin, mZMax));
      mHeightmapShape
          = std::dynamic_pointer_cast<HeightmapShape<S>>(mTerrain->getShape());
      if (!mHeightmapShape) {
        return;
      }
    }

    mHeightmapShape->setHeightField(
        generateHeightField<S>(mXResolution, mYResolution, mZMin, mZMax));
    const auto xStride = mXResolution > 1 ? mXResolution - 1 : 1u;
    const auto yStride = mYResolution > 1 ? mYResolution - 1 : 1u;
    Eigen::Matrix<S, 3, 1> scale(
        mXSize / static_cast<S>(xStride),
        mYSize / static_cast<S>(yStride),
        S(1));
    mHeightmapShape->setScale(scale);
    mTerrain->setShape(mHeightmapShape);
  }

  void render() override
  {
    ImGui::SetNextWindowPos(ImVec2(10, 20));
    ImGui::SetNextWindowSize(ImVec2(360, 600));
    ImGui::SetNextWindowBgAlpha(0.5f);
    if (!ImGui::Begin(
            "Heightmap Demo",
            nullptr,
            ImGuiWindowFlags_MenuBar | ImGuiWindowFlags_HorizontalScrollbar)) {
      // Early out if the window is collapsed, as an optimization.
      ImGui::End();
      return;
    }

    // Menu
    if (ImGui::BeginMenuBar()) {
      if (ImGui::BeginMenu("Menu")) {
        if (ImGui::MenuItem("Exit")) {
          mViewer->setDone(true);
        }
        ImGui::EndMenu();
      }
      if (ImGui::BeginMenu("Help")) {
        if (ImGui::MenuItem("About DART")) {
          mViewer->showAbout();
        }
        ImGui::EndMenu();
      }
      ImGui::EndMenuBar();
    }

    ImGui::Text("Heightmap rendering example");
    ImGui::Spacing();
    ImGui::TextWrapped(
        "Tweak the controls below to regenerate the heightmap. The grid stays "
        "aligned with the terrain so you can check updates in real time.");

    if (ImGui::CollapsingHeader("Help")) {
      ImGui::PushTextWrapPos(ImGui::GetCursorPos().x + 320);
      ImGui::Text("User Guid:\n");
      ImGui::Text("%s", mViewer->getInstructions().c_str());
      ImGui::PopTextWrapPos();
    }

    if (ImGui::CollapsingHeader("Simulation", ImGuiTreeNodeFlags_DefaultOpen)) {
      int e = mViewer->isSimulating() ? 0 : 1;
      if (mViewer->isAllowingSimulation()) {
        if (ImGui::RadioButton("Play", &e, 0) && !mViewer->isSimulating()) {
          mViewer->simulate(true);
        }
        ImGui::SameLine();
        if (ImGui::RadioButton("Pause", &e, 1) && mViewer->isSimulating()) {
          mViewer->simulate(false);
        }
      }
    }

    if (mTerrain) {
      if (ImGui::CollapsingHeader(
              "Heightmap", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("Terrain");

        auto aspect = mTerrain->getVisualAspect();
        bool display = !aspect->isHidden();
        if (ImGui::Checkbox("Show##Terrain", &display)) {
          if (display) {
            aspect->show();
          } else {
            aspect->hide();
          }
        }

        auto shape = std::dynamic_pointer_cast<dynamics::HeightmapShapef>(
            mTerrain->getShape());
        DART_ASSERT(shape);

        int xResolution = static_cast<int>(mXResolution);
        if (ImGui::InputInt("X Resolution", &xResolution, 5, 10)) {
          if (xResolution < 5) {
            xResolution = 5;
          }

          if (static_cast<int>(mXResolution) != xResolution) {
            mXResolution = xResolution;
            updateHeightmapShape();
          }
        }

        int yResolution = static_cast<int>(mYResolution);
        if (ImGui::InputInt("Y Resolution", &yResolution, 5, 10)) {
          if (yResolution < 5) {
            yResolution = 5;
          }

          if (static_cast<int>(mYResolution) != yResolution) {
            mYResolution = yResolution;
            updateHeightmapShape();
          }
        }

        ImGui::Separator();

        if (ImGui::InputFloat("X Size", &mXSize, 0.1, 0.2)) {
          if (mXSize < 0.1) {
            mXSize = 0.1;
          }

          updateHeightmapShape();
        }

        if (ImGui::InputFloat("Y Size", &mYSize, 0.1, 0.2)) {
          if (mYSize < 0.1) {
            mYSize = 0.1;
          }

          updateHeightmapShape();
        }

        ImGui::Separator();

        if (ImGui::InputFloat("Z Min", &mZMin, 0.05, 0.1)) {
          updateHeightmapShape();
        }

        if (ImGui::InputFloat("Z Max", &mZMax, 0.05, 0.1)) {
          updateHeightmapShape();
        }

        ImGui::Separator();

        auto visualAspect = mTerrain->getVisualAspect();

        float color[4];
        auto visualColor = visualAspect->getRGBA();
        color[0] = static_cast<float>(visualColor[0]);
        color[1] = static_cast<float>(visualColor[1]);
        color[2] = static_cast<float>(visualColor[2]);
        color[3] = static_cast<float>(visualColor[3]);
        if (ImGui::ColorEdit4("Color##Heightmap", color)) {
          visualColor[0] = static_cast<double>(color[0]);
          visualColor[1] = static_cast<double>(color[1]);
          visualColor[2] = static_cast<double>(color[2]);
          visualColor[3] = static_cast<double>(color[3]);
          visualAspect->setColor(visualColor);
        }
      }
    }

    if (mGrid) {
      if (ImGui::CollapsingHeader("Grid", ImGuiTreeNodeFlags_None)) {
        ImGui::Text("Grid");

        bool display = mGrid->isDisplayed();
        if (ImGui::Checkbox("Show##Grid", &display)) {
          mGrid->display(display);
        }

        if (display) {
          int e = static_cast<int>(mGrid->getPlaneType());
          if (mViewer->isAllowingSimulation()) {
            if (ImGui::RadioButton("XY-Plane", &e, 0)) {
              mGrid->setPlaneType(gui::GridVisual::PlaneType::XY);
            }
            ImGui::SameLine();
            if (ImGui::RadioButton("YZ-Plane", &e, 1)) {
              mGrid->setPlaneType(gui::GridVisual::PlaneType::YZ);
            }
            ImGui::SameLine();
            if (ImGui::RadioButton("ZX-Plane", &e, 2)) {
              mGrid->setPlaneType(gui::GridVisual::PlaneType::ZX);
            }
          }

          static Eigen::Vector3f offset;
          ImGui::Columns(3);
          offset = mGrid->getOffset().cast<float>();
          if (ImGui::InputFloat("X", &offset[0], 0.1f, 0.5f, "%.1f")) {
            mGrid->setOffset(offset.cast<double>());
          }
          ImGui::NextColumn();
          if (ImGui::InputFloat("Y", &offset[1], 0.1f, 0.5f, "%.1f")) {
            mGrid->setOffset(offset.cast<double>());
          }
          ImGui::NextColumn();
          if (ImGui::InputFloat("Z", &offset[2], 0.1f, 0.5f, "%.1f")) {
            mGrid->setOffset(offset.cast<double>());
          }
          ImGui::Columns(1);

          static int cellCount;
          cellCount = static_cast<int>(mGrid->getNumCells());
          if (ImGui::InputInt("Line Count", &cellCount, 1, 5)) {
            if (cellCount < 0) {
              cellCount = 0;
            }
            mGrid->setNumCells(static_cast<std::size_t>(cellCount));
          }

          static float cellStepSize;
          cellStepSize = static_cast<float>(mGrid->getMinorLineStepSize());
          if (ImGui::InputFloat(
                  "Line Step Size", &cellStepSize, 0.001f, 0.1f)) {
            mGrid->setMinorLineStepSize(static_cast<double>(cellStepSize));
          }

          static int minorLinesPerMajorLine;
          minorLinesPerMajorLine
              = static_cast<int>(mGrid->getNumMinorLinesPerMajorLine());
          if (ImGui::InputInt(
                  "Minor Lines per Major Line",
                  &minorLinesPerMajorLine,
                  1,
                  5)) {
            if (minorLinesPerMajorLine < 0) {
              minorLinesPerMajorLine = 0;
            }
            mGrid->setNumMinorLinesPerMajorLine(
                static_cast<std::size_t>(minorLinesPerMajorLine));
          }

          static float axisLineWidth;
          axisLineWidth = mGrid->getAxisLineWidth();
          if (ImGui::InputFloat(
                  "Axis Line Width", &axisLineWidth, 1.f, 2.f, "%.0f")) {
            mGrid->setAxisLineWidth(axisLineWidth);
          }

          static float majorLineWidth;
          majorLineWidth = mGrid->getMajorLineWidth();
          if (ImGui::InputFloat(
                  "Major Line Width", &majorLineWidth, 1.f, 2.f, "%.0f")) {
            mGrid->setMajorLineWidth(majorLineWidth);
          }

          static float majorColor[3];
          auto internalmajorColor = mGrid->getMajorLineColor();
          majorColor[0] = static_cast<float>(internalmajorColor.x());
          majorColor[1] = static_cast<float>(internalmajorColor.y());
          majorColor[2] = static_cast<float>(internalmajorColor.z());
          if (ImGui::ColorEdit3("Major Line Color", majorColor)) {
            internalmajorColor[0] = static_cast<double>(majorColor[0]);
            internalmajorColor[1] = static_cast<double>(majorColor[1]);
            internalmajorColor[2] = static_cast<double>(majorColor[2]);
            mGrid->setMajorLineColor(internalmajorColor);
          }

          static float minorLineWidth;
          minorLineWidth = mGrid->getMinorLineWidth();
          if (ImGui::InputFloat(
                  "Minor Line Width", &minorLineWidth, 1.f, 2.f, "%.0f")) {
            mGrid->setMinorLineWidth(minorLineWidth);
          }

          float minorColor[3];
          auto internalMinorColor = mGrid->getMinorLineColor();
          minorColor[0] = static_cast<float>(internalMinorColor.x());
          minorColor[1] = static_cast<float>(internalMinorColor.y());
          minorColor[2] = static_cast<float>(internalMinorColor.z());
          if (ImGui::ColorEdit3("Minor Line Color", minorColor)) {
            internalMinorColor[0] = static_cast<double>(minorColor[0]);
            internalMinorColor[1] = static_cast<double>(minorColor[1]);
            internalMinorColor[2] = static_cast<double>(minorColor[2]);
            mGrid->setMinorLineColor(internalMinorColor);
          }
        }
      }
    }

    ImGui::End();
  }

protected:
  dart::gui::ImGuiViewer* mViewer;
  HeightmapWorld* mNode;
  dynamics::SimpleFramePtr mTerrain;
  ::osg::ref_ptr<gui::GridVisual> mGrid;
  std::size_t mXResolution;
  std::size_t mYResolution;
  float mXSize;
  float mYSize;
  float mZMin;
  float mZMax;
  std::shared_ptr<HeightmapShape<S>> mHeightmapShape;
};

int main(int argc, char* argv[])
{
  CLI::App app("Heightmap example");
  double guiScale = 1.0;
  std::string demo = "interactive";
  app.add_option("--gui-scale", guiScale, "Scale factor for ImGui widgets")
      ->check(CLI::PositiveNumber);
  app.add_option(
         "--demo",
         demo,
         "Demo mode: interactive or alignment (heightmap + box comparison)")
      ->check(CLI::IsMember({"interactive", "alignment"}));
  CLI11_PARSE(app, argc, argv);

  auto world = dart::simulation::World::create();
  const bool alignmentDemo = demo == "alignment";
  if (alignmentDemo) {
    setupAlignmentDemo(world);
  } else {
    world->setGravity(Eigen::Vector3d::Zero());
  }

  // Use a wider height range out of the box so the surface is visible above
  // the grid without fiddling with the controls.
  constexpr auto xResolution = 100u;
  constexpr auto yResolution = 100u;
  constexpr auto xSize = 2.f;
  constexpr auto ySize = 2.f;
  constexpr auto zMin = -0.1f;
  constexpr auto zMax = 0.4f;
  dynamics::SimpleFramePtr terrain;
  if (!alignmentDemo) {
    terrain = createHeightmapFrame<float>(
        xResolution, yResolution, xSize, ySize, zMin, zMax);
    world->addSimpleFrame(terrain);
    DART_ASSERT(world->getNumSimpleFrames() == 1u);
  }

  // Create an instance of our customized WorldNode
  ::osg::ref_ptr<HeightmapWorld> node = new HeightmapWorld(world);
  node->setNumStepsPerCycle(16);

  // Create the Viewer instance
  dart::gui::ImGuiViewer viewer;
  viewer.setImGuiScale(static_cast<float>(guiScale));
  viewer.getImGuiHandler()->setFontScale(static_cast<float>(guiScale));
  viewer.addWorldNode(node);
  viewer.simulate(true);

  // Create grid
  ::osg::ref_ptr<gui::GridVisual> grid = new gui::GridVisual();
  // Sink the grid slightly so the heightmap surface is not z-fighting with it.
  grid->setOffset(Eigen::Vector3d(0.0, 0.0, -0.01));

  // Add control widget for atlas
  if (!alignmentDemo) {
    viewer.getImGuiHandler()->addWidget(
        std::make_shared<HeightmapWidget<float>>(
            &viewer,
            node.get(),
            terrain,
            grid.get(),
            xResolution,
            yResolution,
            xSize,
            ySize,
            zMin,
            zMax));
  }

  viewer.addAttachment(grid);

  // Print out instructions
  std::cout << viewer.getInstructions() << std::endl;

  // Set up the window to be 1280x720 pixels
  viewer.setUpViewInWindow(0, 0, 1280, 720);

  if (alignmentDemo) {
    viewer.getCameraManipulator()->setHomePosition(
        ::osg::Vec3(5.2f, 4.4f, 2.3f),
        ::osg::Vec3(1.5f, 0.0f, 0.3f),
        ::osg::Vec3(-0.2f, -0.2f, 0.95f));
  } else {
    viewer.getCameraManipulator()->setHomePosition(
        ::osg::Vec3(2.57f, 3.14f, 1.64f),
        ::osg::Vec3(0.00f, 0.00f, 0.30f),
        ::osg::Vec3(-0.24f, -0.25f, 0.94f));
  }
  // We need to re-dirty the CameraManipulator by passing it into the viewer
  // again, so that the viewer knows to update its HomePosition setting
  viewer.setCameraManipulator(viewer.getCameraManipulator());

  // Begin the application loop
  viewer.run();
}
