/*
 * Copyright (c) 2011-2019, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include <cmath>

#include <dart/dart.hpp>
#include <dart/external/imgui/imgui.h>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>

using namespace dart;
using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::math;

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

  typename HeightmapShape<S>::HeightField data(yResolution, xResolution);
  for (auto i = 0u; i < yResolution; ++i)
  {
    for (auto j = 0u; j < xResolution; ++j)
    {
      data(i, j) = math::Random::uniform(zMin, zMax);
    }
  }
  auto scale = Vector3(xSize / xResolution, ySize / yResolution, 1);

  auto terrainShape = std::make_shared<HeightmapShape<S>>();
  terrainShape->setScale(scale);
  terrainShape->setHeightField(data);

  return std::move(terrainShape);
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
  auto tf = terrainFrame->getRelativeTransform();
  tf.translation()[0] = -static_cast<double>(xSize) / 2.0;
  tf.translation()[1] = +static_cast<double>(ySize) / 2.0;
  terrainFrame->setRelativeTransform(tf);

  terrainFrame->createVisualAspect();

  // TODO(JS): Remove?
  auto terrainShape = createHeightmapShape(
      xResolution, yResolution, xSize, ySize, zMin, zMax);
  terrainFrame->setShape(terrainShape);

  return terrainFrame;
}

class HeightmapWorld : public gui::osg::WorldNode
{
public:
  explicit HeightmapWorld(simulation::WorldPtr world)
    : gui::osg::WorldNode(std::move(world))
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
class HeightmapWidget : public dart::gui::osg::ImGuiWidget
{
public:
  HeightmapWidget(
      dart::gui::osg::ImGuiViewer* viewer,
      HeightmapWorld* node,
      dynamics::SimpleFramePtr terrain,
      gui::osg::GridVisual* grid,
      std::size_t xResolution = 20u,
      std::size_t yResolution = 20u,
      S xSize = S(2),
      S ySize = S(2),
      S zMin = S(0.0),
      S zMax = S(0.1))
    : mViewer(viewer), mNode(node), mTerrain(std::move(terrain)), mGrid(grid)
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
    mTerrain->setShape(createHeightmapShape(
        mXResolution, mYResolution, mXSize, mYSize, mZMin, mZMax));

    auto tf = mTerrain->getRelativeTransform();
    tf.translation()[0] = -static_cast<double>(mXSize) / 2.0;
    tf.translation()[1] = +static_cast<double>(mYSize) / 2.0;
    mTerrain->setRelativeTransform(tf);
  }

  void render() override
  {
    ImGui::SetNextWindowPos(ImVec2(10, 20));
    ImGui::SetNextWindowSize(ImVec2(360, 600));
    ImGui::SetNextWindowBgAlpha(0.5f);
    if (!ImGui::Begin(
            "Point Cloud & Voxel Grid Demo",
            nullptr,
            ImGuiWindowFlags_MenuBar | ImGuiWindowFlags_HorizontalScrollbar))
    {
      // Early out if the window is collapsed, as an optimization.
      ImGui::End();
      return;
    }

    // Menu
    if (ImGui::BeginMenuBar())
    {
      if (ImGui::BeginMenu("Menu"))
      {
        if (ImGui::MenuItem("Exit"))
          mViewer->setDone(true);
        ImGui::EndMenu();
      }
      if (ImGui::BeginMenu("Help"))
      {
        if (ImGui::MenuItem("About DART"))
          mViewer->showAbout();
        ImGui::EndMenu();
      }
      ImGui::EndMenuBar();
    }

    ImGui::Text("Heightmap rendering example");
    ImGui::Spacing();
    ImGui::TextWrapped("TODO.");

    if (ImGui::CollapsingHeader("Help"))
    {
      ImGui::PushTextWrapPos(ImGui::GetCursorPos().x + 320);
      ImGui::Text("User Guid:\n");
      ImGui::Text("%s", mViewer->getInstructions().c_str());
      ImGui::PopTextWrapPos();
    }

    if (ImGui::CollapsingHeader("Simulation", ImGuiTreeNodeFlags_DefaultOpen))
    {
      int e = mViewer->isSimulating() ? 0 : 1;
      if (mViewer->isAllowingSimulation())
      {
        if (ImGui::RadioButton("Play", &e, 0) && !mViewer->isSimulating())
          mViewer->simulate(true);
        ImGui::SameLine();
        if (ImGui::RadioButton("Pause", &e, 1) && mViewer->isSimulating())
          mViewer->simulate(false);
      }
    }

    if (mTerrain)
    {
      if (ImGui::CollapsingHeader("Heightmap", ImGuiTreeNodeFlags_DefaultOpen))
      {
        ImGui::Text("Terrain");

        auto aspect = mTerrain->getVisualAspect();
        bool display = !aspect->isHidden();
        if (ImGui::Checkbox("Show##Terrain", &display))
        {
          if (display)
            aspect->show();
          else
            aspect->hide();
        }

        auto shape = std::dynamic_pointer_cast<dynamics::HeightmapShapef>(
            mTerrain->getShape());
        assert(shape);

        int xResolution = static_cast<int>(mXResolution);
        if (ImGui::InputInt("X Resolution", &xResolution, 5, 10))
        {
          if (xResolution < 5)
            xResolution = 5;

          if (static_cast<int>(mXResolution) != xResolution)
          {
            mXResolution = xResolution;
            updateHeightmapShape();
          }
        }

        int yResolution = static_cast<int>(mYResolution);
        if (ImGui::InputInt("Y Resolution", &yResolution, 5, 10))
        {
          if (yResolution < 5)
            yResolution = 5;

          if (static_cast<int>(mYResolution) != yResolution)
          {
            mYResolution = yResolution;
            updateHeightmapShape();
          }
        }

        ImGui::Separator();

        if (ImGui::InputFloat("X Size", &mXSize, 0.1, 0.2))
        {
          if (mXSize < 0.1)
            mXSize = 0.1;

          updateHeightmapShape();
        }

        if (ImGui::InputFloat("Y Size", &mYSize, 0.1, 0.2))
        {
          if (mYSize < 0.1)
            mYSize = 0.1;

          updateHeightmapShape();
        }

        ImGui::Separator();

        if (ImGui::InputFloat("Z Min", &mZMin, 0.05, 0.1))
          updateHeightmapShape();

        if (ImGui::InputFloat("Z Max", &mZMax, 0.05, 0.1))
          updateHeightmapShape();

        ImGui::Separator();

        auto visualAspect = mTerrain->getVisualAspect();

        float color[4];
        auto visualColor = visualAspect->getRGBA();
        color[0] = static_cast<float>(visualColor[0]);
        color[1] = static_cast<float>(visualColor[1]);
        color[2] = static_cast<float>(visualColor[2]);
        color[3] = static_cast<float>(visualColor[3]);
        if (ImGui::ColorEdit4("Color##Heightmap", color))
        {
          visualColor[0] = static_cast<double>(color[0]);
          visualColor[1] = static_cast<double>(color[1]);
          visualColor[2] = static_cast<double>(color[2]);
          visualColor[3] = static_cast<double>(color[3]);
          visualAspect->setColor(visualColor);
        }
      }
    }

    if (mGrid)
    {
      if (ImGui::CollapsingHeader("Grid", ImGuiTreeNodeFlags_None))
      {
        ImGui::Text("Grid");

        bool display = mGrid->isDisplayed();
        if (ImGui::Checkbox("Show##Grid", &display))
          mGrid->display(display);

        if (display)
        {
          int e = static_cast<int>(mGrid->getPlaneType());
          if (mViewer->isAllowingSimulation())
          {
            if (ImGui::RadioButton("XY-Plane", &e, 0))
              mGrid->setPlaneType(gui::osg::GridVisual::PlaneType::XY);
            ImGui::SameLine();
            if (ImGui::RadioButton("YZ-Plane", &e, 1))
              mGrid->setPlaneType(gui::osg::GridVisual::PlaneType::YZ);
            ImGui::SameLine();
            if (ImGui::RadioButton("ZX-Plane", &e, 2))
              mGrid->setPlaneType(gui::osg::GridVisual::PlaneType::ZX);
          }

          static Eigen::Vector3f offset;
          ImGui::Columns(3);
          offset = mGrid->getOffset().cast<float>();
          if (ImGui::InputFloat("X", &offset[0], 0.1f, 0.5f, "%.1f"))
            mGrid->setOffset(offset.cast<double>());
          ImGui::NextColumn();
          if (ImGui::InputFloat("Y", &offset[1], 0.1f, 0.5f, "%.1f"))
            mGrid->setOffset(offset.cast<double>());
          ImGui::NextColumn();
          if (ImGui::InputFloat("Z", &offset[2], 0.1f, 0.5f, "%.1f"))
            mGrid->setOffset(offset.cast<double>());
          ImGui::Columns(1);

          static int cellCount;
          cellCount = static_cast<int>(mGrid->getNumCells());
          if (ImGui::InputInt("Line Count", &cellCount, 1, 5))
          {
            if (cellCount < 0)
              cellCount = 0;
            mGrid->setNumCells(static_cast<std::size_t>(cellCount));
          }

          static float cellStepSize;
          cellStepSize = static_cast<float>(mGrid->getMinorLineStepSize());
          if (ImGui::InputFloat("Line Step Size", &cellStepSize, 0.001f, 0.1f))
          {
            mGrid->setMinorLineStepSize(static_cast<double>(cellStepSize));
          }

          static int minorLinesPerMajorLine;
          minorLinesPerMajorLine
              = static_cast<int>(mGrid->getNumMinorLinesPerMajorLine());
          if (ImGui::InputInt(
                  "Minor Lines per Major Line", &minorLinesPerMajorLine, 1, 5))
          {
            if (minorLinesPerMajorLine < 0)
              minorLinesPerMajorLine = 0;
            mGrid->setNumMinorLinesPerMajorLine(
                static_cast<std::size_t>(minorLinesPerMajorLine));
          }

          static float axisLineWidth;
          axisLineWidth = mGrid->getAxisLineWidth();
          if (ImGui::InputFloat(
                  "Axis Line Width", &axisLineWidth, 1.f, 2.f, "%.0f"))
          {
            mGrid->setAxisLineWidth(axisLineWidth);
          }

          static float majorLineWidth;
          majorLineWidth = mGrid->getMajorLineWidth();
          if (ImGui::InputFloat(
                  "Major Line Width", &majorLineWidth, 1.f, 2.f, "%.0f"))
          {
            mGrid->setMajorLineWidth(majorLineWidth);
          }

          static float majorColor[3];
          auto internalmajorColor = mGrid->getMajorLineColor();
          majorColor[0] = static_cast<float>(internalmajorColor.x());
          majorColor[1] = static_cast<float>(internalmajorColor.y());
          majorColor[2] = static_cast<float>(internalmajorColor.z());
          if (ImGui::ColorEdit3("Major Line Color", majorColor))
          {
            internalmajorColor[0] = static_cast<double>(majorColor[0]);
            internalmajorColor[1] = static_cast<double>(majorColor[1]);
            internalmajorColor[2] = static_cast<double>(majorColor[2]);
            mGrid->setMajorLineColor(internalmajorColor);
          }

          static float minorLineWidth;
          minorLineWidth = mGrid->getMinorLineWidth();
          if (ImGui::InputFloat(
                  "Minor Line Width", &minorLineWidth, 1.f, 2.f, "%.0f"))
          {
            mGrid->setMinorLineWidth(minorLineWidth);
          }

          float minorColor[3];
          auto internalMinorColor = mGrid->getMinorLineColor();
          minorColor[0] = static_cast<float>(internalMinorColor.x());
          minorColor[1] = static_cast<float>(internalMinorColor.y());
          minorColor[2] = static_cast<float>(internalMinorColor.z());
          if (ImGui::ColorEdit3("Minor Line Color", minorColor))
          {
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
  dart::gui::osg::ImGuiViewer* mViewer;
  HeightmapWorld* mNode;
  dynamics::SimpleFramePtr mTerrain;
  ::osg::ref_ptr<gui::osg::GridVisual> mGrid;
  std::size_t mXResolution;
  std::size_t mYResolution;
  float mXSize;
  float mYSize;
  float mZMin;
  float mZMax;
};

int main()
{
  auto world = dart::simulation::World::create();
  world->setGravity(Eigen::Vector3d::Zero());

  auto terrain = createHeightmapFrame<float>(100u, 100u, 2.f, 2.f, 0.f, 0.1f);
  world->addSimpleFrame(terrain);

  assert(world->getNumSimpleFrames() == 1u);

  // Create an instance of our customized WorldNode
  ::osg::ref_ptr<HeightmapWorld> node = new HeightmapWorld(world);
  node->setNumStepsPerCycle(16);

  // Create the Viewer instance
  dart::gui::osg::ImGuiViewer viewer;
  viewer.addWorldNode(node);
  viewer.simulate(true);

  // Create grid
  ::osg::ref_ptr<gui::osg::GridVisual> grid = new gui::osg::GridVisual();

  // Add control widget for atlas
  viewer.getImGuiHandler()->addWidget(std::make_shared<HeightmapWidget<float>>(
      &viewer, node.get(), terrain, grid));

  viewer.addAttachment(grid);

  // Print out instructions
  std::cout << viewer.getInstructions() << std::endl;

  // Set up the window to be 1280x720 pixels
  viewer.setUpViewInWindow(0, 0, 1280, 720);

  viewer.getCameraManipulator()->setHomePosition(
      ::osg::Vec3(2.57f, 3.14f, 1.64f),
      ::osg::Vec3(0.00f, 0.00f, 0.30f),
      ::osg::Vec3(-0.24f, -0.25f, 0.94f));
  // We need to re-dirty the CameraManipulator by passing it into the viewer
  // again, so that the viewer knows to update its HomePosition setting
  viewer.setCameraManipulator(viewer.getCameraManipulator());

  // Begin the application loop
  viewer.run();
}
