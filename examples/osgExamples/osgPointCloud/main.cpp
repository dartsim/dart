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

static const std::string& robotName = "KR5";

class PointCloudWorld : public gui::osg::WorldNode
{
public:
  explicit PointCloudWorld(
      simulation::WorldPtr world, dynamics::SkeletonPtr robot)
    : gui::osg::WorldNode(std::move(world)), mRobot(std::move(robot))
  {
    auto pointCloudFrame = mWorld->getSimpleFrame("point cloud");
    auto voxelGridFrame = mWorld->getSimpleFrame("voxel");

    mPointCloudShape = std::dynamic_pointer_cast<dynamics::PointCloudShape>(
        pointCloudFrame->getShape());
    mVoxelGridShape = std::dynamic_pointer_cast<dynamics::VoxelGridShape>(
        voxelGridFrame->getShape());

    mPointCloudVisualAspect = pointCloudFrame->getVisualAspect();
    mVoxelGridVisualAspect = voxelGridFrame->getVisualAspect();

    assert(mVoxelGridShape);
  }

  // Triggered at the beginning of each simulation step
  void customPreStep() override
  {
    if (!mRobot)
      return;

    if (!mUpdate)
      return;

    // Set robot pose
    Eigen::VectorXd pos = mRobot->getPositions();
    pos += 0.01 * Eigen::VectorXd::Random(pos.size());
    mRobot->setPositions(pos);

    // Generate point cloud from robot meshes
    auto pointCloud = generatePointCloud(500);

    // Update sensor position
    static double time = 0.0;
    const double dt = 0.001;
    const double radius = 1.0;
    Eigen::Vector3d center = Eigen::Vector3d(0.0, 0.1, 0.0);
    Eigen::Vector3d sensorPos = center;
    sensorPos[0] = radius * std::sin(time);
    sensorPos[1] = radius * std::cos(time);
    sensorPos[2] = 0.5 + 0.25 * std::sin(time * 2.0);
    time += dt;
    auto sensorFrame = mWorld->getSimpleFrame("sensor");
    assert(sensorFrame);
    sensorFrame->setTranslation(sensorPos);

    // Update point cloud
    mPointCloudShape->setPoints(pointCloud);

    // Update voxel
    mVoxelGridShape->updateOccupancy(pointCloud, sensorPos);
  }

  dynamics::VisualAspect* getPointCloudVisualAspect()
  {
    return mPointCloudVisualAspect;
  }

  dynamics::VisualAspect* getVoxelGridVisualAspect()
  {
    return mVoxelGridVisualAspect;
  }

  void setUpdate(bool update)
  {
    mUpdate = update;
  }

  bool getUpdate() const
  {
    return mUpdate;
  }

protected:
  octomap::Pointcloud generatePointCloud(std::size_t numPoints)
  {
    octomap::Pointcloud pointCloud;

    const auto numBodies = mRobot->getNumBodyNodes();
    assert(numBodies > 0);
    while (true)
    {
      const auto bodyIndex
          = math::Random::uniform<std::size_t>(0, numBodies - 1);
      auto body = mRobot->getBodyNode(bodyIndex);
      auto shapeNodes = body->getShapeNodesWith<dynamics::VisualAspect>();
      if (shapeNodes.empty())
        continue;

      const auto shapeIndex
          = math::Random::uniform<std::size_t>(0, shapeNodes.size() - 1);
      auto shapeNode = shapeNodes[shapeIndex];
      auto shape = shapeNode->getShape();
      assert(shape);

      if (!shape->is<dynamics::MeshShape>())
        continue;
      auto mesh = std::static_pointer_cast<dynamics::MeshShape>(shape);

      auto assimpScene = mesh->getMesh();
      assert(assimpScene);

      if (assimpScene->mNumMeshes < 1)
        continue;
      const auto meshIndex
          = math::Random::uniform<std::size_t>(0, assimpScene->mNumMeshes - 1);

      auto assimpMesh = assimpScene->mMeshes[meshIndex];
      auto numVertices = assimpMesh->mNumVertices;

      auto vertexIndex
          = math::Random::uniform<unsigned int>(0, numVertices - 1);
      auto vertex = assimpMesh->mVertices[vertexIndex];

      Eigen::Isometry3d tf = shapeNode->getWorldTransform();
      Eigen::Vector3d eigenVertex
          = Eigen::Vector3f(vertex.x, vertex.y, vertex.z).cast<double>();
      eigenVertex = tf * eigenVertex;

      pointCloud.push_back(
          static_cast<float>(eigenVertex.x()),
          static_cast<float>(eigenVertex.y()),
          static_cast<float>(eigenVertex.z()));

      if (pointCloud.size() == numPoints)
        return pointCloud;
    }
  }

  SkeletonPtr mRobot;

  std::shared_ptr<dynamics::PointCloudShape> mPointCloudShape;
  std::shared_ptr<dynamics::VoxelGridShape> mVoxelGridShape;

  dynamics::VisualAspect* mPointCloudVisualAspect;
  dynamics::VisualAspect* mVoxelGridVisualAspect;

  bool mUpdate{true};
};

class PointCloudWidget : public dart::gui::osg::ImGuiWidget
{
public:
  PointCloudWidget(
      dart::gui::osg::ImGuiViewer* viewer,
      PointCloudWorld* node,
      gui::osg::GridVisual* grid)
    : mViewer(viewer), mNode(node), mGrid(grid)
  {
    // Do nothing
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

    ImGui::Text("Point cloud and voxel grid rendering example");
    ImGui::Spacing();
    ImGui::TextWrapped(
        "The robot is moving by random joint velocities. The small blue boxes "
        "and orange boxes represent point cloud and voxel grid, respectively. "
        "The moving red sphere represents the sensor origin that generates the "
        "point cloud.");

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

      int robotUpdate = mNode->getUpdate() ? 0 : 1;
      if (ImGui::RadioButton("Run Robot Updating", &robotUpdate, 0)
          && mNode->getUpdate())
        mNode->setUpdate(true);
      ImGui::SameLine();
      if (ImGui::RadioButton("Stop Robot Updating", &robotUpdate, 1)
          && mNode->getUpdate())
        mNode->setUpdate(false);
    }

    if (ImGui::CollapsingHeader("View", ImGuiTreeNodeFlags_DefaultOpen))
    {
      if (mViewer->isAllowingSimulation())
      {
        bool pcShow = !mNode->getPointCloudVisualAspect()->isHidden();
        if (ImGui::Checkbox("Point Cloud", &pcShow))
        {
          if (pcShow)
            mNode->getPointCloudVisualAspect()->show();
          else
            mNode->getPointCloudVisualAspect()->hide();
        }

        bool vgShow = !mNode->getVoxelGridVisualAspect()->isHidden();
        if (ImGui::Checkbox("Voxel Grid", &vgShow))
        {
          if (vgShow)
            mNode->getVoxelGridVisualAspect()->show();
          else
            mNode->getVoxelGridVisualAspect()->hide();
        }
      }

      if (ImGui::CollapsingHeader("Grid", ImGuiTreeNodeFlags_None))
      {
        assert(mGrid);
        ImGui::Text("Grid");

        bool display = mGrid->isDisplayed();
        if (ImGui::Checkbox("Show", &display))
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
  osg::ref_ptr<dart::gui::osg::ImGuiViewer> mViewer;
  PointCloudWorld* mNode;
  ::osg::ref_ptr<gui::osg::GridVisual> mGrid;
};

dynamics::SkeletonPtr createRobot(const std::string& name)
{
  auto urdfParser = dart::utils::DartLoader();

  // Load the robot
  auto robot
      = urdfParser.parseSkeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf");

  // Rotate the robot so that z is upwards (default transform is not Identity)
  robot->getJoint(0)->setTransformFromParentBodyNode(
      Eigen::Isometry3d::Identity());

  robot->setName(name);

  return robot;
}

dynamics::SkeletonPtr createGround()
{
  auto urdfParser = dart::utils::DartLoader();

  auto ground = urdfParser.parseSkeleton("dart://sample/urdf/KR5/ground.urdf");

  // Rotate and move the ground so that z is upwards
  Eigen::Isometry3d ground_tf
      = ground->getJoint(0)->getTransformFromParentBodyNode();
  ground_tf.pretranslate(Eigen::Vector3d(0, 0, 0.5));
  ground_tf.rotate(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(1, 0, 0)));
  ground->getJoint(0)->setTransformFromParentBodyNode(ground_tf);

  return ground;
}

dynamics::SimpleFramePtr createVoxelFrame(double resolution = 0.01)
{
  auto voxelShape
      = ::std::make_shared<dart::dynamics::VoxelGridShape>(resolution);
  auto voxelFrame = ::dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World());
  voxelFrame->setName("voxel");
  voxelFrame->setShape(voxelShape);
  auto visualAspect = voxelFrame->createVisualAspect();
  visualAspect->setRGBA(Color::Orange(0.5));

  return voxelFrame;
}

dynamics::SimpleFramePtr createPointCloudFrame()
{
  auto pointCloudShape
      = ::std::make_shared<::dart::dynamics::PointCloudShape>();
  auto pointCloudFrame = ::dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World());
  pointCloudFrame->setName("point cloud");
  pointCloudFrame->setShape(pointCloudShape);
  auto visualAspect = pointCloudFrame->createVisualAspect();
  visualAspect->setRGB(Color::Blue());

  return pointCloudFrame;
}

dynamics::SimpleFramePtr createSensorFrame()
{
  auto sphereShape = ::std::make_shared<dart::dynamics::SphereShape>(0.05);
  auto sensorFrame = ::dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World());
  sensorFrame->setName("sensor");
  sensorFrame->setShape(sphereShape);
  auto visualAspect = sensorFrame->createVisualAspect();
  visualAspect->setRGB(Color::Red());

  return sensorFrame;
}

int main()
{
  auto world = dart::simulation::World::create();
  world->setGravity(Eigen::Vector3d::Zero());

  auto robot = createRobot(robotName);
  world->addSkeleton(robot);

  auto ground = createGround();
  world->addSkeleton(ground);

  auto pointCloud = createPointCloudFrame();
  world->addSimpleFrame(pointCloud);

  auto voxel = createVoxelFrame(0.05);
  world->addSimpleFrame(voxel);

  auto sensor = createSensorFrame();
  world->addSimpleFrame(sensor);

  // Create an instance of our customized WorldNode
  ::osg::ref_ptr<PointCloudWorld> node = new PointCloudWorld(world, robot);
  node->setNumStepsPerCycle(16);

  // Create the Viewer instance
  osg::ref_ptr<dart::gui::osg::ImGuiViewer> viewer
      = new dart::gui::osg::ImGuiViewer();
  viewer->addWorldNode(node);
  viewer->simulate(true);

  // Create grid
  ::osg::ref_ptr<gui::osg::GridVisual> grid = new gui::osg::GridVisual();

  // Add control widget for atlas
  viewer->getImGuiHandler()->addWidget(
      std::make_shared<PointCloudWidget>(viewer, node.get(), grid));

  viewer->addAttachment(grid);

  // Print out instructions
  std::cout << viewer->getInstructions() << std::endl;

  // Set up the window to be 1280x720 pixels
  viewer->setUpViewInWindow(0, 0, 1280, 720);

  viewer->getCameraManipulator()->setHomePosition(
      ::osg::Vec3(2.57f, 3.14f, 1.64f),
      ::osg::Vec3(0.00f, 0.00f, 0.30f),
      ::osg::Vec3(-0.24f, -0.25f, 0.94f));
  // We need to re-dirty the CameraManipulator by passing it into the viewer
  // again, so that the viewer knows to update its HomePosition setting
  viewer->setCameraManipulator(viewer->getCameraManipulator());

  // Begin the application loop
  viewer->run();
}
