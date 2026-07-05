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

// Ported from examples/point_cloud: a KR5 arm random-walks its joints while
// an orbiting "sensor" frame samples a point cloud (either on the robot's
// visual meshes or uniformly in a box) that feeds a PointCloudShape and a
// VoxelGridShape occupancy grid.
//
// Compiled only when DART was built with octomap support (HAVE_OCTOMAP, from
// <dart/config.hpp> -- the same macro dart/dynamics/{PointCloud,VoxelGrid}
// Shape.{hpp,cpp} themselves are guarded by), mirroring
// examples/CMakeLists.txt's `if(HAVE_OCTOMAP) add_subdirectory(point_cloud)`
// gate. No examples/demos/CMakeLists.txt change is needed for this: unlike
// the TinyDNN/Python optional deps (which need their own find_package calls
// wired into this project's CMakeLists.txt), HAVE_OCTOMAP is already baked
// into the installed dart/config.hpp by DART's own configure step, so a
// plain #if here is sufficient; when octomap is absent this translation unit
// compiles to nothing and Scenes.hpp/Registry.cpp likewise omit the
// declaration/registration under the same guard.
//
// Deviations from the original: the function-local `static double time`
// sensor-orbit accumulator and the widget's function-local `static` grid
// editor fields are promoted to scene state (the same fix already applied
// to heightmap in B3 for an identical portRisk: they would otherwise persist
// across scene rebuilds/switches instead of resetting).
// generatePointCloudOnRobot's original `while (true)` retries until it finds a
// mesh-shaped visual shape node on a randomly chosen body/shape; kept, but
// bounded by a generous attempt cap that falls back to an in-box sample if ever
// exhausted, so a future robot asset with no visual meshes cannot hang the
// frame loop (ground rule: runtime state must never crash/hang the app). The
// z-height color gradient's `maxZ` starts at
// std::numeric_limits<double>::min() (a tiny positive value, not the most
// negative double) -- a pre-existing quirk that under-clips the color scale
// for all-negative-z clouds -- kept byte-for-byte as in the original. Menu
// bar (Exit/About DART), the Play/Pause radio, and the Help section are
// dropped as redundant with host chrome (Simulation toolbar, window close,
// Diagnostics log), matching every other B2-B4 ImGui-panel port.

#include "Scenes.hpp"

#include <dart/config.hpp>

#if HAVE_OCTOMAP

  #include <dart/gui/osg/osg.hpp>

  #include <dart/utils/urdf/urdf.hpp>
  #include <dart/utils/utils.hpp>

  #include <dart/common/Deprecated.hpp>

  #include <dart/dart.hpp>

  #include <algorithm>
  #include <memory>
  #include <stdexcept>

  #include <cmath>

namespace dart_demos {

namespace {

using dart::dynamics::SkeletonPtr;

//==============================================================================
enum class PointSamplingMode
{
  SampleOnRobot,
  SampleInBox
};

//==============================================================================
/// Per-instance state captured by this scene's preStep/renderPanel lambdas.
/// The sensor-orbit time and the grid editor's fields are scene-owned
/// members (see the file comment) rather than the original's function-local
/// statistics.
struct PointCloudState
{
  SkeletonPtr robot;
  dart::dynamics::SimpleFramePtr sensorFrame;

  std::shared_ptr<dart::dynamics::PointCloudShape> pointCloudShape;
  std::shared_ptr<dart::dynamics::VoxelGridShape> voxelGridShape;
  dart::dynamics::VisualAspect* pointCloudVisualAspect = nullptr;
  dart::dynamics::VisualAspect* voxelGridVisualAspect = nullptr;

  ::osg::ref_ptr<dart::gui::osg::GridVisual> grid;

  PointSamplingMode samplingMode = PointSamplingMode::SampleOnRobot;
  bool update = true;
  double sensorOrbitTime = 0.0;

  // Grid editor fields, promoted from function-local statistics; see the file
  // comment (same treatment as HeightmapScene's gridXxx members).
  bool gridFieldsInitialized = false;
  Eigen::Vector3f gridOffset = Eigen::Vector3f::Zero();
  int gridLineCount = 0;
  float gridLineStepSize = 0.f;
  int gridMinorLinesPerMajor = 0;
  float gridAxisLineWidth = 0.f;
  float gridMajorLineWidth = 0.f;
  float gridMinorLineWidth = 0.f;
  Eigen::Vector4f gridMajorColor = Eigen::Vector4f::Zero();
  Eigen::Vector4f gridMinorColor = Eigen::Vector4f::Zero();
};

//==============================================================================
void ensureGridFieldsInitialized(PointCloudState& state)
{
  if (state.gridFieldsInitialized || !state.grid)
    return;

  state.gridOffset = state.grid->getOffset().cast<float>();
  state.gridLineCount = static_cast<int>(state.grid->getNumCells());
  state.gridLineStepSize
      = static_cast<float>(state.grid->getMinorLineStepSize());
  state.gridMinorLinesPerMajor
      = static_cast<int>(state.grid->getNumMinorLinesPerMajorLine());
  state.gridAxisLineWidth = state.grid->getAxisLineWidth();
  state.gridMajorLineWidth = state.grid->getMajorLineWidth();
  state.gridMinorLineWidth = state.grid->getMinorLineWidth();
  state.gridMajorColor = state.grid->getMajorLineColor().cast<float>();
  state.gridMinorColor = state.grid->getMinorLineColor().cast<float>();
  state.gridFieldsInitialized = true;
}

//==============================================================================
/// Rejection-samples `numPoints` vertices from the robot's visual meshes, as
/// the original did, but bounded by a generous attempt cap (see the file
/// comment) instead of an unconditional `while (true)`: if the robot ever has
/// no mesh-shaped visual shape nodes, this falls back to uniform in-box
/// sampling for the remainder rather than hanging the frame loop.
octomap::Pointcloud generatePointCloudOnRobot(
    const SkeletonPtr& robot, std::size_t numPoints)
{
  octomap::Pointcloud pointCloud;
  pointCloud.reserve(numPoints);

  const auto numBodies = robot->getNumBodyNodes();
  if (numBodies == 0)
    return pointCloud;

  constexpr std::size_t kMaxAttempts = 200000;
  std::size_t attempts = 0;

  while (pointCloud.size() < numPoints && attempts < kMaxAttempts) {
    ++attempts;

    const auto bodyIndex
        = dart::math::Random::uniform<std::size_t>(0, numBodies - 1);
    auto* body = robot->getBodyNode(bodyIndex);
    const auto numShapeNodes
        = body->getNumShapeNodesWith<dart::dynamics::VisualAspect>();
    if (numShapeNodes == 0)
      continue;

    const auto shapeIndex
        = dart::math::Random::uniform<std::size_t>(0, numShapeNodes - 1);
    auto* shapeNode
        = body->getShapeNodeWith<dart::dynamics::VisualAspect>(shapeIndex);
    auto shape = shapeNode->getShape();
    if (!shape || !shape->is<dart::dynamics::MeshShape>())
      continue;
    auto mesh = std::static_pointer_cast<dart::dynamics::MeshShape>(shape);

    DART_SUPPRESS_DEPRECATED_BEGIN
    auto* assimpScene = mesh->getMesh();
    DART_SUPPRESS_DEPRECATED_END
    if (!assimpScene || assimpScene->mNumMeshes < 1)
      continue;

    const auto meshIndex = dart::math::Random::uniform<unsigned int>(
        0, assimpScene->mNumMeshes - 1);
    auto* assimpMesh = assimpScene->mMeshes[meshIndex];
    const auto numVertices = assimpMesh->mNumVertices;
    if (numVertices == 0)
      continue;

    const auto vertexIndex
        = dart::math::Random::uniform<unsigned int>(0, numVertices - 1);
    const auto& vertex = assimpMesh->mVertices[vertexIndex];

    const Eigen::Isometry3d tf = shapeNode->getWorldTransform();
    Eigen::Vector3d eigenVertex
        = tf * Eigen::Vector3f(vertex.x, vertex.y, vertex.z).cast<double>();

    pointCloud.push_back(
        static_cast<float>(eigenVertex.x()),
        static_cast<float>(eigenVertex.y()),
        static_cast<float>(eigenVertex.z()));
  }

  return pointCloud;
}

//==============================================================================
octomap::Pointcloud generatePointCloudInBox(
    std::size_t numPoints,
    const Eigen::Vector3d& min,
    const Eigen::Vector3d& max)
{
  octomap::Pointcloud pointCloud;
  pointCloud.reserve(numPoints);

  for (std::size_t i = 0; i < numPoints; ++i) {
    const Eigen::Vector3d point = dart::math::Random::uniform(min, max);
    pointCloud.push_back(
        static_cast<float>(point.x()),
        static_cast<float>(point.y()),
        static_cast<float>(point.z()));
  }

  return pointCloud;
}

//==============================================================================
/// Z-height red-blue gradient, alpha 0.75, clipped to [0.1, 0.9]. `maxZ`'s
/// std::numeric_limits<double>::min() start (a tiny positive value rather
/// than the most negative double) is a pre-existing quirk kept byte-for-byte
/// from the original -- see the file comment.
std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>
generatePointCloudColors(
    const dart::dynamics::PointCloudShape& pointCloudShape,
    const octomap::Pointcloud& pointCloud)
{
  const auto& points = pointCloudShape.getPoints();
  double minZ = std::numeric_limits<double>::max();
  double maxZ = std::numeric_limits<double>::min();
  for (const auto& point : points) {
    minZ = std::min(minZ, point.z());
    maxZ = std::max(maxZ, point.z());
  }
  const double diffZ
      = std::max(std::abs(maxZ - minZ), std::numeric_limits<double>::min());

  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>
      colors;
  colors.reserve(pointCloud.size());
  for (const auto& point : pointCloud) {
    float r
        = (point.z() - static_cast<float>(minZ)) / static_cast<float>(diffZ);
    float g = 0.0f;
    float b = 1.f - r;
    r = dart::math::clip(r, 0.1f, 0.9f);
    g = dart::math::clip(g, 0.1f, 0.9f);
    b = dart::math::clip(b, 0.1f, 0.9f);
    colors.emplace_back(Eigen::Vector4f(r, g, b, 0.75f).cast<double>());
  }

  return colors;
}

//==============================================================================
SkeletonPtr createRobot()
{
  dart::utils::DartLoader loader;
  auto robot
      = loader.parseSkeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf");
  if (!robot)
    return nullptr;

  // Rotate so that Z is upwards (the default URDF transform is not Identity).
  robot->getJoint(0)->setTransformFromParentBodyNode(
      Eigen::Isometry3d::Identity());
  robot->setName("KR5");
  return robot;
}

//==============================================================================
SkeletonPtr createGround()
{
  dart::utils::DartLoader loader;
  auto ground = loader.parseSkeleton("dart://sample/urdf/KR5/ground.urdf");
  if (!ground)
    return nullptr;

  Eigen::Isometry3d tf = ground->getJoint(0)->getTransformFromParentBodyNode();
  tf.pretranslate(Eigen::Vector3d(0, 0, 0.5));
  tf.rotate(Eigen::AngleAxisd(
      dart::math::constantsd::pi() / 2, Eigen::Vector3d(1, 0, 0)));
  ground->getJoint(0)->setTransformFromParentBodyNode(tf);
  return ground;
}

} // namespace

//==============================================================================
DemoScene makePointCloudScene()
{
  DemoScene scene;
  scene.id = "point_cloud";
  scene.title = "Point Cloud";
  scene.category = "Visualization";
  scene.summary
      = "Animated point-cloud and voxel-grid sensor rendering on a robot.";

  scene.factory = [] {
    auto world = dart::simulation::World::create();
    world->setGravity(Eigen::Vector3d::Zero());

    auto robot = createRobot();
    if (!robot)
      throw std::runtime_error(
          "failed to load dart://sample/urdf/KR5/KR5 sixx R650.urdf");
    world->addSkeleton(robot);

    auto ground = createGround();
    if (!ground)
      throw std::runtime_error(
          "failed to load dart://sample/urdf/KR5/ground.urdf");
    world->addSkeleton(ground);

    auto state = std::make_shared<PointCloudState>();
    state->robot = robot;

    auto pointCloudShape = std::make_shared<dart::dynamics::PointCloudShape>();
    pointCloudShape->setPointShapeType(dart::dynamics::PointCloudShape::BOX);
    auto pointCloudFrame = dart::dynamics::SimpleFrame::createShared(
        dart::dynamics::Frame::World());
    pointCloudFrame->setName("point cloud");
    pointCloudFrame->setShape(pointCloudShape);
    auto* pointCloudVisual = pointCloudFrame->createVisualAspect();
    pointCloudVisual->setRGB(dart::Color::Blue());
    world->addSimpleFrame(pointCloudFrame);
    pointCloudShape->setColorMode(
        dart::dynamics::PointCloudShape::BIND_PER_POINT);

    auto voxelGridShape
        = std::make_shared<dart::dynamics::VoxelGridShape>(0.05);
    auto voxelFrame = dart::dynamics::SimpleFrame::createShared(
        dart::dynamics::Frame::World());
    voxelFrame->setName("voxel");
    voxelFrame->setShape(voxelGridShape);
    auto* voxelVisual = voxelFrame->createVisualAspect();
    voxelVisual->setRGBA(dart::Color::Orange(0.5));
    voxelVisual->show();
    world->addSimpleFrame(voxelFrame);

    auto sensorShape = std::make_shared<dart::dynamics::SphereShape>(0.05);
    auto sensorFrame = dart::dynamics::SimpleFrame::createShared(
        dart::dynamics::Frame::World());
    sensorFrame->setName("sensor");
    sensorFrame->setShape(sensorShape);
    sensorFrame->createVisualAspect()->setRGB(dart::Color::Red());
    world->addSimpleFrame(sensorFrame);

    state->pointCloudShape = pointCloudShape;
    state->voxelGridShape = voxelGridShape;
    state->pointCloudVisualAspect = pointCloudVisual;
    state->voxelGridVisualAspect = voxelVisual;
    state->sensorFrame = sensorFrame;
    state->grid = new dart::gui::osg::GridVisual();

    DemoSceneSetup setup;
    setup.world = world;
    setup.cameraHome = CameraHome{
        ::osg::Vec3d(2.57, 3.14, 1.64),
        ::osg::Vec3d(0.00, 0.00, 0.30),
        ::osg::Vec3d(-0.24, -0.25, 0.94)};

    setup.preStep = [state] {
      if (!state->update)
        return;

      Eigen::VectorXd pos = state->robot->getPositions();
      pos += 0.01 * Eigen::VectorXd::Random(pos.size());
      state->robot->setPositions(pos);

      constexpr std::size_t kNumPoints = 500;
      octomap::Pointcloud pointCloud;
      switch (state->samplingMode) {
        case PointSamplingMode::SampleOnRobot:
          pointCloud = generatePointCloudOnRobot(state->robot, kNumPoints);
          break;
        case PointSamplingMode::SampleInBox:
          pointCloud = generatePointCloudInBox(
              kNumPoints,
              Eigen::Vector3d::Constant(-0.5),
              Eigen::Vector3d::Constant(0.5));
          break;
      }

      constexpr double dt = 0.001;
      constexpr double radius = 1.0;
      const Eigen::Vector3d center(0.0, 0.1, 0.0);
      Eigen::Vector3d sensorPos = center;
      sensorPos[0] = radius * std::sin(state->sensorOrbitTime);
      sensorPos[1] = radius * std::cos(state->sensorOrbitTime);
      sensorPos[2] = 0.5 + 0.25 * std::sin(state->sensorOrbitTime * 2.0);
      state->sensorOrbitTime += dt;
      state->sensorFrame->setTranslation(sensorPos);

      state->pointCloudShape->setPoints(pointCloud);
      state->pointCloudShape->setColors(
          generatePointCloudColors(*state->pointCloudShape, pointCloud));
      state->voxelGridShape->updateOccupancy(pointCloud, sensorPos);
    };

    setup.onActivate = [state](DemoHostContext& ctx) {
      ctx.addAttachment(state->grid.get());
    };

    setup.renderPanel = [state] {
      ensureGridFieldsInitialized(*state);

      ImGui::TextWrapped(
          "The robot moves by random joint velocities. Blue boxes are the "
          "point cloud; orange boxes are the voxel grid. The red sphere is "
          "the orbiting sensor origin.");

      if (ImGui::CollapsingHeader(
              "Simulation", ImGuiTreeNodeFlags_DefaultOpen)) {
        bool updating = state->update;
        if (ImGui::Checkbox("Run Robot Updating", &updating))
          state->update = updating;

        int samplingMode
            = state->samplingMode == PointSamplingMode::SampleOnRobot ? 0 : 1;
        if (ImGui::RadioButton("Sample on robot", &samplingMode, 0))
          state->samplingMode = PointSamplingMode::SampleOnRobot;
        ImGui::SameLine();
        if (ImGui::RadioButton("Sample in box", &samplingMode, 1))
          state->samplingMode = PointSamplingMode::SampleInBox;
      }

      if (ImGui::CollapsingHeader("View", ImGuiTreeNodeFlags_DefaultOpen)) {
        bool pcShow = !state->pointCloudVisualAspect->isHidden();
        if (ImGui::Checkbox("Point Cloud", &pcShow)) {
          if (pcShow)
            state->pointCloudVisualAspect->show();
          else
            state->pointCloudVisualAspect->hide();
        }

        if (pcShow) {
          auto& pointCloudShape = *state->pointCloudShape;

          const char* colorModeItems[]
              = {"Use shape color", "Bind overall", "Bind per point"};
          int colorMode = pointCloudShape.getColorMode();
          ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.6f);
          if (ImGui::Combo(
                  "Color Mode",
                  &colorMode,
                  colorModeItems,
                  IM_ARRAYSIZE(colorModeItems))) {
            pointCloudShape.setColorMode(
                static_cast<dart::dynamics::PointCloudShape::ColorMode>(
                    colorMode));
          }
          if (colorMode == dart::dynamics::PointCloudShape::USE_SHAPE_COLOR) {
            Eigen::Vector4d rgba = state->pointCloudVisualAspect->getRGBA();
            float color[4]
                = {static_cast<float>(rgba[0]),
                   static_cast<float>(rgba[1]),
                   static_cast<float>(rgba[2]),
                   static_cast<float>(rgba[3])};
            if (ImGui::ColorEdit4("Color##PointCloud", color)) {
              state->pointCloudVisualAspect->setColor(
                  Eigen::Vector4d(color[0], color[1], color[2], color[3]));
            }
          }

          const char* shapeTypeItems[]
              = {"Box", "Billboard Square", "Billboard Circle", "Point"};
          int shapeType = pointCloudShape.getPointShapeType();
          ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.6f);
          if (ImGui::Combo(
                  "Point Shape Type",
                  &shapeType,
                  shapeTypeItems,
                  IM_ARRAYSIZE(shapeTypeItems))) {
            pointCloudShape.setPointShapeType(
                static_cast<dart::dynamics::PointCloudShape::PointShapeType>(
                    shapeType));
          }

          float visualSize
              = static_cast<float>(pointCloudShape.getVisualSize());
          ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.6f);
          if (ImGui::InputFloat(
                  "Visual Size", &visualSize, 0.01f, 0.02f, "%.2f")
              && std::isfinite(visualSize)) {
            visualSize = std::max(0.01f, visualSize);
            pointCloudShape.setVisualSize(static_cast<double>(visualSize));
          }
        }

        ImGui::Separator();

        bool vgShow = !state->voxelGridVisualAspect->isHidden();
        if (ImGui::Checkbox("Voxel Grid", &vgShow)) {
          if (vgShow)
            state->voxelGridVisualAspect->show();
          else
            state->voxelGridVisualAspect->hide();
        }
        if (vgShow) {
          Eigen::Vector4d rgba = state->voxelGridVisualAspect->getRGBA();
          float color[4]
              = {static_cast<float>(rgba[0]),
                 static_cast<float>(rgba[1]),
                 static_cast<float>(rgba[2]),
                 static_cast<float>(rgba[3])};
          if (ImGui::ColorEdit4("Voxel grid color", color)) {
            state->voxelGridVisualAspect->setColor(
                Eigen::Vector4d(color[0], color[1], color[2], color[3]));
          }
        }
      }

      if (ImGui::CollapsingHeader("Grid")) {
        bool display = state->grid->isDisplayed();
        if (ImGui::Checkbox("Show##Grid", &display))
          state->grid->display(display);

        if (display) {
          int plane = static_cast<int>(state->grid->getPlaneType());
          if (ImGui::RadioButton("XY-Plane", &plane, 0))
            state->grid->setPlaneType(
                dart::gui::osg::GridVisual::PlaneType::XY);
          ImGui::SameLine();
          if (ImGui::RadioButton("YZ-Plane", &plane, 1))
            state->grid->setPlaneType(
                dart::gui::osg::GridVisual::PlaneType::YZ);
          ImGui::SameLine();
          if (ImGui::RadioButton("ZX-Plane", &plane, 2))
            state->grid->setPlaneType(
                dart::gui::osg::GridVisual::PlaneType::ZX);

          const char* const kOffsetLabels[3] = {"X", "Y", "Z"};
          ImGui::Columns(3);
          for (int c = 0; c < 3; ++c) {
            float off = state->gridOffset[c];
            if (ImGui::InputFloat(kOffsetLabels[c], &off, 0.1f, 0.5f, "%.1f")
                && std::isfinite(off) && off != state->gridOffset[c]) {
              state->gridOffset[c] = off;
              state->grid->setOffset(state->gridOffset.cast<double>());
            }
            if (c < 2)
              ImGui::NextColumn();
          }
          ImGui::Columns(1);

          int lineCount = state->gridLineCount;
          if (ImGui::InputInt("Line Count", &lineCount, 1, 5)) {
            lineCount = std::max(0, lineCount);
            if (lineCount != state->gridLineCount) {
              state->gridLineCount = lineCount;
              state->grid->setNumCells(
                  static_cast<std::size_t>(state->gridLineCount));
            }
          }

          float lineStep = state->gridLineStepSize;
          if (ImGui::InputFloat("Line Step Size", &lineStep, 0.001f, 0.1f)
              && std::isfinite(lineStep)
              && lineStep != state->gridLineStepSize) {
            state->gridLineStepSize = lineStep;
            state->grid->setMinorLineStepSize(
                static_cast<double>(state->gridLineStepSize));
          }

          int minorPerMajor = state->gridMinorLinesPerMajor;
          if (ImGui::InputInt(
                  "Minor Lines per Major Line", &minorPerMajor, 1, 5)) {
            minorPerMajor = std::max(0, minorPerMajor);
            if (minorPerMajor != state->gridMinorLinesPerMajor) {
              state->gridMinorLinesPerMajor = minorPerMajor;
              state->grid->setNumMinorLinesPerMajorLine(
                  static_cast<std::size_t>(state->gridMinorLinesPerMajor));
            }
          }

          float axisWidth = state->gridAxisLineWidth;
          if (ImGui::InputFloat("Axis Line Width", &axisWidth, 1.f, 2.f, "%.0f")
              && std::isfinite(axisWidth)) {
            axisWidth = std::max(0.f, axisWidth);
            if (axisWidth != state->gridAxisLineWidth) {
              state->gridAxisLineWidth = axisWidth;
              state->grid->setAxisLineWidth(state->gridAxisLineWidth);
            }
          }

          float majorWidth = state->gridMajorLineWidth;
          if (ImGui::InputFloat(
                  "Major Line Width", &majorWidth, 1.f, 2.f, "%.0f")
              && std::isfinite(majorWidth)) {
            majorWidth = std::max(0.f, majorWidth);
            if (majorWidth != state->gridMajorLineWidth) {
              state->gridMajorLineWidth = majorWidth;
              state->grid->setMajorLineWidth(state->gridMajorLineWidth);
            }
          }

          if (ImGui::ColorEdit3(
                  "Major Line Color", state->gridMajorColor.data()))
            state->grid->setMajorLineColor(
                state->gridMajorColor.cast<double>());

          float minorWidth = state->gridMinorLineWidth;
          if (ImGui::InputFloat(
                  "Minor Line Width", &minorWidth, 1.f, 2.f, "%.0f")
              && std::isfinite(minorWidth)) {
            minorWidth = std::max(0.f, minorWidth);
            if (minorWidth != state->gridMinorLineWidth) {
              state->gridMinorLineWidth = minorWidth;
              state->grid->setMinorLineWidth(state->gridMinorLineWidth);
            }
          }

          if (ImGui::ColorEdit3(
                  "Minor Line Color", state->gridMinorColor.data()))
            state->grid->setMinorLineColor(
                state->gridMinorColor.cast<double>());
        }
      }
    };

    return setup;
  };

  return scene;
}

} // namespace dart_demos

#endif // HAVE_OCTOMAP
