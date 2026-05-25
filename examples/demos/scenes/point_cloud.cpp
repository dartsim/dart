/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the "BSD-style" License.
 */

#include "scenes.hpp"

#include "../../gui_source_grid.hpp"

#include <dart/gui/panel.hpp>

#include <dart/simulation/world.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/joint.hpp>
#include <dart/dynamics/line_segment_shape.hpp>
#include <dart/dynamics/mesh_shape.hpp>
#include <dart/dynamics/point_cloud_shape.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/simple_frame.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/sphere_shape.hpp>
#include <dart/dynamics/voxel_grid_shape.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <dart/math/constants.hpp>

#include <dart/io/read.hpp>

#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <algorithm>
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include <cmath>
#include <cstddef>

namespace dart::examples::demos {

namespace {

constexpr const char* kRobotName = "KR5";
constexpr const char* kPointCloudFrameName = "visual_point_cloud";
constexpr const char* kVoxelGridFrameName = "visual_voxel_grid";
constexpr const char* kSensorFrameName = "point_cloud_sensor";
constexpr const char* kGridName = "point_cloud_grid";
constexpr std::size_t kPointCount = 500;

using Vector3List
    = std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>;
using Vector4List
    = std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>;

enum class PointSamplingMode
{
  SampleOnRobot,
  SampleInBox,
};

struct PointCloudState
{
  dart::simulation::WorldPtr world;
  dart::dynamics::SkeletonPtr robot;
  Eigen::VectorXd initialRobotPositions;
  std::shared_ptr<dart::dynamics::SimpleFrame> pointCloudFrame;
  std::shared_ptr<dart::dynamics::PointCloudShape> pointCloudShape;
  dart::dynamics::VisualAspect* pointCloudVisual = nullptr;
  std::shared_ptr<dart::dynamics::SimpleFrame> sensorFrame;
  dart::dynamics::VisualAspect* sensorVisual = nullptr;
  dart::examples::SourceOwnedGridState grid;
  std::shared_ptr<dart::dynamics::SimpleFrame> voxelGridFrame;
  std::shared_ptr<dart::dynamics::VoxelGridShape> voxelGridShape;
  dart::dynamics::VisualAspect* voxelGridVisual = nullptr;
  PointSamplingMode samplingMode = PointSamplingMode::SampleOnRobot;
  dart::dynamics::PointCloudShape::ColorMode colorMode
      = dart::dynamics::PointCloudShape::BIND_OVERALL;
  dart::dynamics::PointCloudShape::PointShapeType pointShapeType
      = dart::dynamics::PointCloudShape::BOX;
  bool updateRobot = true;
  bool pointCloudVisible = true;
  bool sensorVisible = true;
  bool voxelGridVisible = true;
  double visualSize = 0.035;
  Eigen::Vector4d pointCloudColor = Eigen::Vector4d(0.20, 0.38, 0.94, 1.0);
  Eigen::Vector4d voxelGridColor = Eigen::Vector4d(0.94, 0.52, 0.20, 0.22);
  double elapsedTime = 0.0;
  std::size_t lastPointCount = 0u;
  std::size_t lastOccupiedVoxelCount = 0u;
};

void applyVisibility(dart::dynamics::VisualAspect* aspect, bool visible)
{
  if (aspect == nullptr) {
    return;
  }

  if (visible) {
    aspect->show();
  } else {
    aspect->hide();
  }
}

void applyPointCloudColor(PointCloudState& state)
{
  if (state.pointCloudVisual != nullptr) {
    state.pointCloudVisual->setRGBA(state.pointCloudColor);
  }
  if (state.pointCloudShape != nullptr
      && state.colorMode == dart::dynamics::PointCloudShape::BIND_OVERALL) {
    state.pointCloudShape->setOverallColor(state.pointCloudColor);
  }
}

void applyVoxelGridColor(PointCloudState& state)
{
  if (state.voxelGridVisual != nullptr) {
    state.voxelGridVisual->setRGBA(state.voxelGridColor);
  }
}

dart::dynamics::SkeletonPtr createFallbackRobot()
{
  auto robot = dart::dynamics::Skeleton::create(kRobotName);
  auto pair = robot->createJointAndBodyNodePair<dart::dynamics::WeldJoint>();
  auto* body = pair.second;
  body->setName("fallback_body");

  const auto shape = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(0.42, 0.24, 0.72));
  auto* shapeNode
      = body->createShapeNodeWith<dart::dynamics::VisualAspect>(shape);
  shapeNode->getVisualAspect()->setRGBA(Eigen::Vector4d(0.36, 0.44, 0.52, 1.0));
  return robot;
}

dart::dynamics::SkeletonPtr createRobot()
{
  auto robot
      = dart::io::readSkeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf");
  if (robot == nullptr) {
    return createFallbackRobot();
  }

  if (robot->getNumJoints() > 0 && robot->getJoint(0) != nullptr) {
    robot->getJoint(0)->setTransformFromParentBodyNode(
        Eigen::Isometry3d::Identity());
  }
  robot->setName(kRobotName);
  return robot;
}

dart::dynamics::SkeletonPtr createFallbackGround()
{
  auto ground = dart::dynamics::Skeleton::create("point_cloud_ground");
  auto pair = ground->createJointAndBodyNodePair<dart::dynamics::WeldJoint>();
  auto* joint = pair.first;
  auto* body = pair.second;

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(0.0, 0.0, -0.035);
  joint->setTransformFromParentBodyNode(transform);

  const auto shape = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(4.0, 4.0, 0.04));
  auto* shapeNode
      = body->createShapeNodeWith<dart::dynamics::VisualAspect>(shape);
  shapeNode->getVisualAspect()->setRGBA(
      Eigen::Vector4d(0.42, 0.44, 0.40, 0.78));
  return ground;
}

dart::dynamics::SkeletonPtr createGround()
{
  auto ground = dart::io::readSkeleton("dart://sample/urdf/KR5/ground.urdf");
  if (ground == nullptr || ground->getNumJoints() == 0
      || ground->getJoint(0) == nullptr) {
    return createFallbackGround();
  }

  Eigen::Isometry3d transform
      = ground->getJoint(0)->getTransformFromParentBodyNode();
  transform.pretranslate(Eigen::Vector3d(0.0, 0.0, 0.5));
  transform.rotate(
      Eigen::AngleAxisd(dart::math::pi / 2.0, Eigen::Vector3d::UnitX()));
  ground->getJoint(0)->setTransformFromParentBodyNode(transform);
  return ground;
}

std::shared_ptr<dart::dynamics::PointCloudShape> createPointCloudShape()
{
  auto shape = std::make_shared<dart::dynamics::PointCloudShape>(0.035);
  shape->setPointShapeType(dart::dynamics::PointCloudShape::BOX);
  shape->setColorMode(dart::dynamics::PointCloudShape::BIND_OVERALL);
  shape->setOverallColor(Eigen::Vector4d(0.20, 0.38, 0.94, 0.78));
  return shape;
}

std::shared_ptr<dart::dynamics::SimpleFrame> createPointCloudFrame(
    const std::shared_ptr<dart::dynamics::PointCloudShape>& shape)
{
  auto frame = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World(), kPointCloudFrameName);
  frame->setShape(shape);
  frame->getVisualAspect(true)->setRGBA(
      Eigen::Vector4d(0.20, 0.38, 0.94, 0.78));
  return frame;
}

std::shared_ptr<dart::dynamics::SimpleFrame> createVoxelGridFrame(
    const std::shared_ptr<dart::dynamics::VoxelGridShape>& shape)
{
  auto frame = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World(), kVoxelGridFrameName);
  frame->setShape(shape);
  frame->getVisualAspect(true)->setRGBA(
      Eigen::Vector4d(0.94, 0.52, 0.20, 0.22));
  frame->getVisualAspect()->setShadowed(false);
  return frame;
}

std::shared_ptr<dart::dynamics::SimpleFrame> createSensorFrame()
{
  auto frame = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World(), kSensorFrameName);
  frame->setShape(std::make_shared<dart::dynamics::SphereShape>(0.05));
  frame->getVisualAspect(true)->setRGBA(Eigen::Vector4d(0.95, 0.18, 0.12, 1.0));
  return frame;
}

Vector3List generatePointCloudInBox(double time, std::size_t numPoints)
{
  Vector3List points;
  points.reserve(numPoints);
  for (std::size_t i = 0; i < numPoints; ++i) {
    const double xMix = static_cast<double>(i % 10u) / 9.0;
    const double yMix = static_cast<double>((i / 10u) % 10u) / 9.0;
    const double zMix = static_cast<double>((i / 100u) % 5u) / 4.0;
    const double wave
        = 0.04 * std::sin(time * 2.0 + static_cast<double>(i) * 0.17);
    points.emplace_back(
        -0.90 + 0.60 * xMix, -0.30 + 0.60 * yMix, 0.55 + 0.55 * zMix + wave);
  }
  return points;
}

Vector3List generatePointCloudOnRobot(
    const dart::dynamics::SkeletonPtr& robot,
    double time,
    std::size_t numPoints)
{
  Vector3List candidates;
  if (robot != nullptr) {
    for (std::size_t bodyIndex = 0; bodyIndex < robot->getNumBodyNodes();
         ++bodyIndex) {
      auto* body = robot->getBodyNode(bodyIndex);
      if (body == nullptr) {
        continue;
      }

      const auto numShapeNodes
          = body->getNumShapeNodesWith<dart::dynamics::VisualAspect>();
      for (std::size_t shapeIndex = 0; shapeIndex < numShapeNodes;
           ++shapeIndex) {
        auto* shapeNode
            = body->getShapeNodeWith<dart::dynamics::VisualAspect>(shapeIndex);
        if (shapeNode == nullptr) {
          continue;
        }

        auto mesh = std::dynamic_pointer_cast<dart::dynamics::MeshShape>(
            shapeNode->getShape());
        if (mesh == nullptr || mesh->getTriMesh() == nullptr) {
          continue;
        }

        const auto& vertices = mesh->getTriMesh()->getVertices();
        if (vertices.empty()) {
          continue;
        }

        const std::size_t stride
            = std::max<std::size_t>(1u, vertices.size() / 160u);
        const Eigen::Isometry3d transform = shapeNode->getWorldTransform();
        const Eigen::Vector3d& scale = mesh->getScale();
        for (std::size_t vertexIndex = 0; vertexIndex < vertices.size();
             vertexIndex += stride) {
          candidates.push_back(
              transform * vertices[vertexIndex].cwiseProduct(scale));
        }
      }
    }
  }

  if (candidates.empty()) {
    return generatePointCloudInBox(time, numPoints);
  }

  Vector3List points;
  points.reserve(numPoints);
  const auto offset = static_cast<std::size_t>(time * 240.0);
  for (std::size_t i = 0; i < numPoints; ++i) {
    const std::size_t index = (offset + i * 37u) % candidates.size();
    points.push_back(candidates[index]);
  }
  return points;
}

Vector4List generatePointCloudColors(const Vector3List& points)
{
  Vector4List colors;
  colors.reserve(points.size());
  if (points.empty()) {
    return colors;
  }

  const auto minmax = std::minmax_element(
      points.begin(),
      points.end(),
      [](const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs) {
        return lhs.z() < rhs.z();
      });
  const double minZ = minmax.first->z();
  const double maxZ = minmax.second->z();
  const double diffZ
      = std::max(std::abs(maxZ - minZ), std::numeric_limits<double>::epsilon());

  for (const Eigen::Vector3d& point : points) {
    const double mix = std::clamp((point.z() - minZ) / diffZ, 0.1, 0.9);
    colors.emplace_back(mix, 0.10, 1.0 - mix, 1.0);
  }
  return colors;
}

void updateRobotPose(PointCloudState& state)
{
  if (state.robot == nullptr || state.initialRobotPositions.size() == 0) {
    return;
  }

  Eigen::VectorXd positions = state.initialRobotPositions;
  const Eigen::VectorXd lowerLimits = state.robot->getPositionLowerLimits();
  const Eigen::VectorXd upperLimits = state.robot->getPositionUpperLimits();
  for (int i = 0; i < positions.size(); ++i) {
    const double amplitude = 0.10 + 0.04 * static_cast<double>(i % 3);
    positions[i] += amplitude * std::sin(state.elapsedTime * 0.9 + 0.41 * i);
    if (i < lowerLimits.size() && std::isfinite(lowerLimits[i])) {
      positions[i] = std::max(positions[i], lowerLimits[i]);
    }
    if (i < upperLimits.size() && std::isfinite(upperLimits[i])) {
      positions[i] = std::min(positions[i], upperLimits[i]);
    }
  }
  state.robot->setPositions(positions);
}

void updateSensor(PointCloudState& state)
{
  constexpr double radius = 1.0;
  const Eigen::Vector3d center(0.0, 0.1, 0.0);
  Eigen::Vector3d sensorPosition = center;
  sensorPosition.x() = radius * std::sin(state.elapsedTime);
  sensorPosition.y() = center.y() + radius * std::cos(state.elapsedTime);
  sensorPosition.z() = 0.5 + 0.25 * std::sin(state.elapsedTime * 2.0);
  state.sensorFrame->setTranslation(sensorPosition);
}

void refreshPointCloud(PointCloudState& state)
{
  updateSensor(state);
  const Vector3List points
      = state.samplingMode == PointSamplingMode::SampleOnRobot
            ? generatePointCloudOnRobot(
                  state.robot, state.elapsedTime, kPointCount)
            : generatePointCloudInBox(state.elapsedTime, kPointCount);
  state.pointCloudShape->setPoint(
      std::span<const Eigen::Vector3d>(points.data(), points.size()));
  if (state.colorMode == dart::dynamics::PointCloudShape::BIND_PER_POINT) {
    const Vector4List colors = generatePointCloudColors(points);
    state.pointCloudShape->setColors(
        std::span<const Eigen::Vector4d>(colors.data(), colors.size()));
  } else if (state.colorMode == dart::dynamics::PointCloudShape::BIND_OVERALL) {
    state.pointCloudShape->setOverallColor(state.pointCloudColor);
  }
  state.lastPointCount = points.size();

  if (state.voxelGridShape != nullptr) {
    state.voxelGridShape->updateOccupancy(
        std::span<const Eigen::Vector3d>(points.data(), points.size()),
        state.sensorFrame->getWorldTransform().translation());
    state.lastOccupiedVoxelCount
        = state.voxelGridShape->getOccupiedCells().size();
  }
}

void stepPointCloud(PointCloudState& state)
{
  if (!state.updateRobot) {
    return;
  }

  const double dt = state.world != nullptr ? state.world->getTimeStep() : 0.001;
  state.elapsedTime += dt;
  updateRobotPose(state);
  refreshPointCloud(state);
}

const char* samplingModeLabel(PointSamplingMode mode)
{
  return mode == PointSamplingMode::SampleOnRobot ? "robot meshes" : "box";
}

const char* colorModeLabel(dart::dynamics::PointCloudShape::ColorMode mode)
{
  switch (mode) {
    case dart::dynamics::PointCloudShape::USE_SHAPE_COLOR:
      return "Use shape color";
    case dart::dynamics::PointCloudShape::BIND_OVERALL:
      return "Bind overall";
    case dart::dynamics::PointCloudShape::BIND_PER_POINT:
      return "Bind per point";
  }
  return "Unknown";
}

const char* pointShapeTypeLabel(
    dart::dynamics::PointCloudShape::PointShapeType type)
{
  switch (type) {
    case dart::dynamics::PointCloudShape::BOX:
      return "Box";
    case dart::dynamics::PointCloudShape::BILLBOARD_SQUARE:
      return "Billboard Square";
    case dart::dynamics::PointCloudShape::BILLBOARD_CIRCLE:
      return "Billboard Circle";
    case dart::dynamics::PointCloudShape::POINT:
      return "Point";
  }
  return "Unknown";
}

void cycleColorMode(PointCloudState& state)
{
  const int next = (static_cast<int>(state.colorMode) + 1) % 3;
  state.colorMode
      = static_cast<dart::dynamics::PointCloudShape::ColorMode>(next);
  state.pointCloudShape->setColorMode(state.colorMode);
  if (state.colorMode == dart::dynamics::PointCloudShape::BIND_OVERALL) {
    state.pointCloudShape->setOverallColor(state.pointCloudColor);
  }
}

void cyclePointShapeType(PointCloudState& state)
{
  const int next = (static_cast<int>(state.pointShapeType) + 1) % 4;
  state.pointShapeType
      = static_cast<dart::dynamics::PointCloudShape::PointShapeType>(next);
  state.pointCloudShape->setPointShapeType(state.pointShapeType);
}

dart::gui::OrbitCamera makePointCloudCamera()
{
  dart::gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d(0.0, 0.0, 0.30);
  camera.yaw = 0.8848934155088675;
  camera.pitch = 0.31896455812752583;
  camera.distance = 4.27318382473771;
  return camera;
}

void addSimulationControls(
    dart::gui::PanelBuilder& builder, dart::gui::PanelContext& context)
{
  if (context.lifecycle == nullptr) {
    return;
  }

  if (builder.button("Play")) {
    context.lifecycle->paused = false;
  }
  builder.sameLine();
  if (builder.button("Pause")) {
    context.lifecycle->paused = true;
  }
  builder.sameLine();
  if (builder.button("Step")) {
    dart::gui::requestSingleStep(*context.lifecycle);
  }
  builder.sameLine();
  if (builder.button("Exit")) {
    dart::gui::requestExit(*context.lifecycle);
  }
}

dart::gui::Panel createPointCloudPanel(std::shared_ptr<PointCloudState> state)
{
  dart::gui::Panel panel;
  panel.title = "Point Cloud & Voxel Grid Demo";
  panel.buildWithContext = [state = std::move(state)](
                               dart::gui::PanelBuilder& builder,
                               dart::gui::PanelContext& context) {
    builder.text("Point cloud and voxel grid rendering example");
    builder.text(
        "The KR5 robot follows deterministic joint motion for repeatable "
        "captures. Blue boxes represent point cloud samples, orange boxes "
        "represent voxel occupancy, and the red sphere is the sensor origin.");
    builder.separator();
    addSimulationControls(builder, context);
    builder.separator();

    if (builder.checkbox("Run Robot Updating", state->updateRobot)
        && state->updateRobot) {
      refreshPointCloud(*state);
    }

    builder.text(
        std::string("Sampling: ") + samplingModeLabel(state->samplingMode));
    builder.text("Points: " + std::to_string(state->lastPointCount));
    builder.text(
        "Occupied voxels: " + std::to_string(state->lastOccupiedVoxelCount));
    if (builder.button("Sample on robot")) {
      state->samplingMode = PointSamplingMode::SampleOnRobot;
      refreshPointCloud(*state);
    }
    builder.sameLine();
    if (builder.button("Sample in box")) {
      state->samplingMode = PointSamplingMode::SampleInBox;
      refreshPointCloud(*state);
    }

    builder.separator();
    if (builder.checkbox("Point Cloud", state->pointCloudVisible)) {
      applyVisibility(state->pointCloudVisual, state->pointCloudVisible);
    }
    if (builder.checkbox("Sensor Origin", state->sensorVisible)) {
      applyVisibility(state->sensorVisual, state->sensorVisible);
    }

    if (builder.checkbox("Voxel Grid", state->voxelGridVisible)) {
      applyVisibility(state->voxelGridVisual, state->voxelGridVisible);
    }

    builder.separator();
    builder.text(
        std::string("Color Mode: ") + colorModeLabel(state->colorMode));
    if (builder.button("Cycle Color Mode")) {
      cycleColorMode(*state);
    }
    if (builder.colorEdit("Point Cloud Color", state->pointCloudColor)) {
      applyPointCloudColor(*state);
    }
    if (builder.colorEdit("Voxel Grid Color", state->voxelGridColor)) {
      applyVoxelGridColor(*state);
    }
    builder.text(
        std::string("Point Shape Type: ")
        + pointShapeTypeLabel(state->pointShapeType));
    if (builder.button("Cycle Point Shape Type")) {
      cyclePointShapeType(*state);
    }
    if (builder.slider("Visual Size", state->visualSize, 0.005, 0.12)) {
      state->visualSize = std::clamp(state->visualSize, 0.005, 0.12);
      state->pointCloudShape->setVisualSize(state->visualSize);
    }

    builder.separator();
    dart::examples::addSourceOwnedGridPanelControls(builder, state->grid);
    builder.separator();
    builder.text("User Guide");
    builder.text("Select a frame, then Ctrl-left drag or use arrow keys.");
    builder.text("Source grid controls edit optional DART line geometry.");
    builder.text("time: " + std::to_string(context.simulationTime));
    builder.text("contacts: " + std::to_string(context.contactCount));
    builder.text("selected: " + context.selectedLabel);
  };
  return panel;
}

std::shared_ptr<PointCloudState> createPointCloudState()
{
  auto state = std::make_shared<PointCloudState>();
  state->world = dart::simulation::World::create("dartsim_point_cloud");
  state->world->setGravity(Eigen::Vector3d::Zero());
  state->world->setTimeStep(0.001);

  state->robot = createRobot();
  state->initialRobotPositions = state->robot->getPositions();
  state->world->addSkeleton(state->robot);
  state->world->addSkeleton(createGround());
  state->grid.visible = false;
  state->grid.lineCount = 16.0;
  state->grid.lineStepSize = 0.2;
  state->grid.zOffset = -0.01;
  state->grid.minorLineColor = Eigen::Vector4d(0.38, 0.48, 0.42, 0.36);
  state->grid.majorLineColor = Eigen::Vector4d(0.48, 0.58, 0.50, 0.52);
  state->grid.axisLineColor = Eigen::Vector4d(0.20, 0.30, 0.24, 0.76);
  dart::examples::attachSourceOwnedGridFrames(
      state->world, state->grid, kGridName);

  state->pointCloudShape = createPointCloudShape();
  state->pointCloudFrame = createPointCloudFrame(state->pointCloudShape);
  state->pointCloudVisual = state->pointCloudFrame->getVisualAspect();
  applyPointCloudColor(*state);
  state->world->addSimpleFrame(state->pointCloudFrame);

  state->voxelGridShape
      = std::make_shared<dart::dynamics::VoxelGridShape>(0.10);
  state->voxelGridFrame = createVoxelGridFrame(state->voxelGridShape);
  state->voxelGridVisual = state->voxelGridFrame->getVisualAspect();
  applyVoxelGridColor(*state);
  state->world->addSimpleFrame(state->voxelGridFrame);

  state->sensorFrame = createSensorFrame();
  state->sensorVisual = state->sensorFrame->getVisualAspect();
  state->world->addSimpleFrame(state->sensorFrame);

  refreshPointCloud(*state);
  return state;
}

} // namespace

dart::gui::ApplicationOptions makePointCloudScene()
{
  auto state = createPointCloudState();

  dart::gui::ApplicationOptions options;
  options.world = state->world;
  options.camera = makePointCloudCamera();
  options.preStep = [state]() {
    stepPointCloud(*state);
  };
  options.panels.push_back(createPointCloudPanel(state));
  return options;
}

} // namespace dart::examples::demos
