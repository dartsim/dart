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
 *   * This code incorporates portions of Open Dynamics Engine
 *     (Copyright (c) 2001-2004, Russell L. Smith. All rights
 *     reserved.) and portions of FCL (Copyright (c) 2011, Willow
 *     Garage, Inc. All rights reserved.), which were released under
 *     the same BSD license as below
 *
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

#include <dart/config.hpp>

#include <dart/gui/application.hpp>
#include <dart/gui/panel.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/simulation/world.hpp>

#include <dart/constraint/constraint_solver.hpp>

#include <dart/collision/collision_option.hpp>
#include <dart/collision/collision_result.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/cone_shape.hpp>
#include <dart/dynamics/convex_mesh_shape.hpp>
#include <dart/dynamics/cylinder_shape.hpp>
#include <dart/dynamics/ellipsoid_shape.hpp>
#include <dart/dynamics/frame.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/inertia.hpp>
#include <dart/dynamics/point_cloud_shape.hpp>
#include <dart/dynamics/shape.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/simple_frame.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/sphere_shape.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <charconv>
#include <functional>
#include <iostream>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include <cctype>

namespace {

constexpr const char* kGroundName = "rigid_shapes_ground";
constexpr const char* kShapePrefix = "rigid_shape_";
constexpr const char* kContactFrameName = "contact_points";

struct RigidShapesConfig
{
  std::string collisionDetector = "file";
  std::size_t maxContacts = 1000;
  double groundThickness = 0.0;
};

std::string toLowerAscii(std::string_view value)
{
  std::string lowered(value);
  std::transform(lowered.begin(), lowered.end(), lowered.begin(), [](char c) {
    return static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  });
  return lowered;
}

bool parseSizeT(std::string_view value, std::size_t& output)
{
  const auto* first = value.data();
  const auto* last = value.data() + value.size();
  const auto result = std::from_chars(first, last, output);
  return result.ec == std::errc{} && result.ptr == last;
}

bool parseDouble(std::string_view value, double& output)
{
  const auto* first = value.data();
  const auto* last = value.data() + value.size();
  const auto result = std::from_chars(first, last, output);
  return result.ec == std::errc{} && result.ptr == last;
}

std::optional<std::string_view> getOptionValue(
    std::string_view argument,
    std::string_view option,
    int& index,
    int argc,
    char* argv[])
{
  if (argument == option) {
    if (index + 1 >= argc) {
      throw std::runtime_error("Missing value for " + std::string(option));
    }
    return std::string_view(argv[++index]);
  }

  const std::string prefix = std::string(option) + "=";
  if (argument.starts_with(prefix)) {
    return argument.substr(prefix.size());
  }

  return std::nullopt;
}

RigidShapesConfig parseRigidShapesConfig(int argc, char* argv[])
{
  RigidShapesConfig config;
  for (int i = 1; i < argc; ++i) {
    const std::string_view argument(argv[i] == nullptr ? "" : argv[i]);
    if (auto value
        = getOptionValue(argument, "--collision-detector", i, argc, argv)) {
      config.collisionDetector = toLowerAscii(*value);
      continue;
    }
    if (auto value
        = getOptionValue(argument, "--max-contacts", i, argc, argv)) {
      if (!parseSizeT(*value, config.maxContacts)) {
        throw std::runtime_error(
            "Invalid --max-contacts value: " + std::string(*value));
      }
      continue;
    }
    if (auto value
        = getOptionValue(argument, "--ground-thickness", i, argc, argv)) {
      if (!parseDouble(*value, config.groundThickness)
          || config.groundThickness < 0.0) {
        throw std::runtime_error(
            "Invalid --ground-thickness value: " + std::string(*value));
      }
      continue;
    }
  }
  return config;
}

bool applyCollisionDetector(
    const dart::simulation::WorldPtr& world, const std::string& detectorName)
{
  if (detectorName == "file") {
    return true;
  }
  if (detectorName == "dart") {
    world->setCollisionDetector(dart::simulation::CollisionDetectorType::Dart);
    return true;
  }
  if (detectorName == "fcl") {
    world->setCollisionDetector(dart::simulation::CollisionDetectorType::Fcl);
    return true;
  }
#if DART_HAVE_BULLET
  if (detectorName == "bullet") {
    world->setCollisionDetector(
        dart::simulation::CollisionDetectorType::Bullet);
    return true;
  }
#endif
#if DART_HAVE_ODE
  if (detectorName == "ode") {
    world->setCollisionDetector(dart::simulation::CollisionDetectorType::Ode);
    return true;
  }
#endif
  return false;
}

dart::dynamics::SkeletonPtr createGround(double requestedThickness)
{
  const double thickness = requestedThickness > 0.0 ? requestedThickness : 0.08;
  auto ground = dart::dynamics::Skeleton::create(kGroundName);
  auto pair = ground->createJointAndBodyNodePair<dart::dynamics::WeldJoint>();
  auto* body = pair.second;

  auto shape = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(7.0, 7.0, thickness));
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(shape);
  shapeNode->getVisualAspect()->setColor(Eigen::Vector3d(0.76, 0.78, 0.76));

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation().z() = -0.5 * thickness;
  pair.first->setTransformFromParentBodyNode(transform);

  return ground;
}

dart::dynamics::SkeletonPtr createDynamicShape(
    const std::string& name,
    std::shared_ptr<dart::dynamics::Shape> shape,
    const Eigen::Isometry3d& transform,
    const Eigen::Vector3d& color,
    double mass = 1.0)
{
  auto skeleton = dart::dynamics::Skeleton::create(name);
  auto pair = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
  auto* joint = pair.first;
  auto* body = pair.second;
  joint->setTransformFromParentBodyNode(transform);

  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(shape);
  shapeNode->getVisualAspect()->setColor(color);
  shapeNode->getDynamicsAspect()->setRestitutionCoeff(0.35);
  shapeNode->getDynamicsAspect()->setFrictionCoeff(0.8);

  body->setInertia(
      dart::dynamics::Inertia(
          mass, Eigen::Vector3d::Zero(), shape->computeInertia(mass)));
  return skeleton;
}

Eigen::Isometry3d makePose(std::size_t index)
{
  static const std::array<Eigen::Vector3d, 6> kPositions{{
      Eigen::Vector3d(-1.4, -0.25, 0.45),
      Eigen::Vector3d(-0.85, 0.25, 0.75),
      Eigen::Vector3d(-0.25, -0.15, 1.05),
      Eigen::Vector3d(0.35, 0.25, 1.35),
      Eigen::Vector3d(0.95, -0.20, 1.65),
      Eigen::Vector3d(1.55, 0.25, 1.95),
  }};

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = kPositions[index % kPositions.size()];
  transform.rotate(
      Eigen::AngleAxisd(
          0.22 * static_cast<double>(index + 1), Eigen::Vector3d::UnitX()));
  transform.rotate(
      Eigen::AngleAxisd(
          0.17 * static_cast<double>(index + 2), Eigen::Vector3d::UnitY()));
  return transform;
}

Eigen::Vector3d makeColor(std::size_t index)
{
  static const std::array<Eigen::Vector3d, 6> kColors{{
      Eigen::Vector3d(0.85, 0.25, 0.22),
      Eigen::Vector3d(0.22, 0.55, 0.88),
      Eigen::Vector3d(0.95, 0.72, 0.24),
      Eigen::Vector3d(0.30, 0.75, 0.42),
      Eigen::Vector3d(0.68, 0.35, 0.82),
      Eigen::Vector3d(0.25, 0.74, 0.72),
  }};
  return kColors[index % kColors.size()];
}

struct RigidShapesState
{
  explicit RigidShapesState(dart::simulation::WorldPtr inputWorld)
    : world(std::move(inputWorld)),
      contactPoints(std::make_shared<dart::dynamics::PointCloudShape>(0.02))
  {
    contactPoints->setDataVariance(dart::dynamics::Shape::DYNAMIC);
    contactPoints->setPointShapeType(
        dart::dynamics::PointCloudShape::BILLBOARD_CIRCLE);
    contactFrame = dart::dynamics::SimpleFrame::createShared(
        dart::dynamics::Frame::World(), kContactFrameName);
    contactFrame->setShape(contactPoints);
    contactFrame->getVisualAspect(true)->setRGBA(
        Eigen::Vector4d(0.9, 0.1, 0.1, 1.0));
    setContactPointsVisible(false);
    if (world != nullptr) {
      world->addSimpleFrame(contactFrame);
    }
  }

  void spawnShape(
      std::string label,
      std::shared_ptr<dart::dynamics::Shape> shape,
      double mass = 1.0)
  {
    if (world == nullptr || shape == nullptr) {
      return;
    }

    const std::size_t index = nextShapeIndex++;
    world->addSkeleton(createDynamicShape(
        std::string(kShapePrefix) + std::move(label) + "_"
            + std::to_string(index),
        std::move(shape),
        makePose(index),
        makeColor(index),
        mass));
  }

  template <class ShapeType, class... Args>
  void spawn(std::string label, Args&&... args)
  {
    spawnShape(
        std::move(label),
        std::make_shared<ShapeType>(std::forward<Args>(args)...));
  }

  void spawnBox()
  {
    spawn<dart::dynamics::BoxShape>("box", Eigen::Vector3d(0.42, 0.28, 0.24));
  }

  void spawnEllipsoid()
  {
    spawn<dart::dynamics::EllipsoidShape>(
        "ellipsoid", Eigen::Vector3d(0.30, 0.20, 0.18));
  }

  void spawnCylinder()
  {
    spawn<dart::dynamics::CylinderShape>("cylinder", 0.18, 0.42);
  }

  void spawnSphere()
  {
    spawn<dart::dynamics::SphereShape>("sphere", 0.22);
  }

  void spawnCone()
  {
    spawn<dart::dynamics::ConeShape>("cone", 0.20, 0.45);
  }

  void spawnConvexMesh()
  {
    auto mesh
        = std::make_shared<dart::dynamics::ConvexMeshShape::TriMeshType>();
    mesh->reserveVertices(10);
    constexpr double bound = 0.24;
    mesh->addVertex(Eigen::Vector3d(-bound, -bound, -bound));
    mesh->addVertex(Eigen::Vector3d(bound, -bound, bound));
    mesh->addVertex(Eigen::Vector3d(-bound, bound, bound));
    mesh->addVertex(Eigen::Vector3d(bound, bound, -bound));
    mesh->addVertex(Eigen::Vector3d(0.0, -bound, 0.5 * bound));
    mesh->addVertex(Eigen::Vector3d(-0.5 * bound, 0.0, -bound));
    mesh->addVertex(Eigen::Vector3d(0.5 * bound, bound, 0.0));
    mesh->addVertex(Eigen::Vector3d(-bound, 0.5 * bound, 0.0));
    mesh->addVertex(Eigen::Vector3d(bound, -0.5 * bound, 0.0));
    mesh->addVertex(Eigen::Vector3d(0.0, 0.0, bound));
    spawnShape(
        "convex_mesh", dart::dynamics::ConvexMeshShape::fromMesh(mesh, true));
  }

  void deleteLast()
  {
    if (world == nullptr || world->getNumSkeletons() <= 1) {
      return;
    }

    for (std::size_t i = world->getNumSkeletons(); i > 0; --i) {
      auto skeleton = world->getSkeleton(i - 1);
      if (skeleton != nullptr
          && skeleton->getName().rfind(kShapePrefix, 0) == 0) {
        world->removeSkeleton(skeleton);
        return;
      }
    }
  }

  void setContactPointsVisible(bool visible)
  {
    contactPointsVisible = visible;
    if (contactFrame != nullptr) {
      contactFrame->getVisualAspect(true)->setHidden(!visible);
    }
    if (!visible && contactPoints != nullptr) {
      contactPoints->removeAllPoints();
      contactPositions.clear();
    }
  }

  void toggleContactPoints()
  {
    setContactPointsVisible(!contactPointsVisible);
  }

  void updateContactPoints()
  {
    if (!contactPointsVisible || world == nullptr || contactPoints == nullptr) {
      return;
    }

    const auto contacts = world->getLastCollisionResult().getContacts();
    contactPositions.clear();
    contactPositions.reserve(contacts.size());
    for (const auto& contact : contacts) {
      contactPositions.push_back(contact.point);
    }

    contactPoints->setPoint(
        std::span<const Eigen::Vector3d>(
            contactPositions.data(), contactPositions.size()));
  }

  dart::simulation::WorldPtr world;
  std::shared_ptr<dart::dynamics::PointCloudShape> contactPoints;
  std::shared_ptr<dart::dynamics::SimpleFrame> contactFrame;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
      contactPositions;
  std::size_t nextShapeIndex = 0;
  bool contactPointsVisible = false;
};

std::shared_ptr<RigidShapesState> createRigidShapesState(
    const dart::simulation::WorldPtr& world)
{
  auto state = std::make_shared<RigidShapesState>(world);
  state->spawnBox();
  state->spawnEllipsoid();
  state->spawnCylinder();
  state->spawnSphere();
  state->spawnCone();
  return state;
}

dart::simulation::WorldPtr createRigidShapesWorld(
    const RigidShapesConfig& config)
{
  auto world = dart::simulation::World::create("rigid_shapes");
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  if (!applyCollisionDetector(world, config.collisionDetector)) {
    throw std::runtime_error(
        "Unsupported collision detector: " + config.collisionDetector);
  }
  if (auto* constraintSolver = world->getConstraintSolver()) {
    constraintSolver->getCollisionOption().maxNumContacts = config.maxContacts;
  }
  world->addSkeleton(createGround(config.groundThickness));
  return world;
}

dart::gui::Panel createControlsPanel(
    const std::shared_ptr<RigidShapesState>& state)
{
  bool gravityEnabled = true;

  dart::gui::Panel panel;
  panel.title = "Rigid Shapes";
  panel.buildWithContext = [state, gravityEnabled](
                               dart::gui::PanelBuilder& builder,
                               dart::gui::PanelContext& context) mutable {
    builder.text("Spawn rigid bodies with different collision shapes.");
    builder.text("Keys: q box, w ellipsoid, e cylinder, r convex mesh.");
    builder.text("Keys: a delete last, c toggle contact points.");
    builder.separator();
    if (context.lifecycle != nullptr) {
      if (builder.button(context.lifecycle->paused ? "Resume" : "Pause")) {
        dart::gui::togglePaused(*context.lifecycle);
      }
      builder.sameLine();
      if (builder.button("Step")) {
        dart::gui::requestSingleStep(*context.lifecycle);
      }
    }

    if (context.world != nullptr
        && builder.checkbox("Gravity", gravityEnabled)) {
      context.world->setGravity(
          gravityEnabled ? Eigen::Vector3d(0.0, 0.0, -9.81)
                         : Eigen::Vector3d::Zero());
    }

    if (builder.button("Box")) {
      state->spawnBox();
    }
    builder.sameLine();
    if (builder.button("Ellipsoid")) {
      state->spawnEllipsoid();
    }
    if (builder.button("Cylinder")) {
      state->spawnCylinder();
    }
    builder.sameLine();
    if (builder.button("Sphere")) {
      state->spawnSphere();
    }
    if (builder.button("Cone")) {
      state->spawnCone();
    }
    builder.sameLine();
    if (builder.button("Convex Mesh")) {
      state->spawnConvexMesh();
    }
    if (builder.button("Delete Last")) {
      state->deleteLast();
    }
    builder.sameLine();
    if (builder.button("Toggle Contacts")) {
      state->toggleContactPoints();
    }
    if (context.world != nullptr) {
      builder.text(
          "skeletons: " + std::to_string(context.world->getNumSkeletons()));
    }
    builder.text("time: " + std::to_string(context.simulationTime));
    builder.text("contacts: " + std::to_string(context.contactCount));
  };
  return panel;
}

dart::gui::OrbitCamera makeRigidShapesCamera()
{
  dart::gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d::Zero();
  camera.yaw = 0.7853981633974483;
  camera.pitch = 0.6154797086703874;
  camera.distance = 3.4641016151377544;
  return camera;
}

dart::gui::RunOptions makeRigidShapesRunDefaults()
{
  dart::gui::RunOptions options;
  options.width = 640;
  options.height = 480;
  return options;
}

dart::gui::KeyboardAction makeRigidShapesAction(
    std::shared_ptr<RigidShapesState> state,
    char key,
    std::string label,
    std::function<void(RigidShapesState&)> callback)
{
  dart::gui::KeyboardAction action;
  action.label = std::move(label);
  action.shortcut = dart::gui::KeyboardShortcut::characterKey(key);
  action.callback = [state = std::move(state), callback = std::move(callback)](
                        dart::gui::KeyboardActionContext&) {
    if (state != nullptr) {
      callback(*state);
    }
  };
  return action;
}

std::vector<dart::gui::KeyboardAction> createRigidShapesKeyboardActions(
    const std::shared_ptr<RigidShapesState>& state)
{
  std::vector<dart::gui::KeyboardAction> actions;
  actions.push_back(makeRigidShapesAction(
      state, 'q', "Spawn box", [](RigidShapesState& s) { s.spawnBox(); }));
  actions.push_back(makeRigidShapesAction(
      state, 'w', "Spawn ellipsoid", [](RigidShapesState& s) {
        s.spawnEllipsoid();
      }));
  actions.push_back(makeRigidShapesAction(
      state, 'e', "Spawn cylinder", [](RigidShapesState& s) {
        s.spawnCylinder();
      }));
  actions.push_back(makeRigidShapesAction(
      state, 'r', "Spawn convex mesh", [](RigidShapesState& s) {
        s.spawnConvexMesh();
      }));
  actions.push_back(makeRigidShapesAction(
      state, 'a', "Delete last spawned shape", [](RigidShapesState& s) {
        s.deleteLast();
      }));
  actions.push_back(makeRigidShapesAction(
      state, 'c', "Toggle contact points", [](RigidShapesState& s) {
        s.toggleContactPoints();
      }));
  return actions;
}

} // namespace

int main(int argc, char* argv[])
{
  try {
    const auto config = parseRigidShapesConfig(argc, argv);
    auto world = createRigidShapesWorld(config);
    auto state = createRigidShapesState(world);

    dart::gui::ApplicationOptions options;
    options.world = world;
    options.runDefaults = makeRigidShapesRunDefaults();
    options.camera = makeRigidShapesCamera();
    options.preStep = [state]() {
      state->updateContactPoints();
    };
    options.keyboardActions = createRigidShapesKeyboardActions(state);
    options.panels.push_back(createControlsPanel(state));
    return dart::gui::runApplication(argc, argv, options);
  } catch (const std::exception& e) {
    std::cerr << "rigid_shapes: " << e.what() << "\n";
    return 1;
  }
}
