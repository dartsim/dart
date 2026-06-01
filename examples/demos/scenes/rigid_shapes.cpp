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
#include "z_up.hpp"

#include <dart/config.hpp>

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

#include <dart/math/constants.hpp>
#include <dart/math/geometry.hpp>
#include <dart/math/random.hpp>

#include <dart/io/read.hpp>

#include <Eigen/Geometry>

#include <functional>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dart::examples::demos {

namespace {

constexpr const char* kShapesUri = "dart://sample/skel/shapes.skel";
constexpr const char* kShapePrefix = "rigid_shape_";
constexpr const char* kContactFrameName = "contact_points";

struct RigidShapesConfig
{
  std::string collisionDetector = "file";
  std::size_t maxContacts = 1000;
  double groundThickness = 0.0;
};

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

bool updateGroundThickness(
    const dart::simulation::WorldPtr& world, double thickness)
{
  if (world == nullptr) {
    return false;
  }

  dart::dynamics::SkeletonPtr groundSkeleton
      = world->getSkeleton("ground skeleton");
  if (groundSkeleton == nullptr) {
    for (std::size_t i = 0; i < world->getNumSkeletons(); ++i) {
      auto skeleton = world->getSkeleton(i);
      if (skeleton != nullptr && skeleton->getBodyNode("ground") != nullptr) {
        groundSkeleton = skeleton;
        break;
      }
    }
  }

  if (groundSkeleton == nullptr) {
    return false;
  }

  auto* groundBody = groundSkeleton->getBodyNode("ground");
  if (groundBody == nullptr) {
    return false;
  }

  const auto updateShapeNode
      = [thickness](dart::dynamics::ShapeNode* shapeNode) {
          if (shapeNode == nullptr) {
            return;
          }

          auto box = std::dynamic_pointer_cast<dart::dynamics::BoxShape>(
              shapeNode->getShape());
          if (box == nullptr) {
            return;
          }

          const Eigen::Vector3d originalSize = box->getSize();
          const Eigen::Vector3d originalTranslation
              = shapeNode->getRelativeTranslation();
          const double originalTop
              = originalTranslation.y() + 0.5 * originalSize.y();

          Eigen::Vector3d size = originalSize;
          size.y() = thickness;
          box->setSize(size);

          Eigen::Vector3d translation = originalTranslation;
          translation.y() = originalTop - 0.5 * thickness;
          shapeNode->setRelativeTranslation(translation);
        };

  groundBody->eachShapeNodeWith<dart::dynamics::VisualAspect>(updateShapeNode);
  groundBody->eachShapeNodeWith<dart::dynamics::CollisionAspect>(
      updateShapeNode);
  return true;
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

Eigen::Isometry3d makeRandomTransform()
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  const Eigen::Vector3d rotation = dart::math::Random::uniform<Eigen::Vector3d>(
      -dart::math::pi, dart::math::pi);
  // Z-up: random lateral X/Y over the ground, random drop height along +Z.
  transform.translation() = Eigen::Vector3d(
      dart::math::Random::uniform(-1.0, 1.0),
      dart::math::Random::uniform(-1.0, 1.0),
      dart::math::Random::uniform(0.5, 1.0));
  transform.linear() = dart::math::expMapRot(rotation);
  return transform;
}

Eigen::Vector3d makeRandomColor()
{
  return dart::math::Random::uniform<Eigen::Vector3d>(0.0, 1.0);
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
      double mass = 10.0)
  {
    if (world == nullptr || shape == nullptr) {
      return;
    }

    const std::size_t index = nextShapeIndex++;
    world->addSkeleton(createDynamicShape(
        std::string(kShapePrefix) + std::move(label) + "_"
            + std::to_string(index),
        std::move(shape),
        makeRandomTransform(),
        makeRandomColor(),
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
    spawn<dart::dynamics::BoxShape>(
        "box", dart::math::Random::uniform<Eigen::Vector3d>(0.05, 0.25));
  }

  void spawnEllipsoid()
  {
    spawn<dart::dynamics::EllipsoidShape>(
        "ellipsoid",
        2.0 * dart::math::Random::uniform<Eigen::Vector3d>(0.025, 0.125));
  }

  void spawnCylinder()
  {
    spawn<dart::dynamics::CylinderShape>(
        "cylinder",
        dart::math::Random::uniform(0.05, 0.25),
        dart::math::Random::uniform(0.1, 0.5));
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
    const int vertexCount = dart::math::Random::uniform<int>(16, 32);
    const double bound = dart::math::Random::uniform(0.1, 0.25);
    mesh->reserveVertices(vertexCount + 4);

    const double seed = bound * 0.6;
    mesh->addVertex(Eigen::Vector3d(-seed, -seed, -seed));
    mesh->addVertex(Eigen::Vector3d(seed, -seed, seed));
    mesh->addVertex(Eigen::Vector3d(-seed, seed, seed));
    mesh->addVertex(Eigen::Vector3d(seed, seed, -seed));

    for (int i = 0; i < vertexCount; ++i) {
      mesh->addVertex(
          dart::math::Random::uniform<Eigen::Vector3d>(-bound, bound));
    }
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
  return std::make_shared<RigidShapesState>(world);
}

dart::simulation::WorldPtr createRigidShapesWorld(
    const RigidShapesConfig& config)
{
  auto world = dart::io::readWorld(kShapesUri);
  if (world == nullptr) {
    throw std::runtime_error(
        "Failed to load rigid-shapes world from " + std::string(kShapesUri));
  }

  if (!applyCollisionDetector(world, config.collisionDetector)) {
    throw std::runtime_error(
        "Unsupported collision detector: " + config.collisionDetector);
  }
  if (auto* constraintSolver = world->getConstraintSolver()) {
    constraintSolver->getCollisionOption().maxNumContacts = config.maxContacts;
  }
  if (config.groundThickness > 0.0
      && !updateGroundThickness(world, config.groundThickness)) {
    throw std::runtime_error("Failed to update ground thickness");
  }
  // shapes.skel is authored Y-up; reorient to the canonical Z-up convention
  // (the spawn poses and gravity toggle are already Z-up).
  reorientWorldToZUp(world);
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
  camera.up = Eigen::Vector3d::UnitZ();
  camera.yaw = 0.7853981633974483;
  camera.pitch = 0.6154797086703874;
  camera.distance = 3.4641016151377544;
  return camera;
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

dart::gui::ApplicationOptions makeRigidShapesScene()
{
  const RigidShapesConfig config;
  auto world = createRigidShapesWorld(config);
  auto state = createRigidShapesState(world);

  dart::gui::ApplicationOptions options;
  options.world = world;
  options.camera = makeRigidShapesCamera();
  options.preStep = [state]() {
    state->updateContactPoints();
  };
  options.keyboardActions = createRigidShapesKeyboardActions(state);
  options.panels.push_back(createControlsPanel(state));
  return options;
}

} // namespace dart::examples::demos
