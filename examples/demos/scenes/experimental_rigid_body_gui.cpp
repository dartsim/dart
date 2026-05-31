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

#include <dart/simulation/experimental/body/collision_shape.hpp>
#include <dart/simulation/experimental/body/rigid_body.hpp>
#include <dart/simulation/experimental/body/rigid_body_options.hpp>
#include <dart/simulation/experimental/world.hpp>

#include <dart/gui/viewer.hpp>

#include <dart/simulation/world.hpp>

#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/capsule_shape.hpp>
#include <dart/dynamics/cylinder_shape.hpp>
#include <dart/dynamics/frame.hpp>
#include <dart/dynamics/mesh_shape.hpp>
#include <dart/dynamics/simple_frame.hpp>
#include <dart/dynamics/sphere_shape.hpp>

#include <dart/math/tri_mesh.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace dart::examples::demos {

namespace {

namespace dynamics = dart::dynamics;
namespace gui = dart::gui;
namespace legacy_sim = dart::simulation;
namespace math = dart::math;
namespace sx = dart::simulation::experimental;

Eigen::Vector4d rgba(double r, double g, double b, double a = 1.0)
{
  return {r, g, b, a};
}

std::shared_ptr<dynamics::Shape> makeVisualShape(
    const sx::CollisionShape& shape)
{
  switch (shape.type) {
    case sx::CollisionShapeType::Sphere:
      return std::make_shared<dynamics::SphereShape>(shape.radius);
    case sx::CollisionShapeType::Box:
      return std::make_shared<dynamics::BoxShape>(2.0 * shape.halfExtents);
    case sx::CollisionShapeType::Capsule:
      // halfExtents.z() is the axial half-height; the cylinder length is twice
      // that (the spherical caps add `radius` at each end).
      return std::make_shared<dynamics::CapsuleShape>(
          shape.radius, 2.0 * shape.halfExtents.z());
    case sx::CollisionShapeType::Cylinder:
      return std::make_shared<dynamics::CylinderShape>(
          shape.radius, 2.0 * shape.halfExtents.z());
    case sx::CollisionShapeType::Mesh: {
      auto mesh = std::make_shared<math::TriMesh<double>>();
      const math::TriMesh<double>::Vertices vertices(
          shape.vertices.begin(), shape.vertices.end());
      math::TriMesh<double>::Triangles triangles;
      triangles.reserve(shape.triangles.size());
      for (const Eigen::Vector3i& triangle : shape.triangles) {
        triangles.emplace_back(
            static_cast<std::size_t>(triangle[0]),
            static_cast<std::size_t>(triangle[1]),
            static_cast<std::size_t>(triangle[2]));
      }
      mesh->setTriangles(vertices, triangles);
      return std::make_shared<dynamics::MeshShape>(
          Eigen::Vector3d::Ones(), std::move(mesh));
    }
  }
  return std::make_shared<dynamics::SphereShape>(0.25);
}

struct RenderBody
{
  sx::RigidBody body;
  dynamics::SimpleFramePtr frame;
  Eigen::Isometry3d initialTransform = Eigen::Isometry3d::Identity();
  Eigen::Vector3d initialLinearVelocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d initialAngularVelocity = Eigen::Vector3d::Zero();
};

struct ExampleState
{
  sx::World physicsWorld;
  legacy_sim::WorldPtr renderWorld
      = legacy_sim::World::create("experimental_rigid_body_gui");
  std::vector<RenderBody> bodies;

  void addBody(
      const std::string& name,
      const sx::RigidBodyOptions& options,
      const sx::CollisionShape& shape,
      const Eigen::Vector4d& color)
  {
    auto body = physicsWorld.addRigidBody(name, options);
    body.setCollisionShape(shape);
    body.setRestitution(options.isStatic ? 0.0 : 0.15);
    body.setFriction(0.85);

    const auto initialTransform = body.getTransform();
    auto frame = dynamics::SimpleFrame::createShared(
        dynamics::Frame::World(), name + "_visual", initialTransform);
    frame->setShape(makeVisualShape(shape));
    frame->getVisualAspect(true)->setRGBA(color);
    renderWorld->addSimpleFrame(frame);
    bodies.push_back(
        {body,
         std::move(frame),
         initialTransform,
         options.linearVelocity,
         options.angularVelocity});
  }

  void syncRenderFrames()
  {
    for (auto& body : bodies) {
      body.frame->setTransform(body.body.getTransform());
    }
  }

  void step()
  {
    physicsWorld.step();
    syncRenderFrames();
  }

  void reset()
  {
    physicsWorld.setTime(0.0);
    renderWorld->reset();

    for (auto& renderBody : bodies) {
      renderBody.body.setTransform(renderBody.initialTransform);
      renderBody.body.setLinearVelocity(renderBody.initialLinearVelocity);
      renderBody.body.setAngularVelocity(renderBody.initialAngularVelocity);
      renderBody.body.clearForce();
      renderBody.body.clearTorque();
    }

    syncRenderFrames();
  }
};

std::shared_ptr<ExampleState> makeExampleState()
{
  auto state = std::make_shared<ExampleState>();

  constexpr double timeStep = 1.0 / 120.0;
  state->physicsWorld.setTimeStep(timeStep);
  state->renderWorld->setTimeStep(timeStep);

  sx::RigidBodyOptions ground;
  ground.position = Eigen::Vector3d(0.0, 0.0, -0.08);
  ground.isStatic = true;
  state->addBody(
      "ground",
      ground,
      sx::CollisionShape::makeBox(Eigen::Vector3d(3.0, 3.0, 0.08)),
      rgba(0.42, 0.45, 0.48));

  for (int i = 0; i < 4; ++i) {
    sx::RigidBodyOptions options;
    options.mass = 1.0 + 0.25 * i;
    options.position
        = Eigen::Vector3d(-0.75 + 0.5 * i, 0.18 * (i % 2), 1.2 + 0.45 * i);
    options.linearVelocity = Eigen::Vector3d(0.45 - 0.25 * i, 0.0, 0.0);
    options.angularVelocity = Eigen::Vector3d(0.0, 0.4 + 0.2 * i, 0.25);

    const double radius = 0.18 + 0.03 * i;
    state->addBody(
        "falling_sphere_" + std::to_string(i),
        options,
        sx::CollisionShape::makeSphere(radius),
        rgba(0.16 + 0.12 * i, 0.42, 0.92 - 0.1 * i));
  }

  sx::RigidBodyOptions box;
  box.mass = 2.0;
  box.position = Eigen::Vector3d(0.85, -0.35, 2.2);
  box.orientation
      = Eigen::Quaterniond(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY()));
  box.angularVelocity = Eigen::Vector3d(0.2, -0.35, 0.1);
  state->addBody(
      "falling_box",
      box,
      sx::CollisionShape::makeBox(Eigen::Vector3d(0.2, 0.24, 0.16)),
      rgba(0.93, 0.56, 0.18));

  state->physicsWorld.enterSimulationMode();
  state->syncRenderFrames();
  return state;
}

gui::OrbitCamera makeCamera()
{
  gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d(0.0, 0.0, 0.65);
  camera.yaw = -0.78;
  camera.pitch = 0.34;
  camera.distance = 5.2;
  return camera;
}

std::vector<gui::KeyboardAction> createKeyboardActions(
    const std::shared_ptr<ExampleState>& state)
{
  gui::KeyboardAction reset;
  reset.label = "reset scene";
  reset.shortcut = gui::KeyboardShortcut::characterKey('r');
  reset.callback = [state](gui::KeyboardActionContext&) {
    state->reset();
  };
  return {std::move(reset)};
}

} // namespace

dart::gui::ApplicationOptions makeExperimentalRigidBodyScene()
{
  const auto state = makeExampleState();

  gui::ApplicationOptions options;
  options.world = state->renderWorld;
  options.camera = makeCamera();
  options.simulateWorld = false;
  options.preStep = [state]() {
    state->step();
  };
  options.keyboardActions = createKeyboardActions(state);
  return options;
}

} // namespace dart::examples::demos
