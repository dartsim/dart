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

#include <dart/gui/panel.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/simulation/world.hpp>

#include <dart/constraint/ball_joint_constraint.hpp>
#include <dart/constraint/constraint_solver.hpp>
#include <dart/constraint/cylindrical_joint_constraint.hpp>
#include <dart/constraint/revolute_joint_constraint.hpp>
#include <dart/constraint/weld_joint_constraint.hpp>

#include <dart/dynamics/arrow_shape.hpp>
#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/frame.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/line_segment_shape.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/simple_frame.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/sphere_shape.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <memory>
#include <string>

namespace dart::examples::demos {

namespace {

namespace constraint = dart::constraint;
namespace dynamics = dart::dynamics;
namespace gui = dart::gui;
namespace simulation = dart::simulation;

constexpr double kBodyMass = 1.0;

struct DemoBody
{
  dynamics::SkeletonPtr skeleton;
  dynamics::FreeJoint* joint = nullptr;
  dynamics::BodyNode* body = nullptr;
};

Eigen::Isometry3d makeTransform(const Eigen::Vector3d& translation)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = translation;
  return transform;
}

Eigen::Vector4d rgba(double r, double g, double b, double a = 1.0)
{
  return {r, g, b, a};
}

void setFreeJointVelocity(
    dynamics::FreeJoint* joint,
    const Eigen::Vector3d& angular,
    const Eigen::Vector3d& linear)
{
  Eigen::Matrix<double, 6, 1> velocity = Eigen::Matrix<double, 6, 1>::Zero();
  velocity.head<3>() = angular;
  velocity.tail<3>() = linear;
  joint->setVelocities(velocity);
}

DemoBody createFreeBox(
    const std::string& name,
    const Eigen::Isometry3d& pose,
    const Eigen::Vector3d& size,
    const Eigen::Vector4d& color,
    const Eigen::Vector3d& visualOffset = Eigen::Vector3d::Zero())
{
  DemoBody result;
  result.skeleton = dynamics::Skeleton::create(name);
  auto pair
      = result.skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>();
  result.joint = pair.first;
  result.body = pair.second;

  result.joint->setPositions(dynamics::FreeJoint::convertToPositions(pose));
  result.body->setMass(kBodyMass);

  for (std::size_t i = 0; i < result.joint->getNumDofs(); ++i) {
    result.joint->setDampingCoefficient(i, 0.02);
  }

  auto shape = std::make_shared<dynamics::BoxShape>(size);
  auto* shapeNode = result.body->createShapeNodeWith<
      dynamics::VisualAspect,
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);
  shapeNode->setRelativeTranslation(visualOffset);
  shapeNode->getVisualAspect()->setRGBA(color);

  return result;
}

dynamics::SimpleFramePtr createBoxMarker(
    const std::string& name,
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& size,
    const Eigen::Vector4d& color)
{
  auto frame = dynamics::SimpleFrame::createShared(
      dynamics::Frame::World(), name, makeTransform(position));
  frame->setShape(std::make_shared<dynamics::BoxShape>(size));
  frame->getVisualAspect(true)->setRGBA(color);
  return frame;
}

dynamics::SimpleFramePtr createSphereMarker(
    const std::string& name,
    const Eigen::Vector3d& position,
    double radius,
    const Eigen::Vector4d& color)
{
  auto frame = dynamics::SimpleFrame::createShared(
      dynamics::Frame::World(), name, makeTransform(position));
  frame->setShape(std::make_shared<dynamics::SphereShape>(radius));
  frame->getVisualAspect(true)->setRGBA(color);
  return frame;
}

dynamics::SimpleFramePtr createLineMarker(
    const std::string& name,
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end,
    double thickness,
    const Eigen::Vector4d& color)
{
  auto frame
      = dynamics::SimpleFrame::createShared(dynamics::Frame::World(), name);
  frame->setShape(
      std::make_shared<dynamics::LineSegmentShape>(start, end, thickness));
  frame->getVisualAspect(true)->setRGBA(color);
  return frame;
}

dynamics::SimpleFramePtr createAxisArrow(
    const std::string& name,
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end,
    const Eigen::Vector4d& color)
{
  auto frame
      = dynamics::SimpleFrame::createShared(dynamics::Frame::World(), name);
  auto arrow = std::make_shared<dynamics::ArrowShape>(
      start,
      end,
      dynamics::ArrowShape::Properties(0.025, 2.2, 0.18, 0.08, 0.18),
      color);
  frame->setShape(arrow);
  frame->getVisualAspect(true)->setRGBA(color);
  return frame;
}

void addLaneGuide(
    const simulation::WorldPtr& world,
    const std::string& name,
    double x,
    const Eigen::Vector4d& color)
{
  world->addSimpleFrame(createBoxMarker(
      name + "_pad",
      Eigen::Vector3d(x, 0.0, 0.02),
      Eigen::Vector3d(1.75, 1.2, 0.04),
      color));
  world->addSimpleFrame(createLineMarker(
      name + "_left_rail",
      Eigen::Vector3d(x - 0.88, -0.62, 0.08),
      Eigen::Vector3d(x - 0.88, 0.62, 0.08),
      2.0,
      rgba(color.x(), color.y(), color.z(), 0.85)));
  world->addSimpleFrame(createLineMarker(
      name + "_right_rail",
      Eigen::Vector3d(x + 0.88, -0.62, 0.08),
      Eigen::Vector3d(x + 0.88, 0.62, 0.08),
      2.0,
      rgba(color.x(), color.y(), color.z(), 0.85)));
}

void addBallJointDemo(const simulation::WorldPtr& world, double x)
{
  const Eigen::Vector3d anchor(x, 0.0, 1.35);
  DemoBody body = createFreeBox(
      "dynamic_ball_body",
      makeTransform(anchor),
      Eigen::Vector3d(0.78, 0.25, 0.25),
      rgba(0.88, 0.25, 0.18),
      Eigen::Vector3d(0.55, 0.0, -0.22));

  setFreeJointVelocity(
      body.joint, Eigen::Vector3d(0.0, 1.35, 0.0), Eigen::Vector3d::Zero());
  world->addSkeleton(body.skeleton);
  world->addSimpleFrame(
      createSphereMarker("ball_anchor", anchor, 0.08, rgba(1.0, 1.0, 1.0)));
  world->addSimpleFrame(createLineMarker(
      "ball_anchor_pin",
      anchor - Eigen::Vector3d(0.0, 0.0, 0.45),
      anchor + Eigen::Vector3d(0.0, 0.0, 0.45),
      3.0,
      rgba(0.86, 0.86, 0.86)));

  world->getConstraintSolver()->addConstraint(
      std::make_shared<constraint::BallJointConstraint>(body.body, anchor));
}

void addRevoluteJointDemo(const simulation::WorldPtr& world, double x)
{
  const Eigen::Vector3d anchor(x, 0.0, 1.35);
  const Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
  DemoBody body = createFreeBox(
      "dynamic_revolute_body",
      makeTransform(anchor),
      Eigen::Vector3d(0.76, 0.24, 0.24),
      rgba(0.93, 0.58, 0.18),
      Eigen::Vector3d(0.55, 0.0, 0.0));

  setFreeJointVelocity(
      body.joint, Eigen::Vector3d(0.0, 0.0, 2.0), Eigen::Vector3d::Zero());
  world->addSkeleton(body.skeleton);
  world->addSimpleFrame(createAxisArrow(
      "revolute_axis",
      anchor - 0.55 * axis,
      anchor + 0.75 * axis,
      rgba(0.95, 0.74, 0.25)));

  world->getConstraintSolver()->addConstraint(
      std::make_shared<constraint::RevoluteJointConstraint>(
          body.body, anchor, axis));
}

void addCylindricalJointDemo(const simulation::WorldPtr& world, double x)
{
  const Eigen::Vector3d axisPoint(x, 0.0, 0.75);
  const Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
  DemoBody body = createFreeBox(
      "dynamic_cylindrical_body",
      makeTransform(axisPoint),
      Eigen::Vector3d(0.30, 0.64, 0.24),
      rgba(0.18, 0.55, 0.92),
      Eigen::Vector3d(0.38, 0.0, 0.0));

  setFreeJointVelocity(
      body.joint,
      Eigen::Vector3d(0.0, 0.0, 2.8),
      Eigen::Vector3d(0.0, 0.0, 0.18));
  world->addSkeleton(body.skeleton);
  world->addSimpleFrame(createAxisArrow(
      "cylindrical_axis",
      axisPoint - 0.25 * axis,
      axisPoint + 1.75 * axis,
      rgba(0.22, 0.82, 1.0)));

  world->getConstraintSolver()->addConstraint(
      std::make_shared<constraint::CylindricalJointConstraint>(
          body.body, axisPoint, axis));
}

void addWeldJointDemo(const simulation::WorldPtr& world, double x)
{
  const Eigen::Vector3d weldPose(x, 0.0, 1.2);
  DemoBody body = createFreeBox(
      "dynamic_weld_body",
      makeTransform(weldPose),
      Eigen::Vector3d(0.74, 0.32, 0.32),
      rgba(0.30, 0.72, 0.36));

  setFreeJointVelocity(
      body.joint,
      Eigen::Vector3d(0.5, 0.0, 0.8),
      Eigen::Vector3d(0.4, 0.0, 0.3));
  world->addSkeleton(body.skeleton);
  world->addSimpleFrame(createSphereMarker(
      "weld_reference", weldPose, 0.08, rgba(0.64, 0.95, 0.62)));
  world->addSimpleFrame(createLineMarker(
      "weld_lock_a",
      weldPose + Eigen::Vector3d(-0.48, -0.34, 0.45),
      weldPose + Eigen::Vector3d(0.48, 0.34, -0.45),
      3.5,
      rgba(0.90, 0.90, 0.90)));
  world->addSimpleFrame(createLineMarker(
      "weld_lock_b",
      weldPose + Eigen::Vector3d(-0.48, 0.34, 0.45),
      weldPose + Eigen::Vector3d(0.48, -0.34, -0.45),
      3.5,
      rgba(0.90, 0.90, 0.90)));

  world->getConstraintSolver()->addConstraint(
      std::make_shared<constraint::WeldJointConstraint>(body.body));
}

simulation::WorldPtr createDynamicJointConstraintWorld()
{
  auto world = simulation::World::create("dynamic_joint_constraints");
  world->setGravity(Eigen::Vector3d::Zero());
  world->setTimeStep(1.0 / 240.0);

  addLaneGuide(world, "ball", -4.5, rgba(0.52, 0.18, 0.14, 0.75));
  addLaneGuide(world, "revolute", -1.5, rgba(0.62, 0.40, 0.14, 0.75));
  addLaneGuide(world, "cylindrical", 1.5, rgba(0.10, 0.36, 0.58, 0.75));
  addLaneGuide(world, "weld", 4.5, rgba(0.16, 0.48, 0.22, 0.75));

  addBallJointDemo(world, -4.5);
  addRevoluteJointDemo(world, -1.5);
  addCylindricalJointDemo(world, 1.5);
  addWeldJointDemo(world, 4.5);
  return world;
}

gui::Panel createDynamicJointConstraintPanel(const simulation::WorldPtr& world)
{
  gui::Panel panel;
  panel.title = "Dynamic Joint Constraints";
  panel.buildWithContext = [world](
                               gui::PanelBuilder& builder,
                               gui::PanelContext& context) {
    if (context.lifecycle != nullptr) {
      if (builder.button(context.lifecycle->paused ? "Resume" : "Pause")) {
        gui::togglePaused(*context.lifecycle);
      }
      builder.sameLine();
      if (builder.button("Step")) {
        gui::requestSingleStep(*context.lifecycle);
      }
    }

    builder.text("time: " + std::to_string(context.simulationTime));
    builder.text(
        "constraints: "
        + std::to_string(world->getConstraintSolver()->getNumConstraints()));
    builder.separator();
    builder.text("Ball: fixed anchor, free rotation");
    builder.text("Revolute: fixed anchor and axis, free spin");
    builder.text("Cylindrical: fixed axis, free slide and spin");
    builder.text("Weld: fixed relative pose");
  };
  return panel;
}

gui::OrbitCamera makeCamera()
{
  gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d(0.0, 0.0, 1.0);
  camera.yaw = -0.92;
  camera.pitch = 0.34;
  camera.distance = 8.4;
  return camera;
}

gui::RunOptions makeRunDefaults()
{
  gui::RunOptions options;
  options.width = 1120;
  options.height = 680;
  return options;
}

} // namespace

dart::gui::ApplicationOptions makeDynamicJointConstraintsScene()
{
  auto world = createDynamicJointConstraintWorld();

  gui::ApplicationOptions options;
  options.world = world;
  options.camera = makeCamera();
  options.runDefaults = makeRunDefaults();
  options.simulateWorld = true;
  options.panels.push_back(createDynamicJointConstraintPanel(world));
  return options;
}

} // namespace dart::examples::demos
