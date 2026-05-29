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

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/frame.hpp>
#include <dart/dynamics/inertia.hpp>
#include <dart/dynamics/joint.hpp>
#include <dart/dynamics/line_segment_shape.hpp>
#include <dart/dynamics/revolute_joint.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/simple_frame.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <dart/math/constants.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <cmath>

namespace dart::examples::demos {

namespace {

struct Assembly
{
  dart::dynamics::SkeletonPtr skeleton;
  dart::dynamics::Joint* referenceJoint = nullptr;
  dart::dynamics::Joint* followerJoint = nullptr;
  dart::dynamics::BodyNode* referenceBody = nullptr;
  dart::dynamics::BodyNode* followerBody = nullptr;
};

struct LinkVisual
{
  dart::dynamics::SimpleFramePtr frame;
  std::shared_ptr<dart::dynamics::LineSegmentShape> shape;
  dart::dynamics::VisualAspect* visual = nullptr;
};

Eigen::Vector3d lerpColor(
    const Eigen::Vector3d& from, const Eigen::Vector3d& to, double amount)
{
  return from + (to - from) * amount;
}

std::string formatDegrees(double radians)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(2)
         << radians * 180.0 / dart::math::pi << " deg";
  return stream.str();
}

class CouplerController
{
public:
  struct PairRegistration
  {
    std::string label;
    bool usesCoupler = false;
    dart::dynamics::SkeletonPtr skeleton;
    dart::dynamics::Joint* referenceJoint = nullptr;
    dart::dynamics::Joint* followerJoint = nullptr;
    dart::dynamics::BodyNode* referenceBody = nullptr;
    dart::dynamics::BodyNode* followerBody = nullptr;
    LinkVisual link;
    double targetAngle = 0.0;
    double torqueLimit = 90.0;
    double proportionalGain = 320.0;
    double dampingGain = 25.0;
  };

  struct PairStatus
  {
    std::string label;
    bool usesCoupler = false;
    bool couplerEnabled = false;
    double targetAngle = 0.0;
    double referencePosition = 0.0;
    double followerPosition = 0.0;
    double desiredFollowerPosition = 0.0;
    double positionError = 0.0;
    double followerLowerLimit = 0.0;
    double followerUpperLimit = 0.0;
    bool followerAtLowerLimit = false;
    bool followerAtUpperLimit = false;
  };

  void addPair(PairRegistration registration)
  {
    PairData pair;
    pair.label = std::move(registration.label);
    pair.usesCoupler = registration.usesCoupler;
    pair.couplerEnabled
        = registration.usesCoupler && registration.followerJoint != nullptr
          && registration.followerJoint->isUsingCouplerConstraint();
    pair.skeleton = std::move(registration.skeleton);
    pair.referenceJoint = registration.referenceJoint;
    pair.followerJoint = registration.followerJoint;
    pair.referenceBody = registration.referenceBody;
    pair.followerBody = registration.followerBody;
    pair.link = std::move(registration.link);
    pair.targetAngle = registration.targetAngle;
    pair.torqueLimit = registration.torqueLimit;
    pair.proportionalGain = registration.proportionalGain;
    pair.dampingGain = registration.dampingGain;
    if (pair.skeleton != nullptr) {
      pair.initialPositions = pair.skeleton->getPositions();
    }

    mPairs.push_back(std::move(pair));
    refreshPairVisual(mPairs.back());
  }

  void update()
  {
    for (auto& pair : mPairs) {
      driveReferenceJoint(pair);
      refreshPairVisual(pair);
    }
  }

  void reset()
  {
    for (auto& pair : mPairs) {
      if (pair.skeleton == nullptr) {
        continue;
      }
      pair.skeleton->setPositions(pair.initialPositions);
      pair.skeleton->setVelocities(
          Eigen::VectorXd::Zero(pair.skeleton->getNumDofs()));
      pair.skeleton->clearExternalForces();
    }
    update();
  }

  std::vector<PairStatus> getStatuses() const
  {
    std::vector<PairStatus> statuses;
    statuses.reserve(mPairs.size());

    for (const auto& pair : mPairs) {
      PairStatus status;
      status.label = pair.label;
      status.usesCoupler = pair.usesCoupler;
      status.couplerEnabled = pair.couplerEnabled;
      status.targetAngle = pair.targetAngle;
      status.referencePosition = pair.referenceJoint != nullptr
                                     ? pair.referenceJoint->getPosition(0)
                                     : 0.0;
      status.followerPosition = pair.followerJoint != nullptr
                                    ? pair.followerJoint->getPosition(0)
                                    : 0.0;
      status.desiredFollowerPosition = computeDesiredFollowerPosition(pair);
      status.positionError
          = status.followerPosition - status.desiredFollowerPosition;
      status.followerLowerLimit
          = pair.followerJoint != nullptr
                ? pair.followerJoint->getPositionLowerLimit(0)
                : 0.0;
      status.followerUpperLimit
          = pair.followerJoint != nullptr
                ? pair.followerJoint->getPositionUpperLimit(0)
                : 0.0;
      status.followerAtLowerLimit
          = status.followerPosition - status.followerLowerLimit < 1e-3;
      status.followerAtUpperLimit
          = status.followerUpperLimit - status.followerPosition < 1e-3;
      statuses.push_back(std::move(status));
    }

    return statuses;
  }

private:
  struct PairData
  {
    std::string label;
    bool usesCoupler = false;
    bool couplerEnabled = false;
    dart::dynamics::SkeletonPtr skeleton;
    dart::dynamics::Joint* referenceJoint = nullptr;
    dart::dynamics::Joint* followerJoint = nullptr;
    dart::dynamics::BodyNode* referenceBody = nullptr;
    dart::dynamics::BodyNode* followerBody = nullptr;
    LinkVisual link;
    Eigen::VectorXd initialPositions;
    double targetAngle = 0.0;
    double torqueLimit = 90.0;
    double proportionalGain = 320.0;
    double dampingGain = 25.0;
  };

  static double computeDesiredFollowerPosition(const PairData& pair)
  {
    if (pair.followerJoint == nullptr) {
      return 0.0;
    }

    const auto& properties = pair.followerJoint->getMimicDofProperties();
    if (properties.empty() || properties[0].mReferenceJoint == nullptr) {
      return 0.0;
    }

    const auto& mimic = properties[0];
    return mimic.mReferenceJoint->getPosition(mimic.mReferenceDofIndex)
               * mimic.mMultiplier
           + mimic.mOffset;
  }

  void refreshPairVisual(PairData& pair)
  {
    if (pair.link.shape == nullptr || pair.referenceBody == nullptr
        || pair.followerBody == nullptr) {
      return;
    }

    pair.link.shape->setVertex(
        0, pair.referenceBody->getWorldTransform().translation());
    pair.link.shape->setVertex(
        1, pair.followerBody->getWorldTransform().translation());

    if (pair.link.visual == nullptr) {
      return;
    }

    const double error = pair.followerJoint != nullptr
                             ? pair.followerJoint->getPosition(0)
                                   - computeDesiredFollowerPosition(pair)
                             : 0.0;
    const double severity = std::min(1.0, std::abs(error) * 8.0);
    const Eigen::Vector3d color = pair.usesCoupler
                                      ? lerpColor(
                                            Eigen::Vector3d(0.6, 0.95, 0.4),
                                            Eigen::Vector3d(1.0, 0.6, 0.2),
                                            severity)
                                      : lerpColor(
                                            Eigen::Vector3d(0.7, 0.7, 0.85),
                                            Eigen::Vector3d(1.0, 0.4, 0.3),
                                            severity);
    pair.link.visual->setColor(color);
  }

  void driveReferenceJoint(PairData& pair)
  {
    if (pair.referenceJoint == nullptr) {
      return;
    }

    const double position = pair.referenceJoint->getPosition(0);
    const double velocity = pair.referenceJoint->getVelocity(0);
    const double rawTorque
        = pair.proportionalGain * (pair.targetAngle - position)
          - pair.dampingGain * velocity;
    const double torque
        = std::clamp(rawTorque, -pair.torqueLimit, pair.torqueLimit);
    pair.referenceJoint->setForce(0, torque);
  }

  std::vector<PairData> mPairs;
};

struct CouplerState
{
  dart::simulation::WorldPtr world;
  std::shared_ptr<CouplerController> controller;
};

void addRodVisual(dart::dynamics::BodyNode* body, const Eigen::Vector3d& color)
{
  auto shape = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(0.5, 0.05, 0.05));
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(shape);
  shapeNode->setRelativeTranslation(Eigen::Vector3d(0.25, 0.0, 0.0));
  shapeNode->getVisualAspect()->setColor(color);
}

Assembly createAssembly(
    const std::string& label,
    const Eigen::Vector3d& baseOffset,
    bool useCouplerConstraint,
    const Eigen::Vector3d& referenceColor,
    const Eigen::Vector3d& followerColor)
{
  auto skeleton = dart::dynamics::Skeleton::create(label + "_rig");

  dart::dynamics::WeldJoint::Properties baseJointProps;
  baseJointProps.mName = label + "_base_joint";
  baseJointProps.mT_ParentBodyToJoint = Eigen::Translation3d(baseOffset);

  dart::dynamics::BodyNode::Properties baseBodyProps;
  baseBodyProps.mName = label + "_base";
  dart::dynamics::Inertia baseInertia;
  baseInertia.setMass(0.1);
  baseInertia.setMoment(1e-4, 1e-4, 1e-4, 0.0, 0.0, 0.0);
  baseBodyProps.mInertia = baseInertia;

  auto basePair
      = skeleton->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(
          nullptr, baseJointProps, baseBodyProps);
  auto* baseBody = basePair.second;

  dart::dynamics::RevoluteJoint::Properties referenceJointProps;
  referenceJointProps.mName = label + "_reference_joint";
  referenceJointProps.mAxis = Eigen::Vector3d::UnitZ();
  referenceJointProps.mT_ParentBodyToJoint
      = Eigen::Translation3d(0.0, 0.15, 0.0);
  referenceJointProps.mT_ChildBodyToJoint
      = Eigen::Translation3d(-0.25, 0.0, 0.0);

  dart::dynamics::BodyNode::Properties referenceBodyProps;
  referenceBodyProps.mName = label + "_reference_body";
  dart::dynamics::Inertia referenceInertia;
  referenceInertia.setMass(1.0);
  referenceInertia.setMoment(0.02, 0.02, 0.03, 0.0, 0.0, 0.0);
  referenceBodyProps.mInertia = referenceInertia;

  auto referencePair
      = skeleton->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>(
          baseBody, referenceJointProps, referenceBodyProps);
  auto* referenceJoint = referencePair.first;
  auto* referenceBody = referencePair.second;
  referenceJoint->setActuatorType(dart::dynamics::Joint::FORCE);
  referenceJoint->setForceLowerLimit(0, -120.0);
  referenceJoint->setForceUpperLimit(0, 120.0);
  referenceJoint->setDampingCoefficient(0, 0.02);
  addRodVisual(referenceBody, referenceColor);

  dart::dynamics::RevoluteJoint::Properties followerJointProps;
  followerJointProps.mName = label + "_follower_joint";
  followerJointProps.mAxis = Eigen::Vector3d::UnitZ();
  followerJointProps.mT_ParentBodyToJoint
      = Eigen::Translation3d(0.0, -0.15, 0.0);
  followerJointProps.mT_ChildBodyToJoint
      = Eigen::Translation3d(-0.25, 0.0, 0.0);

  dart::dynamics::BodyNode::Properties followerBodyProps;
  followerBodyProps.mName = label + "_follower_body";
  dart::dynamics::Inertia followerInertia;
  followerInertia.setMass(1.0);
  followerInertia.setMoment(0.02, 0.02, 0.03, 0.0, 0.0, 0.0);
  followerBodyProps.mInertia = followerInertia;

  auto followerPair
      = skeleton->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>(
          baseBody, followerJointProps, followerBodyProps);
  auto* followerJoint = followerPair.first;
  auto* followerBody = followerPair.second;
  followerJoint->setActuatorType(dart::dynamics::Joint::MIMIC);
  followerJoint->setMimicJoint(referenceJoint, -1.0, 0.0);
  followerJoint->setUseCouplerConstraint(useCouplerConstraint);
  followerJoint->setForceLowerLimit(0, -120.0);
  followerJoint->setForceUpperLimit(0, 120.0);
  followerJoint->setDampingCoefficient(0, 0.02);
  followerJoint->setPositionLowerLimit(0, -0.35);
  followerJoint->setPositionUpperLimit(0, 0.35);
  followerJoint->setLimitEnforcement(true);
  addRodVisual(followerBody, followerColor);

  referenceJoint->setPosition(0, 0.0);
  followerJoint->setPosition(0, 0.0);

  return {skeleton, referenceJoint, followerJoint, referenceBody, followerBody};
}

void addLimitGuide(
    dart::simulation::World& world,
    const std::string& name,
    dart::dynamics::Joint* followerJoint,
    double angle,
    const Eigen::Vector3d& color)
{
  auto* revolute = dynamic_cast<dart::dynamics::RevoluteJoint*>(followerJoint);
  if (revolute == nullptr) {
    return;
  }

  auto frame = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World(), name);
  auto line = std::make_shared<dart::dynamics::LineSegmentShape>(0.05f);
  line->addVertex(Eigen::Vector3d::Zero());
  line->addVertex(Eigen::Vector3d::UnitX() * 0.65);
  line->addConnection(0, 1);
  frame->setShape(line);
  frame->createVisualAspect()->setColor(color);

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  if (const auto* parent = followerJoint->getParentBodyNode()) {
    transform = parent->getWorldTransform();
  }
  transform = transform * followerJoint->getTransformFromParentBodyNode();
  transform.rotate(Eigen::AngleAxisd(angle, revolute->getAxis()));
  transform.translate(Eigen::Vector3d(0.0, 0.0, 0.02));
  frame->setRelativeTransform(transform);
  world.addSimpleFrame(frame);
}

LinkVisual addPairLink(
    dart::simulation::World& world,
    const std::string& name,
    float width,
    const Eigen::Vector3d& color,
    const Assembly& assembly)
{
  auto frame = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World(), name);
  auto line = std::make_shared<dart::dynamics::LineSegmentShape>(width);
  line->addVertex(assembly.referenceBody->getWorldTransform().translation());
  line->addVertex(assembly.followerBody->getWorldTransform().translation());
  line->addConnection(0, 1);
  frame->setShape(line);
  auto* visual = frame->createVisualAspect();
  visual->setColor(color);
  world.addSimpleFrame(frame);
  return {frame, line, visual};
}

void addSceneGrid(dart::simulation::World& world)
{
  auto frame = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World(), "coupler_constraint_grid");
  auto grid = std::make_shared<dart::dynamics::LineSegmentShape>(1.0f);
  constexpr int cellCount = 25;
  constexpr double step = 0.05;
  constexpr double halfExtent = cellCount * step;
  for (int i = -cellCount; i <= cellCount; ++i) {
    const double coordinate = static_cast<double>(i) * step;
    const auto xStart
        = grid->addVertex(Eigen::Vector3d(-halfExtent, coordinate, -0.02));
    grid->addVertex(Eigen::Vector3d(halfExtent, coordinate, -0.02), xStart);
    const auto yStart
        = grid->addVertex(Eigen::Vector3d(coordinate, -halfExtent, -0.02));
    grid->addVertex(Eigen::Vector3d(coordinate, halfExtent, -0.02), yStart);
  }
  frame->setShape(grid);
  frame->createVisualAspect()->setRGBA(Eigen::Vector4d(0.42, 0.42, 0.42, 0.45));
  world.addSimpleFrame(frame);
}

std::shared_ptr<CouplerState> createCouplerState()
{
  constexpr double targetAngle = 45.0 * dart::math::pi / 180.0;

  auto state = std::make_shared<CouplerState>();
  state->world = dart::simulation::World::create("coupler_constraint");
  state->world->setGravity(Eigen::Vector3d::Zero());
  state->world->setTimeStep(1e-3);

  auto coupler = createAssembly(
      "coupler",
      Eigen::Vector3d(-0.45, 0.0, 0.0),
      true,
      Eigen::Vector3d(0.85, 0.35, 0.25),
      Eigen::Vector3d(0.25, 0.58, 0.92));
  auto motor = createAssembly(
      "motor",
      Eigen::Vector3d(0.45, 0.0, 0.0),
      false,
      Eigen::Vector3d(0.68, 0.32, 0.70),
      Eigen::Vector3d(0.25, 0.75, 0.70));

  state->world->addSkeleton(coupler.skeleton);
  state->world->addSkeleton(motor.skeleton);

  addLimitGuide(
      *state->world,
      "coupler_lower_limit",
      coupler.followerJoint,
      coupler.followerJoint->getPositionLowerLimit(0),
      Eigen::Vector3d(0.92, 0.35, 0.35));
  addLimitGuide(
      *state->world,
      "coupler_upper_limit",
      coupler.followerJoint,
      coupler.followerJoint->getPositionUpperLimit(0),
      Eigen::Vector3d(0.35, 0.92, 0.35));
  addLimitGuide(
      *state->world,
      "motor_lower_limit",
      motor.followerJoint,
      motor.followerJoint->getPositionLowerLimit(0),
      Eigen::Vector3d(0.92, 0.35, 0.35));
  addLimitGuide(
      *state->world,
      "motor_upper_limit",
      motor.followerJoint,
      motor.followerJoint->getPositionUpperLimit(0),
      Eigen::Vector3d(0.35, 0.92, 0.35));
  auto couplerLink = addPairLink(
      *state->world,
      "coupler_link",
      0.06f,
      Eigen::Vector3d(0.95, 0.95, 0.2),
      coupler);
  auto motorLink = addPairLink(
      *state->world,
      "motor_link",
      0.04f,
      Eigen::Vector3d(0.75, 0.75, 0.9),
      motor);
  addSceneGrid(*state->world);

  state->controller = std::make_shared<CouplerController>();
  state->controller->addPair({
      .label = "Coupler pair (left)",
      .usesCoupler = true,
      .skeleton = coupler.skeleton,
      .referenceJoint = coupler.referenceJoint,
      .followerJoint = coupler.followerJoint,
      .referenceBody = coupler.referenceBody,
      .followerBody = coupler.followerBody,
      .link = std::move(couplerLink),
      .targetAngle = targetAngle,
      .torqueLimit = 90.0,
      .proportionalGain = 320.0,
      .dampingGain = 25.0,
  });
  state->controller->addPair({
      .label = "Mimic motor pair (right)",
      .usesCoupler = false,
      .skeleton = motor.skeleton,
      .referenceJoint = motor.referenceJoint,
      .followerJoint = motor.followerJoint,
      .referenceBody = motor.referenceBody,
      .followerBody = motor.followerBody,
      .link = std::move(motorLink),
      .targetAngle = targetAngle,
      .torqueLimit = 90.0,
      .proportionalGain = 320.0,
      .dampingGain = 25.0,
  });

  return state;
}

dart::gui::Panel createControlsPanel(const std::shared_ptr<CouplerState>& state)
{
  dart::gui::Panel panel;
  panel.title = "Coupler Constraint";
  panel.buildWithContext = [state](
                               dart::gui::PanelBuilder& builder,
                               dart::gui::PanelContext& context) {
    builder.text(
        "Left rig: CouplerConstraint (bilateral). Right rig: Mimic motor "
        "(servo).");
    builder.text(
        "Both reference joints chase a 45 deg command while followers are "
        "limited to +/-20 deg.");
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
    if (builder.button("Reset")) {
      state->controller->reset();
    }
    builder.text("Press r to reset both rigs.");
    builder.separator();

    for (const auto& status : state->controller->getStatuses()) {
      builder.text(status.label);
      builder.text(
          "Constraint: "
          + std::string(
              status.usesCoupler ? "Coupler (bilateral)"
                                 : "Mimic motor (servo)"));
      builder.text("Reference target: " + formatDegrees(status.targetAngle));
      builder.text("Reference: " + formatDegrees(status.referencePosition));
      builder.text("Follower: " + formatDegrees(status.followerPosition));
      builder.text(
          "Follower limits: [" + formatDegrees(status.followerLowerLimit) + ", "
          + formatDegrees(status.followerUpperLimit) + "]");
      builder.text(
          "Desired mimic: " + formatDegrees(status.desiredFollowerPosition));
      builder.text("Error: " + formatDegrees(status.positionError));
      if (status.followerAtLowerLimit) {
        builder.text("Lower limit engaged");
      } else if (status.followerAtUpperLimit) {
        builder.text("Upper limit engaged");
      }
      builder.separator();
    }
    builder.text("time: " + std::to_string(context.simulationTime));
  };
  return panel;
}

std::vector<dart::gui::KeyboardAction> createCouplerKeyboardActions(
    const std::shared_ptr<CouplerController>& controller)
{
  dart::gui::KeyboardAction reset;
  reset.label = "reset both rigs";
  reset.shortcut = dart::gui::KeyboardShortcut::characterKey('r');
  reset.callback = [controller](dart::gui::KeyboardActionContext&) {
    controller->reset();
    std::cout << "Reset both rigs to their initial configuration\n";
  };
  return {std::move(reset)};
}

dart::gui::OrbitCamera makeCouplerCamera()
{
  dart::gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d(0.4, 0.0, 0.2);
  camera.yaw = 0.9380474917927134;
  camera.pitch = 0.49357845291184255;
  camera.distance = 2.1118712081942874;
  return camera;
}

} // namespace

dart::gui::ApplicationOptions makeCouplerConstraintScene()
{
  auto state = createCouplerState();

  dart::gui::ApplicationOptions options;
  options.world = state->world;
  options.camera = makeCouplerCamera();
  options.preStep = [controller = state->controller]() {
    controller->update();
  };
  options.panels.push_back(createControlsPanel(state));
  options.keyboardActions = createCouplerKeyboardActions(state->controller);
  return options;
}

} // namespace dart::examples::demos
