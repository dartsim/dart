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

#include <dart/gui/panel.hpp>
#include <dart/gui/viewer.hpp>

#include <dart/simulation/world.hpp>

#include <dart/constraint/constraint_solver.hpp>
#include <dart/constraint/weld_joint_constraint.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <dart/io/read.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dart::examples::demos {

namespace {

constexpr const char* kJointConstraintsWorldUri
    = "dart://sample/skel/fullbody1.skel";
constexpr const char* kGroundSkeletonName = "ground skeleton";
constexpr const char* kBipedSkeletonName = "fullbody1";

dart::simulation::WorldPtr createJointConstraintsWorld()
{
  auto world = dart::io::readWorld(kJointConstraintsWorldUri);
  if (world == nullptr) {
    throw std::runtime_error(
        "Failed to load joint_constraints world from "
        + std::string(kJointConstraintsWorldUri));
  }
  // The fullbody world is authored Y-up; reorient to the canonical Z-up
  // convention. The PD controller tracks joint angles and the balance feedback
  // uses the sagittal (X) center-of-mass/pressure offset, both invariant under
  // the rigid RotX(+90deg) rotation, so the biped still balances.
  reorientWorldToZUp(world);

  auto ground = world->getSkeleton("ground skeleton");
  if (ground == nullptr) {
    throw std::runtime_error("joint_constraints world is missing ground");
  }

  auto biped = world->getSkeleton(kBipedSkeletonName);
  if (biped == nullptr) {
    throw std::runtime_error("joint_constraints world is missing fullbody1");
  }

  const std::vector<std::size_t> genCoordIds{
      1,  // global orientation y
      4,  // global position y
      6,  // left hip
      9,  // left knee
      10, // left ankle
      13, // right hip
      16, // right knee
      17, // right ankle
      21, // lower back
  };
  Eigen::VectorXd initConfig(9);
  initConfig << -0.1, 0.2, 0.2, -0.5, 0.3, 0.2, -0.5, 0.3, -0.1;
  biped->setPositions(genCoordIds, initConfig);
  return world;
}

class JointConstraintsController
{
public:
  JointConstraintsController(
      dart::simulation::WorldPtr world, dart::dynamics::SkeletonPtr biped)
    : mWorld(std::move(world)), mBiped(std::move(biped))
  {
    if (mWorld == nullptr || mBiped == nullptr) {
      throw std::runtime_error(
          "joint_constraints controller is missing world state");
    }

    mHeelLeft = mBiped->getBodyNode("h_heel_left");
    if (mHeelLeft == nullptr) {
      throw std::runtime_error(
          "joint_constraints world is missing h_heel_left");
    }

    const Eigen::Index dofs = static_cast<Eigen::Index>(mBiped->getNumDofs());
    mKp = Eigen::MatrixXd::Identity(dofs, dofs);
    mKd = Eigen::MatrixXd::Identity(dofs, dofs);
    mTorques = Eigen::VectorXd::Zero(dofs);
    mDesiredDofs = mBiped->getPositions();

    for (Eigen::Index i = 0; i < std::min<Eigen::Index>(6, dofs); ++i) {
      mKp(i, i) = 0.0;
      mKd(i, i) = 0.0;
    }
    for (Eigen::Index i = 6; i < std::min<Eigen::Index>(22, dofs); ++i) {
      mKp(i, i) = 200.0;
      mKd(i, i) = 100.0;
    }
    for (Eigen::Index i = 22; i < dofs; ++i) {
      mKp(i, i) = 20.0;
      mKd(i, i) = 10.0;
    }
  }

  void preStep()
  {
    if (auto* spine = mBiped->getBodyNode("h_spine")) {
      spine->addExtForce(mForce);
    }
    if (mImpulseDuration > 0) {
      --mImpulseDuration;
    }
    if (mImpulseDuration <= 0) {
      mImpulseDuration = 0;
      mForce.setZero();
    }

    computeTorques(mBiped->getPositions(), mBiped->getVelocities());
    mBiped->setForces(mTorques);
  }

  void perturb(
      const Eigen::Vector3d& force, const char* message, int frames = 100)
  {
    mForce = force;
    mImpulseDuration = frames;
    if (message != nullptr) {
      std::cout << message << std::endl;
    }
  }

  void toggleHarness()
  {
    auto* solver = mWorld->getConstraintSolver();
    auto* pelvis = mBiped->getBodyNode("h_pelvis");
    if (solver == nullptr || pelvis == nullptr) {
      return;
    }

    if (mHarnessOn) {
      if (mWeldJoint != nullptr) {
        solver->removeConstraint(mWeldJoint);
      }
      mWeldJoint.reset();
      mHarnessOn = false;
      std::cout << "Harness off" << std::endl;
      return;
    }

    mWeldJoint
        = std::make_shared<dart::constraint::WeldJointConstraint>(pelvis);
    solver->addConstraint(mWeldJoint);
    mHarnessOn = true;
    std::cout << "Harness on" << std::endl;
  }

  bool harnessOn() const
  {
    return mHarnessOn;
  }

  int impulseDuration() const
  {
    return mImpulseDuration;
  }

private:
  void computeTorques(
      const Eigen::VectorXd& positions, const Eigen::VectorXd& velocities)
  {
    const double dt = mWorld->getTimeStep();
    const Eigen::VectorXd constraintForces = mBiped->getConstraintForces();
    const Eigen::MatrixXd inverseMass
        = (mBiped->getMassMatrix() + mKd * dt).inverse();
    const Eigen::VectorXd proportional
        = -mKp * (positions + velocities * dt - mDesiredDofs);
    const Eigen::VectorXd derivative = -mKd * velocities;
    const Eigen::VectorXd acceleration
        = inverseMass
          * (-mBiped->getCoriolisAndGravityForces() + proportional + derivative
             + constraintForces);
    mTorques = proportional + derivative - mKd * acceleration * dt;

    const Eigen::Vector3d centerOfMass = mBiped->getCOM();
    const Eigen::Vector3d centerOfPressure
        = mHeelLeft->getTransform() * Eigen::Vector3d(0.05, 0.0, 0.0);
    const Eigen::Vector2d offset(
        centerOfMass[0] - centerOfPressure[0],
        centerOfMass[1] - centerOfPressure[1]);
    if (mTorques.size() > 26 && offset[0] < 0.1) {
      const double sagittalOffset = centerOfMass[0] - centerOfPressure[0];
      constexpr double kAnkleHipGain = 20.0;
      constexpr double kBackGain = 10.0;
      constexpr double kDerivativeGain = 100.0;
      const double correction
          = kDerivativeGain * (mPreviousSagittalOffset - sagittalOffset);
      mTorques[17] += -kAnkleHipGain * sagittalOffset + correction;
      mTorques[25] += -kBackGain * sagittalOffset + correction;
      mTorques[19] += -kAnkleHipGain * sagittalOffset + correction;
      mTorques[26] += -kBackGain * sagittalOffset + correction;
      mPreviousSagittalOffset = sagittalOffset;
    }

    for (Eigen::Index i = 0; i < std::min<Eigen::Index>(6, mTorques.size());
         ++i) {
      mTorques[i] = 0.0;
    }
  }

  dart::simulation::WorldPtr mWorld;
  dart::dynamics::SkeletonPtr mBiped;
  dart::dynamics::BodyNode* mHeelLeft = nullptr;
  double mPreviousSagittalOffset = 0.0;
  Eigen::VectorXd mTorques;
  Eigen::VectorXd mDesiredDofs;
  Eigen::MatrixXd mKp;
  Eigen::MatrixXd mKd;
  int mImpulseDuration = 0;
  Eigen::Vector3d mForce = Eigen::Vector3d::Zero();
  bool mHarnessOn = false;
  std::shared_ptr<dart::constraint::WeldJointConstraint> mWeldJoint;
};

dart::gui::OrbitCamera makeJointConstraintsCamera()
{
  dart::gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d::Zero();
  camera.up = Eigen::Vector3d::UnitZ();
  camera.yaw = 0.5404195002705842;
  camera.pitch = 0.4758822496604165;
  camera.distance = 6.557438524302;
  return camera;
}

dart::gui::KeyboardAction makePerturbAction(
    const std::shared_ptr<JointConstraintsController>& controller,
    char key,
    std::string label,
    const Eigen::Vector3d& force,
    const char* message)
{
  dart::gui::KeyboardAction action;
  action.label = std::move(label);
  action.shortcut = dart::gui::KeyboardShortcut::characterKey(key);
  action.callback
      = [controller, force, message](dart::gui::KeyboardActionContext&) {
          controller->perturb(force, message);
        };
  return action;
}

std::vector<dart::gui::KeyboardAction> createJointConstraintsKeyboardActions(
    const std::shared_ptr<JointConstraintsController>& controller)
{
  std::vector<dart::gui::KeyboardAction> actions;
  actions.reserve(5);
  actions.push_back(makePerturbAction(
      controller,
      '1',
      "Apply joint-constraints forward perturbation",
      Eigen::Vector3d(40.0, 0.0, 0.0),
      "push forward"));
  actions.push_back(makePerturbAction(
      controller,
      '2',
      "Apply joint-constraints backward perturbation",
      Eigen::Vector3d(-40.0, 0.0, 0.0),
      "push backward"));
  actions.push_back(makePerturbAction(
      controller,
      '3',
      "Apply joint-constraints right perturbation",
      Eigen::Vector3d(0.0, 50.0, 0.0),
      "push right"));
  actions.push_back(makePerturbAction(
      controller,
      '4',
      "Apply joint-constraints left perturbation",
      Eigen::Vector3d(0.0, -50.0, 0.0),
      "push left"));

  dart::gui::KeyboardAction toggleHarness;
  toggleHarness.label = "Toggle joint-constraints harness";
  toggleHarness.shortcut = dart::gui::KeyboardShortcut::characterKey('h');
  toggleHarness.callback = [controller](dart::gui::KeyboardActionContext&) {
    controller->toggleHarness();
  };
  actions.push_back(std::move(toggleHarness));
  return actions;
}

dart::gui::Panel createJointConstraintsPanel(
    const std::shared_ptr<JointConstraintsController>& controller)
{
  dart::gui::Panel panel;
  panel.title = "Joint Constraints";
  panel.buildWithContext = [controller](
                               dart::gui::PanelBuilder& builder,
                               dart::gui::PanelContext& context) {
    builder.text("Balanced fullbody controller with constraint perturbations");
    builder.text("'1'-'4': programmed perturbations");
    builder.text("'h': toggle harness on/off");
    builder.text("space bar: simulation on/off");
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
    if (builder.button("Push forward")) {
      controller->perturb(Eigen::Vector3d(40.0, 0.0, 0.0), "push forward");
    }
    builder.sameLine();
    if (builder.button("Push backward")) {
      controller->perturb(Eigen::Vector3d(-40.0, 0.0, 0.0), "push backward");
    }
    if (builder.button("Push right")) {
      controller->perturb(Eigen::Vector3d(0.0, 50.0, 0.0), "push right");
    }
    builder.sameLine();
    if (builder.button("Push left")) {
      controller->perturb(Eigen::Vector3d(0.0, -50.0, 0.0), "push left");
    }
    if (builder.button(
            controller->harnessOn() ? "Remove harness" : "Add harness")) {
      controller->toggleHarness();
    }
    builder.text(
        "harness: " + std::string(controller->harnessOn() ? "on" : "off"));
    builder.text(
        "impulse frames: " + std::to_string(controller->impulseDuration()));
    builder.text("time: " + std::to_string(context.simulationTime));
    builder.text("contacts: " + std::to_string(context.contactCount));
  };
  return panel;
}

} // namespace

dart::gui::ApplicationOptions makeJointConstraintsScene()
{
  auto world = createJointConstraintsWorld();
  auto biped = world->getSkeleton(kBipedSkeletonName);
  auto controller = std::make_shared<JointConstraintsController>(world, biped);

  dart::gui::ApplicationOptions options;
  options.world = world;
  options.camera = makeJointConstraintsCamera();
  options.preStep = [controller]() {
    controller->preStep();
  };
  options.keyboardActions = createJointConstraintsKeyboardActions(controller);
  options.panels.push_back(createJointConstraintsPanel(controller));
  return options;
}

} // namespace dart::examples::demos
