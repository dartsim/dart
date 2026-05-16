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
 *   INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *   SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 *   OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 *   EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <dart/gui/application.hpp>
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
#include <vector>

namespace {

constexpr const char* kJointConstraintsWorldUri
    = "dart://sample/skel/fullbody1.skel";
constexpr const char* kGroundSkeletonName = "joint_constraints_ground";
constexpr const char* kBipedSkeletonName = "joint_constraints_biped";

void colorBiped(const dart::dynamics::SkeletonPtr& biped)
{
  const std::size_t numBodies = biped->getNumBodyNodes();
  for (std::size_t i = 0; i < numBodies; ++i) {
    auto* body = biped->getBodyNode(i);
    if (body == nullptr) {
      continue;
    }
    const double t = numBodies <= 1 ? 0.0
                                    : static_cast<double>(i)
                                          / static_cast<double>(numBodies - 1);
    body->setColor(
        Eigen::Vector3d(0.24 + 0.34 * t, 0.52 - 0.20 * t, 0.34 + 0.24 * t));
  }

  if (auto* head = biped->getBodyNode("h_head")) {
    head->setColor(Eigen::Vector3d(0.84, 0.68, 0.50));
  }
  if (auto* spine = biped->getBodyNode("h_spine")) {
    spine->setColor(Eigen::Vector3d(0.30, 0.58, 0.34));
  }
}

dart::simulation::WorldPtr createJointConstraintsWorld()
{
  auto world = dart::io::readWorld(kJointConstraintsWorldUri);
  if (world == nullptr) {
    throw std::runtime_error(
        "Failed to load joint_constraints world from "
        + std::string(kJointConstraintsWorldUri));
  }
  world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));

  auto ground = world->getSkeleton("ground skeleton");
  if (ground == nullptr) {
    throw std::runtime_error("joint_constraints world is missing ground");
  }
  ground->setName(kGroundSkeletonName);
  if (auto* body = ground->getBodyNode("ground")) {
    body->setColor(Eigen::Vector3d(0.42, 0.46, 0.44));
  }

  auto biped = world->getSkeleton("fullbody1");
  if (biped == nullptr) {
    throw std::runtime_error("joint_constraints world is missing fullbody1");
  }
  biped->setName(kBipedSkeletonName);

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

  colorBiped(biped);
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

  void perturb(const Eigen::Vector3d& force, int frames = 100)
  {
    mForce = force;
    mImpulseDuration = frames;
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
      return;
    }

    mWeldJoint
        = std::make_shared<dart::constraint::WeldJointConstraint>(pelvis);
    solver->addConstraint(mWeldJoint);
    mHarnessOn = true;
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
        centerOfMass[2] - centerOfPressure[2]);
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

dart::gui::Panel createJointConstraintsPanel(
    const std::shared_ptr<JointConstraintsController>& controller)
{
  dart::gui::Panel panel;
  panel.title = "Joint Constraints";
  panel.buildWithContext = [controller](
                               dart::gui::PanelBuilder& builder,
                               dart::gui::PanelContext& context) {
    builder.text("Balanced fullbody controller with constraint perturbations");
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
      controller->perturb(Eigen::Vector3d(40.0, 0.0, 0.0));
    }
    builder.sameLine();
    if (builder.button("Push backward")) {
      controller->perturb(Eigen::Vector3d(-40.0, 0.0, 0.0));
    }
    if (builder.button("Push right")) {
      controller->perturb(Eigen::Vector3d(0.0, 0.0, 50.0));
    }
    builder.sameLine();
    if (builder.button("Push left")) {
      controller->perturb(Eigen::Vector3d(0.0, 0.0, -50.0));
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

int main(int argc, char* argv[])
{
  try {
    auto world = createJointConstraintsWorld();
    auto biped = world->getSkeleton(kBipedSkeletonName);
    auto controller
        = std::make_shared<JointConstraintsController>(world, biped);

    dart::gui::ApplicationOptions options;
    options.world = world;
    options.preStep = [controller]() {
      controller->preStep();
    };
    options.panels.push_back(createJointConstraintsPanel(controller));

    return dart::gui::runApplication(argc, argv, options);
  } catch (const std::exception& e) {
    std::cerr << "joint_constraints: " << e.what() << "\n";
    return 1;
  }
}
