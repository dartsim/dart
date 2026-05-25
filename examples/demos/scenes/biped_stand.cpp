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
#include <dart/dynamics/degree_of_freedom.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <dart/io/read.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace dart::examples::demos {

namespace {

constexpr const char* kBipedStandWorldUri = "dart://sample/skel/fullbody1.skel";
constexpr const char* kBipedStandGroundName = "biped_stand_ground";
constexpr const char* kBipedStandSkeletonName = "biped_stand_biped";

dart::dynamics::DegreeOfFreedom* getRequiredDof(
    const dart::dynamics::SkeletonPtr& skeleton, const char* name)
{
  auto* dof = skeleton == nullptr ? nullptr : skeleton->getDof(name);
  if (dof == nullptr) {
    throw std::runtime_error(
        "biped_stand world is missing DOF: " + std::string(name));
  }
  return dof;
}

void setRequiredDofPosition(
    const dart::dynamics::SkeletonPtr& skeleton, const char* name, double value)
{
  getRequiredDof(skeleton, name)->setPosition(value);
}

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
        Eigen::Vector3d(0.30 + 0.34 * t, 0.48 + 0.18 * t, 0.78 - 0.30 * t));
  }

  if (auto* head = biped->getBodyNode("h_head")) {
    head->setColor(Eigen::Vector3d(0.86, 0.68, 0.50));
  }
  if (auto* spine = biped->getBodyNode("h_spine")) {
    spine->setColor(Eigen::Vector3d(0.34, 0.50, 0.76));
  }
}

dart::simulation::WorldPtr createBipedStandWorld()
{
  auto world = dart::io::readWorld(kBipedStandWorldUri);
  if (world == nullptr) {
    throw std::runtime_error(
        "Failed to load biped_stand world from "
        + std::string(kBipedStandWorldUri));
  }
  world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));

  auto ground = world->getSkeleton("ground skeleton");
  if (ground == nullptr) {
    throw std::runtime_error("biped_stand world is missing ground");
  }
  ground->setName(kBipedStandGroundName);
  if (auto* body = ground->getBodyNode("ground")) {
    body->setColor(Eigen::Vector3d(0.48, 0.50, 0.48));
  }

  auto biped = world->getSkeleton("fullbody1");
  if (biped == nullptr) {
    throw std::runtime_error("biped_stand world is missing fullbody1");
  }
  biped->setName(kBipedStandSkeletonName);

  setRequiredDofPosition(biped, "j_pelvis_rot_y", -0.20);
  setRequiredDofPosition(biped, "j_thigh_left_z", 0.15);
  setRequiredDofPosition(biped, "j_shin_left", -0.40);
  setRequiredDofPosition(biped, "j_heel_left_1", 0.25);
  setRequiredDofPosition(biped, "j_thigh_right_z", 0.15);
  setRequiredDofPosition(biped, "j_shin_right", -0.40);
  setRequiredDofPosition(biped, "j_heel_right_1", 0.25);
  setRequiredDofPosition(biped, "j_abdomen_2", 0.00);

  colorBiped(biped);
  return world;
}

class BipedStandController
{
public:
  BipedStandController(
      dart::simulation::WorldPtr world, dart::dynamics::SkeletonPtr biped)
    : mWorld(std::move(world)), mBiped(std::move(biped))
  {
    if (mWorld == nullptr || mBiped == nullptr) {
      throw std::runtime_error("biped_stand controller is missing world state");
    }

    mLeftHeel = mBiped->getBodyNode("h_heel_left");
    if (mLeftHeel == nullptr) {
      throw std::runtime_error("biped_stand world is missing h_heel_left");
    }

    mLeftFoot0 = getRequiredDof(mBiped, "j_heel_left_1")->getIndexInSkeleton();
    mLeftFoot1 = getRequiredDof(mBiped, "j_toe_left")->getIndexInSkeleton();
    mRightFoot0
        = getRequiredDof(mBiped, "j_heel_right_1")->getIndexInSkeleton();
    mRightFoot1 = getRequiredDof(mBiped, "j_toe_right")->getIndexInSkeleton();

    const Eigen::Index dofs = static_cast<Eigen::Index>(mBiped->getNumDofs());
    mKp = Eigen::MatrixXd::Identity(dofs, dofs);
    mKd = Eigen::MatrixXd::Identity(dofs, dofs);
    mTorques = Eigen::VectorXd::Zero(dofs);
    mDesiredDofs = mBiped->getPositions();

    for (Eigen::Index i = 0; i < std::min<Eigen::Index>(6, dofs); ++i) {
      mKp(i, i) = 0.0;
      mKd(i, i) = 0.0;
    }
    for (Eigen::Index i = 6; i < dofs; ++i) {
      mKp(i, i) = 400.0;
      mKd(i, i) = 40.0;
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

    const double dt = mWorld->getTimeStep();
    const Eigen::VectorXd positions = mBiped->getPositions();
    const Eigen::VectorXd velocities = mBiped->getVelocities();
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
        = mLeftHeel->getTransform() * Eigen::Vector3d(0.05, 0.0, 0.0);
    const double offset = centerOfMass[0] - centerOfPressure[0];
    if (offset < 0.1 && offset > 0.0) {
      applyAnkleStrategy(offset, 200.0, 100.0, 10.0);
    } else if (offset > -0.2 && offset < -0.05) {
      applyAnkleStrategy(offset, 2000.0, 100.0, 100.0);
    }

    for (Eigen::Index i = 0; i < std::min<Eigen::Index>(6, mTorques.size());
         ++i) {
      mTorques[i] = 0.0;
    }
    mBiped->setForces(mTorques);
  }

  void perturb(const Eigen::Vector3d& force, int frames = 100)
  {
    mForce = force;
    mImpulseDuration = frames;
  }

  int impulseDuration() const
  {
    return mImpulseDuration;
  }

private:
  void applyAnkleStrategy(
      double offset, double ankleGain, double toeGain, double derivativeGain)
  {
    const double correction = derivativeGain * (mPreviousOffset - offset);
    mTorques[static_cast<Eigen::Index>(mLeftFoot0)]
        += -ankleGain * offset + correction;
    mTorques[static_cast<Eigen::Index>(mLeftFoot1)]
        += -toeGain * offset + correction;
    mTorques[static_cast<Eigen::Index>(mRightFoot0)]
        += -ankleGain * offset + correction;
    mTorques[static_cast<Eigen::Index>(mRightFoot1)]
        += -toeGain * offset + correction;
    mPreviousOffset = offset;
  }

  dart::simulation::WorldPtr mWorld;
  dart::dynamics::SkeletonPtr mBiped;
  dart::dynamics::BodyNode* mLeftHeel = nullptr;
  Eigen::VectorXd mTorques;
  Eigen::VectorXd mDesiredDofs;
  Eigen::MatrixXd mKp;
  Eigen::MatrixXd mKd;
  std::size_t mLeftFoot0 = 0;
  std::size_t mLeftFoot1 = 0;
  std::size_t mRightFoot0 = 0;
  std::size_t mRightFoot1 = 0;
  double mPreviousOffset = 0.0;
  int mImpulseDuration = 0;
  Eigen::Vector3d mForce = Eigen::Vector3d::Zero();
};

dart::gui::Panel createBipedStandPanel(
    const std::shared_ptr<BipedStandController>& controller)
{
  dart::gui::Panel panel;
  panel.title = "Biped Stand";
  panel.buildWithContext = [controller](
                               dart::gui::PanelBuilder& builder,
                               dart::gui::PanelContext& context) {
    builder.text("SPD standing controller with panel perturbations");
    builder.text("Press space to start simulation.");
    builder.text("Keys: 1 +X, 2 -X, 3 +Z, 4 -Z push for 100 frames.");
    builder.text("1: Push robot with +50 along x-axis N for 100 frames");
    builder.text("2: Push robot with -50 along x-axis N for 100 frames");
    builder.text("3: Push robot with +50 along z-axis N for 100 frames");
    builder.text("4: Push robot with -50 along z-axis N for 100 frames");
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
    if (builder.button("Push +X")) {
      controller->perturb(Eigen::Vector3d(50.0, 0.0, 0.0));
    }
    builder.sameLine();
    if (builder.button("Push -X")) {
      controller->perturb(Eigen::Vector3d(-50.0, 0.0, 0.0));
    }
    if (builder.button("Push +Z")) {
      controller->perturb(Eigen::Vector3d(0.0, 0.0, 50.0));
    }
    builder.sameLine();
    if (builder.button("Push -Z")) {
      controller->perturb(Eigen::Vector3d(0.0, 0.0, -50.0));
    }
    builder.text(
        "impulse frames: " + std::to_string(controller->impulseDuration()));
    builder.text("time: " + std::to_string(context.simulationTime));
    builder.text("contacts: " + std::to_string(context.contactCount));
  };
  return panel;
}

dart::gui::OrbitCamera makeBipedStandCamera()
{
  dart::gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d::Zero();
  camera.yaw = 0.4636476090008061;
  camera.pitch = 0.7297276562269663;
  camera.distance = 4.5;
  return camera;
}

dart::gui::KeyboardAction makeBipedStandAction(
    std::shared_ptr<BipedStandController> controller,
    char key,
    std::string label,
    const Eigen::Vector3d& force)
{
  dart::gui::KeyboardAction action;
  action.label = std::move(label);
  action.shortcut = dart::gui::KeyboardShortcut::characterKey(key);
  action.callback = [controller = std::move(controller),
                     force](dart::gui::KeyboardActionContext&) {
    if (controller != nullptr) {
      controller->perturb(force);
    }
  };
  return action;
}

std::vector<dart::gui::KeyboardAction> createBipedStandKeyboardActions(
    const std::shared_ptr<BipedStandController>& controller)
{
  std::vector<dart::gui::KeyboardAction> actions;
  actions.push_back(makeBipedStandAction(
      controller, '1', "Push +X", Eigen::Vector3d(50.0, 0.0, 0.0)));
  actions.push_back(makeBipedStandAction(
      controller, '2', "Push -X", Eigen::Vector3d(-50.0, 0.0, 0.0)));
  actions.push_back(makeBipedStandAction(
      controller, '3', "Push +Z", Eigen::Vector3d(0.0, 0.0, 50.0)));
  actions.push_back(makeBipedStandAction(
      controller, '4', "Push -Z", Eigen::Vector3d(0.0, 0.0, -50.0)));
  return actions;
}

} // namespace

dart::gui::ApplicationOptions makeBipedStandScene()
{
  auto world = createBipedStandWorld();
  auto biped = world->getSkeleton(kBipedStandSkeletonName);
  auto controller = std::make_shared<BipedStandController>(world, biped);

  dart::gui::ApplicationOptions options;
  options.world = world;
  options.camera = makeBipedStandCamera();
  options.preStep = [controller]() {
    controller->preStep();
  };
  options.keyboardActions = createBipedStandKeyboardActions(controller);
  options.panels.push_back(createBipedStandPanel(controller));
  return options;
}

} // namespace dart::examples::demos
