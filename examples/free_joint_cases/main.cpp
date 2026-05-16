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

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/shape_frame.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <dart/math/constants.hpp>
#include <dart/math/geometry.hpp>
#include <dart/math/math_types.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <charconv>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include <cmath>

namespace {

constexpr std::size_t kCaseCount = 5;
constexpr const char* kActiveSkeletonPrefix = "free_joint_case_";
constexpr const char* kReferenceSkeletonPrefix = "free_joint_reference_";

struct CaseMetrics
{
  double translationalBlockError{0.0};
  double worldJacobianMaxError{0.0};
  double worldJacobianClassicDerivMaxError{0.0};
  double relativeJacobianDotMaxError{0.0};
};

enum class GroundTruthModel
{
  TorqueFreeRigidBody,
  ConstantWorldTwist,
};

struct FreeJointCasesConfig
{
  double numericDt{1e-6};
  double simulationDt{1e-3};
  GroundTruthModel groundTruthModel{GroundTruthModel::TorqueFreeRigidBody};
  bool useSphericalInertia{false};
  int torqueFreeSubsteps{10};
};

struct TorqueFreeState
{
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d omegaBody{Eigen::Vector3d::Zero()};
};

struct FreeJointFixture
{
  std::string name;
  dart::dynamics::SkeletonPtr skeleton;
  dart::dynamics::FreeJoint* joint = nullptr;
  dart::dynamics::BodyNode* body = nullptr;
  dart::dynamics::SkeletonPtr referenceSkeleton;
  dart::dynamics::FreeJoint* referenceJoint = nullptr;
  dart::dynamics::ShapeNode* referenceShapeNode = nullptr;
  Eigen::Matrix3d anisotropicInertia{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d sphericalInertia{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d inertia{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d inertiaInv{Eigen::Matrix3d::Identity()};
  TorqueFreeState torqueFreeState;
  Eigen::Vector6d initialPositions{Eigen::Vector6d::Zero()};
  Eigen::Vector6d initialVelocities{Eigen::Vector6d::Zero()};
  double initialEnergy{0.0};
  Eigen::Vector3d jacobianOffset{Eigen::Vector3d(0.1, 0.0, 0.0)};
  CaseMetrics metrics;
};

struct TorqueFreeDeriv
{
  Eigen::Vector4d orientationDot{Eigen::Vector4d::Zero()};
  Eigen::Vector3d omegaBodyDot{Eigen::Vector3d::Zero()};
};

std::string formatDouble(double value, int precision = 3)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

Eigen::Vector3d computeOmegaBodyDot(
    const Eigen::Matrix3d& inertia,
    const Eigen::Matrix3d& inertiaInv,
    const Eigen::Vector3d& omegaBody)
{
  return -inertiaInv * (omegaBody.cross(inertia * omegaBody));
}

Eigen::Vector4d computeQuaternionCoeffDot(
    const Eigen::Quaterniond& orientation, const Eigen::Vector3d& omegaBody)
{
  const Eigen::Quaterniond omegaQuat(
      0.0, omegaBody.x(), omegaBody.y(), omegaBody.z());
  Eigen::Quaterniond qdot = orientation * omegaQuat;
  qdot.coeffs() *= 0.5;
  return qdot.coeffs();
}

TorqueFreeDeriv evaluateTorqueFreeDeriv(
    const TorqueFreeState& state,
    const Eigen::Matrix3d& inertia,
    const Eigen::Matrix3d& inertiaInv)
{
  TorqueFreeDeriv deriv;
  deriv.orientationDot
      = computeQuaternionCoeffDot(state.orientation, state.omegaBody);
  deriv.omegaBodyDot
      = computeOmegaBodyDot(inertia, inertiaInv, state.omegaBody);
  return deriv;
}

void integrateTorqueFreeStep(
    TorqueFreeState& state,
    const Eigen::Matrix3d& inertia,
    const Eigen::Matrix3d& inertiaInv,
    double dt)
{
  if (!(dt > 0.0)) {
    return;
  }

  const TorqueFreeDeriv k1
      = evaluateTorqueFreeDeriv(state, inertia, inertiaInv);

  TorqueFreeState s2 = state;
  s2.orientation.coeffs() += 0.5 * dt * k1.orientationDot;
  s2.omegaBody += 0.5 * dt * k1.omegaBodyDot;
  s2.orientation.normalize();
  const TorqueFreeDeriv k2 = evaluateTorqueFreeDeriv(s2, inertia, inertiaInv);

  TorqueFreeState s3 = state;
  s3.orientation.coeffs() += 0.5 * dt * k2.orientationDot;
  s3.omegaBody += 0.5 * dt * k2.omegaBodyDot;
  s3.orientation.normalize();
  const TorqueFreeDeriv k3 = evaluateTorqueFreeDeriv(s3, inertia, inertiaInv);

  TorqueFreeState s4 = state;
  s4.orientation.coeffs() += dt * k3.orientationDot;
  s4.omegaBody += dt * k3.omegaBodyDot;
  s4.orientation.normalize();
  const TorqueFreeDeriv k4 = evaluateTorqueFreeDeriv(s4, inertia, inertiaInv);

  state.orientation.coeffs()
      += (dt / 6.0)
         * (k1.orientationDot + 2.0 * k2.orientationDot
            + 2.0 * k3.orientationDot + k4.orientationDot);
  state.omegaBody += (dt / 6.0)
                     * (k1.omegaBodyDot + 2.0 * k2.omegaBodyDot
                        + 2.0 * k3.omegaBodyDot + k4.omegaBodyDot);
  state.orientation.normalize();
}

Eigen::Isometry3d computeConstantTwistTransform(
    const FreeJointFixture& fixture, double time)
{
  const Eigen::Isometry3d initial
      = dart::dynamics::FreeJoint::convertToTransform(fixture.initialPositions);
  Eigen::Isometry3d expected = initial;
  expected.linear()
      = dart::math::expMapRot(fixture.initialVelocities.head<3>() * time)
        * initial.linear();
  expected.translation()
      = initial.translation() + fixture.initialVelocities.tail<3>() * time;
  return expected;
}

CaseMetrics computeMetrics(const FreeJointFixture& fixture, double dt)
{
  using dart::dynamics::Frame;
  using dart::dynamics::FreeJoint;

  CaseMetrics metrics;

  const dart::math::Jacobian worldJacobian = fixture.body->getWorldJacobian();
  const Eigen::Matrix3d translationBlock
      = worldJacobian.bottomRows<3>().rightCols<3>();
  metrics.translationalBlockError
      = (translationBlock - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff();

  auto clone = fixture.skeleton->cloneSkeleton(fixture.name + "_clone");
  clone->setPositions(fixture.skeleton->getPositions());
  clone->setVelocities(fixture.skeleton->getVelocities());

  auto* cloneBody = clone->getBodyNode(0);
  auto* cloneJoint = dynamic_cast<FreeJoint*>(clone->getJoint(0));
  const Eigen::Vector6d savedPositions = cloneJoint->getPositions();
  const Eigen::Vector6d savedVelocities = cloneJoint->getVelocities();
  const Eigen::Vector3d offset = fixture.jacobianOffset;
  const dart::math::Jacobian jacobian = cloneBody->getWorldJacobian(offset);

  double maxJacobianError = 0.0;
  for (int i = 0; i < 6; ++i) {
    Eigen::Vector6d velocities = Eigen::Vector6d::Zero();
    velocities[i] = 1.0;
    cloneJoint->setVelocities(velocities);

    const Eigen::Isometry3d transform0 = cloneBody->getWorldTransform();
    const Eigen::Matrix3d rotation0 = transform0.linear();
    const Eigen::Vector3d position0 = transform0 * offset;

    clone->integratePositions(dt);

    const Eigen::Isometry3d transform1 = cloneBody->getWorldTransform();
    const Eigen::Matrix3d rotation1 = transform1.linear();
    const Eigen::Vector3d position1 = transform1 * offset;

    Eigen::Vector6d numeric;
    numeric.head<3>()
        = dart::math::logMap(rotation1 * rotation0.transpose()) / dt;
    numeric.tail<3>() = (position1 - position0) / dt;

    maxJacobianError = std::max(
        maxJacobianError, (numeric - jacobian.col(i)).cwiseAbs().maxCoeff());

    cloneJoint->setPositions(savedPositions);
  }
  metrics.worldJacobianMaxError = maxJacobianError;

  cloneJoint->setPositions(savedPositions);
  cloneJoint->setVelocities(savedVelocities);

  const Eigen::Vector6d q0 = cloneJoint->getPositions();
  const Eigen::Vector6d qdot = cloneJoint->getVelocities();
  const dart::math::Jacobian jacobianDeriv
      = cloneBody->getJacobianClassicDeriv(offset);

  clone->integratePositions(dt);
  const Eigen::Vector6d velocityPlus
      = cloneBody->getSpatialVelocity(offset, Frame::World(), Frame::World());
  cloneJoint->setPositions(q0);

  clone->integratePositions(-dt);
  const Eigen::Vector6d velocityMinus
      = cloneBody->getSpatialVelocity(offset, Frame::World(), Frame::World());
  cloneJoint->setPositions(q0);

  const Eigen::Vector6d numeric = (velocityPlus - velocityMinus) / (2.0 * dt);
  const Eigen::Vector6d predicted = jacobianDeriv * qdot;
  metrics.worldJacobianClassicDerivMaxError
      = (numeric - predicted).cwiseAbs().maxCoeff();

  const Eigen::Matrix6d relJacobian0 = cloneJoint->getRelativeJacobian();
  const Eigen::Matrix6d relJacobianDot
      = cloneJoint->getRelativeJacobianTimeDeriv();
  clone->integratePositions(dt);
  const Eigen::Matrix6d relJacobian1 = cloneJoint->getRelativeJacobian();
  const Eigen::Matrix6d relJacobianFiniteDifference
      = (relJacobian1 - relJacobian0) / dt;

  metrics.relativeJacobianDotMaxError
      = (relJacobianDot - relJacobianFiniteDifference).cwiseAbs().maxCoeff();

  return metrics;
}

FreeJointFixture createFixture(
    std::size_t index,
    const char* label,
    const Eigen::Isometry3d& pose,
    const Eigen::Vector6d& velocities,
    const Eigen::Vector4d& color)
{
  using dart::dynamics::BoxShape;
  using dart::dynamics::CollisionAspect;
  using dart::dynamics::DynamicsAspect;
  using dart::dynamics::FreeJoint;
  using dart::dynamics::Skeleton;
  using dart::dynamics::VisualAspect;

  FreeJointFixture fixture;
  fixture.name = label;
  fixture.skeleton = Skeleton::create(
      std::string(kActiveSkeletonPrefix) + std::to_string(index));
  auto [joint, body]
      = fixture.skeleton->createJointAndBodyNodePair<FreeJoint>();
  fixture.joint = joint;
  fixture.body = body;
  body->setName(label);
  body->setMass(1.0);
  body->setMomentOfInertia(0.02, 0.04, 0.06);
  body->setCollidable(false);
  fixture.anisotropicInertia = Eigen::Vector3d(0.02, 0.04, 0.06).asDiagonal();
  fixture.sphericalInertia = fixture.anisotropicInertia.diagonal().mean()
                             * Eigen::Matrix3d::Identity();
  fixture.inertia = fixture.anisotropicInertia;
  fixture.inertiaInv = fixture.inertia.inverse();

  const Eigen::Vector3d boxSize(0.3, 0.2, 0.15);
  auto shapeNode = body->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(std::make_shared<BoxShape>(boxSize));
  shapeNode->getVisualAspect()->setRGBA(color);

  joint->setTransformFromParentBodyNode(Eigen::Isometry3d::Identity());
  joint->setTransformFromChildBodyNode(Eigen::Isometry3d::Identity());
  fixture.initialPositions = FreeJoint::convertToPositions(pose);
  fixture.initialVelocities = velocities;
  joint->setPositions(fixture.initialPositions);
  joint->setVelocities(fixture.initialVelocities);
  fixture.initialEnergy = fixture.skeleton->computeKineticEnergy();

  const Eigen::Matrix3d rotation = pose.linear();
  fixture.torqueFreeState.orientation = Eigen::Quaterniond(rotation);
  fixture.torqueFreeState.orientation.normalize();
  fixture.torqueFreeState.omegaBody
      = rotation.transpose() * velocities.head<3>();

  fixture.referenceSkeleton = Skeleton::create(
      std::string(kReferenceSkeletonPrefix) + std::to_string(index));
  fixture.referenceSkeleton->setMobile(false);
  auto [referenceJoint, referenceBody]
      = fixture.referenceSkeleton->createJointAndBodyNodePair<FreeJoint>();
  fixture.referenceJoint = referenceJoint;
  referenceBody->setName(std::string(label) + " reference");
  referenceBody->setCollidable(false);

  Eigen::Vector4d referenceColor = color;
  referenceColor.head<3>()
      = 0.25 * Eigen::Vector3d::Ones() + 0.75 * referenceColor.head<3>();
  referenceColor[3] = std::min(referenceColor[3], 0.22);

  fixture.referenceShapeNode = referenceBody->createShapeNodeWith<VisualAspect>(
      std::make_shared<BoxShape>(boxSize * 1.05));
  fixture.referenceShapeNode->getVisualAspect()->setRGBA(referenceColor);
  fixture.referenceShapeNode->getVisualAspect()->setShadowed(false);

  referenceJoint->setTransformFromParentBodyNode(Eigen::Isometry3d::Identity());
  referenceJoint->setTransformFromChildBodyNode(Eigen::Isometry3d::Identity());
  referenceJoint->setPositions(fixture.initialPositions);
  referenceJoint->setVelocities(Eigen::Vector6d::Zero());

  return fixture;
}

class FreeJointCasesController
{
public:
  FreeJointCasesController(
      dart::simulation::WorldPtr world,
      std::vector<FreeJointFixture> fixtures,
      const FreeJointCasesConfig& config)
    : mWorld(std::move(world)),
      mFixtures(std::move(fixtures)),
      mNumericDt(config.numericDt),
      mGroundTruthModel(config.groundTruthModel),
      mUseSphericalInertia(config.useSphericalInertia),
      mTorqueFreeSubsteps(std::max(1, config.torqueFreeSubsteps))
  {
    if (mWorld == nullptr) {
      throw std::runtime_error("free_joint_cases controller is missing world");
    }
    applyInertiaMode();
    resetCases();
    recomputeMetrics();
  }

  void preStep()
  {
    const double targetTime = mWorld->getTime() + mWorld->getTimeStep();
    if (targetTime < mTorqueFreeTime) {
      resetTorqueFreeState();
    }
    advanceTorqueFreeState(targetTime);
    updateReferenceBodies(targetTime);
  }

  void resetCases()
  {
    mWorld->reset();
    for (auto& fixture : mFixtures) {
      if (fixture.joint != nullptr) {
        fixture.joint->setPositions(fixture.initialPositions);
        fixture.joint->setVelocities(fixture.initialVelocities);
        fixture.joint->setAccelerations(Eigen::Vector6d::Zero());
        fixture.initialEnergy = fixture.skeleton->computeKineticEnergy();
      }
      if (fixture.referenceJoint != nullptr) {
        fixture.referenceJoint->setPositions(fixture.initialPositions);
        fixture.referenceJoint->setVelocities(Eigen::Vector6d::Zero());
      }
    }
    resetTorqueFreeState();
    updateReferenceBodies(0.0);
  }

  void recomputeMetrics()
  {
    for (auto& fixture : mFixtures) {
      fixture.metrics = computeMetrics(fixture, mNumericDt);
    }
  }

  void setReferenceVisible(bool visible)
  {
    mReferenceVisible = visible;
    for (auto& fixture : mFixtures) {
      if (fixture.referenceShapeNode != nullptr) {
        fixture.referenceShapeNode->getVisualAspect()->setHidden(!visible);
      }
    }
  }

  bool referenceVisible() const
  {
    return mReferenceVisible;
  }

  void setUseSphericalInertia(bool value)
  {
    if (mUseSphericalInertia == value) {
      return;
    }
    mUseSphericalInertia = value;
    applyInertiaMode();
    resetCases();
    recomputeMetrics();
  }

  bool usesSphericalInertia() const
  {
    return mUseSphericalInertia;
  }

  void setGroundTruthModel(GroundTruthModel model)
  {
    if (mGroundTruthModel == model) {
      return;
    }
    mGroundTruthModel = model;
    resetTorqueFreeState();
    updateReferenceBodies(mWorld->getTime());
  }

  GroundTruthModel groundTruthModel() const
  {
    return mGroundTruthModel;
  }

  std::string groundTruthModelLabel() const
  {
    return mGroundTruthModel == GroundTruthModel::TorqueFreeRigidBody
               ? "torque-free rigid body"
               : "constant world twist";
  }

  void setTorqueFreeSubsteps(int substeps)
  {
    mTorqueFreeSubsteps = std::max(1, substeps);
    resetTorqueFreeState();
  }

  int torqueFreeSubsteps() const
  {
    return mTorqueFreeSubsteps;
  }

  double numericDt() const
  {
    return mNumericDt;
  }

  std::size_t caseCount() const
  {
    return mFixtures.size();
  }

  std::vector<std::string> metricRows() const
  {
    std::vector<std::string> rows;
    rows.reserve(mFixtures.size());
    for (const auto& fixture : mFixtures) {
      const double energy = fixture.skeleton->computeKineticEnergy();
      const double denominator = std::max(1.0, fixture.initialEnergy);
      const double relativeEnergyError
          = (energy - fixture.initialEnergy) / denominator;

      rows.push_back(
          fixture.name + ": KE " + formatDouble(energy, 4) + ", dKE/KE0 "
          + formatDouble(relativeEnergyError, 4) + ", |Jt-I|inf "
          + formatDouble(fixture.metrics.translationalBlockError, 3)
          + ", |Jnum-J|inf "
          + formatDouble(fixture.metrics.worldJacobianMaxError, 3)
          + ", |dVdot-FD|inf "
          + formatDouble(fixture.metrics.worldJacobianClassicDerivMaxError, 3)
          + ", |relJdot-FD|inf "
          + formatDouble(fixture.metrics.relativeJacobianDotMaxError, 3));
    }
    return rows;
  }

private:
  void applyInertiaMode()
  {
    for (auto& fixture : mFixtures) {
      const Eigen::Matrix3d& inertia = mUseSphericalInertia
                                           ? fixture.sphericalInertia
                                           : fixture.anisotropicInertia;
      fixture.inertia = inertia;
      fixture.inertiaInv = inertia.inverse();
      if (fixture.body != nullptr) {
        fixture.body->setMomentOfInertia(
            inertia(0, 0),
            inertia(1, 1),
            inertia(2, 2),
            inertia(0, 1),
            inertia(0, 2),
            inertia(1, 2));
      }
    }
  }

  void resetTorqueFreeState()
  {
    mTorqueFreeTime = 0.0;
    for (auto& fixture : mFixtures) {
      const Eigen::Isometry3d initial
          = dart::dynamics::FreeJoint::convertToTransform(
              fixture.initialPositions);
      fixture.torqueFreeState.orientation
          = Eigen::Quaterniond(initial.linear());
      fixture.torqueFreeState.orientation.normalize();
      fixture.torqueFreeState.omegaBody
          = initial.linear().transpose() * fixture.initialVelocities.head<3>();
    }
  }

  void advanceTorqueFreeState(double targetTime)
  {
    const double timeStep = mWorld->getTimeStep();
    if (!(timeStep > 0.0) || !(targetTime > mTorqueFreeTime)) {
      return;
    }

    const double baseStep
        = timeStep / static_cast<double>(std::max(1, mTorqueFreeSubsteps));
    double time = mTorqueFreeTime;
    while (time + 1e-12 < targetTime) {
      const double dt = std::min(baseStep, targetTime - time);
      for (auto& fixture : mFixtures) {
        integrateTorqueFreeStep(
            fixture.torqueFreeState, fixture.inertia, fixture.inertiaInv, dt);
      }
      time += dt;
    }
    mTorqueFreeTime = targetTime;
  }

  void updateReferenceBodies(double time)
  {
    for (auto& fixture : mFixtures) {
      if (fixture.referenceJoint == nullptr) {
        continue;
      }

      Eigen::Isometry3d expected = Eigen::Isometry3d::Identity();
      if (mGroundTruthModel == GroundTruthModel::TorqueFreeRigidBody) {
        expected.linear()
            = fixture.torqueFreeState.orientation.toRotationMatrix();
        expected.translation() = fixture.initialPositions.tail<3>()
                                 + fixture.initialVelocities.tail<3>() * time;
      } else {
        expected = computeConstantTwistTransform(fixture, time);
      }
      fixture.referenceJoint->setPositions(
          dart::dynamics::FreeJoint::convertToPositions(expected));
    }
  }

  dart::simulation::WorldPtr mWorld;
  std::vector<FreeJointFixture> mFixtures;
  double mNumericDt = 1e-6;
  GroundTruthModel mGroundTruthModel = GroundTruthModel::TorqueFreeRigidBody;
  bool mUseSphericalInertia = false;
  int mTorqueFreeSubsteps = 10;
  double mTorqueFreeTime = 0.0;
  bool mReferenceVisible = true;
};

struct FreeJointCasesScene
{
  dart::simulation::WorldPtr world;
  std::shared_ptr<FreeJointCasesController> controller;
};

enum class ParseResult
{
  Ok,
  Help,
};

bool parseDouble(std::string_view value, double& output)
{
  const auto* first = value.data();
  const auto* last = value.data() + value.size();
  const auto result = std::from_chars(first, last, output);
  return result.ec == std::errc{} && result.ptr == last;
}

bool parseInt(std::string_view value, int& output)
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

void printUsage(const char* executable)
{
  std::cout << "Usage: " << executable
            << " [--numeric-dt DT] [--dt DT] [--spherical-inertia]\n"
            << "       [--ground-truth torque-free|constant]\n"
            << "       [--ground-truth-substeps N]\n"
            << "       [common dart::gui flags such as --headless, --frames,\n"
            << "        --screenshot, --out, --width, --height, --gui-scale]\n";
}

ParseResult parseFreeJointCasesConfig(
    int argc, char* argv[], FreeJointCasesConfig& config)
{
  for (int i = 1; i < argc; ++i) {
    const std::string_view argument(argv[i] == nullptr ? "" : argv[i]);
    if (argument == "--help" || argument == "-h") {
      printUsage(argv[0]);
      return ParseResult::Help;
    }
    if (auto value = getOptionValue(argument, "--numeric-dt", i, argc, argv)) {
      if (!parseDouble(*value, config.numericDt) || !(config.numericDt > 0.0)) {
        throw std::runtime_error(
            "Invalid --numeric-dt value: " + std::string(*value));
      }
      continue;
    }
    if (auto value = getOptionValue(argument, "--dt", i, argc, argv)) {
      if (!parseDouble(*value, config.simulationDt)
          || !(config.simulationDt > 0.0)) {
        throw std::runtime_error("Invalid --dt value: " + std::string(*value));
      }
      continue;
    }
    if (argument == "--spherical-inertia") {
      config.useSphericalInertia = true;
      continue;
    }
    if (auto value
        = getOptionValue(argument, "--ground-truth", i, argc, argv)) {
      if (*value == "torque-free") {
        config.groundTruthModel = GroundTruthModel::TorqueFreeRigidBody;
      } else if (*value == "constant") {
        config.groundTruthModel = GroundTruthModel::ConstantWorldTwist;
      } else {
        throw std::runtime_error(
            "Invalid --ground-truth value: " + std::string(*value));
      }
      continue;
    }
    if (auto value
        = getOptionValue(argument, "--ground-truth-substeps", i, argc, argv)) {
      if (!parseInt(*value, config.torqueFreeSubsteps)
          || config.torqueFreeSubsteps < 1) {
        throw std::runtime_error(
            "Invalid --ground-truth-substeps value: " + std::string(*value));
      }
      continue;
    }
  }

  return ParseResult::Ok;
}

FreeJointCasesScene createFreeJointCasesScene(
    const FreeJointCasesConfig& config)
{
  FreeJointCasesScene scene;
  scene.world = dart::simulation::World::create("free_joint_cases");
  scene.world->setGravity(Eigen::Vector3d::Zero());
  scene.world->setTimeStep(config.simulationDt);

  std::vector<FreeJointFixture> fixtures;
  fixtures.reserve(kCaseCount);

  {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    Eigen::Vector6d velocity = Eigen::Vector6d::Zero();
    velocity.tail<3>() = Eigen::Vector3d(0.7, 0.0, 0.0);
    fixtures.push_back(createFixture(
        0,
        "linear_only",
        pose,
        velocity,
        Eigen::Vector4d(0.15, 0.35, 0.95, 0.7)));
  }

  {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(2.0, 0.0, 0.0);
    Eigen::Vector6d velocity = Eigen::Vector6d::Zero();
    velocity.head<3>() = Eigen::Vector3d(0.0, 0.9, 0.0);
    fixtures.push_back(createFixture(
        1,
        "angular_only",
        pose,
        velocity,
        Eigen::Vector4d(0.90, 0.12, 0.12, 0.7)));
  }

  {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(4.0, 0.0, 0.0);
    pose.linear() = dart::math::expMapRot(Eigen::Vector3d(0.6, -0.3, 0.2));
    Eigen::Vector6d velocity = Eigen::Vector6d::Zero();
    velocity.head<3>() = Eigen::Vector3d(0.6, -0.3, 0.4);
    velocity.tail<3>() = Eigen::Vector3d(0.5, 0.2, -0.1);
    fixtures.push_back(createFixture(
        2,
        "linear_angular",
        pose,
        velocity,
        Eigen::Vector4d(0.24, 0.78, 0.32, 0.7)));
  }

  {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(6.0, 0.0, 0.0);
    pose.linear() = dart::math::expMapRot(
        Eigen::Vector3d(dart::math::pi - 1e-6, 0.0, 0.0));
    Eigen::Vector6d velocity = Eigen::Vector6d::Zero();
    velocity.head<3>() = Eigen::Vector3d(0.2, 0.8, -0.4);
    velocity.tail<3>() = Eigen::Vector3d(0.3, -0.1, 0.2);
    fixtures.push_back(createFixture(
        3,
        "near_pi_rotation",
        pose,
        velocity,
        Eigen::Vector4d(0.95, 0.58, 0.10, 0.7)));
  }

  {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(8.0, 0.0, 0.0);
    pose.linear() = dart::math::expMapRot(Eigen::Vector3d(0.3, 1.2, -0.4));
    Eigen::Vector6d velocity = Eigen::Vector6d::Zero();
    velocity.head<3>() = Eigen::Vector3d(1.2, 0.9, -1.0);
    velocity.tail<3>() = Eigen::Vector3d(0.2, 0.3, 0.0);
    fixtures.push_back(createFixture(
        4,
        "high_omega_multi_axis",
        pose,
        velocity,
        Eigen::Vector4d(0.82, 0.22, 0.78, 0.7)));
  }

  for (const auto& fixture : fixtures) {
    scene.world->addSkeleton(fixture.skeleton);
    scene.world->addSkeleton(fixture.referenceSkeleton);
  }

  scene.controller = std::make_shared<FreeJointCasesController>(
      scene.world, std::move(fixtures), config);
  scene.controller->setReferenceVisible(true);
  return scene;
}

dart::gui::OrbitCamera makeFreeJointCasesCamera()
{
  dart::gui::OrbitCamera camera;
  camera.target = Eigen::Vector3d(2.5, 0.0, 0.0);
  camera.yaw = -dart::math::pi / 2.0;
  camera.pitch = std::asin(4.0 / std::sqrt(116.0));
  camera.distance = std::sqrt(116.0);
  return camera;
}

dart::gui::RunOptions makeFreeJointCasesRunDefaults()
{
  dart::gui::RunOptions options;
  options.width = 1280;
  options.height = 720;
  return options;
}

dart::gui::Panel createFreeJointCasesPanel(
    const std::shared_ptr<FreeJointCasesController>& controller)
{
  dart::gui::Panel panel;
  panel.title = "FreeJoint Cases";
  panel.buildWithContext = [controller](
                               dart::gui::PanelBuilder& builder,
                               dart::gui::PanelContext& context) {
    builder.text("FreeJoint cases with transparent reference bodies");
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
    builder.sameLine();
    if (builder.button("Reset cases")) {
      controller->resetCases();
    }
    builder.sameLine();
    if (builder.button("Numeric checks")) {
      controller->recomputeMetrics();
    }
    bool showReferences = controller->referenceVisible();
    if (builder.checkbox("Show references", showReferences)) {
      controller->setReferenceVisible(showReferences);
    }
    bool sphericalInertia = controller->usesSphericalInertia();
    if (builder.checkbox("Use spherical inertia", sphericalInertia)) {
      controller->setUseSphericalInertia(sphericalInertia);
    }
    bool constantTwist = controller->groundTruthModel()
                         == GroundTruthModel::ConstantWorldTwist;
    if (builder.checkbox("Constant world twist", constantTwist)) {
      controller->setGroundTruthModel(
          constantTwist ? GroundTruthModel::ConstantWorldTwist
                        : GroundTruthModel::TorqueFreeRigidBody);
    }
    double torqueFreeSubsteps
        = static_cast<double>(controller->torqueFreeSubsteps());
    if (builder.slider(
            "Torque-free substeps", torqueFreeSubsteps, 1.0, 200.0)) {
      controller->setTorqueFreeSubsteps(
          static_cast<int>(std::lround(torqueFreeSubsteps)));
    }
    builder.text("cases: " + std::to_string(controller->caseCount()));
    builder.text("time: " + std::to_string(context.simulationTime));
    builder.text("numeric dt: " + formatDouble(controller->numericDt(), 6));
    builder.text("ground truth: " + controller->groundTruthModelLabel());
    builder.separator();
    builder.text(
        "metrics: KE, dKE/KE0, |Jt-I|inf, |Jnum-J|inf, |dVdot-FD|inf, "
        "|relJdot-FD|inf");
    for (const auto& row : controller->metricRows()) {
      builder.text(row);
    }
  };
  return panel;
}

} // namespace

int main(int argc, char* argv[])
{
  try {
    FreeJointCasesConfig config;
    if (parseFreeJointCasesConfig(argc, argv, config) == ParseResult::Help) {
      return 0;
    }

    FreeJointCasesScene scene = createFreeJointCasesScene(config);

    dart::gui::ApplicationOptions options;
    options.world = scene.world;
    options.runDefaults = makeFreeJointCasesRunDefaults();
    options.camera = makeFreeJointCasesCamera();
    options.preStep = [controller = scene.controller]() {
      controller->preStep();
    };
    options.panels.push_back(createFreeJointCasesPanel(scene.controller));

    return dart::gui::runApplication(argc, argv, options);
  } catch (const std::exception& e) {
    std::cerr << "free_joint_cases: " << e.what() << "\n";
    return 1;
  }
}
