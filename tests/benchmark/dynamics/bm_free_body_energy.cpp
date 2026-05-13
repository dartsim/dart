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

#include <dart/simulation/world.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <dart/math/geometry.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <benchmark/benchmark.h>

#include <algorithm>
#include <array>
#include <limits>

#include <cmath>

namespace {

struct FreeBodyCase
{
  Eigen::Matrix3d inertia;
  Eigen::Vector3d omega;
  Eigen::Vector3d localCom = Eigen::Vector3d::Zero();
  Eigen::Vector3d linearVelocity = Eigen::Vector3d::Zero();
};

struct DriftSummary
{
  double maxRelativeEnergyDrift = 0.0;
  double maxRelativeAngularMomentumDrift = 0.0;
  double maxRelativeWorldAngularMomentumVectorDrift = 0.0;
  double maxRelativeWorldLinearMomentumVectorDrift = 0.0;
  double maxMidpointResidual = 0.0;
  int maxNewtonIterations = 0;
  int midpointSamples = 0;
  int newtonIterations = 0;
  int newtonUnconvergedSteps = 0;
};

struct ForcedTorqueSummary
{
  double maxRelativeAngularImpulseError = 0.0;
  double minEnergyGainPerStep = std::numeric_limits<double>::infinity();
  double finalEnergyGain = 0.0;
};

struct DampedSummary
{
  double maxEnergyIncreasePerStep = -std::numeric_limits<double>::infinity();
  double finalEnergyRatio = std::numeric_limits<double>::infinity();
};

struct GravitySummary
{
  double maxVelocityError = 0.0;
  double maxPositionError = 0.0;
};

struct GeneralizedForceSummary
{
  double maxVelocityError = 0.0;
  double maxPositionError = 0.0;
  double finalEnergyGain = 0.0;
};

struct AccelerationCommandSummary
{
  double maxVelocityError = 0.0;
  double maxPositionError = 0.0;
  double finalEnergyGain = 0.0;
};

struct MidpointStats
{
  double residualNorm = std::numeric_limits<double>::infinity();
  int iterations = 0;
  bool converged = false;
};

const std::array<FreeBodyCase, 4>& cases()
{
  static const std::array<FreeBodyCase, 4> data = [] {
    std::array<FreeBodyCase, 4> result;

    result[0].inertia = Eigen::Vector3d(0.7, 1.0, 1.2).asDiagonal();
    result[0].omega = Eigen::Vector3d(8.0, -4.0, 6.0);

    result[1].inertia = Eigen::Vector3d(0.4, 1.1, 1.3).asDiagonal();
    result[1].omega = Eigen::Vector3d(-12.0, 7.0, 3.0);
    result[1].localCom = Eigen::Vector3d(0.03, -0.02, 0.04);
    result[1].linearVelocity = Eigen::Vector3d(0.2, -0.1, 0.05);

    result[2].inertia = Eigen::Vector3d(0.8, 0.9, 1.4).asDiagonal();
    result[2].omega = Eigen::Vector3d(3.5, 11.0, -5.0);
    result[2].localCom = Eigen::Vector3d(-0.05, 0.02, 0.01);
    result[2].linearVelocity = Eigen::Vector3d(-0.15, 0.08, 0.03);

    result[3].inertia << 0.9, 0.04, -0.03, 0.04, 1.1, 0.02, -0.03, 0.02, 1.3;
    result[3].omega = Eigen::Vector3d(9.0, -6.0, 4.0);
    result[3].localCom = Eigen::Vector3d(0.08, -0.04, 0.02);
    result[3].linearVelocity = Eigen::Vector3d(0.1, 0.12, -0.07);

    return result;
  }();

  return data;
}

const FreeBodyCase& principalAxisCase()
{
  static const FreeBodyCase data = [] {
    FreeBodyCase result;
    result.inertia = Eigen::Vector3d(0.7, 1.0, 1.2).asDiagonal();
    result.omega = Eigen::Vector3d(12.0, 0.0, 0.0);
    return result;
  }();

  return data;
}

const FreeBodyCase& gravityCase()
{
  static const FreeBodyCase data = [] {
    FreeBodyCase result;
    result.inertia = Eigen::Vector3d(0.7, 1.0, 1.2).asDiagonal();
    result.linearVelocity = Eigen::Vector3d(0.2, -0.1, 1.5);
    return result;
  }();

  return data;
}

const FreeBodyCase& generalizedForceCase()
{
  static const FreeBodyCase data = [] {
    FreeBodyCase result;
    result.inertia = Eigen::Vector3d(0.7, 1.0, 1.2).asDiagonal();
    result.linearVelocity = Eigen::Vector3d(0.2, -0.1, 0.0);
    return result;
  }();

  return data;
}

const Eigen::Vector3d& principalAxisTorque()
{
  static const Eigen::Vector3d torque(0.6, 0.0, 0.0);
  return torque;
}

bool isResidualConverged(
    const Eigen::Matrix3d& inertia,
    const Eigen::Vector3d& omega,
    const Eigen::Vector3d& midpoint,
    double dt,
    double residualNorm)
{
  constexpr double absoluteTolerance = 1e-13;
  constexpr double relativeTolerance = 1e-12;

  const Eigen::Vector3d angularMomentum = inertia * midpoint;
  const double residualScale
      = ((2.0 / dt) * inertia * (midpoint - omega)).norm()
        + midpoint.cross(angularMomentum).norm();
  const double tolerance
      = absoluteTolerance + relativeTolerance * std::max(1.0, residualScale);

  if (!std::isfinite(residualNorm) || !std::isfinite(tolerance)) {
    return false;
  }

  return residualNorm <= tolerance;
}

double dtFromState(const benchmark::State& state)
{
  return static_cast<double>(state.range(0)) * 1e-6;
}

int stepsFromState(const benchmark::State& state)
{
  return static_cast<int>(state.range(1));
}

double kineticEnergy(
    const Eigen::Matrix3d& inertia, const Eigen::Vector3d& omega)
{
  return 0.5 * omega.dot(inertia * omega);
}

double angularMomentumNorm(
    const Eigen::Matrix3d& inertia, const Eigen::Vector3d& omega)
{
  return (inertia * omega).norm();
}

dart::dynamics::SkeletonPtr createFreeBodySkeleton(const FreeBodyCase& bodyCase)
{
  auto skeleton = dart::dynamics::Skeleton::create();
  auto pair = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
  auto* joint = pair.first;
  auto* body = pair.second;

  body->setMass(1.0);
  body->setLocalCOM(bodyCase.localCom);
  body->setMomentOfInertia(
      bodyCase.inertia(0, 0),
      bodyCase.inertia(1, 1),
      bodyCase.inertia(2, 2),
      bodyCase.inertia(0, 1),
      bodyCase.inertia(0, 2),
      bodyCase.inertia(1, 2));

  Eigen::Vector6d velocity = Eigen::Vector6d::Zero();
  velocity.head<3>() = bodyCase.omega;
  velocity.tail<3>() = bodyCase.linearVelocity;
  joint->setVelocities(velocity);

  for (std::size_t i = 0; i < joint->getNumDofs(); ++i) {
    joint->setDampingCoefficient(i, 0.0);
    joint->setCoulombFriction(i, 0.0);
  }

  return skeleton;
}

dart::simulation::WorldPtr createFreeBodyWorld(
    const FreeBodyCase& bodyCase, double dt)
{
  auto world = dart::simulation::World::create();
  world->setGravity(Eigen::Vector3d::Zero());
  world->setTimeStep(dt);
  world->addSkeleton(createFreeBodySkeleton(bodyCase));
  return world;
}

dart::simulation::WorldPtr createFreeBodyWorld(
    const FreeBodyCase& bodyCase, double dt, const Eigen::Vector3d& gravity)
{
  auto world = dart::simulation::World::create();
  world->setGravity(gravity);
  world->setTimeStep(dt);
  world->addSkeleton(createFreeBodySkeleton(bodyCase));
  return world;
}

Eigen::Vector6d worldSpatialMomentum(
    const dart::dynamics::SkeletonPtr& skeleton)
{
  Eigen::Vector6d momentum = Eigen::Vector6d::Zero();
  skeleton->eachBodyNode([&](const dart::dynamics::BodyNode* body) {
    momentum += dart::math::dAdInvT(
        body->getWorldTransform(),
        body->getSpatialInertia() * body->getSpatialVelocity());
  });
  return momentum;
}

void setFreeJointDamping(
    const dart::dynamics::SkeletonPtr& skeleton, double damping)
{
  auto* joint = skeleton->getJoint(0);
  for (std::size_t i = 0; i < joint->getNumDofs(); ++i) {
    joint->setDampingCoefficient(i, damping);
    joint->setCoulombFriction(i, 0.0);
    joint->setSpringStiffness(i, 0.0);
  }
}

Eigen::Vector3d explicitEulerStep(
    const Eigen::Matrix3d& inertia,
    const Eigen::LDLT<Eigen::Matrix3d>& inertiaSolver,
    const Eigen::Vector3d& omega,
    double dt)
{
  const Eigen::Vector3d angularMomentum = inertia * omega;
  const Eigen::Vector3d omegaDot
      = -inertiaSolver.solve(omega.cross(angularMomentum));
  return omega + omegaDot * dt;
}

Eigen::Vector3d angularAcceleration(
    const Eigen::Matrix3d& inertia,
    const Eigen::LDLT<Eigen::Matrix3d>& inertiaSolver,
    const Eigen::Vector3d& omega)
{
  const Eigen::Vector3d angularMomentum = inertia * omega;
  return -inertiaSolver.solve(omega.cross(angularMomentum));
}

Eigen::Vector3d rungeKutta4Step(
    const Eigen::Matrix3d& inertia,
    const Eigen::LDLT<Eigen::Matrix3d>& inertiaSolver,
    const Eigen::Vector3d& omega,
    double dt)
{
  const Eigen::Vector3d k1 = angularAcceleration(inertia, inertiaSolver, omega);
  const Eigen::Vector3d k2
      = angularAcceleration(inertia, inertiaSolver, omega + 0.5 * dt * k1);
  const Eigen::Vector3d k3
      = angularAcceleration(inertia, inertiaSolver, omega + 0.5 * dt * k2);
  const Eigen::Vector3d k4
      = angularAcceleration(inertia, inertiaSolver, omega + dt * k3);

  return omega + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

Eigen::Vector3d rungeKutta4TwoHalfSteps(
    const Eigen::Matrix3d& inertia,
    const Eigen::LDLT<Eigen::Matrix3d>& inertiaSolver,
    const Eigen::Vector3d& omega,
    double dt)
{
  const double halfDt = 0.5 * dt;
  const Eigen::Vector3d midpoint
      = rungeKutta4Step(inertia, inertiaSolver, omega, halfDt);
  return rungeKutta4Step(inertia, inertiaSolver, midpoint, halfDt);
}

Eigen::Vector3d midpointStep(
    const Eigen::Matrix3d& inertia,
    const Eigen::LDLT<Eigen::Matrix3d>& inertiaSolver,
    const Eigen::Vector3d& omega,
    double dt,
    int maxIterations,
    MidpointStats* stats = nullptr)
{
  Eigen::Vector3d midpoint = omega;
  double residualNorm = std::numeric_limits<double>::infinity();
  int iterations = 0;
  bool converged = false;

  for (int i = 0; i < maxIterations; ++i) {
    const Eigen::Vector3d angularMomentum = inertia * midpoint;
    const Eigen::Vector3d residual = (2.0 / dt) * inertia * (midpoint - omega)
                                     + midpoint.cross(angularMomentum);
    residualNorm = residual.norm();

    if (isResidualConverged(inertia, omega, midpoint, dt, residualNorm)) {
      converged = true;
      break;
    }

    const Eigen::Matrix3d jacobian
        = (2.0 / dt) * inertia - dart::math::makeSkewSymmetric(angularMomentum)
          + dart::math::makeSkewSymmetric(midpoint) * inertia;
    const Eigen::Vector3d step = jacobian.ldlt().solve(residual);
    midpoint -= step;
    ++iterations;

    if (step.norm() < 1e-13) {
      const Eigen::Vector3d nextAngularMomentum = inertia * midpoint;
      const Eigen::Vector3d nextResidual
          = (2.0 / dt) * inertia * (midpoint - omega)
            + midpoint.cross(nextAngularMomentum);
      residualNorm = nextResidual.norm();
      converged
          = isResidualConverged(inertia, omega, midpoint, dt, residualNorm);
      break;
    }
  }

  if (!converged) {
    const Eigen::Vector3d angularMomentum = inertia * midpoint;
    const Eigen::Vector3d residual = (2.0 / dt) * inertia * (midpoint - omega)
                                     + midpoint.cross(angularMomentum);
    residualNorm = residual.norm();
    converged = isResidualConverged(inertia, omega, midpoint, dt, residualNorm);
  }

  if (stats != nullptr) {
    stats->residualNorm = residualNorm;
    stats->iterations = iterations;
    stats->converged = converged;
  }

  (void)inertiaSolver;
  return 2.0 * midpoint - omega;
}

Eigen::Vector3d midpointStep1(
    const Eigen::Matrix3d& inertia,
    const Eigen::LDLT<Eigen::Matrix3d>& inertiaSolver,
    const Eigen::Vector3d& omega,
    double dt)
{
  return midpointStep(inertia, inertiaSolver, omega, dt, 1);
}

Eigen::Vector3d midpointStep2(
    const Eigen::Matrix3d& inertia,
    const Eigen::LDLT<Eigen::Matrix3d>& inertiaSolver,
    const Eigen::Vector3d& omega,
    double dt)
{
  return midpointStep(inertia, inertiaSolver, omega, dt, 2);
}

Eigen::Vector3d midpointStep4(
    const Eigen::Matrix3d& inertia,
    const Eigen::LDLT<Eigen::Matrix3d>& inertiaSolver,
    const Eigen::Vector3d& omega,
    double dt)
{
  return midpointStep(inertia, inertiaSolver, omega, dt, 4);
}

Eigen::Vector3d midpointStep8(
    const Eigen::Matrix3d& inertia,
    const Eigen::LDLT<Eigen::Matrix3d>& inertiaSolver,
    const Eigen::Vector3d& omega,
    double dt)
{
  return midpointStep(inertia, inertiaSolver, omega, dt, 8);
}

template <typename StepFn>
DriftSummary summarizeDrift(double dt, int numSteps, StepFn step)
{
  DriftSummary summary;

  for (const FreeBodyCase& bodyCase : cases()) {
    Eigen::Vector3d omega = bodyCase.omega;
    const Eigen::LDLT<Eigen::Matrix3d> inertiaSolver(bodyCase.inertia);

    const double energy0 = kineticEnergy(bodyCase.inertia, omega);
    const double momentum0 = angularMomentumNorm(bodyCase.inertia, omega);

    for (int i = 0; i < numSteps; ++i) {
      omega = step(bodyCase.inertia, inertiaSolver, omega, dt);

      const double relativeEnergyDrift
          = std::abs(kineticEnergy(bodyCase.inertia, omega) - energy0)
            / std::max(1.0, std::abs(energy0));
      const double relativeMomentumDrift
          = std::abs(angularMomentumNorm(bodyCase.inertia, omega) - momentum0)
            / std::max(1.0, std::abs(momentum0));

      summary.maxRelativeEnergyDrift
          = std::max(summary.maxRelativeEnergyDrift, relativeEnergyDrift);
      summary.maxRelativeAngularMomentumDrift = std::max(
          summary.maxRelativeAngularMomentumDrift, relativeMomentumDrift);
    }
  }

  return summary;
}

DriftSummary summarizeMidpointDrift(
    double dt, int numSteps, int maxNewtonIterations)
{
  DriftSummary summary;

  for (const FreeBodyCase& bodyCase : cases()) {
    Eigen::Vector3d omega = bodyCase.omega;
    const Eigen::LDLT<Eigen::Matrix3d> inertiaSolver(bodyCase.inertia);

    const double energy0 = kineticEnergy(bodyCase.inertia, omega);
    const double momentum0 = angularMomentumNorm(bodyCase.inertia, omega);

    for (int i = 0; i < numSteps; ++i) {
      MidpointStats stats;
      omega = midpointStep(
          bodyCase.inertia,
          inertiaSolver,
          omega,
          dt,
          maxNewtonIterations,
          &stats);

      const double relativeEnergyDrift
          = std::abs(kineticEnergy(bodyCase.inertia, omega) - energy0)
            / std::max(1.0, std::abs(energy0));
      const double relativeMomentumDrift
          = std::abs(angularMomentumNorm(bodyCase.inertia, omega) - momentum0)
            / std::max(1.0, std::abs(momentum0));

      summary.maxRelativeEnergyDrift
          = std::max(summary.maxRelativeEnergyDrift, relativeEnergyDrift);
      summary.maxRelativeAngularMomentumDrift = std::max(
          summary.maxRelativeAngularMomentumDrift, relativeMomentumDrift);
      summary.maxMidpointResidual
          = std::max(summary.maxMidpointResidual, stats.residualNorm);
      summary.maxNewtonIterations
          = std::max(summary.maxNewtonIterations, stats.iterations);
      summary.midpointSamples += 1;
      summary.newtonIterations += stats.iterations;

      if (!stats.converged) {
        summary.newtonUnconvergedSteps += 1;
      }
    }
  }

  return summary;
}

DriftSummary summarizeWorldStepDrift(double dt, int numSteps)
{
  DriftSummary summary;

  for (const FreeBodyCase& bodyCase : cases()) {
    auto world = createFreeBodyWorld(bodyCase, dt);
    const auto skeleton = world->getSkeleton(0);

    const double energy0 = skeleton->computeKineticEnergy();
    const Eigen::Vector6d worldMomentum0 = worldSpatialMomentum(skeleton);
    const Eigen::Vector3d worldAngularMomentum0 = worldMomentum0.head<3>();
    const Eigen::Vector3d worldLinearMomentum0 = worldMomentum0.tail<3>();
    const double momentum0 = worldAngularMomentum0.norm();
    const double linearMomentum0 = worldLinearMomentum0.norm();

    for (int i = 0; i < numSteps; ++i) {
      world->step();

      const double energy = skeleton->computeKineticEnergy();
      const Eigen::Vector6d currentWorldMomentum
          = worldSpatialMomentum(skeleton);
      const Eigen::Vector3d currentWorldAngularMomentum
          = currentWorldMomentum.head<3>();
      const Eigen::Vector3d currentWorldLinearMomentum
          = currentWorldMomentum.tail<3>();
      const double momentum = currentWorldAngularMomentum.norm();
      const double linearMomentum = currentWorldLinearMomentum.norm();

      if (!std::isfinite(energy) || !std::isfinite(momentum)
          || !std::isfinite(linearMomentum)) {
        summary.maxRelativeEnergyDrift
            = std::numeric_limits<double>::infinity();
        summary.maxRelativeAngularMomentumDrift
            = std::numeric_limits<double>::infinity();
        summary.maxRelativeWorldLinearMomentumVectorDrift
            = std::numeric_limits<double>::infinity();
        break;
      }

      const double relativeEnergyDrift
          = std::abs(energy - energy0) / std::max(1.0, std::abs(energy0));
      const double relativeMomentumDrift
          = std::abs(momentum - momentum0) / std::max(1.0, std::abs(momentum0));
      const double relativeWorldAngularMomentumVectorDrift
          = (currentWorldAngularMomentum - worldAngularMomentum0).norm()
            / std::max(1.0, std::abs(momentum0));
      const double relativeWorldLinearMomentumVectorDrift
          = (currentWorldLinearMomentum - worldLinearMomentum0).norm()
            / std::max(1.0, std::abs(linearMomentum0));

      summary.maxRelativeEnergyDrift
          = std::max(summary.maxRelativeEnergyDrift, relativeEnergyDrift);
      summary.maxRelativeAngularMomentumDrift = std::max(
          summary.maxRelativeAngularMomentumDrift, relativeMomentumDrift);
      summary.maxRelativeWorldAngularMomentumVectorDrift = std::max(
          summary.maxRelativeWorldAngularMomentumVectorDrift,
          relativeWorldAngularMomentumVectorDrift);
      summary.maxRelativeWorldLinearMomentumVectorDrift = std::max(
          summary.maxRelativeWorldLinearMomentumVectorDrift,
          relativeWorldLinearMomentumVectorDrift);
    }
  }

  return summary;
}

DriftSummary summarizeWorldStepDrift(
    const FreeBodyCase& bodyCase, double dt, int numSteps)
{
  DriftSummary summary;
  auto world = createFreeBodyWorld(bodyCase, dt);
  const auto skeleton = world->getSkeleton(0);

  const double energy0 = skeleton->computeKineticEnergy();
  const Eigen::Vector6d worldMomentum0 = worldSpatialMomentum(skeleton);
  const Eigen::Vector3d worldAngularMomentum0 = worldMomentum0.head<3>();
  const Eigen::Vector3d worldLinearMomentum0 = worldMomentum0.tail<3>();
  const double momentum0 = worldAngularMomentum0.norm();
  const double linearMomentum0 = worldLinearMomentum0.norm();

  for (int i = 0; i < numSteps; ++i) {
    world->step();

    const double energy = skeleton->computeKineticEnergy();
    const Eigen::Vector6d currentWorldMomentum = worldSpatialMomentum(skeleton);
    const Eigen::Vector3d currentWorldAngularMomentum
        = currentWorldMomentum.head<3>();
    const Eigen::Vector3d currentWorldLinearMomentum
        = currentWorldMomentum.tail<3>();
    const double momentum = currentWorldAngularMomentum.norm();
    const double linearMomentum = currentWorldLinearMomentum.norm();

    if (!std::isfinite(energy) || !std::isfinite(momentum)
        || !std::isfinite(linearMomentum)) {
      summary.maxRelativeEnergyDrift = std::numeric_limits<double>::infinity();
      summary.maxRelativeAngularMomentumDrift
          = std::numeric_limits<double>::infinity();
      summary.maxRelativeWorldLinearMomentumVectorDrift
          = std::numeric_limits<double>::infinity();
      break;
    }

    const double relativeEnergyDrift
        = std::abs(energy - energy0) / std::max(1.0, std::abs(energy0));
    const double relativeMomentumDrift
        = std::abs(momentum - momentum0) / std::max(1.0, std::abs(momentum0));
    const double relativeWorldAngularMomentumVectorDrift
        = (currentWorldAngularMomentum - worldAngularMomentum0).norm()
          / std::max(1.0, std::abs(momentum0));
    const double relativeWorldLinearMomentumVectorDrift
        = (currentWorldLinearMomentum - worldLinearMomentum0).norm()
          / std::max(1.0, std::abs(linearMomentum0));

    summary.maxRelativeEnergyDrift
        = std::max(summary.maxRelativeEnergyDrift, relativeEnergyDrift);
    summary.maxRelativeAngularMomentumDrift = std::max(
        summary.maxRelativeAngularMomentumDrift, relativeMomentumDrift);
    summary.maxRelativeWorldAngularMomentumVectorDrift = std::max(
        summary.maxRelativeWorldAngularMomentumVectorDrift,
        relativeWorldAngularMomentumVectorDrift);
    summary.maxRelativeWorldLinearMomentumVectorDrift = std::max(
        summary.maxRelativeWorldLinearMomentumVectorDrift,
        relativeWorldLinearMomentumVectorDrift);
  }

  return summary;
}

GravitySummary summarizeGravityWorldStep(double dt, int numSteps)
{
  GravitySummary summary;
  const Eigen::Vector3d gravity(0.0, 0.0, -9.81);
  auto world = createFreeBodyWorld(gravityCase(), dt, gravity);
  const auto skeleton = world->getSkeleton(0);
  auto* joint = static_cast<dart::dynamics::FreeJoint*>(skeleton->getJoint(0));

  Eigen::Vector3d expectedVelocity = gravityCase().linearVelocity;
  Eigen::Vector3d expectedPosition = Eigen::Vector3d::Zero();

  for (int i = 0; i < numSteps; ++i) {
    expectedVelocity += gravity * dt;
    expectedPosition += expectedVelocity * dt;

    world->step();

    summary.maxVelocityError = std::max(
        summary.maxVelocityError,
        (joint->getVelocitiesStatic().tail<3>() - expectedVelocity).norm());
    summary.maxPositionError = std::max(
        summary.maxPositionError,
        (joint->getRelativeTransform().translation() - expectedPosition)
            .norm());
  }

  return summary;
}

GravitySummary summarizeGravityRungeKutta4(double dt, int numSteps)
{
  GravitySummary summary;
  const Eigen::Vector3d gravity(0.0, 0.0, -9.81);
  auto world = createFreeBodyWorld(gravityCase(), dt, gravity);
  const auto skeleton = world->getSkeleton(0);
  auto* joint = static_cast<dart::dynamics::FreeJoint*>(skeleton->getJoint(0));

  const Eigen::Vector3d initialVelocity = gravityCase().linearVelocity;
  const Eigen::Vector3d initialPosition = Eigen::Vector3d::Zero();

  for (int i = 0; i < numSteps; ++i) {
    world->stepUnconstrainedRungeKutta4();

    const double t = dt * static_cast<double>(i + 1);
    const Eigen::Vector3d expectedVelocity = initialVelocity + gravity * t;
    const Eigen::Vector3d expectedPosition
        = initialPosition + initialVelocity * t + 0.5 * gravity * t * t;

    summary.maxVelocityError = std::max(
        summary.maxVelocityError,
        (joint->getVelocitiesStatic().tail<3>() - expectedVelocity).norm());
    summary.maxPositionError = std::max(
        summary.maxPositionError,
        (joint->getRelativeTransform().translation() - expectedPosition)
            .norm());
  }

  return summary;
}

GeneralizedForceSummary summarizeGeneralizedForceWorldStep(
    double dt, int numSteps)
{
  GeneralizedForceSummary summary;
  auto world = createFreeBodyWorld(generalizedForceCase(), dt);
  const auto skeleton = world->getSkeleton(0);
  auto* joint = static_cast<dart::dynamics::FreeJoint*>(skeleton->getJoint(0));

  Eigen::Vector6d generalizedForce = Eigen::Vector6d::Zero();
  generalizedForce[3] = 1.0;
  const Eigen::Vector3d linearAcceleration(generalizedForce[3], 0.0, 0.0);
  Eigen::Vector3d expectedVelocity = generalizedForceCase().linearVelocity;
  Eigen::Vector3d expectedPosition = Eigen::Vector3d::Zero();
  const double energy0 = skeleton->computeKineticEnergy();

  for (int i = 0; i < numSteps; ++i) {
    expectedVelocity += linearAcceleration * dt;
    expectedPosition += expectedVelocity * dt;

    joint->setForces(generalizedForce);
    world->step();

    summary.maxVelocityError = std::max(
        summary.maxVelocityError,
        (joint->getVelocitiesStatic().tail<3>() - expectedVelocity).norm());
    summary.maxPositionError = std::max(
        summary.maxPositionError,
        (joint->getRelativeTransform().translation() - expectedPosition)
            .norm());
  }

  summary.finalEnergyGain = skeleton->computeKineticEnergy() - energy0;
  return summary;
}

AccelerationCommandSummary summarizeAccelerationCommandWorldStep(
    double dt, int numSteps)
{
  AccelerationCommandSummary summary;
  auto world = createFreeBodyWorld(generalizedForceCase(), dt);
  const auto skeleton = world->getSkeleton(0);
  auto* joint = static_cast<dart::dynamics::FreeJoint*>(skeleton->getJoint(0));
  joint->setActuatorType(dart::dynamics::Joint::ACCELERATION);

  const Eigen::Vector3d linearAcceleration(1.0, 0.0, 0.0);
  Eigen::Vector3d expectedVelocity = generalizedForceCase().linearVelocity;
  Eigen::Vector3d expectedPosition = Eigen::Vector3d::Zero();
  const double energy0 = skeleton->computeKineticEnergy();

  for (int i = 0; i < numSteps; ++i) {
    expectedVelocity += linearAcceleration * dt;
    expectedPosition += expectedVelocity * dt;

    joint->setCommand(3, linearAcceleration.x());
    world->step();

    summary.maxVelocityError = std::max(
        summary.maxVelocityError,
        (joint->getVelocitiesStatic().tail<3>() - expectedVelocity).norm());
    summary.maxPositionError = std::max(
        summary.maxPositionError,
        (joint->getRelativeTransform().translation() - expectedPosition)
            .norm());
  }

  summary.finalEnergyGain = skeleton->computeKineticEnergy() - energy0;
  return summary;
}

DampedSummary summarizeDampedWorldStep(double dt, int numSteps)
{
  DampedSummary summary;
  auto world = createFreeBodyWorld(principalAxisCase(), dt);
  const auto skeleton = world->getSkeleton(0);
  setFreeJointDamping(skeleton, 0.4);

  const double energy0 = skeleton->computeKineticEnergy();
  double previousEnergy = energy0;

  for (int i = 0; i < numSteps; ++i) {
    world->step();

    const double energy = skeleton->computeKineticEnergy();
    summary.maxEnergyIncreasePerStep
        = std::max(summary.maxEnergyIncreasePerStep, energy - previousEnergy);
    previousEnergy = energy;
  }

  summary.finalEnergyRatio = previousEnergy / energy0;
  return summary;
}

ForcedTorqueSummary summarizeForcedTorqueWorldStep(double dt, int numSteps)
{
  ForcedTorqueSummary summary;
  auto world = createFreeBodyWorld(principalAxisCase(), dt);
  const auto skeleton = world->getSkeleton(0);
  auto* body = skeleton->getBodyNode(0);

  const Eigen::Vector6d worldMomentum0 = worldSpatialMomentum(skeleton);
  const Eigen::Vector3d worldAngularMomentum0 = worldMomentum0.head<3>();
  const double energy0 = skeleton->computeKineticEnergy();
  const Eigen::Vector3d torque = principalAxisTorque();

  for (int i = 0; i < numSteps; ++i) {
    const double previousEnergy = skeleton->computeKineticEnergy();

    body->addExtTorque(torque, false);
    world->step();

    const Eigen::Vector3d observedAngularImpulse
        = worldSpatialMomentum(skeleton).head<3>() - worldAngularMomentum0;
    const Eigen::Vector3d expectedAngularImpulse
        = torque * dt * static_cast<double>(i + 1);
    const double relativeAngularImpulseError
        = (observedAngularImpulse - expectedAngularImpulse).norm()
          / std::max(1.0, expectedAngularImpulse.norm());

    summary.maxRelativeAngularImpulseError = std::max(
        summary.maxRelativeAngularImpulseError, relativeAngularImpulseError);
    summary.minEnergyGainPerStep = std::min(
        summary.minEnergyGainPerStep,
        skeleton->computeKineticEnergy() - previousEnergy);
  }

  summary.finalEnergyGain = skeleton->computeKineticEnergy() - energy0;
  return summary;
}

void addCounters(benchmark::State& state, const DriftSummary& summary)
{
  state.counters["max_rel_energy_drift"] = summary.maxRelativeEnergyDrift;
  state.counters["max_rel_ang_mom_drift"]
      = summary.maxRelativeAngularMomentumDrift;
  if (summary.maxRelativeWorldAngularMomentumVectorDrift > 0.0) {
    state.counters["max_rel_world_ang_mom_vector_drift"]
        = summary.maxRelativeWorldAngularMomentumVectorDrift;
  }
  if (summary.maxRelativeWorldLinearMomentumVectorDrift > 0.0) {
    state.counters["max_rel_world_linear_mom_vector_drift"]
        = summary.maxRelativeWorldLinearMomentumVectorDrift;
  }

  if (summary.midpointSamples > 0) {
    state.counters["max_midpoint_residual"] = summary.maxMidpointResidual;
    state.counters["max_newton_iters"] = summary.maxNewtonIterations;
    state.counters["avg_newton_iters"]
        = static_cast<double>(summary.newtonIterations)
          / static_cast<double>(summary.midpointSamples);
    state.counters["newton_unconverged_steps"] = summary.newtonUnconvergedSteps;
  }
}

void addCounters(benchmark::State& state, const ForcedTorqueSummary& summary)
{
  state.counters["max_rel_torque_impulse_error"]
      = summary.maxRelativeAngularImpulseError;
  state.counters["min_energy_gain_per_step"] = summary.minEnergyGainPerStep;
  state.counters["final_energy_gain"] = summary.finalEnergyGain;
}

void addCounters(benchmark::State& state, const DampedSummary& summary)
{
  state.counters["max_energy_increase_per_step"]
      = summary.maxEnergyIncreasePerStep;
  state.counters["final_energy_ratio"] = summary.finalEnergyRatio;
}

void addCounters(benchmark::State& state, const GravitySummary& summary)
{
  state.counters["max_gravity_velocity_error"] = summary.maxVelocityError;
  state.counters["max_gravity_position_error"] = summary.maxPositionError;
}

void addCounters(
    benchmark::State& state, const GeneralizedForceSummary& summary)
{
  state.counters["max_generalized_force_velocity_error"]
      = summary.maxVelocityError;
  state.counters["max_generalized_force_position_error"]
      = summary.maxPositionError;
  state.counters["generalized_force_final_energy_gain"]
      = summary.finalEnergyGain;
}

void addCounters(
    benchmark::State& state, const AccelerationCommandSummary& summary)
{
  state.counters["max_acceleration_command_velocity_error"]
      = summary.maxVelocityError;
  state.counters["max_acceleration_command_position_error"]
      = summary.maxPositionError;
  state.counters["acceleration_command_final_energy_gain"]
      = summary.finalEnergyGain;
}

template <typename StepFn>
void runBenchmark(benchmark::State& state, StepFn step)
{
  const double dt = dtFromState(state);
  const int numSteps = stepsFromState(state);

  for (auto _ : state) {
    for (const FreeBodyCase& bodyCase : cases()) {
      Eigen::Vector3d omega = bodyCase.omega;
      const Eigen::LDLT<Eigen::Matrix3d> inertiaSolver(bodyCase.inertia);

      for (int i = 0; i < numSteps; ++i) {
        omega = step(bodyCase.inertia, inertiaSolver, omega, dt);
      }

      benchmark::DoNotOptimize(omega.data());
    }
  }

  state.SetItemsProcessed(
      state.iterations() * static_cast<int64_t>(cases().size() * numSteps));
}

void runWorldStepBenchmark(benchmark::State& state)
{
  const double dt = dtFromState(state);
  const int numSteps = stepsFromState(state);

  for (auto _ : state) {
    for (const FreeBodyCase& bodyCase : cases()) {
      state.PauseTiming();
      auto world = createFreeBodyWorld(bodyCase, dt);
      const auto skeleton = world->getSkeleton(0);
      state.ResumeTiming();

      for (int i = 0; i < numSteps; ++i) {
        world->step();

        if (!skeleton->getVelocities().allFinite()) {
          break;
        }
      }

      benchmark::DoNotOptimize(skeleton->getVelocities().data());
    }
  }

  state.SetItemsProcessed(
      state.iterations() * static_cast<int64_t>(cases().size() * numSteps));
}

void runWorldStepBenchmark(
    benchmark::State& state, const FreeBodyCase& bodyCase)
{
  const double dt = dtFromState(state);
  const int numSteps = stepsFromState(state);

  for (auto _ : state) {
    state.PauseTiming();
    auto world = createFreeBodyWorld(bodyCase, dt);
    const auto skeleton = world->getSkeleton(0);
    state.ResumeTiming();

    for (int i = 0; i < numSteps; ++i) {
      world->step();

      if (!skeleton->getVelocities().allFinite()) {
        break;
      }
    }

    benchmark::DoNotOptimize(skeleton->getVelocities().data());
  }

  state.SetItemsProcessed(state.iterations() * static_cast<int64_t>(numSteps));
}

void runAccelerationCommandWorldStepBenchmark(benchmark::State& state)
{
  const double dt = dtFromState(state);
  const int numSteps = stepsFromState(state);
  const Eigen::Vector3d linearAcceleration(1.0, 0.0, 0.0);

  for (auto _ : state) {
    state.PauseTiming();
    auto world = createFreeBodyWorld(generalizedForceCase(), dt);
    const auto skeleton = world->getSkeleton(0);
    auto* joint
        = static_cast<dart::dynamics::FreeJoint*>(skeleton->getJoint(0));
    joint->setActuatorType(dart::dynamics::Joint::ACCELERATION);
    state.ResumeTiming();

    for (int i = 0; i < numSteps; ++i) {
      joint->setCommand(3, linearAcceleration.x());
      world->step();

      if (!skeleton->getVelocities().allFinite()) {
        break;
      }
    }

    benchmark::DoNotOptimize(skeleton->getVelocities().data());
  }

  state.SetItemsProcessed(state.iterations() * static_cast<int64_t>(numSteps));
}

void runGeneralizedForceWorldStepBenchmark(benchmark::State& state)
{
  const double dt = dtFromState(state);
  const int numSteps = stepsFromState(state);
  Eigen::Vector6d generalizedForce = Eigen::Vector6d::Zero();
  generalizedForce[3] = 1.0;

  for (auto _ : state) {
    state.PauseTiming();
    auto world = createFreeBodyWorld(generalizedForceCase(), dt);
    const auto skeleton = world->getSkeleton(0);
    auto* joint
        = static_cast<dart::dynamics::FreeJoint*>(skeleton->getJoint(0));
    state.ResumeTiming();

    for (int i = 0; i < numSteps; ++i) {
      joint->setForces(generalizedForce);
      world->step();

      if (!skeleton->getVelocities().allFinite()) {
        break;
      }
    }

    benchmark::DoNotOptimize(skeleton->getVelocities().data());
  }

  state.SetItemsProcessed(state.iterations() * static_cast<int64_t>(numSteps));
}

void runGravityWorldStepBenchmark(benchmark::State& state)
{
  const double dt = dtFromState(state);
  const int numSteps = stepsFromState(state);
  const Eigen::Vector3d gravity(0.0, 0.0, -9.81);

  for (auto _ : state) {
    state.PauseTiming();
    auto world = createFreeBodyWorld(gravityCase(), dt, gravity);
    const auto skeleton = world->getSkeleton(0);
    state.ResumeTiming();

    for (int i = 0; i < numSteps; ++i) {
      world->step();

      if (!skeleton->getVelocities().allFinite()) {
        break;
      }
    }

    benchmark::DoNotOptimize(skeleton->getVelocities().data());
  }

  state.SetItemsProcessed(state.iterations() * static_cast<int64_t>(numSteps));
}

void runGravityRungeKutta4Benchmark(benchmark::State& state)
{
  const double dt = dtFromState(state);
  const int numSteps = stepsFromState(state);
  const Eigen::Vector3d gravity(0.0, 0.0, -9.81);

  for (auto _ : state) {
    state.PauseTiming();
    auto world = createFreeBodyWorld(gravityCase(), dt, gravity);
    const auto skeleton = world->getSkeleton(0);
    state.ResumeTiming();

    for (int i = 0; i < numSteps; ++i) {
      world->stepUnconstrainedRungeKutta4();

      if (!skeleton->getVelocities().allFinite()) {
        break;
      }
    }

    benchmark::DoNotOptimize(skeleton->getVelocities().data());
  }

  state.SetItemsProcessed(state.iterations() * static_cast<int64_t>(numSteps));
}

void runDampedWorldStepBenchmark(benchmark::State& state)
{
  const double dt = dtFromState(state);
  const int numSteps = stepsFromState(state);

  for (auto _ : state) {
    state.PauseTiming();
    auto world = createFreeBodyWorld(principalAxisCase(), dt);
    const auto skeleton = world->getSkeleton(0);
    setFreeJointDamping(skeleton, 0.4);
    state.ResumeTiming();

    for (int i = 0; i < numSteps; ++i) {
      world->step();

      if (!skeleton->getVelocities().allFinite()) {
        break;
      }
    }

    benchmark::DoNotOptimize(skeleton->getVelocities().data());
  }

  state.SetItemsProcessed(state.iterations() * static_cast<int64_t>(numSteps));
}

void runForcedTorqueWorldStepBenchmark(benchmark::State& state)
{
  const double dt = dtFromState(state);
  const int numSteps = stepsFromState(state);
  const Eigen::Vector3d torque = principalAxisTorque();

  for (auto _ : state) {
    state.PauseTiming();
    auto world = createFreeBodyWorld(principalAxisCase(), dt);
    const auto skeleton = world->getSkeleton(0);
    auto* body = skeleton->getBodyNode(0);
    state.ResumeTiming();

    for (int i = 0; i < numSteps; ++i) {
      body->addExtTorque(torque, false);
      world->step();

      if (!skeleton->getVelocities().allFinite()) {
        break;
      }
    }

    benchmark::DoNotOptimize(skeleton->getVelocities().data());
  }

  state.SetItemsProcessed(state.iterations() * static_cast<int64_t>(numSteps));
}

} // namespace

static void BM_FreeBodyEnergy_ExplicitEuler(benchmark::State& state)
{
  runBenchmark(state, explicitEulerStep);
  addCounters(
      state,
      summarizeDrift(
          dtFromState(state), stepsFromState(state), explicitEulerStep));
}

static void BM_FreeBodyEnergy_WorldStep(benchmark::State& state)
{
  runWorldStepBenchmark(state);
  state.PauseTiming();
  addCounters(
      state,
      summarizeWorldStepDrift(dtFromState(state), stepsFromState(state)));
  state.ResumeTiming();
}

static void BM_FreeBodyEnergy_PrincipalAxisWorldStep(benchmark::State& state)
{
  runWorldStepBenchmark(state, principalAxisCase());
  state.PauseTiming();
  addCounters(
      state,
      summarizeWorldStepDrift(
          principalAxisCase(), dtFromState(state), stepsFromState(state)));
  state.ResumeTiming();
}

static void BM_FreeBodyEnergy_GravityWorldStep(benchmark::State& state)
{
  runGravityWorldStepBenchmark(state);
  state.PauseTiming();
  addCounters(
      state,
      summarizeGravityWorldStep(dtFromState(state), stepsFromState(state)));
  state.ResumeTiming();
}

static void BM_FreeBodyEnergy_GravityRungeKutta4(benchmark::State& state)
{
  runGravityRungeKutta4Benchmark(state);
  state.PauseTiming();
  addCounters(
      state,
      summarizeGravityRungeKutta4(dtFromState(state), stepsFromState(state)));
  state.ResumeTiming();
}

static void BM_FreeBodyEnergy_GeneralizedForceWorldStep(benchmark::State& state)
{
  runGeneralizedForceWorldStepBenchmark(state);
  state.PauseTiming();
  addCounters(
      state,
      summarizeGeneralizedForceWorldStep(
          dtFromState(state), stepsFromState(state)));
  state.ResumeTiming();
}

static void BM_FreeBodyEnergy_AccelerationCommandWorldStep(
    benchmark::State& state)
{
  runAccelerationCommandWorldStepBenchmark(state);
  state.PauseTiming();
  addCounters(
      state,
      summarizeAccelerationCommandWorldStep(
          dtFromState(state), stepsFromState(state)));
  state.ResumeTiming();
}

static void BM_FreeBodyEnergy_DampedWorldStep(benchmark::State& state)
{
  runDampedWorldStepBenchmark(state);
  state.PauseTiming();
  addCounters(
      state,
      summarizeDampedWorldStep(dtFromState(state), stepsFromState(state)));
  state.ResumeTiming();
}

static void BM_FreeBodyEnergy_ForcedTorqueWorldStep(benchmark::State& state)
{
  runForcedTorqueWorldStepBenchmark(state);
  state.PauseTiming();
  addCounters(
      state,
      summarizeForcedTorqueWorldStep(
          dtFromState(state), stepsFromState(state)));
  state.ResumeTiming();
}

static void BM_FreeBodyEnergy_RungeKutta4(benchmark::State& state)
{
  runBenchmark(state, rungeKutta4Step);
  addCounters(
      state,
      summarizeDrift(
          dtFromState(state), stepsFromState(state), rungeKutta4Step));
}

static void BM_FreeBodyEnergy_RungeKutta4TwoHalfSteps(benchmark::State& state)
{
  runBenchmark(state, rungeKutta4TwoHalfSteps);
  addCounters(
      state,
      summarizeDrift(
          dtFromState(state), stepsFromState(state), rungeKutta4TwoHalfSteps));
}

static void BM_FreeBodyEnergy_Midpoint1(benchmark::State& state)
{
  runBenchmark(state, midpointStep1);
  addCounters(
      state,
      summarizeMidpointDrift(dtFromState(state), stepsFromState(state), 1));
}

static void BM_FreeBodyEnergy_Midpoint2(benchmark::State& state)
{
  runBenchmark(state, midpointStep2);
  addCounters(
      state,
      summarizeMidpointDrift(dtFromState(state), stepsFromState(state), 2));
}

static void BM_FreeBodyEnergy_Midpoint4(benchmark::State& state)
{
  runBenchmark(state, midpointStep4);
  addCounters(
      state,
      summarizeMidpointDrift(dtFromState(state), stepsFromState(state), 4));
}

static void BM_FreeBodyEnergy_Midpoint8(benchmark::State& state)
{
  runBenchmark(state, midpointStep8);
  addCounters(
      state,
      summarizeMidpointDrift(dtFromState(state), stepsFromState(state), 8));
}

BENCHMARK(BM_FreeBodyEnergy_ExplicitEuler)
    ->Args({1000, 1000})
    ->Args({5000, 1000})
    ->Args({10000, 1000});
BENCHMARK(BM_FreeBodyEnergy_WorldStep)
    ->Args({1000, 1000})
    ->Args({5000, 1000})
    ->Args({10000, 1000});
BENCHMARK(BM_FreeBodyEnergy_PrincipalAxisWorldStep)
    ->Args({1000, 1000})
    ->Args({5000, 1000})
    ->Args({10000, 1000});
BENCHMARK(BM_FreeBodyEnergy_GravityWorldStep)
    ->Args({1000, 1000})
    ->Args({5000, 1000})
    ->Args({10000, 1000});
BENCHMARK(BM_FreeBodyEnergy_GravityRungeKutta4)
    ->Args({1000, 1000})
    ->Args({5000, 1000})
    ->Args({10000, 1000});
BENCHMARK(BM_FreeBodyEnergy_GeneralizedForceWorldStep)
    ->Args({1000, 1000})
    ->Args({5000, 1000})
    ->Args({10000, 1000});
BENCHMARK(BM_FreeBodyEnergy_AccelerationCommandWorldStep)
    ->Args({1000, 1000})
    ->Args({5000, 1000})
    ->Args({10000, 1000});
BENCHMARK(BM_FreeBodyEnergy_DampedWorldStep)
    ->Args({1000, 1000})
    ->Args({5000, 1000})
    ->Args({10000, 1000});
BENCHMARK(BM_FreeBodyEnergy_ForcedTorqueWorldStep)
    ->Args({1000, 1000})
    ->Args({5000, 1000})
    ->Args({10000, 1000});
BENCHMARK(BM_FreeBodyEnergy_RungeKutta4)
    ->Args({1000, 1000})
    ->Args({5000, 1000})
    ->Args({10000, 1000});
BENCHMARK(BM_FreeBodyEnergy_RungeKutta4TwoHalfSteps)
    ->Args({1000, 1000})
    ->Args({5000, 1000})
    ->Args({10000, 1000});
BENCHMARK(BM_FreeBodyEnergy_Midpoint1)
    ->Args({1000, 1000})
    ->Args({5000, 1000})
    ->Args({10000, 1000});
BENCHMARK(BM_FreeBodyEnergy_Midpoint2)
    ->Args({1000, 1000})
    ->Args({5000, 1000})
    ->Args({10000, 1000});
BENCHMARK(BM_FreeBodyEnergy_Midpoint4)
    ->Args({1000, 1000})
    ->Args({5000, 1000})
    ->Args({10000, 1000});
BENCHMARK(BM_FreeBodyEnergy_Midpoint8)
    ->Args({1000, 1000})
    ->Args({5000, 1000})
    ->Args({10000, 1000});

BENCHMARK_MAIN();
