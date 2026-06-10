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
 *     copyright notice and this list of conditions in the documentation
 *     and/or other materials provided with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <dart/simulation/detail/newton_barrier/barrier_kernel.hpp>
#include <dart/simulation/detail/newton_barrier/psd_projection.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <limits>

#include <cmath>

namespace dart::simulation::detail::newton_barrier {

inline constexpr double kDefaultRestitutionActivationSpeed = 1e-3;
inline constexpr double kDefaultFallingBoxTimeStep = 1e-3;
inline constexpr double kDefaultFallingBoxMass = 1.0;
inline constexpr double kDefaultFallingBoxGravity = 9.81;
inline constexpr double kDefaultFallingBoxActivationDistance = 1e-3;
inline constexpr double kDefaultFallingBoxBarrierStiffness = 1.0;
inline constexpr double kDefaultFallingBoxYoungModulus = 1e6;
inline constexpr double kDefaultRayleighDampingCoefficient = 0.0;

//==============================================================================
inline double sanitizeUnitInterval(const double value)
{
  if (!std::isfinite(value)) {
    return 0.0;
  }
  return std::clamp(value, 0.0, 1.0);
}

//==============================================================================
struct RestitutionVelocityTarget
{
  bool active = false;
  double approachVelocity = 0.0;
  double coefficient = 0.0;
  double activationSpeed = kDefaultRestitutionActivationSpeed;
  double targetSeparatingVelocity = 0.0;
};

//==============================================================================
inline RestitutionVelocityTarget makeRestitutionVelocityTarget(
    const double approachVelocity,
    const double coefficient,
    const double activationSpeed = kDefaultRestitutionActivationSpeed)
{
  RestitutionVelocityTarget target;
  target.approachVelocity = approachVelocity;
  target.coefficient = sanitizeUnitInterval(coefficient);
  target.activationSpeed
      = std::isfinite(activationSpeed) ? std::max(0.0, activationSpeed) : 0.0;

  if (!std::isfinite(approachVelocity) || target.coefficient <= 0.0
      || !(approachVelocity < -target.activationSpeed)) {
    return target;
  }

  target.active = true;
  target.targetSeparatingVelocity = -target.coefficient * approachVelocity;
  return target;
}

//==============================================================================
template <int Size>
struct Bdf2StepHistory
{
  using Vector = Eigen::Matrix<double, Size, 1>;

  bool active = true;
  bool hasPreviousPosition = false;
  Vector previousPosition = Vector::Zero();
  Vector currentPosition = Vector::Zero();
  Vector currentVelocity = Vector::Zero();
};

//==============================================================================
template <int Size>
struct Bdf2InertialTerm
{
  using Vector = Eigen::Matrix<double, Size, 1>;

  bool active = false;
  bool restarted = false;
  int order = 0;
  double timeStep = 0.0;
  double mass = 0.0;
  double scalarWeight = 0.0;
  Vector targetPosition = Vector::Zero();
};

//==============================================================================
template <int Size>
struct Bdf2VelocityUpdate
{
  using Vector = Eigen::Matrix<double, Size, 1>;

  bool active = false;
  bool restarted = false;
  int order = 0;
  double timeStep = 0.0;
  Vector velocity = Vector::Zero();
};

//==============================================================================
template <int Size>
[[nodiscard]] bool isFiniteVector(const Eigen::Matrix<double, Size, 1>& vector)
{
  return vector.allFinite();
}

//==============================================================================
template <int Size>
[[nodiscard]] Bdf2InertialTerm<Size> makeBdf2InertialTerm(
    const Bdf2StepHistory<Size>& history,
    const double mass,
    const double timeStep)
{
  Bdf2InertialTerm<Size> term;
  term.timeStep = timeStep;
  term.mass = mass;

  const bool validCurrent = history.active && std::isfinite(timeStep)
                            && timeStep > 0.0 && std::isfinite(mass)
                            && mass > 0.0
                            && isFiniteVector<Size>(history.currentPosition)
                            && isFiniteVector<Size>(history.currentVelocity);
  if (!validCurrent) {
    return term;
  }

  const double inverseTimeStepSquared = 1.0 / (timeStep * timeStep);
  if (!std::isfinite(inverseTimeStepSquared)) {
    return term;
  }

  term.active = true;
  if (history.hasPreviousPosition
      && isFiniteVector<Size>(history.previousPosition)) {
    term.order = 2;
    term.scalarWeight = 2.25 * mass * inverseTimeStepSquared;
    term.targetPosition
        = (4.0 * history.currentPosition - history.previousPosition) / 3.0;
    return term;
  }

  term.restarted = true;
  term.order = 1;
  term.scalarWeight = mass * inverseTimeStepSquared;
  term.targetPosition
      = history.currentPosition + timeStep * history.currentVelocity;
  return term;
}

//==============================================================================
template <int Size>
[[nodiscard]] double evaluateInertialEnergy(
    const Bdf2InertialTerm<Size>& term,
    const Eigen::Matrix<double, Size, 1>& position)
{
  if (!term.active || !isFiniteVector<Size>(position)
      || !std::isfinite(term.scalarWeight)) {
    return 0.0;
  }

  const Eigen::Matrix<double, Size, 1> displacement
      = position - term.targetPosition;
  return 0.5 * term.scalarWeight * displacement.squaredNorm();
}

//==============================================================================
template <int Size>
[[nodiscard]] Bdf2VelocityUpdate<Size> makeBdf2VelocityUpdate(
    const Bdf2StepHistory<Size>& history,
    const Eigen::Matrix<double, Size, 1>& acceptedPosition,
    const double timeStep)
{
  Bdf2VelocityUpdate<Size> update;
  update.timeStep = timeStep;

  const bool validCurrent = history.active && std::isfinite(timeStep)
                            && timeStep > 0.0
                            && isFiniteVector<Size>(history.currentPosition)
                            && isFiniteVector<Size>(acceptedPosition);
  if (!validCurrent) {
    return update;
  }

  update.active = true;
  if (history.hasPreviousPosition
      && isFiniteVector<Size>(history.previousPosition)) {
    update.order = 2;
    update.velocity = (3.0 * acceptedPosition - 4.0 * history.currentPosition
                       + history.previousPosition)
                      / (2.0 * timeStep);
    return update;
  }

  update.restarted = true;
  update.order = 1;
  update.velocity = (acceptedPosition - history.currentPosition) / timeStep;
  return update;
}

//==============================================================================
template <int Size>
[[nodiscard]] Bdf2StepHistory<Size> advanceBdf2History(
    const Bdf2StepHistory<Size>& history,
    const Eigen::Matrix<double, Size, 1>& acceptedPosition,
    const Eigen::Matrix<double, Size, 1>& acceptedVelocity)
{
  Bdf2StepHistory<Size> next = history;
  next.hasPreviousPosition = history.active
                             && isFiniteVector<Size>(history.currentPosition)
                             && isFiniteVector<Size>(acceptedPosition)
                             && isFiniteVector<Size>(acceptedVelocity);
  if (!next.hasPreviousPosition) {
    next.active = false;
    return next;
  }

  next.previousPosition = history.currentPosition;
  next.currentPosition = acceptedPosition;
  next.currentVelocity = acceptedVelocity;
  return next;
}

//==============================================================================
struct FallingBoxEnergyDiagnosticOptions
{
  /// Time step in seconds. The diagnostic records it for sweep provenance.
  double timeStep = kDefaultFallingBoxTimeStep;
  /// Rigid/stiff-body mass in kilograms.
  double mass = kDefaultFallingBoxMass;
  /// Box center height in meters above the reference plane.
  double height = 0.0;
  /// Vertical velocity in meters per second; sign is irrelevant to energy.
  double velocity = 0.0;
  /// Gravity magnitude in meters per second squared.
  double gravity = kDefaultFallingBoxGravity;
  /// Unsigned contact clearance in meters.
  double clearance = kDefaultFallingBoxActivationDistance;
  /// Barrier activation distance in meters.
  double activationDistance = kDefaultFallingBoxActivationDistance;
  /// Barrier stiffness coefficient kappa in the clamped-log energy scale.
  double barrierStiffness = kDefaultFallingBoxBarrierStiffness;
  /// Young's modulus in pascals for the stiff-body elastic diagnostic proxy.
  double youngModulus = kDefaultFallingBoxYoungModulus;
  /// Compression magnitude in meters for the elastic diagnostic proxy.
  double compression = 0.0;
  /// Geometry scale that maps Young's modulus and compression to joules.
  double elasticEnergyScale = 1e-6;
};

//==============================================================================
struct FallingBoxEnergyDiagnostic
{
  bool active = false;
  bool barrierActive = false;
  double timeStep = 0.0;
  double kineticEnergy = 0.0;
  double gravitationalEnergy = 0.0;
  double barrierEnergy = 0.0;
  double elasticEnergy = 0.0;
  double totalEnergy = 0.0;
};

//==============================================================================
inline FallingBoxEnergyDiagnostic makeFallingBoxEnergyDiagnostic(
    const FallingBoxEnergyDiagnosticOptions& options)
{
  FallingBoxEnergyDiagnostic diagnostic;
  diagnostic.timeStep = options.timeStep;

  const bool valid
      = std::isfinite(options.timeStep) && options.timeStep > 0.0
        && std::isfinite(options.mass) && options.mass > 0.0
        && std::isfinite(options.height) && std::isfinite(options.velocity)
        && std::isfinite(options.gravity) && options.gravity >= 0.0
        && std::isfinite(options.clearance) && options.clearance >= 0.0
        && std::isfinite(options.activationDistance)
        && options.activationDistance > 0.0
        && std::isfinite(options.barrierStiffness)
        && options.barrierStiffness >= 0.0
        && std::isfinite(options.youngModulus) && options.youngModulus >= 0.0
        && std::isfinite(options.compression) && options.compression >= 0.0
        && std::isfinite(options.elasticEnergyScale)
        && options.elasticEnergyScale >= 0.0;
  if (!valid) {
    return diagnostic;
  }

  diagnostic.active = true;
  diagnostic.kineticEnergy
      = 0.5 * options.mass * options.velocity * options.velocity;
  diagnostic.gravitationalEnergy
      = options.mass * options.gravity * options.height;

  const double squaredClearance = options.clearance * options.clearance;
  const double squaredActivation
      = options.activationDistance * options.activationDistance;
  const auto barrier = c2ClampedLogBarrier(squaredClearance, squaredActivation);
  diagnostic.barrierActive = barrier.active;
  diagnostic.barrierEnergy = options.barrierStiffness * barrier.value;

  diagnostic.elasticEnergy = 0.5 * options.youngModulus
                             * options.elasticEnergyScale * options.compression
                             * options.compression;
  diagnostic.totalEnergy
      = diagnostic.kineticEnergy + diagnostic.gravitationalEnergy
        + diagnostic.barrierEnergy + diagnostic.elasticEnergy;
  return diagnostic;
}

//==============================================================================
template <int Size>
struct RayleighDampingTerm
{
  using Vector = Eigen::Matrix<double, Size, 1>;
  using Matrix = Eigen::Matrix<double, Size, Size>;

  bool active = false;
  double coefficient = kDefaultRayleighDampingCoefficient;
  double timeStep = 0.0;
  double scale = 0.0;
  double energy = 0.0;
  Vector gradient = Vector::Zero();
  Vector generalizedForce = Vector::Zero();
  Matrix hessian = Matrix::Zero();
};

//==============================================================================
template <int Size>
[[nodiscard]] RayleighDampingTerm<Size> makeSemiImplicitRayleighDampingTerm(
    const Eigen::Matrix<double, Size, 1>& displacement,
    const Eigen::Matrix<double, Size, Size>& potentialHessian,
    const double coefficient,
    const double timeStep)
{
  RayleighDampingTerm<Size> term;
  term.coefficient = coefficient;
  term.timeStep = timeStep;

  const bool valid = std::isfinite(coefficient) && coefficient > 0.0
                     && std::isfinite(timeStep) && timeStep > 0.0
                     && isFiniteVector<Size>(displacement)
                     && potentialHessian.allFinite();
  if (!valid) {
    return term;
  }

  const auto psdHessian = projectSymmetricMatrixToPsd<Size>(potentialHessian);
  if (!psdHessian.allFinite()) {
    return term;
  }

  term.scale = coefficient / timeStep;
  if (!std::isfinite(term.scale)) {
    return term;
  }

  term.active = true;
  term.hessian = term.scale * psdHessian;
  term.gradient = term.hessian * displacement;
  term.generalizedForce = -term.gradient;
  term.energy = 0.5 * displacement.dot(term.gradient);
  return term;
}

} // namespace dart::simulation::detail::newton_barrier
