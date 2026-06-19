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

#include "dart/simulation/diff/rollout.hpp"

#include "dart/simulation/common/exceptions.hpp"
#include "dart/simulation/world.hpp"

#include <utility>

namespace dart::simulation::diff {

namespace {

//==============================================================================
void validateSupportedDifferentiableRolloutLayout(const World& world)
{
  const std::size_t rigidDofs = world.getNumRigidBodyDofs();
  DART_SIMULATION_THROW_T_IF(
      rigidDofs != 0 && rigidDofs != world.getNumDofs(),
      NotImplementedException,
      "diff::rollout(): mixed rigid-body plus multibody differentiable "
      "rollouts are not supported until full-world step Jacobians are "
      "assembled; use a rigid-only or multibody-only differentiable World");
}

} // namespace

//==============================================================================
RolloutTrajectory rollout(
    World& world,
    const Eigen::VectorXd& initialStateVector,
    const Eigen::MatrixXd& controlSequence,
    std::size_t steps)
{
  DART_SIMULATION_THROW_T_IF(
      steps == 0,
      InvalidArgumentException,
      "diff::rollout(): steps must be >= 1");

  validateSupportedDifferentiableRolloutLayout(world);

  const auto stateSize = static_cast<Eigen::Index>(2 * world.getNumDofs());
  const auto controlSize = static_cast<Eigen::Index>(world.getNumEfforts());

  DART_SIMULATION_THROW_T_IF(
      initialStateVector.size() != stateSize,
      InvalidArgumentException,
      "diff::rollout(): initialStateVector has size {} but expected {} "
      "(= 2 * num_dofs)",
      initialStateVector.size(),
      stateSize);

  DART_SIMULATION_THROW_T_IF(
      controlSequence.rows() != static_cast<Eigen::Index>(steps)
          || controlSequence.cols() != controlSize,
      InvalidArgumentException,
      "diff::rollout(): controlSequence is {}x{} but expected {}x{} "
      "(= steps x num_efforts)",
      controlSequence.rows(),
      controlSequence.cols(),
      steps,
      controlSize);

  RolloutTrajectory trajectory;
  trajectory.states.reserve(steps + 1);
  trajectory.stepDerivatives.reserve(steps);

  // Forward: seed the initial state, then chain single steps. Each step's
  // Jacobian is captured at its pre-step configuration (with that step's
  // control applied), matching the per-step autograd `timestep` chaining.
  world.setStateVector(initialStateVector);
  trajectory.states.push_back(initialStateVector);

  for (std::size_t t = 0; t < steps; ++t) {
    const Eigen::VectorXd control
        = controlSequence.row(static_cast<Eigen::Index>(t)).transpose();
    world.setControlVector(control);
    world.step();
    trajectory.stepDerivatives.push_back(world.getStepDerivatives());
    trajectory.states.push_back(world.getStateVector());
  }

  return trajectory;
}

//==============================================================================
RolloutGradient RolloutTrajectory::rolloutVjp(
    const Eigen::VectorXd& finalStateGrad) const
{
  const std::size_t steps = numSteps();

  // Equivalent to rolloutVjpPerStep with only the final-state gradient nonzero,
  // expressed directly so the common (final-loss) case avoids allocating the
  // zero intermediate gradients.
  DART_SIMULATION_THROW_T_IF(
      steps == 0,
      InvalidArgumentException,
      "RolloutTrajectory::rolloutVjp(): empty trajectory has no gradient");

  const Eigen::Index stateSize = stepDerivatives.front().stateJacobian.rows();
  DART_SIMULATION_THROW_T_IF(
      finalStateGrad.size() != stateSize,
      InvalidArgumentException,
      "RolloutTrajectory::rolloutVjp(): finalStateGrad has size {} but the "
      "rollout final state has size {}",
      finalStateGrad.size(),
      stateSize);

  RolloutGradient gradient;
  gradient.controlGrads.resize(steps);

  // Reverse-mode chain: adjoint g = dL/dx_{t+1}, pulled back through step t.
  Eigen::VectorXd g = finalStateGrad;
  for (std::size_t t = steps; t-- > 0;) {
    const StepDerivatives& d = stepDerivatives[t];
    gradient.controlGrads[t] = d.controlJacobian.transpose() * g;
    g = d.stateJacobian.transpose() * g;
  }
  gradient.initialStateGrad = std::move(g);

  return gradient;
}

//==============================================================================
RolloutGradient RolloutTrajectory::rolloutVjpPerStep(
    const std::vector<Eigen::VectorXd>& stateGrads) const
{
  const std::size_t steps = numSteps();
  DART_SIMULATION_THROW_T_IF(
      steps == 0,
      InvalidArgumentException,
      "RolloutTrajectory::rolloutVjpPerStep(): empty trajectory has no "
      "gradient");

  DART_SIMULATION_THROW_T_IF(
      stateGrads.size() != steps + 1,
      InvalidArgumentException,
      "RolloutTrajectory::rolloutVjpPerStep(): stateGrads has {} entries but "
      "expected {} (= steps + 1)",
      stateGrads.size(),
      steps + 1);

  const Eigen::Index stateSize = stepDerivatives.front().stateJacobian.rows();
  for (std::size_t t = 0; t < stateGrads.size(); ++t) {
    DART_SIMULATION_THROW_T_IF(
        stateGrads[t].size() != stateSize,
        InvalidArgumentException,
        "RolloutTrajectory::rolloutVjpPerStep(): stateGrads[{}] has size {} "
        "but expected {} (= 2 * ndof)",
        t,
        stateGrads[t].size(),
        stateSize);
  }

  RolloutGradient gradient;
  gradient.controlGrads.resize(steps);

  // Accumulate the per-state local gradient into the adjoint as it is pulled
  // back through the chain.
  Eigen::VectorXd g = stateGrads[steps];
  for (std::size_t t = steps; t-- > 0;) {
    const StepDerivatives& d = stepDerivatives[t];
    gradient.controlGrads[t] = d.controlJacobian.transpose() * g;
    g = stateGrads[t] + d.stateJacobian.transpose() * g;
  }
  gradient.initialStateGrad = std::move(g);

  return gradient;
}

} // namespace dart::simulation::diff
