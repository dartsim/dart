/*
 * Copyright (c) 2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#include "dart/dynamics/Ikfast.hpp"

#include "dart/dynamics/detail/ikfast.h"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/IkfastSolver.hpp"
#include "dart/dynamics/DegreeOfFreedom.hpp"

namespace dart {
namespace dynamics {

namespace {

//==============================================================================
void toEigenTypes(
    const IkReal* eetrans, const IkReal* eerot, Eigen::Isometry3d& tf)
{
  tf.linear()(0, 0) = eerot[0*3+0];
  tf.linear()(0, 1) = eerot[0*3+1];
  tf.linear()(0, 2) = eerot[0*3+2];
  tf.linear()(1, 0) = eerot[1*3+0];
  tf.linear()(1, 1) = eerot[1*3+1];
  tf.linear()(1, 2) = eerot[1*3+2];
  tf.linear()(2, 0) = eerot[2*3+0];
  tf.linear()(2, 1) = eerot[2*3+1];
  tf.linear()(2, 2) = eerot[2*3+2];

  tf.translation() << eetrans[0], eetrans[1], eetrans[2];
}

//==============================================================================
void toIkfastTypes(
    const Eigen::Isometry3d& tf,
    std::array<IkReal, 3>& eetrans,
    std::array<IkReal, 9>& eerot)
{
  eerot[0*3+0] = tf.linear()(0, 0);
  eerot[0*3+1] = tf.linear()(0, 1);
  eerot[0*3+2] = tf.linear()(0, 2);
  eerot[1*3+0] = tf.linear()(1, 0);
  eerot[1*3+1] = tf.linear()(1, 1);
  eerot[1*3+2] = tf.linear()(1, 2);
  eerot[2*3+0] = tf.linear()(2, 0);
  eerot[2*3+1] = tf.linear()(2, 1);
  eerot[2*3+2] = tf.linear()(2, 2);

  eetrans[0] = tf.translation()[0];
  eetrans[1] = tf.translation()[1];
  eetrans[2] = tf.translation()[2];
}

//==============================================================================
void toIkfastTypes(
    const Ikfast* ikfast,
    const ikfast::IkSolutionBase<IkReal>& ikfastSolution,
    InverseKinematics::Analytical::Solution& solution)
{
  const auto solver = ikfast->getSolver();
  const auto ik = ikfast->getIK();
  const auto dofIndices = ik->getDofs();
  const auto skel = ik->getNode()->getSkeleton();
  const auto dofs = skel->getDofs();
  // TODO(JS): simplify

  std::vector<IkReal> solutionValues(solver->getNumJoints());
  std::vector<IkReal> freeValues(solver->getNumFreeParameters());

  ikfastSolution.GetSolution(solutionValues, freeValues);

  bool valid = true;

  solution.mConfig.resize(solutionValues.size());
  for (auto i = 0u; i < solution.mConfig.size(); ++i)
  {
    solution.mConfig[i] = solutionValues[i];

    if (solutionValues[i] < dofs[dofIndices[i]]->getPositionLowerLimit())
      valid = false;

    if (solutionValues[i] > dofs[dofIndices[i]]->getPositionUpperLimit())
      valid = false;
  }

  solution.mValidity
      = valid ? InverseKinematics::Analytical::VALID
              : InverseKinematics::Analytical::OUT_OF_REACH;
}

//==============================================================================
void toIkfastTypes(
    const Ikfast* ikfast,
    const ikfast::IkSolutionList<IkReal>& ikfastSolutions,
    std::vector<InverseKinematics::Analytical::Solution>& solutions)
{
  const auto numSolutions = ikfastSolutions.GetNumSolutions();

  solutions.resize(numSolutions);

  for (auto i = 0u; i < numSolutions; ++i)
  {
    const auto& ikfastSolution = ikfastSolutions.GetSolution(i);
    toIkfastTypes(ikfast, ikfastSolution, solutions[i]);
  }
}

} // namespace (anonymous)

//==============================================================================
Ikfast::Ikfast(InverseKinematics* ik,
    const std::shared_ptr<IkfastSolver>& ikfastSolver,
    const std::string& methodName,
    const InverseKinematics::Analytical::Properties& properties)
  : Analytical(ik, methodName, properties),
    mSolver{ikfastSolver},
    mConfigured{false}
{
  // Do nothing
}

//==============================================================================
std::unique_ptr<InverseKinematics::GradientMethod> Ikfast::clone(
    InverseKinematics* newIK) const
{
  return dart::common::make_unique<Ikfast>(
        newIK, mSolver, "todo", getAnalyticalProperties());
}

//==============================================================================
const std::vector<std::size_t>& Ikfast::getDofs() const
{
  if(!mConfigured)
    configure();

  return mDofs;
}

//==============================================================================
std::shared_ptr<IkfastSolver> Ikfast::getSolver() const
{
  return mSolver;
}

//==============================================================================
void Ikfast::configure() const
{
  const auto metaSkeleton = mIK->getNode()->getSkeleton();

  const auto dofs = metaSkeleton->getDofs();
  const auto numDofs = dofs.size();

  const auto ikfastNumJoints = mSolver->getNumJoints();
  const auto ikfastNumFreeParams = mSolver->getNumFreeParameters();
  const auto ikfastFreeParams = mSolver->getFreeParameters();

  if (numDofs != static_cast<std::size_t>(ikfastNumJoints))
  {
    dterr << "[Ikfast::configure] Failed to configure: "
          << "the DOFs don't agree with the target skeleton and the IKFast "
          << "solver's.\n";
    return;
  }

  for (auto i = 0u; i < numDofs; ++i)
  {
    bool isFreeJoint = false;
    for (auto j = 0; j < ikfastNumFreeParams; ++j)
    {
      if (i == static_cast<std::size_t>(ikfastFreeParams[j]))
      {
        isFreeJoint = true;
        break;
      }
    }

    if (isFreeJoint)
      continue;

    mDofs.push_back(dofs[i]->getIndexInSkeleton());
  }
  // Note: It is possible to be index mismatch between IKFast's joints and
  // metaSkeleton's joints.

  mExtraJointValues.resize(ikfastNumFreeParams);

  mConfigured = true;
}

//==============================================================================
auto Ikfast::computeSolutions(const Eigen::Isometry3d& desiredBodyTf)
    -> const std::vector<InverseKinematics::Analytical::Solution>&
{
  mSolutions.clear();

  if (!mConfigured)
  {
    configure();

    if (!mConfigured)
    {
      dtwarn << "[Ikfast::computeSolutions] This analytical IK was not able "
             << "to configure properly, so it will not be able to compute "
             << "solutions. Returning an empty list of solutions.\n";
      return mSolutions;
    }
  }

  if (!mSolver)
  {
    dtwarn << "[IKFast::computeSolutions] Attempting to perform an IK without "
           << "solver function. Returning empty solution.\n";
    return mSolutions;
  }

  toIkfastTypes(desiredBodyTf, eetrans, eerot);

  ikfast::IkSolutionList<IkReal> solutions;
  const auto success = mSolver->computeIk(
      eetrans.data(), eerot.data(), mExtraJointValues.data(), solutions);

  if (!success)
  {
//    dtwarn << "[IKFast::computeSolutions] Failed to get ik solutions using the "
//           << "IKFast solver. Returning an empty list of solutions.\n";
    return mSolutions;
  }
  else
  {
    dtmsg << "[IKFast::computeSolutions] Solved!.\n";
  }

  // Convert solutions
  toIkfastTypes(this, solutions, mSolutions);

  return mSolutions;
}

} // namespace dynamics
} // namespace dart
