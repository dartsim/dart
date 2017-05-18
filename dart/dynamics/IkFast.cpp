/*
 * Copyright (c) 2017, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2017, Personal Robotics Lab, Carnegie Mellon University
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

#include "dart/dynamics/IkFast.hpp"

#include "dart/external/ikfast/ikfast.h"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/DegreeOfFreedom.hpp"

namespace dart {
namespace dynamics {

namespace {

//==============================================================================
void convertTransform(
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
void convertTransform(
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
bool isFreeJoint(int numFreeParams, const int* freeParams, int index)
{
  for (auto i = 0; i < numFreeParams; ++i)
  {
    if (index == freeParams[i])
      return true;
  }

  return false;
}

//==============================================================================
void convertIkSolution(
    const IkFast* ikfast,
    int numJoints,
    int numFreeParameters,
    int* freeParameters,
    const ikfast::IkSolutionBase<IkReal>& ikfastSolution,
    InverseKinematics::Analytical::Solution& solution)
{
  const auto ik = ikfast->getIK();
  const auto dofIndices = ikfast->getDofs();
  const auto skel = ik->getNode()->getSkeleton();

  std::vector<IkReal> solutionValues(numJoints);
  std::vector<IkReal> freeValues(numFreeParameters);

  ikfastSolution.GetSolution(solutionValues, freeValues);

  bool limitViolated = false;

  auto index = 0u;
  solution.mConfig.resize(dofIndices.size());
  for (auto i = 0u; i < solutionValues.size(); ++i)
  {
    if (isFreeJoint(numFreeParameters, freeParameters, i))
      continue;

    solution.mConfig[index] = solutionValues[i];

    const auto dofIndex = dofIndices[index];

    if (solutionValues[i] < skel->getDof(dofIndex)->getPositionLowerLimit())
    {
      limitViolated = true;
      break;
    }

    if (solutionValues[i] > skel->getDof(dofIndex)->getPositionUpperLimit())
    {
      limitViolated = true;
      break;
    }

    index++;
  }

  solution.mValidity
      = limitViolated ? InverseKinematics::Analytical::LIMIT_VIOLATED
                      : InverseKinematics::Analytical::VALID;
}

//==============================================================================
void convertIkSolutions(
    const IkFast* ikfast,
    int numJoints,
    int numFreeParameters,
    int* freeParameters,
    const ikfast::IkSolutionList<IkReal>& ikfastSolutions,
    std::vector<InverseKinematics::Analytical::Solution>& solutions)
{
  const auto numSolutions = ikfastSolutions.GetNumSolutions();

  solutions.resize(numSolutions);

  for (auto i = 0u; i < numSolutions; ++i)
  {
    const auto& ikfastSolution = ikfastSolutions.GetSolution(i);
    convertIkSolution(
        ikfast,
        numJoints,
        numFreeParameters,
        freeParameters,
        ikfastSolution,
        solutions[i]);
  }
}

} // namespace (anonymous)

//==============================================================================
IkFast::IkFast(
    InverseKinematics* ik,
    const std::vector<std::size_t>& dofMap,
    const std::vector<std::size_t>& freeDofMap,
    const std::string& methodName,
    const InverseKinematics::Analytical::Properties& properties)
  : Analytical{ik, methodName, properties}, mConfigured{false}
{
  setExtraDofUtilization(UNUSED);

  mDofs = dofMap;
  mFreeDofs = freeDofMap;
}

//==============================================================================
auto IkFast::getDofs() const -> const std::vector<std::size_t>&
{
  if (!mConfigured)
  {
    configure();

    if (!mConfigured)
    {
      dtwarn << "[IkFast::getDofs] This analytical IK was not able "
             << "to configure properly, so it will not be able to compute "
             << "solutions. Returning an empty list of dofs.\n";
      assert(mDofs.empty());
    }
  }

  return mDofs;
}

//==============================================================================
bool IkFast::isConfigured() const
{
  return mConfigured;
}

//==============================================================================
bool checkDofMapValidity(
    const InverseKinematics* ik,
    const std::vector<std::size_t>& dofMap,
    const std::vector<DegreeOfFreedom*>& dependentDofs,
    const std::string& dofMapName)
{
  for (const auto& dof : dofMap)
  {
    bool found = false;
    for (auto dependentDof : dependentDofs)
    {
      const auto dependentDofIndex = dependentDof->getIndexInSkeleton();
      if (dof == dependentDofIndex)
      {
        found = true;
        break;
      }
    }

    if (!found)
    {
      dterr << "[IkFast::configure] Failed to configure. An element of the "
            << "given " << dofMapName << " '" << dof
            << "' is not a dependent dofs of Node '" << ik->getNode()->getName()
            << "'.\n";
      return false;
    }
  }

  return true;
}

//==============================================================================
void IkFast::configure() const
{
  const auto ikFastNumJoints = getNumJoints();
  const auto ikFastNumFreeJoints = getNumFreeParameters();
  const auto ikFastNumNonFreeJoints = ikFastNumJoints - ikFastNumFreeJoints;

  if (static_cast<std::size_t>(ikFastNumNonFreeJoints) != mDofs.size())
  {
    dterr << "[IkFast::configure] Failed to configure. Received a joint map of "
          << "size '" << mDofs.size() << "' but the actual dofs IkFast is '"
          << ikFastNumNonFreeJoints << "'.\n";
    return;
  }

  if (static_cast<std::size_t>(ikFastNumFreeJoints) != mFreeDofs.size())
  {
    dterr << "[IkFast::configure] Failed to configure. Received a free joint "
          << "map of size '" << mDofs.size()
          << "' but the actual dofs IkFast is '" << ikFastNumFreeJoints
          << "'.\n";
    return;
  }

  const auto dependentDofs = mIK->getNode()->getDependentDofs();

  if (!checkDofMapValidity(mIK.get(), mDofs, dependentDofs, "dof map"))
    return;

  if (!checkDofMapValidity(mIK.get(), mFreeDofs, dependentDofs, "free dof map"))
    return;

  mFreeParams.resize(ikFastNumFreeJoints);

  mConfigured = true;
}

//==============================================================================
auto IkFast::computeSolutions(const Eigen::Isometry3d& desiredBodyTf)
    -> const std::vector<InverseKinematics::Analytical::Solution>&
{
  mSolutions.clear();

  if (!mConfigured)
  {
    configure();

    if (!mConfigured)
    {
      dtwarn << "[IkFast::computeSolutions] This analytical IK was not able "
             << "to configure properly, so it will not be able to compute "
             << "solutions. Returning an empty list of solutions.\n";
      return mSolutions;
    }
  }

  convertTransform(desiredBodyTf, mTargetTranspose, mTargetRotation);

  const auto dofs = mIK->getNode()->getSkeleton()->getDofs();
  const auto ikFastNumFreeParams = getNumFreeParameters();
  const auto ikFastFreeParams = getFreeParameters();
  for (auto i = 0; i < ikFastNumFreeParams; ++i)
    mFreeParams[i] = dofs[ikFastFreeParams[i]]->getPosition();

  ikfast::IkSolutionList<IkReal> solutions;
  const auto success = computeIk(
      mTargetTranspose.data(),
      mTargetRotation.data(),
      mFreeParams.data(),
      solutions);

  if (success)
  {
    convertIkSolutions(
          this,
          getNumJoints(),
          getNumFreeParameters(),
          getFreeParameters(),
          solutions, mSolutions);
  }

  return mSolutions;
}

} // namespace dynamics
} // namespace dart
