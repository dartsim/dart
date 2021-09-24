/*
 * Copyright (c) 2011-2021, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/DegreeOfFreedom.hpp"
#include "dart/dynamics/RevoluteJoint.hpp"
#include "dart/external/ikfast/ikfast.h"

namespace dart {
namespace dynamics {

namespace {

//==============================================================================
void convertTransform(
    const Eigen::Isometry3d& tf,
    std::array<IkReal, 3>& eetrans,
    std::array<IkReal, 9>& eerot)
{
  eerot[0 * 3 + 0] = tf.linear()(0, 0);
  eerot[0 * 3 + 1] = tf.linear()(0, 1);
  eerot[0 * 3 + 2] = tf.linear()(0, 2);
  eerot[1 * 3 + 0] = tf.linear()(1, 0);
  eerot[1 * 3 + 1] = tf.linear()(1, 1);
  eerot[1 * 3 + 2] = tf.linear()(1, 2);
  eerot[2 * 3 + 0] = tf.linear()(2, 0);
  eerot[2 * 3 + 1] = tf.linear()(2, 1);
  eerot[2 * 3 + 2] = tf.linear()(2, 2);

  eetrans[0] = tf.translation()[0];
  eetrans[1] = tf.translation()[1];
  eetrans[2] = tf.translation()[2];
}

//==============================================================================
void convertTransform(
    const std::array<IkReal, 3>& eetrans,
    const std::array<IkReal, 9>& eerot,
    Eigen::Isometry3d& tf)
{
  tf.setIdentity();

  tf(0, 0) = eerot[0 * 3 + 0];
  tf(0, 1) = eerot[0 * 3 + 1];
  tf(0, 2) = eerot[0 * 3 + 2];
  tf(1, 0) = eerot[1 * 3 + 0];
  tf(1, 1) = eerot[1 * 3 + 1];
  tf(1, 2) = eerot[1 * 3 + 2];
  tf(2, 0) = eerot[2 * 3 + 0];
  tf(2, 1) = eerot[2 * 3 + 1];
  tf(2, 2) = eerot[2 * 3 + 2];

  tf(3, 0) = eetrans[0];
  tf(3, 1) = eetrans[1];
  tf(3, 2) = eetrans[2];
}

//==============================================================================
std::vector<bool> getFreeJointFlags(
    int numParams, int numFreeParams, const int* freeParams)
{
  std::vector<bool> flags(numParams, false);

  for (int i = 0; i < numFreeParams; ++i)
  {
    assert(freeParams[i] < numParams);
    flags[freeParams[i]] = true;
  }

  return flags;
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

  const auto freeJointFlags
      = getFreeJointFlags(numJoints, numFreeParameters, freeParameters);

  bool limitViolated = false;

  auto index = 0u;
  assert(solutionValues.size());
  solution.mConfig.resize(dofIndices.size());
  for (auto i = 0u; i < solutionValues.size(); ++i)
  {
    const auto isFreeJoint = freeJointFlags[i];
    if (isFreeJoint)
      continue;

    const auto dofIndex = dofIndices[index];
    const auto* dof = skel->getDof(dofIndex);
    const auto* joint = dof->getJoint();

    auto solutionValue = solutionValues[i];

    const auto lb = dof->getPositionLowerLimit();
    const auto ub = dof->getPositionUpperLimit();

    if (joint->getType() == RevoluteJoint::getStaticType())
    {
      // TODO(JS): Apply this to any DegreeOfFreedom whose configuration space
      // is SO(2).

      const auto currentValue = dof->getPosition();
      if (!wrapCyclicSolution(currentValue, lb, ub, solutionValue))
      {
        limitViolated = true;
        break;
      }
    }
    else
    {
      if (solutionValues[i] < lb)
      {
        limitViolated = true;
        break;
      }

      if (solutionValues[i] > ub)
      {
        limitViolated = true;
        break;
      }
    }

    solution.mConfig[index] = solutionValue;

    index++;
  }

  solution.mValidity = limitViolated
                           ? InverseKinematics::Analytical::LIMIT_VIOLATED
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

//==============================================================================
bool checkDofMapValidity(
    const InverseKinematics* ik,
    const std::vector<std::size_t>& dofMap,
    const std::string& dofMapName)
{
  // dependentDofs are the dependent DOFs of the BodyNode that is associated
  // with ik. This function returns true if all the indices in dofMap are found
  // in dependentDofs. Returns false otherwise.
  const auto& dependentDofs = ik->getNode()->getDependentDofs();

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

} // namespace

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
const std::vector<std::size_t>& IkFast::getFreeDofs() const
{
  return mFreeDofs;
}

//==============================================================================
bool IkFast::isConfigured() const
{
  return mConfigured;
}

//==============================================================================
std::size_t IkFast::getNumFreeParameters2() const
{
  return static_cast<std::size_t>(getNumFreeParameters());
}

//==============================================================================
std::size_t IkFast::getNumJoints2() const
{
  return static_cast<std::size_t>(getNumJoints());
}

//==============================================================================
IkFast::IkType IkFast::getIkType2() const
{
  // Following conversion is referred from:
  // https://github.com/rdiankov/openrave/blob/b1ebe135b4217823ebdf56d9af5fe89b29723603/include/openrave/openrave.h#L575-L623

  const int type = getIkType();

  if (type == 0)
    return IkType::UNKNOWN;
  else if (type == 0x67000001)
    return IkType::TRANSFORM_6D;
  else if (type == 0x34000002)
    return IkType::ROTATION_3D;
  else if (type == 0x34000003)
    return IkType::TRANSLATION_3D;
  else if (type == 0x34000004)
    return IkType::DIRECTION_3D;
  else if (type == 0x34000005)
    return IkType::RAY_4D;
  else if (type == 0x34000006)
    return IkType::LOOKAT_3D;
  else if (type == 0x34000007)
    return IkType::TRANSLATION_DIRECTION_5D;
  else if (type == 0x34000008)
    return IkType::TRANSLATION_XY_2D;
  else if (type == 0x34000009)
    return IkType::TRANSLATION_XY_ORIENTATION_3D;
  else if (type == 0x3400000a)
    return IkType::TRANSLATION_LOCAL_GLOBAL_6D;
  else if (type == 0x3400000b)
    return IkType::TRANSLATION_X_AXIS_ANGLE_4D;
  else if (type == 0x3400000c)
    return IkType::TRANSLATION_Y_AXIS_ANGLE_4D;
  else if (type == 0x3400000d)
    return IkType::TRANSLATION_Z_AXIS_ANGLE_4D;
  else if (type == 0x3400000e)
    return IkType::TRANSLATION_X_AXIS_ANGLE_Z_NORM_4D;
  else if (type == 0x3400000f)
    return IkType::TRANSLATION_Y_AXIS_ANGLE_X_NORM_4D;
  else if (type == 0x34000010)
    return IkType::TRANSLATION_Z_AXIS_ANGLE_Y_NORM_4D;

  return IkType::UNKNOWN;
}

//==============================================================================
const std::string IkFast::getKinematicsHash2() const
{
  return const_cast<IkFast*>(this)->getKinematicsHash();
}

//==============================================================================
std::string IkFast::getIkFastVersion2() const
{
  return const_cast<IkFast*>(this)->getIkFastVersion();
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

  if (!checkDofMapValidity(mIK.get(), mDofs, "dof map"))
    return;

  if (!checkDofMapValidity(mIK.get(), mFreeDofs, "free dof map"))
    return;

  mFreeParams.resize(static_cast<std::size_t>(ikFastNumFreeJoints));

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
        solutions,
        mSolutions);
  }

  return mSolutions;
}

//==============================================================================
Eigen::Isometry3d IkFast::computeFk(const Eigen::VectorXd& parameters)
{
  const std::size_t ikFastNumNonFreeJoints
      = getNumJoints2() - getNumFreeParameters2();
  if (static_cast<std::size_t>(parameters.size()) != ikFastNumNonFreeJoints)
  {
    dtwarn << "[IkFast::computeFk] The dimension of given joint positions "
           << "doesn't agree with the number of joints of this IkFast solver. "
           << "Returning identity.\n";
    return Eigen::Isometry3d::Identity();
  }

  std::array<IkReal, 3> eetrans;
  std::array<IkReal, 9> eerot;
  computeFk(parameters.data(), eetrans.data(), eerot.data());

  Eigen::Isometry3d tf;
  convertTransform(eetrans, eerot, tf);
  return tf;
}

//==============================================================================
bool wrapCyclicSolution(
    double currentValue, double lb, double ub, double& solutionValue)
{
  if (lb > ub)
    return false;

  const auto pi2 = math::constantsd::two_pi();

  if (currentValue < lb)
  {
    const auto diff_lb = lb - solutionValue;
    const auto lb_ceil = solutionValue + std::ceil(diff_lb / pi2) * pi2;
    assert(lb <= lb_ceil);
    if (lb_ceil <= ub)
    {
      solutionValue = lb_ceil;
    }
    else
    {
      return false;
    }
  }
  else if (ub < currentValue)
  {
    const auto diff_ub = ub - solutionValue;
    const auto ub_floor = solutionValue + std::floor(diff_ub / pi2) * pi2;
    assert(ub_floor <= ub);
    if (lb <= ub_floor)
    {
      solutionValue = ub_floor;
    }
    else
    {
      return false;
    }
  }
  else
  {
    const auto diff_curr = currentValue - solutionValue;
    const auto curr_floor = solutionValue + std::floor(diff_curr / pi2) * pi2;
    const auto curr_ceil = solutionValue + std::ceil(diff_curr / pi2) * pi2;

    bool found = false;

    if (lb <= curr_floor)
    {
      solutionValue = curr_floor;
      found = true;
    }

    if (curr_ceil <= ub)
    {
      if (found)
      {
        if (std::abs(curr_floor - currentValue)
            > std::abs(curr_ceil - currentValue))
        {
          solutionValue = curr_ceil;
          found = true;
        }
      }
      else
      {
        solutionValue = curr_ceil;
        found = true;
      }
    }

    if (!found)
    {
      return false;
    }
  }

  return true;
}

} // namespace dynamics
} // namespace dart
