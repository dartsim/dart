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

#include "dart/dynamics/ContactInverseDynamics.hpp"

#include "dart/common/Console.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/DegreeOfFreedom.hpp"
#include "dart/dynamics/Joint.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/math/Constants.hpp"
#include "dart/math/NonNegativeLeastSquares.hpp"

#include <algorithm>
#include <limits>

#include <cmath>

namespace dart {
namespace dynamics {

namespace {

//==============================================================================
// Edge directions of the linearized Coulomb friction cone for the given
// world-frame unit normal and friction coefficient. A zero friction
// coefficient yields the normal direction only.
Eigen::Matrix3Xd computeConeGenerators(
    const Eigen::Vector3d& unitNormal,
    double frictionCoeff,
    std::size_t numBasis)
{
  if (frictionCoeff <= 0.0) {
    return unitNormal;
  }

  const Eigen::Vector3d reference = std::abs(unitNormal.x()) < 0.9
                                        ? Eigen::Vector3d::UnitX()
                                        : Eigen::Vector3d::UnitY();
  const Eigen::Vector3d tangent1
      = (reference - reference.dot(unitNormal) * unitNormal).normalized();
  const Eigen::Vector3d tangent2 = unitNormal.cross(tangent1);

  Eigen::Matrix3Xd generators(3, numBasis);
  for (std::size_t i = 0; i < numBasis; ++i) {
    const double angle = 2.0 * math::constantsd::pi() * static_cast<double>(i)
                         / static_cast<double>(numBasis);
    generators.col(static_cast<Eigen::Index>(i))
        = (unitNormal
           + frictionCoeff
                 * (std::cos(angle) * tangent1 + std::sin(angle) * tangent2))
              .normalized();
  }

  return generators;
}

} // namespace

//==============================================================================
ContactInverseDynamics::ContactInverseDynamics(SkeletonPtr skeleton)
  : mSkeleton(std::move(skeleton)),
    mRegularization(1e-8),
    mResidualTolerance(1e-6),
    mHasUnactuatedDofsOverride(false)
{
  if (!mSkeleton) {
    dterr << "[ContactInverseDynamics] Constructed with a null Skeleton; "
          << "compute() will fail until a valid Skeleton is provided.\n";
  }
}

//==============================================================================
const SkeletonPtr& ContactInverseDynamics::getSkeleton() const
{
  return mSkeleton;
}

//==============================================================================
void ContactInverseDynamics::setContacts(const std::vector<Contact>& contacts)
{
  mContacts = contacts;
}

//==============================================================================
const std::vector<ContactInverseDynamics::Contact>&
ContactInverseDynamics::getContacts() const
{
  return mContacts;
}

//==============================================================================
void ContactInverseDynamics::setRegularization(double regularization)
{
  if (regularization < 0.0) {
    dtwarn << "[ContactInverseDynamics] Ignoring negative regularization ["
           << regularization << "].\n";
    return;
  }
  mRegularization = regularization;
}

//==============================================================================
double ContactInverseDynamics::getRegularization() const
{
  return mRegularization;
}

//==============================================================================
void ContactInverseDynamics::setResidualTolerance(double tolerance)
{
  if (tolerance <= 0.0) {
    dtwarn << "[ContactInverseDynamics] Ignoring non-positive residual "
           << "tolerance [" << tolerance << "].\n";
    return;
  }
  mResidualTolerance = tolerance;
}

//==============================================================================
double ContactInverseDynamics::getResidualTolerance() const
{
  return mResidualTolerance;
}

//==============================================================================
void ContactInverseDynamics::setUnactuatedDofs(
    const std::vector<std::size_t>& indices)
{
  mUnactuatedDofs = indices;
  std::sort(mUnactuatedDofs.begin(), mUnactuatedDofs.end());
  mUnactuatedDofs.erase(
      std::unique(mUnactuatedDofs.begin(), mUnactuatedDofs.end()),
      mUnactuatedDofs.end());
  mHasUnactuatedDofsOverride = true;
}

//==============================================================================
std::vector<std::size_t> ContactInverseDynamics::getUnactuatedDofs() const
{
  if (mHasUnactuatedDofsOverride) {
    return mUnactuatedDofs;
  }

  std::vector<std::size_t> dofs;
  if (!mSkeleton) {
    return dofs;
  }

  for (std::size_t tree = 0; tree < mSkeleton->getNumTrees(); ++tree) {
    const Joint* rootJoint = mSkeleton->getRootJoint(tree);
    if (rootJoint && rootJoint->getNumDofs() == 6u) {
      for (std::size_t i = 0; i < rootJoint->getNumDofs(); ++i) {
        dofs.push_back(rootJoint->getDof(i)->getIndexInSkeleton());
      }
    }
  }

  std::sort(dofs.begin(), dofs.end());
  return dofs;
}

//==============================================================================
ContactInverseDynamics::Result ContactInverseDynamics::compute(
    bool withExternalForces, bool withDampingForces, bool withSpringForces)
{
  Result result;

  if (!mSkeleton) {
    dterr << "[ContactInverseDynamics::compute] No Skeleton is set.\n";
    return result;
  }

  const std::size_t numDofs = mSkeleton->getNumDofs();

  for (const Contact& contact : mContacts) {
    if (!contact.bodyNode
        || contact.bodyNode->getSkeleton().get() != mSkeleton.get()) {
      dterr << "[ContactInverseDynamics::compute] Contact BodyNode is null or "
            << "does not belong to Skeleton [" << mSkeleton->getName()
            << "].\n";
      return result;
    }
    if (!contact.localOffset.allFinite() || !contact.normal.allFinite()
        || !std::isfinite(contact.frictionCoeff)) {
      dterr << "[ContactInverseDynamics::compute] Contact on BodyNode ["
            << contact.bodyNode->getName()
            << "] has non-finite values; localOffset, normal, and "
            << "frictionCoeff must all be finite.\n";
      return result;
    }
    if (contact.normal.norm() < 1e-12) {
      dterr << "[ContactInverseDynamics::compute] Contact on BodyNode ["
            << contact.bodyNode->getName() << "] has a zero normal.\n";
      return result;
    }
    if (contact.frictionCoeff < 0.0) {
      dterr << "[ContactInverseDynamics::compute] Contact on BodyNode ["
            << contact.bodyNode->getName()
            << "] has a negative friction coefficient ["
            << contact.frictionCoeff << "].\n";
      return result;
    }
    if (contact.frictionCoeff > 0.0 && contact.numBasis < 3) {
      dterr << "[ContactInverseDynamics::compute] Contact on BodyNode ["
            << contact.bodyNode->getName() << "] requests numBasis ["
            << contact.numBasis << "], but at least 3 friction-cone edge "
            << "directions are required.\n";
      return result;
    }
  }

  const std::vector<std::size_t> unactuated = getUnactuatedDofs();
  for (const std::size_t index : unactuated) {
    if (index >= numDofs) {
      dterr << "[ContactInverseDynamics::compute] Unactuated DOF index ["
            << index << "] is out of range for Skeleton ["
            << mSkeleton->getName() << "] with [" << numDofs << "] DOFs.\n";
      return result;
    }
  }

  // Inverse dynamics for the current state. Restore the joint forces and the
  // joint commands afterwards: setForces() overwrites the commands of
  // FORCE-actuated joints, so the commands must be restored last.
  const Eigen::VectorXd savedForces = mSkeleton->getForces();
  const Eigen::VectorXd savedCommands = mSkeleton->getCommands();
  mSkeleton->computeInverseDynamics(
      withExternalForces, withDampingForces, withSpringForces);
  const Eigen::VectorXd fullForces = mSkeleton->getForces();
  mSkeleton->setForces(savedForces);
  mSkeleton->setCommands(savedCommands);

  const std::size_t numUnactuated = unactuated.size();
  const auto residualOf = [&](const Eigen::VectorXd& forces) {
    Eigen::VectorXd residual(numUnactuated);
    for (std::size_t i = 0; i < numUnactuated; ++i) {
      residual[static_cast<Eigen::Index>(i)]
          = forces[static_cast<Eigen::Index>(unactuated[i])];
    }
    return residual;
  };

  const Eigen::VectorXd target = residualOf(fullForces);
  const double feasibilityScale = std::max(
      1.0, target.size() > 0 ? target.lpNorm<Eigen::Infinity>() : 0.0);

  if (mContacts.empty() || numUnactuated == 0) {
    result.jointForces = fullForces;
    result.contactForces.assign(mContacts.size(), Eigen::Vector3d::Zero());
    result.unactuatedResidual = target;
    result.feasible = numUnactuated == 0
                      || target.lpNorm<Eigen::Infinity>()
                             <= mResidualTolerance * feasibilityScale;
    return result;
  }

  // Columns of the generalized-force contribution of every friction-cone
  // edge direction of every contact.
  Eigen::Index numColumns = 0;
  for (const Contact& contact : mContacts) {
    numColumns += contact.frictionCoeff > 0.0
                      ? static_cast<Eigen::Index>(contact.numBasis)
                      : 1;
  }

  mGenerators.resize(mContacts.size());
  mFullColumns.resize(static_cast<Eigen::Index>(numDofs), numColumns);
  Eigen::Index column = 0;
  for (std::size_t k = 0; k < mContacts.size(); ++k) {
    const Contact& contact = mContacts[k];
    mGenerators[k] = computeConeGenerators(
        contact.normal.normalized(), contact.frictionCoeff, contact.numBasis);

    const math::LinearJacobian jacobian = mSkeleton->getLinearJacobian(
        contact.bodyNode, contact.localOffset, Frame::World());

    mFullColumns.middleCols(column, mGenerators[k].cols()).noalias()
        = jacobian.transpose() * mGenerators[k];
    column += mGenerators[k].cols();
  }

  // Nonnegative least squares on the unactuated rows with Tikhonov
  // regularization rows appended for uniqueness and even distribution.
  const Eigen::Index numRows = static_cast<Eigen::Index>(numUnactuated);
  mSystem.resize(numRows + numColumns, numColumns);
  for (Eigen::Index r = 0; r < numRows; ++r) {
    mSystem.row(r) = mFullColumns.row(static_cast<Eigen::Index>(unactuated[r]));
  }
  mSystem.bottomRows(numColumns)
      = std::sqrt(mRegularization)
        * Eigen::MatrixXd::Identity(numColumns, numColumns);

  mRhs.setZero(numRows + numColumns);
  mRhs.head(numRows) = target;

  // First pass with a dual tolerance matched to the feasibility tolerance:
  // near-redundant friction-cone directions otherwise keep the active-set
  // loop busy chasing improvements far below the requested accuracy. Fall
  // back to the solver's strict automatic tolerance when the loose pass does
  // not reach the requested residual.
  const double looseTolerance = 0.1 * mResidualTolerance * feasibilityScale;
  bool converged = math::solveNonNegativeLeastSquares(
      mSystem, mRhs, mCoefficients, looseTolerance);
  const auto residualNormOf = [&]() {
    return (mSystem.topRows(numRows) * mCoefficients - target)
        .lpNorm<Eigen::Infinity>();
  };
  if (!converged || residualNormOf() > mResidualTolerance * feasibilityScale) {
    const Eigen::VectorXd looseCoefficients = mCoefficients;
    const double looseResidual = converged
                                     ? residualNormOf()
                                     : std::numeric_limits<double>::infinity();
    const bool strictConverged
        = math::solveNonNegativeLeastSquares(mSystem, mRhs, mCoefficients);
    if (!strictConverged && looseResidual < residualNormOf()) {
      // Neither pass met the tolerance; keep the better iterate.
      mCoefficients = looseCoefficients;
      converged = true;
    } else {
      converged = strictConverged;
    }
  }

  // Recover per-contact forces and the corrected joint forces.
  result.contactForces.resize(mContacts.size());
  column = 0;
  for (std::size_t k = 0; k < mContacts.size(); ++k) {
    const Eigen::Index numGenerators = mGenerators[k].cols();
    result.contactForces[k].noalias()
        = mGenerators[k] * mCoefficients.segment(column, numGenerators);
    column += numGenerators;
  }

  result.jointForces = fullForces;
  result.jointForces.noalias() -= mFullColumns * mCoefficients;
  result.unactuatedResidual = residualOf(result.jointForces);
  result.feasible = converged
                    && result.unactuatedResidual.lpNorm<Eigen::Infinity>()
                           <= mResidualTolerance * feasibilityScale;

  return result;
}

} // namespace dynamics
} // namespace dart
