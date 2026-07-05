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

#include "dart/constraint/BoxedLcpConstraintSolver.hpp"

#include "dart/common/Console.hpp"
#include "dart/common/Macros.hpp"
#include "dart/common/Profile.hpp"
#include "dart/constraint/ConstraintBase.hpp"
#include "dart/constraint/ContactConstraint.hpp"
#include "dart/constraint/DantzigBoxedLcpSolver.hpp"
#include "dart/constraint/PgsBoxedLcpSolver.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/FreeJoint.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/lcpsolver/Lemke.hpp"
#include "dart/lcpsolver/dantzig/DantzigLcp.hpp"

#include <algorithm>
#include <array>
#include <iomanip>
#include <iostream>
#include <memory>

#include <cassert>
#include <cmath>

namespace dart {
namespace constraint {

namespace {

constexpr std::size_t kInlineConstraintCount = 8;
constexpr std::size_t kInlineLcpVectorSize = 12;
constexpr std::size_t kInlineLcpMatrixSize = 192;

struct BoxedLcpThreadScratch
{
  std::vector<ConstraintBase*> constraintPtrStorage;
  std::vector<std::size_t> constraintDimStorage;
  std::vector<std::size_t> constraintOffsetStorage;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> lcpA;
  Eigen::VectorXd lcpX;
  Eigen::VectorXd lcpB;
  Eigen::VectorXd lcpW;
  Eigen::VectorXd lcpLo;
  Eigen::VectorXd lcpHi;
  Eigen::VectorXi lcpFIndex;
};

BoxedLcpThreadScratch& boxedLcpThreadScratch()
{
  static thread_local BoxedLcpThreadScratch scratch;
  return scratch;
}

bool usesInlineLcpBuffer(std::size_t n, std::size_t matrixSize)
{
  return n <= kInlineLcpVectorSize && matrixSize <= kInlineLcpMatrixSize;
}

void reserveDantzigScratch(const BoxedLcpSolverPtr& solver, std::size_t n)
{
  auto dantzigSolver = std::dynamic_pointer_cast<DantzigBoxedLcpSolver>(solver);
  if (dantzigSolver)
    dantzigSolver->reserve(n);
}

} // namespace

//==============================================================================
BoxedLcpConstraintSolver::BoxedLcpConstraintSolver(
    double timeStep,
    BoxedLcpSolverPtr boxedLcpSolver,
    BoxedLcpSolverPtr secondaryBoxedLcpSolver)
  : BoxedLcpConstraintSolver(
      std::move(boxedLcpSolver), std::move(secondaryBoxedLcpSolver))
{
  setTimeStep(timeStep);
}

//==============================================================================
BoxedLcpConstraintSolver::BoxedLcpConstraintSolver()
  : BoxedLcpConstraintSolver(std::make_shared<DantzigBoxedLcpSolver>())
{
  // Do nothing
}

//==============================================================================
BoxedLcpConstraintSolver::BoxedLcpConstraintSolver(
    BoxedLcpSolverPtr boxedLcpSolver)
  : BoxedLcpConstraintSolver(
      std::move(boxedLcpSolver), std::make_shared<PgsBoxedLcpSolver>())
{
  // Do nothing
}

//==============================================================================
BoxedLcpConstraintSolver::BoxedLcpConstraintSolver(
    BoxedLcpSolverPtr boxedLcpSolver, BoxedLcpSolverPtr secondaryBoxedLcpSolver)
  : ConstraintSolver()
{
  if (boxedLcpSolver) {
    setBoxedLcpSolver(std::move(boxedLcpSolver));
  } else {
    dtwarn << "[BoxedLcpConstraintSolver] Attempting to construct with nullptr "
           << "LCP solver, which is not allowed. Using Dantzig solver "
           << "instead.\n";
    setBoxedLcpSolver(std::make_shared<DantzigBoxedLcpSolver>());
  }

  setSecondaryBoxedLcpSolver(std::move(secondaryBoxedLcpSolver));
}

//==============================================================================
void BoxedLcpConstraintSolver::setBoxedLcpSolver(BoxedLcpSolverPtr lcpSolver)
{
  if (!lcpSolver) {
    dtwarn << "[BoxedLcpConstraintSolver::setBoxedLcpSolver] "
           << "nullptr for boxed LCP solver is not allowed.\n";
    return;
  }

  if (lcpSolver == mSecondaryBoxedLcpSolver) {
    dtwarn << "[BoxedLcpConstraintSolver::setBoxedLcpSolver] Attempting to set "
           << "a primary LCP solver that is the same with the secondary LCP "
           << "solver, which is discouraged. Ignoring this request.\n";
  }

  mBoxedLcpSolver = std::move(lcpSolver);
}

//==============================================================================
ConstBoxedLcpSolverPtr BoxedLcpConstraintSolver::getBoxedLcpSolver() const
{
  return mBoxedLcpSolver;
}

//==============================================================================
void BoxedLcpConstraintSolver::setSecondaryBoxedLcpSolver(
    BoxedLcpSolverPtr lcpSolver)
{
  if (lcpSolver == mBoxedLcpSolver) {
    dtwarn << "[BoxedLcpConstraintSolver::setBoxedLcpSolver] Attempting to set "
           << "the secondary LCP solver that is identical to the primary LCP "
           << "solver, which is redundant. Please use different solvers or set "
           << "the secondary LCP solver to nullptr.\n";
  }

  mSecondaryBoxedLcpSolver = std::move(lcpSolver);
}

//==============================================================================
ConstBoxedLcpSolverPtr BoxedLcpConstraintSolver::getSecondaryBoxedLcpSolver()
    const
{
  return mSecondaryBoxedLcpSolver;
}

//==============================================================================
void BoxedLcpConstraintSolver::reserveConstrainedGroupScratch(
    const ConstrainedGroup& group)
{
  const auto& constraints = group.mConstraints;
  const std::size_t numConstraints = constraints.size();
  if (numConstraints == 0u)
    return;

  auto& scratch = boxedLcpThreadScratch();
  if (numConstraints > kInlineConstraintCount) {
    scratch.constraintPtrStorage.reserve(numConstraints);
    scratch.constraintDimStorage.reserve(numConstraints);
    scratch.constraintOffsetStorage.reserve(numConstraints);
  }

  std::size_t n = 0u;
  for (const auto& constraint : constraints) {
    n += constraint->getDimension();
  }

  if (n == 0u)
    return;

  const int nSkip = ::dart::lcpsolver::dantzig::padding(static_cast<int>(n));
  const std::size_t matrixSize = n * static_cast<std::size_t>(nSkip);
  if (!usesInlineLcpBuffer(n, matrixSize)) {
    scratch.lcpA.resize(static_cast<Eigen::Index>(n), nSkip);
    scratch.lcpX.resize(static_cast<Eigen::Index>(n));
    scratch.lcpB.resize(static_cast<Eigen::Index>(n));
    scratch.lcpW.resize(static_cast<Eigen::Index>(n));
    scratch.lcpLo.resize(static_cast<Eigen::Index>(n));
    scratch.lcpHi.resize(static_cast<Eigen::Index>(n));
    scratch.lcpFIndex.resize(static_cast<Eigen::Index>(n));
  }

  reserveDantzigScratch(mBoxedLcpSolver, n);
  reserveDantzigScratch(mSecondaryBoxedLcpSolver, n);
}

//==============================================================================
void BoxedLcpConstraintSolver::solveConstrainedGroup(ConstrainedGroup& group)
{
  DART_PROFILE_SCOPED;

  // Build LCP terms by aggregating them from constraints
  const auto& constraints = group.mConstraints;
  const std::size_t numConstraints = constraints.size();
  if (numConstraints == 0u)
    return;

  std::array<ConstraintBase*, kInlineConstraintCount> inlineConstraintPtrs;
  std::array<std::size_t, kInlineConstraintCount> inlineConstraintDims;
  std::array<std::size_t, kInlineConstraintCount> inlineConstraintOffsets;

  auto& scratch = boxedLcpThreadScratch();
  auto& constraintPtrStorage = scratch.constraintPtrStorage;
  auto& constraintDimStorage = scratch.constraintDimStorage;
  auto& constraintOffsetStorage = scratch.constraintOffsetStorage;

  ConstraintBase** constraintPtrs = nullptr;
  std::size_t* constraintDims = nullptr;
  std::size_t* constraintOffsets = nullptr;

  if (numConstraints <= kInlineConstraintCount) {
    constraintPtrs = inlineConstraintPtrs.data();
    constraintDims = inlineConstraintDims.data();
    constraintOffsets = inlineConstraintOffsets.data();
  } else {
    constraintPtrStorage.resize(numConstraints);
    constraintDimStorage.resize(numConstraints);
    constraintOffsetStorage.resize(numConstraints);
    constraintPtrs = constraintPtrStorage.data();
    constraintDims = constraintDimStorage.data();
    constraintOffsets = constraintOffsetStorage.data();
  }

  std::size_t n = 0u;
  for (std::size_t i = 0; i < numConstraints; ++i) {
    constraintPtrs[i] = constraints[i].get();
    constraintDims[i] = constraintPtrs[i]->getDimension();
    n += constraintDims[i];
  }

  // If there is no constraint, then just return.
  if (0u == n)
    return;

  const int nSkip = ::dart::lcpsolver::dantzig::padding(static_cast<int>(n));
  const std::size_t matrixSize = n * static_cast<std::size_t>(nSkip);
  const bool useInlineLcpBuffer = usesInlineLcpBuffer(n, matrixSize);
  std::array<double, kInlineLcpMatrixSize> inlineA;
  std::array<double, kInlineLcpVectorSize> inlineX;
  std::array<double, kInlineLcpVectorSize> inlineB;
  std::array<double, kInlineLcpVectorSize> inlineW;
  std::array<double, kInlineLcpVectorSize> inlineLo;
  std::array<double, kInlineLcpVectorSize> inlineHi;
  std::array<int, kInlineLcpVectorSize> inlineFIndex;

  double* a = nullptr;
  double* x = nullptr;
  double* b = nullptr;
  double* w = nullptr;
  double* lo = nullptr;
  double* hi = nullptr;
  int* fIndex = nullptr;

  auto& lcpA = scratch.lcpA;
  auto& lcpX = scratch.lcpX;
  auto& lcpB = scratch.lcpB;
  auto& lcpW = scratch.lcpW;
  auto& lcpLo = scratch.lcpLo;
  auto& lcpHi = scratch.lcpHi;
  auto& lcpFIndex = scratch.lcpFIndex;

  if (useInlineLcpBuffer) {
#if !DART_BUILD_MODE_RELEASE
    std::fill_n(inlineA.data(), matrixSize, 0.0);
#endif
    std::fill_n(inlineX.data(), n, 0.0);
    std::fill_n(inlineW.data(), n, 0.0);
    std::fill_n(inlineFIndex.data(), n, -1);
    a = inlineA.data();
    x = inlineX.data();
    b = inlineB.data();
    w = inlineW.data();
    lo = inlineLo.data();
    hi = inlineHi.data();
    fIndex = inlineFIndex.data();
  } else {
#if DART_BUILD_MODE_RELEASE
    lcpA.resize(n, nSkip);
#else // debug
    lcpA.setZero(n, nSkip);
#endif
    lcpX.resize(n);
    lcpX.setZero();
    lcpB.resize(n);
    lcpW.setZero(n); // set w to 0
    lcpLo.resize(n);
    lcpHi.resize(n);
    lcpFIndex.setConstant(n, -1); // set findex to -1
    a = lcpA.data();
    x = lcpX.data();
    b = lcpB.data();
    w = lcpW.data();
    lo = lcpLo.data();
    hi = lcpHi.data();
    fIndex = lcpFIndex.data();
  }

  // Compute offset indices
  constraintOffsets[0] = 0;
  for (std::size_t i = 1; i < numConstraints; ++i) {
    const std::size_t previousDim = constraintDims[i - 1];
    DART_ASSERT(previousDim > 0);
    constraintOffsets[i] = constraintOffsets[i - 1] + previousDim;
  }

  std::array<ContactConstraint*, kInlineConstraintCount> inlineContactPtrs;
  dynamics::BodyNode* directBody = group.mSingleReactiveBodyNode;
  dynamics::Skeleton* directSkeleton = group.mSingleReactiveSkeleton;
  bool useDirectSingleFreeBody = group.mAllSingleReactiveContacts
                                 && group.mSingleReactiveContactsShareBody
                                 && group.mAllExactContactConstraints
                                 && directBody != nullptr
                                 && directSkeleton != nullptr
                                 && numConstraints <= kInlineConstraintCount;
  if (useDirectSingleFreeBody) {
    for (std::size_t i = 0; i < numConstraints; ++i) {
      inlineContactPtrs[i] = static_cast<ContactConstraint*>(constraintPtrs[i]);
    }
  }

  if (useDirectSingleFreeBody) {
    const auto* parentJoint = directBody->getParentJoint();
    const auto* freeJoint
        = dynamic_cast<const dynamics::FreeJoint*>(parentJoint);
    useDirectSingleFreeBody
        = directSkeleton != nullptr && directSkeleton->getNumBodyNodes() == 1u
          && directSkeleton->getNumDofs() == 6u
          && directBody->getParentBodyNode() == nullptr
          && directBody->getNumChildBodyNodes() == 0u && freeJoint != nullptr
          && parentJoint != nullptr
          && (parentJoint->getActuatorType() == dynamics::Joint::FORCE
              || parentJoint->getActuatorType() == dynamics::Joint::PASSIVE);
  }

  const auto resetLcpTerms = [&]() {
    if (useInlineLcpBuffer) {
#if !DART_BUILD_MODE_RELEASE
      std::fill_n(inlineA.data(), matrixSize, 0.0);
#endif
      std::fill_n(x, n, 0.0);
      std::fill_n(w, n, 0.0);
      std::fill_n(fIndex, n, -1);
    } else {
#if !DART_BUILD_MODE_RELEASE
      lcpA.setZero(n, nSkip);
#endif
      lcpX.setZero();
      lcpW.setZero();
      lcpFIndex.setConstant(n, -1);
    }
  };

  const auto constructLcpTerms = [&]() {
    resetLcpTerms();
    DART_PROFILE_SCOPED_N("Construct LCP");
    ConstraintInfo constInfo;
    constInfo.invTimeStep = 1.0 / mTimeStep;

    if (useDirectSingleFreeBody) {
      const Eigen::Matrix6d& articulatedInertia
          = directBody->getArticulatedInertia();
      static const Eigen::Matrix6d identityInertia
          = Eigen::Matrix6d::Identity();
      const bool hasIdentityInertia
          = articulatedInertia.cwiseEqual(identityInertia).all();
      Eigen::LDLT<Eigen::Matrix6d> inertiaDecomposition;
      if (!hasIdentityInertia) {
        inertiaDecomposition.compute(articulatedInertia);
      }

      if (!hasIdentityInertia
          && inertiaDecomposition.info() != Eigen::Success) {
        useDirectSingleFreeBody = false;
      } else {
        for (std::size_t i = 0; i < numConstraints; ++i) {
          ContactConstraint* constraint = inlineContactPtrs[i];
          const std::size_t offset = constraintOffsets[i];
          const std::size_t dim = constraintDims[i];

          constInfo.x = x + offset;
          constInfo.lo = lo + offset;
          constInfo.hi = hi + offset;
          constInfo.b = b + offset;
          constInfo.findex = fIndex + offset;
          constInfo.w = w + offset;

          {
            DART_PROFILE_SCOPED_N("Fill lo, hi, b, w");
            constraint->getInformation(&constInfo);
          }

          {
            DART_PROFILE_SCOPED_N("Fill A direct single body");
            for (std::size_t j = 0; j < dim; ++j) {
              const std::size_t row = offset + j;
              if (fIndex[row] >= 0)
                fIndex[row] += offset;

              Eigen::Vector6d bodyVelocityChange
                  = constraint->getSpatialNormalForSingleReactiveBody(j);
              if (!hasIdentityInertia)
                bodyVelocityChange
                    = inertiaDecomposition.solve(bodyVelocityChange);

              std::size_t index = nSkip * row + offset;
              constraint->getVelocityChangeFromSingleBody(
                  bodyVelocityChange, a + index, true, j);
              for (std::size_t k = i + 1; k < numConstraints; ++k) {
                index = nSkip * row + constraintOffsets[k];
                inlineContactPtrs[k]->getVelocityChangeFromSingleBody(
                    bodyVelocityChange, a + index, false, 0u);
              }
            }
          }
        }
      }
    }

    if (!useDirectSingleFreeBody) {
      for (std::size_t i = 0; i < numConstraints; ++i) {
        ConstraintBase* constraint = constraintPtrs[i];
        const std::size_t offset = constraintOffsets[i];
        const std::size_t dim = constraintDims[i];

        constInfo.x = x + offset;
        constInfo.lo = lo + offset;
        constInfo.hi = hi + offset;
        constInfo.b = b + offset;
        constInfo.findex = fIndex + offset;
        constInfo.w = w + offset;

        // Fill vectors: lo, hi, b, w
        {
          DART_PROFILE_SCOPED_N("Fill lo, hi, b, w");
          constraint->getInformation(&constInfo);
        }

        // Fill a matrix by impulse tests: A
        {
          DART_PROFILE_SCOPED_N("Fill A");
          constraint->excite();
          for (std::size_t j = 0; j < dim; ++j) {
            // Adjust findex for global index
            const std::size_t row = offset + j;
            if (fIndex[row] >= 0)
              fIndex[row] += offset;

            // Apply impulse for impulse test
            {
              DART_PROFILE_SCOPED_N("Unit impulse test");
              constraint->applyUnitImpulse(j);
            }

            // Fill upper triangle blocks of A matrix
            {
              DART_PROFILE_SCOPED_N("Fill upper triangle of A");
              std::size_t index = nSkip * row + offset;
              constraint->getVelocityChange(a + index, true);
              for (std::size_t k = i + 1; k < numConstraints; ++k) {
                index = nSkip * row + constraintOffsets[k];
                constraintPtrs[k]->getVelocityChange(a + index, false);
              }
            }
          }
        }

        {
          DART_PROFILE_SCOPED_N("Unexcite");
          constraint->unexcite();
        }
      }
    }

    {
      // Fill lower triangle blocks of A matrix
      DART_PROFILE_SCOPED_N("Fill lower triangle of A");
      for (std::size_t row = 1; row < n; ++row) {
        for (std::size_t col = 0; col < row; ++col) {
          a[nSkip * row + col] = a[nSkip * col + row];
        }
      }
    }
  };

  // For each constraint
  constructLcpTerms();

#if !defined(NDEBUG)
  if (!isSymmetric(n, a)) {
    dtwarn << "[BoxedLcpConstraintSolver::solveConstrainedGroup] LCP matrix is "
              "not symmetric. Continuing to avoid assertion failure.\n";
  }
#endif

  // Print LCP formulation
  //  dtdbg << "Before solve:" << std::endl;
  //  print(n, A, x, lo, hi, b, w, findex);
  //  std::cout << std::endl;

  // Solve LCP using the primary solver and fallback to secondary solver when
  // the primary solver failed.
  const bool earlyTermination = (mSecondaryBoxedLcpSolver != nullptr);
  DART_ASSERT(mBoxedLcpSolver);
  bool success
      = mBoxedLcpSolver->solve(n, a, x, b, 0, lo, hi, fIndex, earlyTermination);

  const auto hasNaN = [](const double* values, std::size_t size) {
    for (std::size_t i = 0; i < size; ++i) {
      if (std::isnan(values[i]))
        return true;
    }
    return false;
  };

  // Sanity check. LCP solvers should not report success with nan values, but
  // it could happen. So we set the success to false for nan values.
  if (success && hasNaN(x, n))
    success = false;

  bool fallbackSuccess = false;
  bool fallbackRan = false;
  if (!success && mSecondaryBoxedLcpSolver) {
    DART_PROFILE_SCOPED_N("Secondary LCP");
    constructLcpTerms();
    fallbackSuccess
        = mSecondaryBoxedLcpSolver->solve(n, a, x, b, 0, lo, hi, fIndex, false);
    fallbackRan = true;
  }

  // Capture the NaN state of the solution BEFORE any scrubbing so that
  // usability is decided from the pre-scrub state. Scrubbing NaN entries must
  // not be allowed to turn a failed solve into an apparently usable one.
  const bool hadNaN = hasNaN(x, n);

  // Treat a finite fallback solution as usable even if the solver reported
  // failure to avoid discarding potentially valid impulses. Note this uses the
  // pre-scrub NaN state (hadNaN), NOT a post-scrub recheck, so a failed
  // fallback that wrote NaNs cannot be promoted to success by scrubbing.
  const bool finalSuccess
      = success || fallbackSuccess || (fallbackRan && !hadNaN);

  if (finalSuccess) {
    // The solution will be used. If an otherwise-usable solution (e.g. a
    // successful fallback) carries stray NaNs, scrub just those entries to 0
    // so the remaining finite impulses can propagate.
    if (hadNaN) {
      for (std::size_t i = 0; i < n; ++i) {
        if (std::isnan(x[i]))
          x[i] = 0.0;
      }
    }
  } else {
    // Fail safe: the solve genuinely failed (including a failed fallback that
    // produced NaNs), so zero the ENTIRE solution rather than propagating
    // partial finite impulses.
    if (hadNaN) {
      dterr << "[BoxedLcpConstraintSolver] The solution of LCP includes NAN "
            << "values. We're setting it zero for "
            << "safety. Consider using more robust solver such as PGS as a "
            << "secondary solver. If this happens even with PGS solver, please "
            << "report this as a bug.\n";
    } else {
      dterr << "[BoxedLcpConstraintSolver] Primary LCP solver failed to find a "
            << "solution. The constraint impulses are set to zero for safety. "
            << "Consider configuring a secondary solver (e.g., PGS) to provide "
            << "a fallback when Dantzig fails.\n";
    }

    std::fill_n(x, n, 0.0);
  }

  // Print LCP formulation
  //  dtdbg << "After solve:" << std::endl;
  //  print(n, A, x, lo, hi, b, w, findex);
  //  std::cout << std::endl;

  // Apply constraint impulses
  {
    DART_PROFILE_SCOPED_N("Apply constraint impulses");
    if (useDirectSingleFreeBody) {
      for (std::size_t i = 0; i < numConstraints; ++i) {
        inlineContactPtrs[i]->applyImpulse(x + constraintOffsets[i]);
      }

      directSkeleton->setImpulseApplied(true);
    } else {
      for (std::size_t i = 0; i < numConstraints; ++i) {
        ConstraintBase* constraint = constraintPtrs[i];
        constraint->applyImpulse(x + constraintOffsets[i]);
        constraint->excite();
      }
    }
  }
}

//==============================================================================
void BoxedLcpConstraintSolver::solvePositionConstrainedGroup(
    ConstrainedGroup& group)
{
  DART_PROFILE_SCOPED;

  // Collect only contact constraints for the position-correction pass.
  std::vector<ContactConstraint*> contacts;
  contacts.reserve(group.getNumConstraints());
  for (std::size_t i = 0; i < group.getNumConstraints(); ++i) {
    const ConstraintBasePtr& constraint = group.getConstraint(i);
    if (auto* contact = dynamic_cast<ContactConstraint*>(constraint.get())) {
      contacts.push_back(contact);
    }
  }

  const std::size_t numConstraints = contacts.size();
  if (numConstraints == 0u)
    return;

  // Compute offsets and total dimension.
  std::vector<std::size_t> offsets(numConstraints);
  std::size_t n = 0u;
  for (std::size_t i = 0; i < numConstraints; ++i) {
    offsets[i] = n;
    n += contacts[i]->getDimension();
  }

  if (0u == n)
    return;

  const int nSkip = ::dart::lcpsolver::dantzig::padding(static_cast<int>(n));

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A;
  A.setZero(n, nSkip);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd b(n);
  Eigen::VectorXd w = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd lo(n);
  Eigen::VectorXd hi(n);
  Eigen::VectorXi fIndex = Eigen::VectorXi::Constant(n, -1);

  const auto constructLcpTerms = [&]() {
    A.setZero(n, nSkip);
    x.setZero();
    w.setZero();
    fIndex.setConstant(n, -1);

    ConstraintInfo constInfo;
    constInfo.invTimeStep = 1.0 / mTimeStep;
    constInfo.phase = ConstraintPhase::Position;
    // useSplitImpulse only affects the velocity phase; leave it at its default
    // (false) here.

    for (std::size_t i = 0; i < numConstraints; ++i) {
      ContactConstraint* constraint = contacts[i];
      const std::size_t offset = offsets[i];
      const std::size_t dim = constraint->getDimension();

      constInfo.x = x.data() + offset;
      constInfo.lo = lo.data() + offset;
      constInfo.hi = hi.data() + offset;
      constInfo.b = b.data() + offset;
      constInfo.findex = fIndex.data() + offset;
      constInfo.w = w.data() + offset;

      constraint->getInformation(&constInfo);

      constraint->excite();
      for (std::size_t j = 0; j < dim; ++j) {
        const std::size_t row = offset + j;
        if (fIndex[static_cast<Eigen::Index>(row)] >= 0)
          fIndex[static_cast<Eigen::Index>(row)] += static_cast<int>(offset);

        constraint->applyUnitImpulse(j);

        std::size_t index = nSkip * row + offset;
        constraint->getVelocityChange(A.data() + index, true);
        for (std::size_t k = i + 1; k < numConstraints; ++k) {
          index = nSkip * row + offsets[k];
          contacts[k]->getVelocityChange(A.data() + index, false);
        }
      }
      constraint->unexcite();
    }

    // Fill lower triangle blocks of A matrix.
    for (std::size_t row = 1; row < n; ++row) {
      for (std::size_t col = 0; col < row; ++col) {
        A.data()[nSkip * row + col] = A.data()[nSkip * col + row];
      }
    }
  };

  constructLcpTerms();

  const bool earlyTermination = (mSecondaryBoxedLcpSolver != nullptr);
  DART_ASSERT(mBoxedLcpSolver);
  bool success = mBoxedLcpSolver->solve(
      n,
      A.data(),
      x.data(),
      b.data(),
      0,
      lo.data(),
      hi.data(),
      fIndex.data(),
      earlyTermination);

  const auto hasNaN = [](const double* values, std::size_t size) {
    for (std::size_t i = 0; i < size; ++i) {
      if (std::isnan(values[i]))
        return true;
    }
    return false;
  };

  if (success && hasNaN(x.data(), n))
    success = false;

  if (!success && mSecondaryBoxedLcpSolver) {
    constructLcpTerms();
    success = mSecondaryBoxedLcpSolver->solve(
        n,
        A.data(),
        x.data(),
        b.data(),
        0,
        lo.data(),
        hi.data(),
        fIndex.data(),
        false);
  }

  // If the solve failed or produced NaNs, skip the position correction for this
  // group (leave positions unchanged) rather than apply garbage impulses.
  if (!success || hasNaN(x.data(), n))
    return;

  for (std::size_t i = 0; i < numConstraints; ++i) {
    contacts[i]->applyPositionImpulse(x.data() + offsets[i]);
  }
}

//==============================================================================
bool BoxedLcpConstraintSolver::isSymmetric(std::size_t n, double* A)
{
  std::size_t nSkip = ::dart::lcpsolver::dantzig::padding(static_cast<int>(n));
  for (std::size_t i = 0; i < n; ++i) {
    for (std::size_t j = 0; j < n; ++j) {
      if (std::abs(A[nSkip * i + j] - A[nSkip * j + i]) > 1e-6) {
        std::cout << "A: " << std::endl;
        for (std::size_t k = 0; k < n; ++k) {
          for (std::size_t l = 0; l < nSkip; ++l) {
            std::cout << std::setprecision(4) << A[k * nSkip + l] << " ";
          }
          std::cout << std::endl;
        }

        std::cout << "A(" << i << ", " << j << "): " << A[nSkip * i + j]
                  << std::endl;
        std::cout << "A(" << j << ", " << i << "): " << A[nSkip * j + i]
                  << std::endl;
        return false;
      }
    }
  }

  return true;
}

//==============================================================================
bool BoxedLcpConstraintSolver::isSymmetric(
    std::size_t n, double* A, std::size_t begin, std::size_t end)
{
  std::size_t nSkip = ::dart::lcpsolver::dantzig::padding(static_cast<int>(n));
  for (std::size_t i = begin; i <= end; ++i) {
    for (std::size_t j = begin; j <= end; ++j) {
      if (std::abs(A[nSkip * i + j] - A[nSkip * j + i]) > 1e-6) {
        std::cout << "A: " << std::endl;
        for (std::size_t k = 0; k < n; ++k) {
          for (std::size_t l = 0; l < nSkip; ++l) {
            std::cout << std::setprecision(4) << A[k * nSkip + l] << " ";
          }
          std::cout << std::endl;
        }

        std::cout << "A(" << i << ", " << j << "): " << A[nSkip * i + j]
                  << std::endl;
        std::cout << "A(" << j << ", " << i << "): " << A[nSkip * j + i]
                  << std::endl;
        return false;
      }
    }
  }

  return true;
}

//==============================================================================
void BoxedLcpConstraintSolver::print(
    std::size_t n,
    double* A,
    double* x,
    double* /*lo*/,
    double* /*hi*/,
    double* b,
    double* w,
    int* findex)
{
  std::size_t nSkip = ::dart::lcpsolver::dantzig::padding(static_cast<int>(n));
  std::cout << "A: " << std::endl;
  for (std::size_t i = 0; i < n; ++i) {
    for (std::size_t j = 0; j < nSkip; ++j) {
      std::cout << std::setprecision(4) << A[i * nSkip + j] << " ";
    }
    std::cout << std::endl;
  }

  std::cout << "b: ";
  for (std::size_t i = 0; i < n; ++i) {
    std::cout << std::setprecision(4) << b[i] << " ";
  }
  std::cout << std::endl;

  std::cout << "w: ";
  for (std::size_t i = 0; i < n; ++i) {
    std::cout << w[i] << " ";
  }
  std::cout << std::endl;

  std::cout << "x: ";
  for (std::size_t i = 0; i < n; ++i) {
    std::cout << x[i] << " ";
  }
  std::cout << std::endl;

  //  std::cout << "lb: ";
  //  for (int i = 0; i < dim; ++i)
  //  {
  //    std::cout << lb[i] << " ";
  //  }
  //  std::cout << std::endl;

  //  std::cout << "ub: ";
  //  for (int i = 0; i < dim; ++i)
  //  {
  //    std::cout << ub[i] << " ";
  //  }
  //  std::cout << std::endl;

  std::cout << "frictionIndex: ";
  for (std::size_t i = 0; i < n; ++i) {
    std::cout << findex[i] << " ";
  }
  std::cout << std::endl;

  double* Ax = new double[n];

  for (std::size_t i = 0; i < n; ++i) {
    Ax[i] = 0.0;
  }

  for (std::size_t i = 0; i < n; ++i) {
    for (std::size_t j = 0; j < n; ++j) {
      Ax[i] += A[i * nSkip + j] * x[j];
    }
  }

  std::cout << "Ax   : ";
  for (std::size_t i = 0; i < n; ++i) {
    std::cout << Ax[i] << " ";
  }
  std::cout << std::endl;

  std::cout << "b + w: ";
  for (std::size_t i = 0; i < n; ++i) {
    std::cout << b[i] + w[i] << " ";
  }
  std::cout << std::endl;

  delete[] Ax;
}

} // namespace constraint
} // namespace dart
