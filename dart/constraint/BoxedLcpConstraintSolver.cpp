/*
 * Copyright (c) 2011-2024, The DART development contributors
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

#include "dart/common/Logging.hpp"
#include "dart/common/Profile.hpp"
#include "dart/constraint/ConstraintBase.hpp"
#include "dart/constraint/DantzigBoxedLcpSolver.hpp"
#include "dart/constraint/PgsBoxedLcpSolver.hpp"
#include "dart/external/odelcpsolver/lcp.h"
#include "dart/lcpsolver/Lemke.hpp"

#if DART_HAVE_Taskflow
  #include <taskflow/algorithm/for_each.hpp>
#endif

#if DART_BUILD_MODE_DEBUG
  #include <iomanip>
  #include <iostream>
#endif
#include <atomic>
#include <future>
#include <thread>

#include <cassert>

namespace dart {
namespace constraint {

namespace {

[[nodiscard]] BoxedLcpSolverPtr createBoxedLcpSolver(
    const BoxedLcpSolverType type)
{
  switch (type) {
    case BoxedLcpSolverType::Dantzig: {
      return std::make_shared<DantzigBoxedLcpSolver>();
    }
    case BoxedLcpSolverType::Pgs: {
      return std::make_shared<PgsBoxedLcpSolver>();
    }
    default: {
      DART_WARN("Unknown BoxedLcpSolverType. Using Dantzig solver instead.");
      return std::make_shared<DantzigBoxedLcpSolver>();
    }
  }
}

#if DART_BUILD_MODE_DEBUG
bool isSymmetric(std::size_t n, double* A)
{
  std::size_t nSkip = dPAD(n);
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
[[maybe_unused]] bool isSymmetric(
    std::size_t n, double* A, std::size_t begin, std::size_t end)
{
  std::size_t nSkip = dPAD(n);
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
[[maybe_unused]] void print(
    std::size_t n,
    double* A,
    double* x,
    double* /*lo*/,
    double* /*hi*/,
    double* b,
    double* w,
    int* findex)
{
  std::size_t nSkip = dPAD(n);
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
#endif

} // namespace

//==============================================================================
DART_SUPPRESS_DEPRECATED_BEGIN
BoxedLcpConstraintSolver::BoxedLcpConstraintSolver(
    double timeStep,
    BoxedLcpSolverPtr boxedLcpSolver,
    BoxedLcpSolverPtr secondaryBoxedLcpSolver)
  : BoxedLcpConstraintSolver(
      std::move(boxedLcpSolver), std::move(secondaryBoxedLcpSolver))
{
  setTimeStep(timeStep);
}
DART_SUPPRESS_DEPRECATED_END

//==============================================================================
BoxedLcpConstraintSolver::BoxedLcpConstraintSolver(const Config& config)
#if DART_HAVE_Taskflow
  : ConstraintSolver(), mConfig(config), mExecutor(4)
#else
  : ConstraintSolver(), mConfig(config)
#endif
{
  // Empty
}

//==============================================================================
DART_SUPPRESS_DEPRECATED_BEGIN
BoxedLcpConstraintSolver::BoxedLcpConstraintSolver(
    BoxedLcpSolverPtr boxedLcpSolver)
  : BoxedLcpConstraintSolver(
      std::move(boxedLcpSolver), std::make_shared<PgsBoxedLcpSolver>())
{
  // Do nothing
}
DART_SUPPRESS_DEPRECATED_END

//==============================================================================
DART_SUPPRESS_DEPRECATED_BEGIN
BoxedLcpConstraintSolver::BoxedLcpConstraintSolver(
    BoxedLcpSolverPtr boxedLcpSolver, BoxedLcpSolverPtr secondaryBoxedLcpSolver)
#if DART_HAVE_Taskflow
  : ConstraintSolver(), mExecutor(4)
#else
  : ConstraintSolver()
#endif
{
  DART_SUPPRESS_DEPRECATED_BEGIN
  if (boxedLcpSolver) {
    setBoxedLcpSolver(std::move(boxedLcpSolver));
  } else {
    DART_WARN(
        "[BoxedLcpConstraintSolver] Attempting to construct with nullptr LCP "
        "solver, which is not allowed. Using Dantzig solver instead.");
    setBoxedLcpSolver(std::make_shared<DantzigBoxedLcpSolver>());
  }
  DART_SUPPRESS_DEPRECATED_END

  setSecondaryBoxedLcpSolver(std::move(secondaryBoxedLcpSolver));
}
DART_SUPPRESS_DEPRECATED_END

//==============================================================================
void BoxedLcpConstraintSolver::setPrimaryBoxedLcpSolverType(
    BoxedLcpSolverType type)
{
  mConfig.primaryBoxedLcpSolver = type;
}

//==============================================================================
BoxedLcpSolverType BoxedLcpConstraintSolver::getPrimaryBoxedLcpSolverType()
    const
{
  return mConfig.primaryBoxedLcpSolver;
}

//==============================================================================
void BoxedLcpConstraintSolver::setSecondaryBoxedLcpSolverType(
    BoxedLcpSolverType type)
{
  mConfig.secondaryBoxedLcpSolver = type;
}

//==============================================================================
BoxedLcpSolverType BoxedLcpConstraintSolver::getSecondaryBoxedLcpSolverType()
    const
{
  return mConfig.secondaryBoxedLcpSolver;
}

//==============================================================================
void BoxedLcpConstraintSolver::setBoxedLcpSolver(BoxedLcpSolverPtr lcpSolver)
{
  if (!lcpSolver) {
    DART_WARN(
        "[BoxedLcpConstraintSolver::setBoxedLcpSolver] nullptr for boxed LCP "
        "solver is not allowed.");
    return;
  }

  if (lcpSolver == mSecondaryBoxedLcpSolver) {
    DART_WARN(
        "[BoxedLcpConstraintSolver::setBoxedLcpSolver] Attempting to set a "
        "primary LCP solver that is the same with the secondary LCP solver, "
        "which is discouraged. Ignoring this request.");
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
    DART_WARN(
        "[BoxedLcpConstraintSolver::setSecondaryBoxedLcpSolver] Attempting "
        "to set the secondary LCP solver that is identical to the primary "
        "LCP solver, which is redundant. Please use different solvers or "
        "set the secondary LCP solver to nullptr.");
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
void solveBoxedLcp(BoxedLcp& lcp, ConstrainedGroup& group)
{
  DART_PROFILE_SCOPED;
  (void)lcp;
  (void)group;

  // Build LCP terms by aggregating them from constraints
  const std::size_t numConstraints = group.getNumConstraints();
  const std::size_t n = group.getTotalDimension();

  // If there is no constraint, then just return.
  if (0u == n) {
    return;
  }

  auto& mA = lcp.A;
  auto& mABackup = lcp.ABackup;
  auto& mX = lcp.x;
  auto& mXBackup = lcp.xBackup;
  auto& mB = lcp.b;
  auto& mBBackup = lcp.bBackup;
  // auto& mW = lcp.w;
  auto& mLo = lcp.lo;
  auto& mLoBackup = lcp.loBackup;
  auto& mHi = lcp.hi;
  auto& mHiBackup = lcp.hiBackup;
  auto& mFIndex = lcp.fIndex;
  auto& mFIndexBackup = lcp.fIndexBackup;
  auto& mOffset = lcp.offset;
  auto& mBoxedLcpSolver = lcp.boxedLcpSolver;
  auto& mSecondaryBoxedLcpSolver = lcp.secondaryBoxedLcpSolver;

  const int nSkip = dPAD(n);
  lcp.A.resize(n, nSkip);
  lcp.x.resize(n);
  lcp.b.resize(n);
  lcp.w.setZero(n);
  lcp.lo.resize(n);
  lcp.hi.resize(n);
  lcp.fIndex.setConstant(n, -1);

  // Compute offset indices
  lcp.offset.resize(numConstraints);
  lcp.offset[0] = 0;
  for (std::size_t i = 1; i < numConstraints; ++i) {
    const auto constraint = group.getConstraint(i - 1);
    assert(constraint->getDimension() > 0);
    lcp.offset[i] = lcp.offset[i - 1] + constraint->getDimension();
  }

  // For each constraint
  {
    DART_PROFILE_SCOPED_N("Construct LCP");
    ConstraintInfo constInfo;
    constInfo.invTimeStep = 1.0 / lcp.timeStep;
    for (std::size_t i = 0; i < numConstraints; ++i) {
      const auto constraint = group.getConstraint(i);

      constInfo.x = lcp.x.data() + lcp.offset[i];
      constInfo.lo = lcp.lo.data() + lcp.offset[i];
      constInfo.hi = lcp.hi.data() + lcp.offset[i];
      constInfo.b = lcp.b.data() + lcp.offset[i];
      constInfo.findex = lcp.fIndex.data() + lcp.offset[i];
      constInfo.w = lcp.w.data() + lcp.offset[i];

      // Fill vectors: lo, hi, b, w
      {
        DART_PROFILE_SCOPED_N("Fill lo, hi, b, w");
        constraint->getInformation(&constInfo);
      }

      // Fill a matrix by impulse tests: A
      {
        DART_PROFILE_SCOPED_N("Fill A");
        constraint->excite();
        for (std::size_t j = 0; j < constraint->getDimension(); ++j) {
          // Adjust findex for global index
          if (lcp.fIndex[lcp.offset[i] + j] >= 0) {
            lcp.fIndex[lcp.offset[i] + j] += lcp.offset[i];
          }

          // Apply impulse for impulse test
          {
            DART_PROFILE_SCOPED_N("Unit impulse test");
            constraint->applyUnitImpulse(j);
          }

          // Fill upper triangle blocks of A matrix
          {
            DART_PROFILE_SCOPED_N("Fill upper triangle of A");
            int index = nSkip * (lcp.offset[i] + j) + lcp.offset[i];
            constraint->getVelocityChange(mA.data() + index, true);
            for (std::size_t k = i + 1; k < numConstraints; ++k) {
              index = nSkip * (lcp.offset[i] + j) + lcp.offset[k];
              group.getConstraint(k)->getVelocityChange(
                  mA.data() + index, false);
            }
          }
        }
      }

      {
        DART_PROFILE_SCOPED_N("Unexcite");
        constraint->unexcite();
      }
    }

    {
      // Fill lower triangle blocks of A matrix
      DART_PROFILE_SCOPED_N("Fill lower triangle of A");
      mA.leftCols(n).triangularView<Eigen::Lower>()
          = mA.leftCols(n).triangularView<Eigen::Upper>().transpose();
    }
  }

  assert(isSymmetric(n, mA.data()));

  // Print LCP formulation
  //  dtdbg << "Before solve:" << std::endl;
  //  print(n, A, x, lo, hi, b, w, findex);
  //  std::cout << std::endl;

  // Solve LCP using the primary solver and fallback to secondary solver when
  // the primary solver failed.
  if (mSecondaryBoxedLcpSolver) {
    // Make backups for the secondary LCP solver because the primary solver
    // modifies the original terms.
    mABackup = mA;
    mXBackup = mX;
    mBBackup = mB;
    mLoBackup = mLo;
    mHiBackup = mHi;
    mFIndexBackup = mFIndex;
  }
  const bool earlyTermination = (mSecondaryBoxedLcpSolver != nullptr);
  assert(mBoxedLcpSolver);
  bool success = mBoxedLcpSolver->solve(
      n,
      mA.data(),
      mX.data(),
      mB.data(),
      0,
      mLo.data(),
      mHi.data(),
      mFIndex.data(),
      earlyTermination);

  // Sanity check. LCP solvers should not report success with nan values, but
  // it could happen. So we set the success to false for nan values.
  if (success && mX.hasNaN()) {
    success = false;
  }

  if (!success && mSecondaryBoxedLcpSolver) {
    DART_PROFILE_SCOPED_N("Secondary LCP");
    mSecondaryBoxedLcpSolver->solve(
        n,
        mABackup.data(),
        mXBackup.data(),
        mBBackup.data(),
        0,
        mLoBackup.data(),
        mHiBackup.data(),
        mFIndexBackup.data(),
        false);
    mX = mXBackup;
  }

  if (mX.hasNaN()) {
    dterr << "[BoxedLcpConstraintSolver] The solution of LCP includes NAN "
          << "values: " << mX.transpose() << ". We're setting it zero for "
          << "safety. Consider using more robust solver such as PGS as a "
          << "secondary solver. If this happens even with PGS solver, please "
          << "report this as a bug.\n";
    mX.setZero();
  }

  // Print LCP formulation
  //  dtdbg << "After solve:" << std::endl;
  //  print(n, A, x, lo, hi, b, w, findex);
  //  std::cout << std::endl;

  // Apply constraint impulses
  {
    DART_PROFILE_SCOPED_N("Apply constraint impulses");
    for (std::size_t i = 0; i < numConstraints; ++i) {
      const ConstraintBasePtr& constraint = group.getConstraint(i);
      constraint->applyImpulse(mX.data() + mOffset[i]);
      constraint->excite();
    }
  }
}

//==============================================================================
void BoxedLcpConstraintSolver::solveConstrainedGroups()
{
  DART_PROFILE_SCOPED;

  const int numGroups = mConstrainedGroups.size();

  // Prepare problems
  {
    DART_PROFILE_SCOPED_N("Prepare problems");
    mProblems.resize(numGroups);
    for (auto& prob : mProblems) {
      if (!prob.boxedLcpSolver) {
        prob.boxedLcpSolver
            = createBoxedLcpSolver(mConfig.primaryBoxedLcpSolver);
      }

      if (!prob.secondaryBoxedLcpSolver) {
        prob.secondaryBoxedLcpSolver
            = createBoxedLcpSolver(mConfig.secondaryBoxedLcpSolver);
      }

      prob.timeStep = mTimeStep;
    }
  }

  // Solve problems
  {
    DART_PROFILE_SCOPED_N("Solve problems");
#if DART_HAVE_Taskflow
    tf::Taskflow mTaskflow;
    mTaskflow.for_each_index(0, numGroups, 1, [&](int i) {
      solveBoxedLcp(mProblems[i], mConstrainedGroups[i]);
    });
    mExecutor.run(mTaskflow).wait();
#else
    for (auto i = 0u; i < mProblems.size(); ++i) {
      solveBoxedLcp(mProblems[i], mConstrainedGroups[i]);
    };
#endif
  }
}

} // namespace constraint
} // namespace dart
