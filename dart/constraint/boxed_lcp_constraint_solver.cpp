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
 *     MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
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

#include "dart/constraint/boxed_lcp_constraint_solver.hpp"

#include "dart/common/diagnostics.hpp"
#include "dart/common/logging.hpp"
#include "dart/constraint/dantzig_boxed_lcp_solver.hpp"
#include "dart/constraint/pgs_boxed_lcp_solver.hpp"
#include "dart/math/lcp/lcp_utils.hpp"
#include "dart/math/lcp/pivoting/dantzig_solver.hpp"
#include "dart/math/lcp/projection/pgs_solver.hpp"

#include <iterator>

DART_SUPPRESS_DEPRECATED_BEGIN

namespace dart {
namespace constraint {

namespace {

void configurePgsSolver(
    const std::shared_ptr<math::PgsSolver>& solver,
    const PgsBoxedLcpSolver::Option& option)
{
  if (!solver) {
    return;
  }

  math::LcpOptions options = solver->getDefaultOptions();
  options.maxIterations = option.mMaxIteration;
  options.absoluteTolerance = option.mDeltaXThreshold;
  options.relativeTolerance = option.mRelativeDeltaXTolerance;
  solver->setDefaultOptions(options);

  math::PgsSolver::Parameters params;
  params.epsilonForDivision = option.mEpsilonForDivision;
  params.randomizeConstraintOrder = option.mRandomizeConstraintOrder;
  solver->setParameters(params);
}

class BoxedLcpSolverAdapter final : public math::LcpSolver
{
public:
  explicit BoxedLcpSolverAdapter(BoxedLcpSolverPtr solver)
    : mSolver(std::move(solver))
  {
    // Do nothing
  }

  const BoxedLcpSolverPtr& getBoxedSolver() const
  {
    return mSolver;
  }

  math::LcpResult solve(
      const math::LcpProblem& problem,
      Eigen::VectorXd& x,
      const math::LcpOptions& options) override
  {
    math::LcpResult result;
    if (!mSolver) {
      result.status = math::LcpSolverStatus::Failed;
      result.message = "Boxed LCP solver is not set";
      return result;
    }

    const auto n = std::ssize(problem.b);
    if (n <= 0) {
      x.resize(0);
      result.status = math::LcpSolverStatus::Success;
      return result;
    }

    if (problem.A.rows() != n || problem.A.cols() != n || problem.lo.size() != n
        || problem.hi.size() != n || problem.findex.size() != n) {
      result.status = math::LcpSolverStatus::InvalidProblem;
      result.message = "Invalid LCP problem dimensions";
      return result;
    }

    const int nSkip = math::padding(static_cast<int>(n));
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
        paddedA(n, nSkip);
    paddedA.setZero();
    paddedA.leftCols(n) = problem.A;

    Eigen::VectorXd xVec = (x.size() == n) ? x : Eigen::VectorXd::Zero(n);
    Eigen::VectorXd bVec = problem.b;
    Eigen::VectorXd loVec = problem.lo;
    Eigen::VectorXd hiVec = problem.hi;
    Eigen::VectorXi findexVec = problem.findex;

    const bool success = mSolver->solve(
        n,
        paddedA.data(),
        xVec.data(),
        bVec.data(),
        0,
        loVec.data(),
        hiVec.data(),
        findexVec.data(),
        options.earlyTermination);

    x = std::move(xVec);
    result.status = success ? math::LcpSolverStatus::Success
                            : math::LcpSolverStatus::Failed;
    return result;
  }

  std::string getName() const override
  {
    return mSolver ? std::string(mSolver->getTypeView()) : "BoxedLcpSolver";
  }

  std::string getCategory() const override
  {
    return "Boxed";
  }

private:
  BoxedLcpSolverPtr mSolver;
};

} // namespace

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
    DART_WARN(
        "[BoxedLcpConstraintSolver] Attempting to construct with nullptr LCP "
        "solver, which is not allowed. Using Dantzig solver instead.");
    setBoxedLcpSolver(std::make_shared<DantzigBoxedLcpSolver>());
  }

  setSecondaryBoxedLcpSolver(std::move(secondaryBoxedLcpSolver));
}

//==============================================================================
void BoxedLcpConstraintSolver::setBoxedLcpSolver(BoxedLcpSolverPtr lcpSolver)
{
  if (!lcpSolver) {
    DART_WARN("nullptr for boxed LCP solver is not allowed.");
    return;
  }

  DART_WARN_IF(
      lcpSolver == mSecondaryBoxedLcpSolver,
      "Attempting to set a primary LCP solver that is the same as the "
      "secondary LCP solver, which is discouraged.");

  mBoxedLcpSolver = std::move(lcpSolver);
  syncLcpSolversFromBoxedSolvers();
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
  DART_WARN_IF(
      lcpSolver == mBoxedLcpSolver,
      "Attempting to set the secondary LCP solver that is identical to the "
      "primary LCP solver, which is redundant. Please use different solvers or "
      "set the secondary LCP solver to nullptr.");

  mSecondaryBoxedLcpSolver = std::move(lcpSolver);
  syncLcpSolversFromBoxedSolvers();
}

//==============================================================================
ConstBoxedLcpSolverPtr BoxedLcpConstraintSolver::getSecondaryBoxedLcpSolver()
    const
{
  return mSecondaryBoxedLcpSolver;
}

//==============================================================================
void BoxedLcpConstraintSolver::solveConstrainedGroup(ConstrainedGroup& group)
{
  syncLcpSolversFromBoxedSolvers();
  ConstraintSolver::solveConstrainedGroup(group);
}

//==============================================================================
void BoxedLcpConstraintSolver::syncLcpSolversFromBoxedSolvers()
{
  if (auto pgs
      = std::dynamic_pointer_cast<PgsBoxedLcpSolver>(mBoxedLcpSolver)) {
    auto primarySolver = std::dynamic_pointer_cast<math::PgsSolver>(mLcpSolver);
    if (!primarySolver) {
      primarySolver = std::make_shared<math::PgsSolver>();
      mLcpSolver = primarySolver;
    }
    configurePgsSolver(primarySolver, pgs->getOption());
  } else if (std::dynamic_pointer_cast<DantzigBoxedLcpSolver>(
                 mBoxedLcpSolver)) {
    if (!std::dynamic_pointer_cast<math::DantzigSolver>(mLcpSolver)) {
      mLcpSolver = std::make_shared<math::DantzigSolver>();
    }
  } else {
    auto adapter = std::dynamic_pointer_cast<BoxedLcpSolverAdapter>(mLcpSolver);
    if (!adapter || adapter->getBoxedSolver() != mBoxedLcpSolver) {
      mLcpSolver = std::make_shared<BoxedLcpSolverAdapter>(mBoxedLcpSolver);
    }
  }

  if (mSecondaryBoxedLcpSolver) {
    if (auto pgs = std::dynamic_pointer_cast<PgsBoxedLcpSolver>(
            mSecondaryBoxedLcpSolver)) {
      auto secondarySolver
          = std::dynamic_pointer_cast<math::PgsSolver>(mSecondaryLcpSolver);
      if (!secondarySolver) {
        secondarySolver = std::make_shared<math::PgsSolver>();
        mSecondaryLcpSolver = secondarySolver;
      }
      configurePgsSolver(secondarySolver, pgs->getOption());
    } else if (std::dynamic_pointer_cast<DantzigBoxedLcpSolver>(
                   mSecondaryBoxedLcpSolver)) {
      if (!std::dynamic_pointer_cast<math::DantzigSolver>(
              mSecondaryLcpSolver)) {
        mSecondaryLcpSolver = std::make_shared<math::DantzigSolver>();
      }
    } else {
      auto adapter = std::dynamic_pointer_cast<BoxedLcpSolverAdapter>(
          mSecondaryLcpSolver);
      if (!adapter || adapter->getBoxedSolver() != mSecondaryBoxedLcpSolver) {
        mSecondaryLcpSolver
            = std::make_shared<BoxedLcpSolverAdapter>(mSecondaryBoxedLcpSolver);
      }
    }
  } else {
    mSecondaryLcpSolver.reset();
  }

  if (auto adapter
      = std::dynamic_pointer_cast<BoxedLcpSolverAdapter>(mLcpSolver)) {
    math::LcpOptions options = adapter->getDefaultOptions();
    options.earlyTermination = (mSecondaryBoxedLcpSolver != nullptr);
    adapter->setDefaultOptions(options);
  }

  if (auto dantzig
      = std::dynamic_pointer_cast<math::DantzigSolver>(mLcpSolver)) {
    math::LcpOptions options = dantzig->getDefaultOptions();
    options.earlyTermination = (mSecondaryBoxedLcpSolver != nullptr);
    dantzig->setDefaultOptions(options);
  }
}

} // namespace constraint
} // namespace dart

DART_SUPPRESS_DEPRECATED_END
