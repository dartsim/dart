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
#include "dart/dynamics/Skeleton.hpp"
#include "dart/lcpsolver/Lemke.hpp"
#include "dart/lcpsolver/dantzig/DantzigLcp.hpp"

#include <algorithm>
#include <array>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <typeinfo>
#include <unordered_map>
#include <vector>

#include <cassert>
#include <cmath>

namespace dart {
namespace constraint {

namespace {

constexpr std::size_t kInlineConstraintCount = 8;
constexpr std::size_t kInlineLcpVectorSize = 12;
constexpr std::size_t kInlineLcpMatrixSize = 192;

struct MatrixFreeContactBodyScratch
{
  bool identityInertia = false;
  Eigen::LDLT<Eigen::Matrix6d> inertiaDecomposition;
  Eigen::Vector6d velocityChange = Eigen::Vector6d::Zero();

  Eigen::Vector6d solve(const Eigen::Vector6d& impulse) const
  {
    if (identityInertia)
      return impulse;

    return inertiaDecomposition.solve(impulse);
  }
};

struct MatrixFreeContactRow
{
  std::array<int, 2> bodyIndices{{-1, -1}};
  std::array<Eigen::Vector6d, 2> spatialNormals;
  std::array<Eigen::Vector6d, 2> unitVelocityChanges;
  double bodyDiagonal = 0.0;
  double diagonal = 0.0;
  double b = 0.0;
  double lo = 0.0;
  double hi = 0.0;
  int fIndex = -1;
};

struct BoxedLcpThreadScratch
{
  std::vector<ConstraintBase*> constraintPtrStorage;
  std::vector<std::size_t> constraintDimStorage;
  std::vector<std::size_t> constraintOffsetStorage;
  std::vector<double> lcpA;
  std::vector<double> lcpX;
  std::vector<double> lcpB;
  std::vector<double> lcpW;
  std::vector<double> lcpLo;
  std::vector<double> lcpHi;
  std::vector<int> lcpFIndex;

  std::vector<ContactConstraint*> matrixFreeContacts;
  std::vector<std::size_t> matrixFreeOffsets;
  std::vector<MatrixFreeContactRow> matrixFreeRows;
  std::vector<double> matrixFreeX;
  std::vector<double> matrixFreeW;
  std::vector<double> matrixFreeLo;
  std::vector<double> matrixFreeHi;
  std::vector<double> matrixFreeB;
  std::vector<int> matrixFreeFIndex;
  std::vector<dynamics::BodyNode*> matrixFreeBodyNodes;
  std::vector<MatrixFreeContactBodyScratch> matrixFreeBodies;
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

void reserveBoxedLcpSolverScratch(
    const BoxedLcpSolverPtr& solver, std::size_t n)
{
  auto dantzigSolver = std::dynamic_pointer_cast<DantzigBoxedLcpSolver>(solver);
  if (dantzigSolver) {
    dantzigSolver->reserve(n);
    return;
  }

  auto pgsSolver = std::dynamic_pointer_cast<PgsBoxedLcpSolver>(solver);
  if (pgsSolver)
    pgsSolver->reserve(n);
}

void reserveMatrixFreeContactScratch(
    BoxedLcpThreadScratch& scratch, std::size_t numConstraints, std::size_t n)
{
  scratch.matrixFreeContacts.reserve(numConstraints);
  scratch.matrixFreeOffsets.reserve(numConstraints);
  scratch.matrixFreeRows.reserve(n);
  scratch.matrixFreeX.reserve(n);
  scratch.matrixFreeW.reserve(n);
  scratch.matrixFreeLo.reserve(n);
  scratch.matrixFreeHi.reserve(n);
  scratch.matrixFreeB.reserve(n);
  scratch.matrixFreeFIndex.reserve(n);

  const std::size_t maxReactiveBodies = std::max(n, 2u * numConstraints);
  scratch.matrixFreeBodyNodes.reserve(maxReactiveBodies);
  scratch.matrixFreeBodies.reserve(maxReactiveBodies);
}

using MatrixFreeContactSolverOptions
    = BoxedLcpConstraintSolver::MatrixFreeContactSolverOptions;

std::mutex& matrixFreeContactOptionsMutex()
{
  static auto* mutex = new std::mutex;
  return *mutex;
}

std::unordered_map<
    const BoxedLcpConstraintSolver*,
    MatrixFreeContactSolverOptions>&
matrixFreeContactOptionsBySolver()
{
  static auto* options = new std::unordered_map<
      const BoxedLcpConstraintSolver*,
      MatrixFreeContactSolverOptions>;
  return *options;
}

MatrixFreeContactSolverOptions sanitizeMatrixFreeContactOptions(
    MatrixFreeContactSolverOptions options)
{
  if (options.mMaxIterations < 1)
    options.mMaxIterations = 1;
  if (!std::isfinite(options.mSor) || options.mSor <= 0.0)
    options.mSor = 1.0;
  if (!std::isfinite(options.mDeltaTolerance)
      || options.mDeltaTolerance < 0.0) {
    options.mDeltaTolerance = 0.0;
  }
  if (!std::isfinite(options.mRelativeDeltaTolerance)
      || options.mRelativeDeltaTolerance < 0.0) {
    options.mRelativeDeltaTolerance = 0.0;
  }
  if (!std::isfinite(options.mEpsilonForDivision)
      || options.mEpsilonForDivision <= 0.0) {
    options.mEpsilonForDivision = 1e-9;
  }
  return options;
}

//==============================================================================
template <typename ExactT, typename DynamicT>
bool isExactDynamicType(const DynamicT* object)
{
  if (object == nullptr)
    return false;

#if defined(__clang__)
  #pragma clang diagnostic push
  #pragma clang diagnostic ignored "-Wpotentially-evaluated-expression"
#endif
  const bool isExact = typeid(*object) == typeid(ExactT);
#if defined(__clang__)
  #pragma clang diagnostic pop
#endif

  return isExact;
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
BoxedLcpConstraintSolver::~BoxedLcpConstraintSolver()
{
  std::lock_guard<std::mutex> lock(matrixFreeContactOptionsMutex());
  matrixFreeContactOptionsBySolver().erase(this);
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
void BoxedLcpConstraintSolver::setMatrixFreeContactSolverOptions(
    const MatrixFreeContactSolverOptions& options)
{
  const auto sanitized = sanitizeMatrixFreeContactOptions(options);
  std::lock_guard<std::mutex> lock(matrixFreeContactOptionsMutex());
  matrixFreeContactOptionsBySolver()[this] = sanitized;
}

//==============================================================================
BoxedLcpConstraintSolver::MatrixFreeContactSolverOptions
BoxedLcpConstraintSolver::getMatrixFreeContactSolverOptions() const
{
  std::lock_guard<std::mutex> lock(matrixFreeContactOptionsMutex());
  const auto& optionsBySolver = matrixFreeContactOptionsBySolver();
  const auto it = optionsBySolver.find(this);
  if (it == optionsBySolver.end())
    return MatrixFreeContactSolverOptions{};

  return it->second;
}

//==============================================================================
void BoxedLcpConstraintSolver::setFromOtherConstraintSolver(
    const ConstraintSolver& other)
{
  ConstraintSolver::setFromOtherConstraintSolver(other);

  if (const auto* boxed
      = dynamic_cast<const BoxedLcpConstraintSolver*>(&other)) {
    setMatrixFreeContactSolverOptions(
        boxed->getMatrixFreeContactSolverOptions());
  } else {
    setMatrixFreeContactSolverOptions(MatrixFreeContactSolverOptions{});
  }
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
    scratch.lcpA.reserve(matrixSize);
    scratch.lcpX.reserve(n);
    scratch.lcpB.reserve(n);
    scratch.lcpW.reserve(n);
    scratch.lcpLo.reserve(n);
    scratch.lcpHi.reserve(n);
    scratch.lcpFIndex.reserve(n);
  }

  const auto matrixFreeOptions = getMatrixFreeContactSolverOptions();
  if (matrixFreeOptions.mEnabled && n >= matrixFreeOptions.mMinRows) {
    reserveMatrixFreeContactScratch(scratch, numConstraints, n);
  }

  reserveBoxedLcpSolverScratch(mBoxedLcpSolver, n);
  reserveBoxedLcpSolverScratch(mSecondaryBoxedLcpSolver, n);
}

//==============================================================================
bool BoxedLcpConstraintSolver::solveMatrixFreeContactGroup(
    ConstrainedGroup& group, bool profileRecording)
{
  const auto options = getMatrixFreeContactSolverOptions();
  if (!options.mEnabled)
    return false;

  const auto& constraints = group.mConstraints;
  const std::size_t numConstraints = constraints.size();
  if (numConstraints == 0u)
    return false;

  std::size_t n = 0u;
  for (const auto& constraint : constraints)
    n += constraint->getDimension();

  if (n < options.mMinRows)
    return false;

  DART_PROFILE_SCOPED_IF_N(
      profileRecording, "BoxedLcpConstraintSolver::matrixFreeContactSolve");

  const auto isSupportedSingleFreeBodySide
      = [](const dynamics::BodyNode* body, const dynamics::Skeleton* skel) {
          if (body == nullptr || skel == nullptr)
            return false;

          const auto* parentJoint = body->getParentJoint();
          return skel->isMobile() && skel->getNumBodyNodes() == 1u
                 && skel->getNumDofs() == 6u
                 && body->getParentBodyNode() == nullptr
                 && body->getNumChildBodyNodes() == 0u
                 && skel->getCachedRootFreeJoint() != nullptr
                 && parentJoint != nullptr
                 && (parentJoint->getActuatorType() == dynamics::Joint::FORCE
                     || parentJoint->getActuatorType()
                            == dynamics::Joint::PASSIVE);
        };

  auto& scratch = boxedLcpThreadScratch();
  reserveMatrixFreeContactScratch(scratch, numConstraints, n);

  auto& contacts = scratch.matrixFreeContacts;
  auto& offsets = scratch.matrixFreeOffsets;
  auto& rows = scratch.matrixFreeRows;
  auto& x = scratch.matrixFreeX;
  auto& w = scratch.matrixFreeW;
  auto& lo = scratch.matrixFreeLo;
  auto& hi = scratch.matrixFreeHi;
  auto& b = scratch.matrixFreeB;
  auto& fIndex = scratch.matrixFreeFIndex;
  auto& bodyNodes = scratch.matrixFreeBodyNodes;
  auto& bodies = scratch.matrixFreeBodies;

  contacts.clear();
  offsets.clear();
  rows.clear();
  bodyNodes.clear();
  bodies.clear();
  contacts.reserve(numConstraints);
  offsets.reserve(numConstraints);
  rows.reserve(n);

  x.resize(n);
  w.resize(n);
  lo.resize(n);
  hi.resize(n);
  b.resize(n);
  fIndex.resize(n);
  std::fill(x.begin(), x.end(), 0.0);
  std::fill(w.begin(), w.end(), 0.0);
  std::fill(fIndex.begin(), fIndex.end(), -1);

  const auto findOrAddBody = [&](dynamics::BodyNode* body) -> int {
    const auto it = std::find(bodyNodes.begin(), bodyNodes.end(), body);
    if (it != bodyNodes.end()) {
      return static_cast<int>(static_cast<std::size_t>(it - bodyNodes.begin()));
    }

    MatrixFreeContactBodyScratch bodyScratch;
    const Eigen::Matrix6d& articulatedInertia = body->getArticulatedInertia();
    static const Eigen::Matrix6d identityInertia = Eigen::Matrix6d::Identity();
    bodyScratch.identityInertia
        = articulatedInertia.cwiseEqual(identityInertia).all();
    if (!bodyScratch.identityInertia) {
      bodyScratch.inertiaDecomposition.compute(articulatedInertia);
      if (bodyScratch.inertiaDecomposition.info() != Eigen::Success)
        return -1;
    }

    const std::size_t index = bodies.size();
    bodyNodes.push_back(body);
    bodies.push_back(std::move(bodyScratch));
    return static_cast<int>(index);
  };

  ConstraintInfo constInfo;
  constInfo.invTimeStep = 1.0 / mTimeStep;

  std::size_t offset = 0u;
  for (const auto& constraintPtr : constraints) {
    if (!isExactDynamicType<ContactConstraint>(constraintPtr.get()))
      return false;

    auto* contact = static_cast<ContactConstraint*>(constraintPtr.get());
    if (contact->mIsSelfCollision)
      return false;

    const std::size_t dim = contact->getDimension();
    if (dim == 0u || (dim != 1u && dim != 3u))
      return false;

    const bool hasReactiveA = contact->mIsReactiveA;
    const bool hasReactiveB = contact->mIsReactiveB;
    if (!hasReactiveA && !hasReactiveB)
      return false;

    if (hasReactiveA
        && !isSupportedSingleFreeBodySide(
            contact->mBodyNodeA, contact->mSkeletonA)) {
      return false;
    }
    if (hasReactiveB
        && !isSupportedSingleFreeBodySide(
            contact->mBodyNodeB, contact->mSkeletonB)) {
      return false;
    }

    contacts.push_back(contact);
    offsets.push_back(offset);

    constInfo.x = x.data() + offset;
    constInfo.lo = lo.data() + offset;
    constInfo.hi = hi.data() + offset;
    constInfo.b = b.data() + offset;
    constInfo.findex = fIndex.data() + offset;
    constInfo.w = w.data() + offset;
    contact->getInformation(&constInfo);

    for (std::size_t local = 0u; local < dim; ++local) {
      const std::size_t rowIndex = offset + local;
      if (fIndex[rowIndex] >= 0)
        fIndex[rowIndex] += static_cast<int>(offset);

      MatrixFreeContactRow row;
      row.b = b[rowIndex];
      row.lo = lo[rowIndex];
      row.hi = hi[rowIndex];
      row.fIndex = fIndex[rowIndex];

      int side = 0;
      if (hasReactiveA) {
        const int bodyIndex = findOrAddBody(contact->mBodyNodeA);
        if (bodyIndex < 0)
          return false;

        row.bodyIndices[side] = bodyIndex;
        row.spatialNormals[side] = contact->mSpatialNormalA.col(local);
        row.unitVelocityChanges[side]
            = bodies[static_cast<std::size_t>(bodyIndex)].solve(
                row.spatialNormals[side]);
        row.bodyDiagonal
            += row.spatialNormals[side].dot(row.unitVelocityChanges[side]);
        ++side;
      }

      if (hasReactiveB) {
        const int bodyIndex = findOrAddBody(contact->mBodyNodeB);
        if (bodyIndex < 0)
          return false;

        row.bodyIndices[side] = bodyIndex;
        row.spatialNormals[side] = contact->mSpatialNormalB.col(local);
        row.unitVelocityChanges[side]
            = bodies[static_cast<std::size_t>(bodyIndex)].solve(
                row.spatialNormals[side]);
        row.bodyDiagonal
            += row.spatialNormals[side].dot(row.unitVelocityChanges[side]);
      }

      double extraDiagonal
          = row.bodyDiagonal * ContactConstraint::mConstraintForceMixing;
      if (local == 1u)
        extraDiagonal += contact->mPrimarySlipCompliance / mTimeStep;
      else if (local == 2u)
        extraDiagonal += contact->mSecondarySlipCompliance / mTimeStep;

      row.diagonal = row.bodyDiagonal + extraDiagonal;
      if (!std::isfinite(row.diagonal)
          || row.diagonal <= options.mEpsilonForDivision) {
        row.diagonal = 0.0;
      }

      rows.push_back(row);
    }

    offset += dim;
  }

  if (rows.size() != n)
    return false;

  const auto projectVelocityChange = [&](const MatrixFreeContactRow& row) {
    double value = 0.0;
    for (int side = 0; side < 2; ++side) {
      const int bodyIndex = row.bodyIndices[side];
      if (bodyIndex < 0)
        continue;

      value += row.spatialNormals[side].dot(
          bodies[static_cast<std::size_t>(bodyIndex)].velocityChange);
    }
    return value;
  };

  const auto applyDelta = [&](const MatrixFreeContactRow& row, double delta) {
    for (int side = 0; side < 2; ++side) {
      const int bodyIndex = row.bodyIndices[side];
      if (bodyIndex < 0)
        continue;

      bodies[static_cast<std::size_t>(bodyIndex)].velocityChange.noalias()
          += row.unitVelocityChanges[side] * delta;
    }
  };

  const auto clampImpulse = [&](const MatrixFreeContactRow& row, double value) {
    double lower = row.lo;
    double upper = row.hi;
    if (row.fIndex >= 0) {
      const double normalImpulse = x[static_cast<std::size_t>(row.fIndex)];
      upper = row.hi * normalImpulse;
      lower = -upper;
    }

    return std::max(lower, std::min(value, upper));
  };

  const double oneMinusSor = 1.0 - options.mSor;
  int numIterations = 0;
  bool solverConverged = false;
  for (int iter = 0; iter < options.mMaxIterations; ++iter) {
    bool converged = true;
    numIterations = iter + 1;
    for (std::size_t rowIndex = 0u; rowIndex < rows.size(); ++rowIndex) {
      const MatrixFreeContactRow& row = rows[rowIndex];
      if (row.diagonal <= options.mEpsilonForDivision)
        continue;

      const double oldX = x[rowIndex];
      const double bodyAx = projectVelocityChange(row);
      double newX = (row.b - bodyAx + row.bodyDiagonal * oldX) / row.diagonal;
      newX = options.mSor * newX + oneMinusSor * oldX;
      newX = clampImpulse(row, newX);
      if (!std::isfinite(newX))
        return false;

      const double delta = newX - oldX;
      if (delta != 0.0) {
        x[rowIndex] = newX;
        applyDelta(row, delta);
      }

      const double tolerance = std::max(
          options.mDeltaTolerance,
          options.mRelativeDeltaTolerance * std::max(1.0, std::abs(newX)));
      if (std::abs(delta) > tolerance)
        converged = false;
    }

    if (converged) {
      solverConverged = true;
      break;
    }
  }

  DART_PROFILE_COUNTER_IF_N(
      profileRecording, "BoxedLcpConstraintSolver::matrixFreeContactRows", n);
  DART_PROFILE_COUNTER_IF_N(
      profileRecording,
      "BoxedLcpConstraintSolver::matrixFreeContactIterations",
      numIterations);
  DART_PROFILE_COUNTER_IF_N(
      profileRecording,
      "BoxedLcpConstraintSolver::matrixFreeContactConverged",
      solverConverged ? 1u : 0u);

  for (std::size_t i = 0u; i < contacts.size(); ++i) {
    contacts[i]->applyImpulse(x.data() + offsets[i]);
    contacts[i]->excite();
  }

  return true;
}

//==============================================================================
void BoxedLcpConstraintSolver::solveConstrainedGroup(ConstrainedGroup& group)
{
  const bool profileRecording
      = dart::common::profile::isProfileRecordingEnabled();
  DART_PROFILE_SCOPED_IF_N(
      profileRecording, "BoxedLcpConstraintSolver::solveConstrainedGroup");

  // Build LCP terms by aggregating them from constraints
  const auto& constraints = group.mConstraints;
  const std::size_t numConstraints = constraints.size();
  if (numConstraints == 0u)
    return;

  if (solveMatrixFreeContactGroup(group, profileRecording))
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
    lcpA.resize(matrixSize);
    lcpX.resize(n);
    lcpB.resize(n);
    lcpW.resize(n);
    lcpLo.resize(n);
    lcpHi.resize(n);
    lcpFIndex.resize(n);
#if !DART_BUILD_MODE_RELEASE
    std::fill_n(lcpA.data(), matrixSize, 0.0);
#endif
    std::fill_n(lcpX.data(), n, 0.0);
    std::fill_n(lcpW.data(), n, 0.0);
    std::fill_n(lcpFIndex.data(), n, -1);
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
    // WP-PG.30: cached per-skeleton (keyed on structural version) instead of
    // repeating this dynamic_cast every step; see
    // Skeleton::getCachedRootFreeJoint(). directBody is directSkeleton's root
    // whenever getNumBodyNodes() == 1u below, so the cached root joint is
    // exactly parentJoint's classification.
    useDirectSingleFreeBody
        = directSkeleton != nullptr && directSkeleton->getNumBodyNodes() == 1u
          && directSkeleton->getNumDofs() == 6u
          && directBody->getParentBodyNode() == nullptr
          && directBody->getNumChildBodyNodes() == 0u
          && directSkeleton->getCachedRootFreeJoint() != nullptr
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
      std::fill_n(a, matrixSize, 0.0);
#endif
      std::fill_n(x, n, 0.0);
      std::fill_n(w, n, 0.0);
      std::fill_n(fIndex, n, -1);
    }
  };

  const auto constructLcpTerms = [&]() {
    DART_PROFILE_SCOPED_IF_N(
        profileRecording, "BoxedLcpConstraintSolver::constructLcpTerms");

    resetLcpTerms();
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

          constraint->getInformation(&constInfo);

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
        constraint->getInformation(&constInfo);

        // Fill a matrix by impulse tests: A
        constraint->excite();
        for (std::size_t j = 0; j < dim; ++j) {
          // Adjust findex for global index
          const std::size_t row = offset + j;
          if (fIndex[row] >= 0)
            fIndex[row] += offset;

          // Apply impulse for impulse test
          constraint->applyUnitImpulse(j);

          // Fill upper triangle blocks of A matrix
          std::size_t index = nSkip * row + offset;
          constraint->getVelocityChange(a + index, true);
          for (std::size_t k = i + 1; k < numConstraints; ++k) {
            index = nSkip * row + constraintOffsets[k];
            constraintPtrs[k]->getVelocityChange(a + index, false);
          }
        }

        constraint->unexcite();
      }
    }

    // Fill lower triangle blocks of A matrix
    for (std::size_t row = 1; row < n; ++row) {
      for (std::size_t col = 0; col < row; ++col) {
        a[nSkip * row + col] = a[nSkip * col + row];
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
  bool success = false;
  {
    DART_PROFILE_SCOPED_IF_N(
        profileRecording, "BoxedLcpConstraintSolver::primarySolve");
    success = mBoxedLcpSolver->solve(
        n, a, x, b, 0, lo, hi, fIndex, earlyTermination);
  }

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
    constructLcpTerms();
    {
      DART_PROFILE_SCOPED_IF_N(
          profileRecording, "BoxedLcpConstraintSolver::fallbackSolve");
      fallbackSuccess = mSecondaryBoxedLcpSolver->solve(
          n, a, x, b, 0, lo, hi, fIndex, false);
    }
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
    DART_PROFILE_SCOPED_IF_N(
        profileRecording, "BoxedLcpConstraintSolver::applyImpulses");

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
