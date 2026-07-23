/*
 * Copyright (c) 2011-2026, The DART development contributors
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

#ifndef DART_CONSTRAINT_DETAIL_EXACTCOULOMBCONTACTROWOPERATOR_HPP_
#define DART_CONSTRAINT_DETAIL_EXACTCOULOMBCONTACTROWOPERATOR_HPP_

#include "dart/constraint/ConstraintBase.hpp"
#include "dart/constraint/ContactConstraint.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/FreeJoint.hpp"
#include "dart/dynamics/Skeleton.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <iterator>
#include <typeinfo>
#include <vector>

namespace dart {
namespace constraint {
namespace detail {

/// Scratch-backed exact-Coulomb contact-row Delassus operator.
///
/// This mirrors the opt-in matrix-free contact structure used by
/// `BoxedLcpConstraintSolver::solveMatrixFreeContactGroup`: for supported
/// single free rigid bodies, each contact row stores its body Jacobian `J`
/// and unit velocity change `M^-1 J^T`, so `W * x` products, per-contact
/// diagonal Delassus blocks, block-column updates, and dense Delassus
/// assembly run through per-body scatter/gather instead of DART impulse
/// tests.
///
/// A group is supported when every constraint is an exactly-typed
/// non-self-collision three-row `ContactConstraint` whose reactive sides are
/// single-body free-joint skeletons with force/passive actuators. `build`
/// returns false for anything else so callers can keep the impulse-test
/// assembly route as the general fallback. The operator excludes DART's
/// constraint-force-mixing and slip-compliance regularization: it represents
/// the paper's pure reduced operator `W = J M^-1 J^T`.
struct ExactCoulombContactRowOperator
{
  struct BodyScratch
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

  struct Row
  {
    std::array<int, 2> bodyIndices{{-1, -1}};
    std::array<Eigen::Vector6d, 2> jacobians;
    std::array<Eigen::Vector6d, 2> unitVelocityChanges;
  };

  /// One (row, side) incidence of a body, used for gather loops.
  struct BodyRowIncidence
  {
    Eigen::Index row = -1;
    int side = 0;
  };

  /// One deterministic contact manifold for colored inner BGS.
  ///
  /// Contacts are grouped by their canonical reactive-body pair. The first
  /// occurrence of a pair fixes the manifold order, and contacts within a
  /// manifold retain the constraint input order. `writeContacts` is the
  /// sorted unique set of accumulator contact triples touched by all of the
  /// manifold's block-column updates.
  struct ColoredBlockGaussSeidelManifold
  {
    std::array<int, 2> canonicalBodyPair{{-1, -1}};
    std::vector<Eigen::Index> contacts;
    std::vector<Eigen::Index> writeContacts;
  };

  /// Greedy deterministic coloring of contact manifolds.
  ///
  /// Each color stores manifold indices in first-manifold order. Manifolds in
  /// one color have pairwise-disjoint accumulator write-contact sets, so their
  /// local contact updates can run concurrently without shared writes.
  struct ColoredBlockGaussSeidelSchedule
  {
    std::vector<ColoredBlockGaussSeidelManifold> manifolds;
    std::vector<std::vector<std::size_t>> colors;

    bool hasUsableParallelism() const
    {
      return std::any_of(colors.begin(), colors.end(), [](const auto& color) {
        return color.size() > 1u;
      });
    }
  };

  /// World-space contact identity and reaction frame for manifold-level
  /// warm starts across solver steps.
  struct ContactFrame
  {
    const dynamics::BodyNode* bodyA = nullptr;
    const dynamics::BodyNode* bodyB = nullptr;
    Eigen::Vector3d point = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();
    Eigen::Vector3d tangent1 = Eigen::Vector3d::Zero();
    Eigen::Vector3d tangent2 = Eigen::Vector3d::Zero();
  };

  /// Extract the world-space contact frame of a three-row friction contact.
  ///
  /// The row impulses act along `normal`, `tangent1`, and `tangent2` in the
  /// world frame, so a solved reaction triple can be stored as one world
  /// impulse vector and re-projected onto the next step's frames. Returns
  /// false for anything but an exactly-typed frictional ContactConstraint.
  static bool extractContactFrame(
      const ConstraintBase* constraint, ContactFrame& frame)
  {
    if (constraint == nullptr)
      return false;

#if defined(__clang__)
  #pragma clang diagnostic push
  #pragma clang diagnostic ignored "-Wpotentially-evaluated-expression"
#endif
    const bool isExactContactType
        = typeid(*constraint) == typeid(ContactConstraint);
#if defined(__clang__)
  #pragma clang diagnostic pop
#endif
    if (!isExactContactType)
      return false;

    const auto* contact = static_cast<const ContactConstraint*>(constraint);
    if (contact->mContact == nullptr || contact->getDimension() != 3u
        || !contact->mIsFrictionOn) {
      return false;
    }

    frame.bodyA = contact->mBodyNodeA;
    frame.bodyB = contact->mBodyNodeB;
    frame.point = contact->mContact->point;
    frame.normal = contact->mContact->normal;
    frame.tangent1 = contact->mTangentBasis.col(0);
    frame.tangent2 = contact->mTangentBasis.col(1);
    return frame.point.allFinite() && frame.normal.allFinite()
           && frame.tangent1.allFinite() && frame.tangent2.allFinite();
  }

  std::vector<BodyScratch> bodies;
  std::vector<Row> rows;
  std::vector<std::vector<BodyRowIncidence>> bodyIncidences;
  ColoredBlockGaussSeidelSchedule coloredBlockGaussSeidelSchedule;

  Eigen::Index getDimension() const
  {
    return static_cast<Eigen::Index>(rows.size());
  }

  std::size_t getContactCount() const
  {
    return rows.size() / 3u;
  }

  bool isBuilt() const
  {
    return !rows.empty();
  }

  /// Build rows from an all-contact constrained group. Returns false and
  /// leaves the operator empty when any constraint is unsupported.
  bool build(const std::vector<ConstraintBase*>& constraints)
  {
    bodies.clear();
    rows.clear();
    bodyIncidences.clear();
    coloredBlockGaussSeidelSchedule = ColoredBlockGaussSeidelSchedule();

    if (constraints.empty())
      return false;

    const auto isSupportedSingleFreeBodySide
        = [](const dynamics::BodyNode* body, const dynamics::Skeleton* skel) {
            if (body == nullptr || skel == nullptr)
              return false;

            const auto* parentJoint = body->getParentJoint();
            const auto hasSupportedActuatorTypes
                = [](const dynamics::Joint* joint) {
                    for (std::size_t i = 0u; i < joint->getNumDofs(); ++i) {
                      const auto actuatorType = joint->getActuatorType(i);
                      if (actuatorType != dynamics::Joint::FORCE
                          && actuatorType != dynamics::Joint::PASSIVE) {
                        return false;
                      }
                    }
                    return true;
                  };

            return skel->isMobile() && skel->getNumBodyNodes() == 1u
                   && skel->getNumDofs() == 6u
                   && body->getParentBodyNode() == nullptr
                   && body->getNumChildBodyNodes() == 0u
                   && parentJoint != nullptr
                   && dynamic_cast<const dynamics::FreeJoint*>(parentJoint)
                          != nullptr
                   && hasSupportedActuatorTypes(parentJoint);
          };

    std::vector<dynamics::BodyNode*> bodyKeys;
    const auto findOrAddBody = [&](dynamics::BodyNode* body) -> int {
      for (std::size_t i = 0u; i < bodyKeys.size(); ++i) {
        if (bodyKeys[i] == body)
          return static_cast<int>(i);
      }

      BodyScratch bodyScratch;
      const Eigen::Matrix6d& articulatedInertia = body->getArticulatedInertia();
      static const Eigen::Matrix6d identityInertia
          = Eigen::Matrix6d::Identity();
      bodyScratch.identityInertia
          = articulatedInertia.cwiseEqual(identityInertia).all();
      if (!bodyScratch.identityInertia) {
        bodyScratch.inertiaDecomposition.compute(articulatedInertia);
        if (bodyScratch.inertiaDecomposition.info() != Eigen::Success)
          return -1;
      }

      const int index = static_cast<int>(bodies.size());
      bodyKeys.push_back(body);
      bodies.push_back(std::move(bodyScratch));
      return index;
    };

    rows.reserve(3u * constraints.size());
    for (ConstraintBase* constraintPtr : constraints) {
      if (constraintPtr == nullptr) {
        clear();
        return false;
      }

#if defined(__clang__)
  #pragma clang diagnostic push
  #pragma clang diagnostic ignored "-Wpotentially-evaluated-expression"
#endif
      const bool isExactContactType
          = typeid(*constraintPtr) == typeid(ContactConstraint);
#if defined(__clang__)
  #pragma clang diagnostic pop
#endif
      if (!isExactContactType) {
        clear();
        return false;
      }

      auto* contact = static_cast<ContactConstraint*>(constraintPtr);
      if (contact->mIsSelfCollision || contact->getDimension() != 3u) {
        clear();
        return false;
      }

      const bool hasReactiveA = contact->mIsReactiveA;
      const bool hasReactiveB = contact->mIsReactiveB;
      if (!hasReactiveA && !hasReactiveB) {
        clear();
        return false;
      }

      if (hasReactiveA
          && !isSupportedSingleFreeBodySide(
              contact->mBodyNodeA, contact->mSkeletonA)) {
        clear();
        return false;
      }
      if (hasReactiveB
          && !isSupportedSingleFreeBodySide(
              contact->mBodyNodeB, contact->mSkeletonB)) {
        clear();
        return false;
      }

      for (int local = 0; local < 3; ++local) {
        Row row;
        int side = 0;
        if (hasReactiveA) {
          const int bodyIndex = findOrAddBody(contact->mBodyNodeA);
          if (bodyIndex < 0) {
            clear();
            return false;
          }

          row.bodyIndices[side] = bodyIndex;
          row.jacobians[side] = contact->mSpatialNormalA.col(local);
          row.unitVelocityChanges[side]
              = bodies[static_cast<std::size_t>(bodyIndex)].solve(
                  row.jacobians[side]);
          ++side;
        }

        if (hasReactiveB) {
          const int bodyIndex = findOrAddBody(contact->mBodyNodeB);
          if (bodyIndex < 0) {
            clear();
            return false;
          }

          row.bodyIndices[side] = bodyIndex;
          row.jacobians[side] = contact->mSpatialNormalB.col(local);
          row.unitVelocityChanges[side]
              = bodies[static_cast<std::size_t>(bodyIndex)].solve(
                  row.jacobians[side]);
        }

        if (!row.jacobians[0].allFinite()
            || !row.unitVelocityChanges[0].allFinite()
            || (row.bodyIndices[1] >= 0
                && (!row.jacobians[1].allFinite()
                    || !row.unitVelocityChanges[1].allFinite()))) {
          clear();
          return false;
        }

        rows.push_back(row);
      }
    }

    bodyIncidences.resize(bodies.size());
    for (Eigen::Index rowIndex = 0;
         rowIndex < static_cast<Eigen::Index>(rows.size());
         ++rowIndex) {
      const Row& row = rows[static_cast<std::size_t>(rowIndex)];
      for (int side = 0; side < 2; ++side) {
        const int bodyIndex = row.bodyIndices[side];
        if (bodyIndex < 0)
          continue;

        bodyIncidences[static_cast<std::size_t>(bodyIndex)].push_back(
            BodyRowIncidence{rowIndex, side});
      }
    }

    buildColoredBlockGaussSeidelSchedule();

    return true;
  }

  void clear()
  {
    bodies.clear();
    rows.clear();
    bodyIncidences.clear();
    coloredBlockGaussSeidelSchedule = ColoredBlockGaussSeidelSchedule();
  }

  const ColoredBlockGaussSeidelSchedule& getColoredBlockGaussSeidelSchedule()
      const
  {
    return coloredBlockGaussSeidelSchedule;
  }

  /// Compute `output = W * input` with one body scatter/gather pass.
  void apply(
      const Eigen::Ref<const Eigen::VectorXd>& input,
      Eigen::Ref<Eigen::VectorXd> output)
  {
    for (BodyScratch& body : bodies)
      body.velocityChange.setZero();

    for (std::size_t rowIndex = 0u; rowIndex < rows.size(); ++rowIndex) {
      const double impulse = input[static_cast<Eigen::Index>(rowIndex)];
      if (impulse == 0.0)
        continue;

      const Row& row = rows[rowIndex];
      for (int side = 0; side < 2; ++side) {
        const int bodyIndex = row.bodyIndices[side];
        if (bodyIndex < 0)
          continue;

        bodies[static_cast<std::size_t>(bodyIndex)].velocityChange.noalias()
            += row.unitVelocityChanges[side] * impulse;
      }
    }

    for (std::size_t rowIndex = 0u; rowIndex < rows.size(); ++rowIndex) {
      const Row& row = rows[rowIndex];
      double value = 0.0;
      for (int side = 0; side < 2; ++side) {
        const int bodyIndex = row.bodyIndices[side];
        if (bodyIndex < 0)
          continue;

        value += row.jacobians[side].dot(
            bodies[static_cast<std::size_t>(bodyIndex)].velocityChange);
      }
      output[static_cast<Eigen::Index>(rowIndex)] = value;
    }
  }

  /// Compute `output = W * input` through two deterministic parallel phases.
  ///
  /// `parallelFor(count, callback)` must synchronously invoke `callback(i)`
  /// exactly once for every index and return the number of participants that
  /// executed work. The first phase owns one body per index and sums that
  /// body's incidences in stored contact order, so no floating-point reduction
  /// is shared between workers. Completion of that call is the barrier before
  /// the second phase gathers into disjoint output rows.
  template <typename ParallelFor>
  std::size_t applyParallel(
      const Eigen::Ref<const Eigen::VectorXd>& input,
      Eigen::Ref<Eigen::VectorXd> output,
      ParallelFor& parallelFor)
  {
    auto scatterBody = [this, &input](std::size_t bodyIndex) {
      BodyScratch& body = bodies[bodyIndex];
      body.velocityChange.setZero();
      for (const BodyRowIncidence& incidence : bodyIncidences[bodyIndex]) {
        const double impulse = input[incidence.row];
        if (impulse == 0.0)
          continue;

        const Row& row = rows[static_cast<std::size_t>(incidence.row)];
        body.velocityChange.noalias()
            += row.unitVelocityChanges[incidence.side] * impulse;
      }
    };
    const std::size_t scatterParticipants
        = parallelFor(bodies.size(), scatterBody);

    auto gatherRow = [this, &output](std::size_t rowIndex) {
      const Row& row = rows[rowIndex];
      double value = 0.0;
      for (int side = 0; side < 2; ++side) {
        const int bodyIndex = row.bodyIndices[side];
        if (bodyIndex < 0)
          continue;

        value += row.jacobians[side].dot(
            bodies[static_cast<std::size_t>(bodyIndex)].velocityChange);
      }
      output[static_cast<Eigen::Index>(rowIndex)] = value;
    };
    const std::size_t gatherParticipants = parallelFor(rows.size(), gatherRow);
    return std::max(scatterParticipants, gatherParticipants);
  }

  /// Accumulate `accumulator += W.middleCols(3 * contact, 3) * delta` by
  /// scattering the contact's three rows into its touched bodies and
  /// gathering only the rows incident to those bodies.
  void accumulateBlockColumns(
      Eigen::Index contact,
      const Eigen::Vector3d& delta,
      Eigen::Ref<Eigen::VectorXd> accumulator) const
  {
    std::array<int, 2> touchedBodies{{-1, -1}};
    std::array<Eigen::Vector6d, 2> deltaVelocities;
    int touchedCount = 0;

    for (int local = 0; local < 3; ++local) {
      const double impulse = delta[local];
      if (impulse == 0.0)
        continue;

      const Row& row = rows[static_cast<std::size_t>(3 * contact + local)];
      for (int side = 0; side < 2; ++side) {
        const int bodyIndex = row.bodyIndices[side];
        if (bodyIndex < 0)
          continue;

        int slot = -1;
        for (int t = 0; t < touchedCount; ++t) {
          if (touchedBodies[static_cast<std::size_t>(t)] == bodyIndex) {
            slot = t;
            break;
          }
        }
        if (slot < 0) {
          slot = touchedCount++;
          touchedBodies[static_cast<std::size_t>(slot)] = bodyIndex;
          deltaVelocities[static_cast<std::size_t>(slot)].setZero();
        }

        deltaVelocities[static_cast<std::size_t>(slot)].noalias()
            += row.unitVelocityChanges[side] * impulse;
      }
    }

    for (int t = 0; t < touchedCount; ++t) {
      const int bodyIndex = touchedBodies[static_cast<std::size_t>(t)];
      const Eigen::Vector6d& deltaVelocity
          = deltaVelocities[static_cast<std::size_t>(t)];
      for (const BodyRowIncidence& incidence :
           bodyIncidences[static_cast<std::size_t>(bodyIndex)]) {
        const Row& row = rows[static_cast<std::size_t>(incidence.row)];
        accumulator[incidence.row]
            += row.jacobians[incidence.side].dot(deltaVelocity);
      }
    }
  }

  /// Extract one contact's 3x3 diagonal Delassus block directly.
  Eigen::Matrix3d diagonalBlock(Eigen::Index contact) const
  {
    Eigen::Matrix3d block = Eigen::Matrix3d::Zero();
    for (int a = 0; a < 3; ++a) {
      const Row& rowA = rows[static_cast<std::size_t>(3 * contact + a)];
      for (int b = 0; b < 3; ++b) {
        const Row& rowB = rows[static_cast<std::size_t>(3 * contact + b)];
        double value = 0.0;
        for (int sideA = 0; sideA < 2; ++sideA) {
          const int bodyA = rowA.bodyIndices[sideA];
          if (bodyA < 0)
            continue;

          for (int sideB = 0; sideB < 2; ++sideB) {
            if (rowB.bodyIndices[sideB] != bodyA)
              continue;

            value += rowA.jacobians[sideA].dot(rowB.unitVelocityChanges[sideB]);
          }
        }
        block(a, b) = value;
      }
    }
    return block;
  }

  /// Assemble the dense Delassus snapshot from the rows without impulse
  /// tests. The upper triangle is mirrored to the lower triangle to match
  /// the impulse-test adapter's symmetric snapshot convention.
  void assembleDense(Eigen::MatrixXd& delassus) const
  {
    const Eigen::Index dimension = getDimension();
    delassus.setZero(dimension, dimension);

    for (const auto& incidences : bodyIncidences) {
      for (const BodyRowIncidence& incidenceA : incidences) {
        const Row& rowA = rows[static_cast<std::size_t>(incidenceA.row)];
        for (const BodyRowIncidence& incidenceB : incidences) {
          if (incidenceB.row < incidenceA.row)
            continue;

          const Row& rowB = rows[static_cast<std::size_t>(incidenceB.row)];
          delassus(incidenceA.row, incidenceB.row)
              += rowA.jacobians[incidenceA.side].dot(
                  rowB.unitVelocityChanges[incidenceB.side]);
        }
      }
    }

    for (Eigen::Index row = 1; row < dimension; ++row) {
      for (Eigen::Index col = 0; col < row; ++col) {
        delassus(row, col) = delassus(col, row);
      }
    }
  }

private:
  void buildColoredBlockGaussSeidelSchedule()
  {
    auto& schedule = coloredBlockGaussSeidelSchedule;
    schedule = ColoredBlockGaussSeidelSchedule();

    for (Eigen::Index contact = 0;
         contact < static_cast<Eigen::Index>(getContactCount());
         ++contact) {
      std::array<int, 2> canonicalBodyPair
          = rows[static_cast<std::size_t>(3 * contact)].bodyIndices;
      if (canonicalBodyPair[1] < canonicalBodyPair[0])
        std::swap(canonicalBodyPair[0], canonicalBodyPair[1]);

      auto manifold = std::find_if(
          schedule.manifolds.begin(),
          schedule.manifolds.end(),
          [&canonicalBodyPair](const auto& candidate) {
            return candidate.canonicalBodyPair == canonicalBodyPair;
          });
      if (manifold == schedule.manifolds.end()) {
        ColoredBlockGaussSeidelManifold next;
        next.canonicalBodyPair = canonicalBodyPair;
        schedule.manifolds.push_back(std::move(next));
        manifold = std::prev(schedule.manifolds.end());
      }
      manifold->contacts.push_back(contact);
    }

    for (auto& manifold : schedule.manifolds) {
      for (const Eigen::Index contact : manifold.contacts) {
        const Row& source = rows[static_cast<std::size_t>(3 * contact)];
        for (const int bodyIndex : source.bodyIndices) {
          if (bodyIndex < 0)
            continue;
          for (const BodyRowIncidence& incidence :
               bodyIncidences[static_cast<std::size_t>(bodyIndex)]) {
            manifold.writeContacts.push_back(incidence.row / 3);
          }
        }
      }
      std::sort(manifold.writeContacts.begin(), manifold.writeContacts.end());
      manifold.writeContacts.erase(
          std::unique(
              manifold.writeContacts.begin(), manifold.writeContacts.end()),
          manifold.writeContacts.end());
    }

    std::vector<std::vector<Eigen::Index>> colorWriteContacts;
    for (std::size_t manifoldIndex = 0u;
         manifoldIndex < schedule.manifolds.size();
         ++manifoldIndex) {
      const auto& manifold = schedule.manifolds[manifoldIndex];
      std::size_t color = 0u;
      for (; color < colorWriteContacts.size(); ++color) {
        std::vector<Eigen::Index> intersection;
        std::set_intersection(
            manifold.writeContacts.begin(),
            manifold.writeContacts.end(),
            colorWriteContacts[color].begin(),
            colorWriteContacts[color].end(),
            std::back_inserter(intersection));
        if (intersection.empty())
          break;
      }

      if (color == colorWriteContacts.size()) {
        schedule.colors.emplace_back();
        colorWriteContacts.emplace_back();
      }
      schedule.colors[color].push_back(manifoldIndex);

      std::vector<Eigen::Index> mergedWriteContacts;
      mergedWriteContacts.reserve(
          colorWriteContacts[color].size() + manifold.writeContacts.size());
      std::set_union(
          colorWriteContacts[color].begin(),
          colorWriteContacts[color].end(),
          manifold.writeContacts.begin(),
          manifold.writeContacts.end(),
          std::back_inserter(mergedWriteContacts));
      colorWriteContacts[color] = std::move(mergedWriteContacts);
    }
  }
};

} // namespace detail
} // namespace constraint
} // namespace dart

#endif // DART_CONSTRAINT_DETAIL_EXACTCOULOMBCONTACTROWOPERATOR_HPP_
