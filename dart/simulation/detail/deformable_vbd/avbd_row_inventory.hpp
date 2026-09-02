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
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <dart/simulation/detail/deformable_vbd/avbd_constraint.hpp>

#include <dart/common/memory_allocator.hpp>
#include <dart/common/stl_allocator.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <limits>
#include <span>
#include <tuple>
#include <type_traits>
#include <vector>

#include <cstddef>
#include <cstdint>

namespace dart::simulation::detail::deformable_vbd {

/// Broad semantic role of one persistent AVBD scalar row. The role is part of
/// the row key so finite-stiffness spring, contact-normal, self-contact normal,
/// friction, attachment, joint, linear/angular motor, and fracture rows for the
/// same bodies/features never alias each other.
enum class AvbdScalarRowRole : std::uint8_t
{
  Generic = 0,
  Attachment,
  ContactNormal,
  SelfContactNormal,
  DeformableSpring,
  FrictionTangent,
  JointLinear,
  JointAngular,
  RigidDistanceSpring,
  MotorLinear,
  MotorAngular,
  Fracture,
};

/// Whether the row carries a Lagrange multiplier (hard/bounded constraint) or
/// only a progressively ramped finite stiffness force.
enum class AvbdScalarRowKind : std::uint8_t
{
  HardConstraint = 0,
  FiniteStiffness,
};

/// Stable identity for a scalar AVBD row. Callers decide how to encode body,
/// feature, contact-manifold, and axis IDs; the inventory only requires those
/// IDs to be stable for rows that should warm start across frames.
struct AvbdScalarRowKey
{
  AvbdScalarRowRole role = AvbdScalarRowRole::Generic;
  std::uint64_t objectA = 0;
  std::uint64_t objectB = 0;
  std::uint64_t featureA = 0;
  std::uint64_t featureB = 0;
  std::uint32_t row = 0;
  std::uint8_t axis = 0;
};

//==============================================================================
inline auto avbdScalarRowKeyTuple(const AvbdScalarRowKey& key)
{
  return std::tuple{
      static_cast<std::uint8_t>(key.role),
      key.objectA,
      key.objectB,
      key.featureA,
      key.featureB,
      key.row,
      key.axis};
}

//==============================================================================
inline bool operator<(const AvbdScalarRowKey& lhs, const AvbdScalarRowKey& rhs)
{
  return avbdScalarRowKeyTuple(lhs) < avbdScalarRowKeyTuple(rhs);
}

//==============================================================================
inline bool operator==(const AvbdScalarRowKey& lhs, const AvbdScalarRowKey& rhs)
{
  return avbdScalarRowKeyTuple(lhs) == avbdScalarRowKeyTuple(rhs);
}

/// Per-step descriptor for an active scalar row.
struct AvbdScalarRowDescriptor
{
  AvbdScalarRowKey key;
  AvbdScalarRowKind kind = AvbdScalarRowKind::HardConstraint;
  AvbdScalarRowBounds bounds;
  double startStiffness = 1.0;
  double materialStiffness = std::numeric_limits<double>::infinity();
  double maxStiffness = std::numeric_limits<double>::infinity();
};

/// Canonical identity for one persistent rigid-contact tangent anchor.
///
/// Unlike a scalar-row key, this identity deliberately excludes the row role
/// and tangent-axis index. The two scalar tangent rows share one contact
/// anchor, and changing the tangent basis must not create or duplicate anchor
/// state.
struct AvbdContactTangentAnchorKey
{
  std::uint64_t objectA = 0;
  std::uint64_t objectB = 0;
  std::uint64_t featureA = 0;
  std::uint64_t featureB = 0;
  std::uint32_t row = 0;
};

//==============================================================================
inline auto avbdContactTangentAnchorKeyTuple(
    const AvbdContactTangentAnchorKey& key)
{
  return std::tuple{
      key.objectA, key.objectB, key.featureA, key.featureB, key.row};
}

//==============================================================================
inline bool operator==(
    const AvbdContactTangentAnchorKey& lhs,
    const AvbdContactTangentAnchorKey& rhs)
{
  return avbdContactTangentAnchorKeyTuple(lhs)
         == avbdContactTangentAnchorKeyTuple(rhs);
}

//==============================================================================
inline bool operator<(
    const AvbdContactTangentAnchorKey& lhs,
    const AvbdContactTangentAnchorKey& rhs)
{
  return avbdContactTangentAnchorKeyTuple(lhs)
         < avbdContactTangentAnchorKeyTuple(rhs);
}

/// One contact-owned pair of feature-local tangent anchors. The local points
/// follow the canonical endpoint order encoded by `key`, not the transient
/// body-A/body-B ordering supplied by collision detection.
struct AvbdContactTangentAnchorState
{
  AvbdContactTangentAnchorKey key;
  Eigen::Vector3d localPointA = Eigen::Vector3d::Zero();
  Eigen::Vector3d localPointB = Eigen::Vector3d::Zero();
  bool valid = false;
  bool sticking = false;
};

/// Detected material-point identity for one rigid contact-normal row.
///
/// This is deliberately separate from `AvbdContactTangentAnchorState`: a
/// sticking tangent anchor keeps its older feature-local points while the
/// collision detector's current material point can move. Keeping both states
/// distinct lets spatial-rank warm starts fail closed without resetting valid
/// single-contact static-friction anchors.
struct AvbdRigidContactIdentityState
{
  AvbdScalarRowKey key;
  Eigen::Vector3d localPointA = Eigen::Vector3d::Zero();
  Eigen::Vector3d localPointB = Eigen::Vector3d::Zero();
};

/// Axis-independent identity for one deformable friction pair. Static-contact
/// and self-contact anchors live in separate inventories, so the fields match
/// the corresponding tangent row key with only the tangent axis removed.
struct AvbdDeformableTangentAnchorKey
{
  std::uint64_t objectA = 0;
  std::uint64_t objectB = 0;
  std::uint64_t featureA = 0;
  std::uint64_t featureB = 0;
  std::uint32_t row = 0;
};

//==============================================================================
inline auto avbdDeformableTangentAnchorKeyTuple(
    const AvbdDeformableTangentAnchorKey& key)
{
  return std::tuple{
      key.objectA, key.objectB, key.featureA, key.featureB, key.row};
}

//==============================================================================
inline bool operator<(
    const AvbdDeformableTangentAnchorKey& lhs,
    const AvbdDeformableTangentAnchorKey& rhs)
{
  return avbdDeformableTangentAnchorKeyTuple(lhs)
         < avbdDeformableTangentAnchorKeyTuple(rhs);
}

//==============================================================================
inline bool operator==(
    const AvbdDeformableTangentAnchorKey& lhs,
    const AvbdDeformableTangentAnchorKey& rhs)
{
  return avbdDeformableTangentAnchorKeyTuple(lhs)
         == avbdDeformableTangentAnchorKeyTuple(rhs);
}

//==============================================================================
inline AvbdDeformableTangentAnchorKey makeAvbdDeformableTangentAnchorKey(
    const AvbdScalarRowKey& key)
{
  return AvbdDeformableTangentAnchorKey{
      key.objectA, key.objectB, key.featureA, key.featureB, key.row};
}

/// Compact persistent state for one deformable/static friction pair.
struct AvbdHalfSpaceTangentAnchorState
{
  AvbdDeformableTangentAnchorKey key;
  Eigen::Vector3d accumulatedDisplacement = Eigen::Vector3d::Zero();
  std::array<Eigen::Vector3d, 2> basis{
      Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY()};
  std::uint32_t vertex = 0;
  bool valid = false;
  bool sticking = false;
};

/// Persistent state for one point-triangle or edge-edge friction pair. The
/// four reference positions reconstruct the prior generalized tangent stencil
/// for exact dual transport after replay.
struct AvbdSelfContactTangentAnchorState
{
  AvbdDeformableTangentAnchorKey key;
  Eigen::Vector3d accumulatedDisplacement = Eigen::Vector3d::Zero();
  std::array<Eigen::Vector3d, 2> basis{
      Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY()};
  std::array<Eigen::Vector3d, 4> referencePositions{
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero()};
  std::array<std::uint32_t, 4> nodes{0, 0, 0, 0};
  bool isEdgeEdge = false;
  bool valid = false;
  bool sticking = false;
};

/// Warm-start parameters applied when an active row key persists across frames.
struct AvbdRowWarmStartOptions
{
  double alpha = 0.99;
  double gamma = 0.99;
  double maxStiffness = std::numeric_limits<double>::infinity();
};

/// One active row plus its persistent scalar state.
struct AvbdScalarRowRecord
{
  AvbdScalarRowDescriptor descriptor;
  AvbdScalarRowState state;
  Eigen::Vector3d direction = Eigen::Vector3d::Zero();
};

//==============================================================================
inline double maxAvbdDescriptorStiffness(
    const AvbdScalarRowDescriptor& descriptor,
    const AvbdRowWarmStartOptions& options)
{
  return std::min(
      {descriptor.materialStiffness,
       descriptor.maxStiffness,
       options.maxStiffness});
}

//==============================================================================
inline AvbdScalarRowState initialAvbdScalarRowState(
    const AvbdScalarRowDescriptor& descriptor,
    const AvbdRowWarmStartOptions& options = {})
{
  AvbdScalarRowState state;
  state.lambda = 0.0;
  state.stiffness = std::min(
      descriptor.startStiffness,
      maxAvbdDescriptorStiffness(descriptor, options));
  return state;
}

//==============================================================================
inline AvbdScalarRowState warmStartAvbdScalarRowState(
    const AvbdScalarRowState& previous,
    const AvbdScalarRowDescriptor& descriptor,
    const AvbdRowWarmStartOptions& options)
{
  const double maxStiffness = maxAvbdDescriptorStiffness(descriptor, options);
  if (descriptor.kind == AvbdScalarRowKind::HardConstraint) {
    return warmStartAvbdHardConstraint(
        previous,
        descriptor.startStiffness,
        options.alpha,
        options.gamma,
        maxStiffness);
  }

  AvbdScalarRowState next;
  next.lambda = 0.0;
  const double lower = std::min(descriptor.startStiffness, maxStiffness);
  next.stiffness
      = std::clamp(options.gamma * previous.stiffness, lower, maxStiffness);
  return next;
}

/// Deterministic active-row inventory for AVBD warm starting. `syncActiveRows`
/// preserves state for rows whose keys persist, drops rows that disappear, and
/// keeps active records in descriptor order so callers can build solver-local
/// row arrays without depending on map iteration order.
class AvbdScalarRowInventory
{
public:
  using RecordAllocator = ::dart::common::StlAllocator<AvbdScalarRowRecord>;
  using RecordVector = std::vector<AvbdScalarRowRecord, RecordAllocator>;
  using DescriptorAllocator
      = ::dart::common::StlAllocator<AvbdScalarRowDescriptor>;
  using DescriptorVector
      = std::vector<AvbdScalarRowDescriptor, DescriptorAllocator>;

  //==============================================================================
  AvbdScalarRowInventory() = default;

  //==============================================================================
  explicit AvbdScalarRowInventory(::dart::common::MemoryAllocator& allocator)
    : mRecords(RecordAllocator{allocator}),
      mPreviousRecords(RecordAllocator{allocator}),
      mDescriptorScratch(DescriptorAllocator{allocator})
  {
  }

  //==============================================================================
  template <
      typename Allocator,
      typename
      = std::enable_if_t<std::is_constructible_v<RecordAllocator, Allocator>>>
  explicit AvbdScalarRowInventory(const Allocator& allocator)
    : mRecords(RecordAllocator{allocator}),
      mPreviousRecords(RecordAllocator{allocator}),
      mDescriptorScratch(DescriptorAllocator{allocator})
  {
  }

  //==============================================================================
  void syncActiveRows(
      std::span<const AvbdScalarRowDescriptor> descriptors,
      const AvbdRowWarmStartOptions& options)
  {
    if (descriptors.empty()) {
      mRecords.clear();
      return;
    }

    if (mRecords.size() == descriptors.size()) {
      bool sameOrder = true;
      for (std::size_t i = 0; i < descriptors.size(); ++i) {
        if (!(mRecords[i].descriptor.key == descriptors[i].key)) {
          sameOrder = false;
          break;
        }
      }

      if (sameOrder) {
        for (std::size_t i = 0; i < descriptors.size(); ++i) {
          const AvbdScalarRowState state = warmStartAvbdScalarRowState(
              mRecords[i].state, descriptors[i], options);
          AvbdScalarRowRecord record;
          record.descriptor = descriptors[i];
          record.state = state;
          mRecords[i] = record;
        }
        return;
      }
    }

    mPreviousRecords.clear();
    mPreviousRecords.insert(
        mPreviousRecords.end(), mRecords.begin(), mRecords.end());
    std::sort(
        mPreviousRecords.begin(),
        mPreviousRecords.end(),
        [](const AvbdScalarRowRecord& lhs, const AvbdScalarRowRecord& rhs) {
          return lhs.descriptor.key < rhs.descriptor.key;
        });

    mRecords.clear();
    mRecords.reserve(descriptors.size());
    for (const AvbdScalarRowDescriptor& descriptor : descriptors) {
      AvbdScalarRowState state = initialAvbdScalarRowState(descriptor, options);
      if (const AvbdScalarRowRecord* previous
          = findInSorted(mPreviousRecords, descriptor.key)) {
        state
            = warmStartAvbdScalarRowState(previous->state, descriptor, options);
      }
      AvbdScalarRowRecord record;
      record.descriptor = descriptor;
      record.state = state;
      mRecords.push_back(record);
    }
  }

  //==============================================================================
  template <typename DescriptorAt>
  void syncActiveRowsByIndex(
      std::size_t descriptorCount,
      DescriptorAt descriptorAt,
      const AvbdRowWarmStartOptions& options)
  {
    syncActiveRowsByIndex(
        descriptorCount,
        [&](std::size_t index) { return descriptorAt(index).key; },
        descriptorAt,
        options);
  }

  //==============================================================================
  template <typename KeyAt, typename DescriptorAt>
  void syncActiveRowsByIndex(
      std::size_t descriptorCount,
      KeyAt keyAt,
      DescriptorAt descriptorAt,
      const AvbdRowWarmStartOptions& options)
  {
    if (descriptorCount == 0u) {
      mRecords.clear();
      mDescriptorScratch.clear();
      return;
    }

    if (mRecords.size() == descriptorCount) {
      bool sameOrder = true;
      for (std::size_t i = 0; i < descriptorCount; ++i) {
        if (!(mRecords[i].descriptor.key == keyAt(i))) {
          sameOrder = false;
          break;
        }
      }

      if (sameOrder) {
        for (std::size_t i = 0; i < descriptorCount; ++i) {
          const AvbdScalarRowDescriptor descriptor = descriptorAt(i);
          const AvbdScalarRowState state = warmStartAvbdScalarRowState(
              mRecords[i].state, descriptor, options);
          AvbdScalarRowRecord record;
          record.descriptor = descriptor;
          record.state = state;
          mRecords[i] = record;
        }
        return;
      }
    }

    mDescriptorScratch.clear();
    mDescriptorScratch.reserve(descriptorCount);
    for (std::size_t i = 0; i < descriptorCount; ++i) {
      mDescriptorScratch.push_back(descriptorAt(i));
    }
    syncActiveRows(mDescriptorScratch, options);
  }

  //==============================================================================
  void reserve(std::size_t capacity)
  {
    mRecords.reserve(capacity);
    mPreviousRecords.reserve(capacity);
    mDescriptorScratch.reserve(capacity);
  }

  //==============================================================================
  [[nodiscard]] std::size_t size() const noexcept
  {
    return mRecords.size();
  }

  //==============================================================================
  [[nodiscard]] bool empty() const noexcept
  {
    return mRecords.empty();
  }

  //==============================================================================
  void clear() noexcept
  {
    mRecords.clear();
    mDescriptorScratch.clear();
  }

  //==============================================================================
  [[nodiscard]] AvbdScalarRowRecord& operator[](std::size_t index) noexcept
  {
    return mRecords[index];
  }

  //==============================================================================
  [[nodiscard]] const AvbdScalarRowRecord& operator[](
      std::size_t index) const noexcept
  {
    return mRecords[index];
  }

  //==============================================================================
  [[nodiscard]] RecordVector& records() noexcept
  {
    return mRecords;
  }

  //==============================================================================
  [[nodiscard]] const RecordVector& records() const noexcept
  {
    return mRecords;
  }

  //==============================================================================
  [[nodiscard]] AvbdScalarRowRecord* find(const AvbdScalarRowKey& key) noexcept
  {
    for (AvbdScalarRowRecord& record : mRecords) {
      if (record.descriptor.key == key) {
        return &record;
      }
    }
    return nullptr;
  }

  //==============================================================================
  [[nodiscard]] const AvbdScalarRowRecord* find(
      const AvbdScalarRowKey& key) const noexcept
  {
    for (const AvbdScalarRowRecord& record : mRecords) {
      if (record.descriptor.key == key) {
        return &record;
      }
    }
    return nullptr;
  }

private:
  //==============================================================================
  [[nodiscard]] static const AvbdScalarRowRecord* findInSorted(
      std::span<const AvbdScalarRowRecord> records,
      const AvbdScalarRowKey& key) noexcept
  {
    const auto match = std::lower_bound(
        records.begin(),
        records.end(),
        key,
        [](const AvbdScalarRowRecord& record, const AvbdScalarRowKey& value) {
          return record.descriptor.key < value;
        });
    if (match != records.end() && match->descriptor.key == key) {
      return &*match;
    }
    return nullptr;
  }

  RecordVector mRecords;
  RecordVector mPreviousRecords;
  DescriptorVector mDescriptorScratch;
};

/// Deterministic allocator-backed inventory for one deformable tangent anchor
/// per two scalar friction rows. Anchor payloads stay out of the scalar row
/// record because most AVBD row families never use them.
template <typename Anchor>
class AvbdDeformableTangentAnchorInventory
{
public:
  using AnchorAllocator = ::dart::common::StlAllocator<Anchor>;
  using AnchorVector = std::vector<Anchor, AnchorAllocator>;

  AvbdDeformableTangentAnchorInventory() = default;

  explicit AvbdDeformableTangentAnchorInventory(
      ::dart::common::MemoryAllocator& allocator)
    : mAnchors(AnchorAllocator{allocator}),
      mPreviousAnchors(AnchorAllocator{allocator})
  {
  }

  template <
      typename Allocator,
      typename
      = std::enable_if_t<std::is_constructible_v<AnchorAllocator, Allocator>>>
  explicit AvbdDeformableTangentAnchorInventory(const Allocator& allocator)
    : mAnchors(AnchorAllocator{allocator}),
      mPreviousAnchors(AnchorAllocator{allocator})
  {
  }

  void syncActiveKeys(std::span<const AvbdDeformableTangentAnchorKey> keys)
  {
    syncActiveKeysByIndex(
        keys.size(), [&](std::size_t index) { return keys[index]; });
  }

  template <typename KeyAt>
  void syncActiveKeysByIndex(std::size_t keyCount, KeyAt keyAt)
  {
    if (keyCount == 0u) {
      mAnchors.clear();
      return;
    }

    if (mAnchors.size() == keyCount) {
      bool sameOrder = true;
      for (std::size_t i = 0; i < keyCount; ++i) {
        if (!(mAnchors[i].key == keyAt(i))) {
          sameOrder = false;
          break;
        }
      }
      if (sameOrder) {
        return;
      }
    }

    mPreviousAnchors.clear();
    mPreviousAnchors.insert(
        mPreviousAnchors.end(), mAnchors.begin(), mAnchors.end());
    std::sort(
        mPreviousAnchors.begin(),
        mPreviousAnchors.end(),
        [](const Anchor& lhs, const Anchor& rhs) { return lhs.key < rhs.key; });

    mAnchors.clear();
    mAnchors.reserve(keyCount);
    for (std::size_t i = 0; i < keyCount; ++i) {
      const AvbdDeformableTangentAnchorKey key = keyAt(i);
      const auto match = std::lower_bound(
          mPreviousAnchors.begin(),
          mPreviousAnchors.end(),
          key,
          [](const Anchor& anchor,
             const AvbdDeformableTangentAnchorKey& value) {
            return anchor.key < value;
          });
      if (match != mPreviousAnchors.end() && match->key == key) {
        mAnchors.push_back(*match);
      } else {
        Anchor anchor;
        anchor.key = key;
        mAnchors.push_back(anchor);
      }
    }
  }

  void reserve(std::size_t capacity)
  {
    mAnchors.reserve(capacity);
    mPreviousAnchors.reserve(capacity);
  }

  void clear() noexcept
  {
    mAnchors.clear();
    mPreviousAnchors.clear();
  }

  [[nodiscard]] std::size_t size() const noexcept
  {
    return mAnchors.size();
  }

  [[nodiscard]] Anchor& operator[](std::size_t index) noexcept
  {
    return mAnchors[index];
  }

  [[nodiscard]] const Anchor& operator[](std::size_t index) const noexcept
  {
    return mAnchors[index];
  }

  [[nodiscard]] AnchorVector& records() noexcept
  {
    return mAnchors;
  }

  [[nodiscard]] const AnchorVector& records() const noexcept
  {
    return mAnchors;
  }

private:
  AnchorVector mAnchors;
  AnchorVector mPreviousAnchors;
};

using AvbdHalfSpaceTangentAnchorInventory
    = AvbdDeformableTangentAnchorInventory<AvbdHalfSpaceTangentAnchorState>;
using AvbdSelfContactTangentAnchorInventory
    = AvbdDeformableTangentAnchorInventory<AvbdSelfContactTangentAnchorState>;

} // namespace dart::simulation::detail::deformable_vbd
