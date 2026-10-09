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

#ifndef DART_CONSTRAINT_DETAIL_CONTACTWARMSTARTCACHE_HPP_
#define DART_CONSTRAINT_DETAIL_CONTACTWARMSTARTCACHE_HPP_

#include <Eigen/Core>

#include <array>
#include <limits>
#include <vector>

#include <cstddef>

namespace dart::collision {
class CollisionGroup;
}

namespace dart::dynamics {
class ShapeFrame;
}

namespace dart::constraint {
class BoxedLcpSolver;
class ConstraintSolver;
} // namespace dart::constraint

namespace dart::constraint::detail {

struct ContactWarmStartSolveResult
{
  const BoxedLcpSolver* solver = nullptr;
  bool success = false;
  bool converged = false;
  double violation = std::numeric_limits<double>::infinity();
};

/// The last friction-solve result on this thread; counters mix islands.
ContactWarmStartSolveResult& contactWarmStartSolveResult();
/// One-shot request to refine an eligible primary's cached initial guess.
const BoxedLcpSolver*& contactWarmStartRefinementSolver();

/// Detector-independent contact history; no API stability promise. Prepare and
/// merge serially; islands read seeds and update only their own entries.
class ContactWarmStartCache
{
public:
  struct Key
  {
    std::array<const dynamics::ShapeFrame*, 2> frames{{nullptr, nullptr}};
    std::array<Eigen::Vector3d, 2> points{
        {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()}};
    /// The contact normal toward body 1, expressed in each body's coordinates.
    std::array<Eigen::Vector3d, 2> normals{
        {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()}};
  };

  struct Seed
  {
    Eigen::Vector3d localImpulse = Eigen::Vector3d::Zero();
    bool matched = false;
    /// Native normal seeds may retain their detector's point tolerance
    /// when the previous shape pair and both local normals remain compatible.
    bool canRetainNative = false;
  };

  static constexpr std::size_t npos = std::numeric_limits<std::size_t>::max();

  /// Reserve both snapshots and lookup scratch for allocation-free steps.
  void reserve(std::size_t count);
  /// Count bounds the contacts added this step. A changed timestep, collision
  /// group or group content invalidates history.
  void begin(
      double timeStep,
      std::size_t count,
      const collision::CollisionGroup* group = nullptr,
      std::size_t contentVersion = 0);
  /// Match each prior contact at most once, using the nearest pair of points.
  /// Each local point must be within 1 mm and each normal within about 2.56
  /// degrees (dot >= 0.999). Native seed eligibility ignores point distances
  /// and prior match consumption. Invalid geometry returns npos.
  std::size_t add(const void* contact, const Key& key);
  const Seed* seed(const void* contact) const;
  /// Both vectors represent the world impulse toward body 1 in each body's
  /// local coordinates. Islands may update distinct contacts concurrently;
  /// non-finite results are discarded at merge.
  void update(
      const void* contact, const std::array<Eigen::Vector3d, 2>& localImpulses);
  /// Keep current keys for native eligibility, and only finite solved impulses
  /// for additional matching.
  void finish();
  void clear();
  /// Number of reusable impulses, excluding native-only contact metadata.
  std::size_t size() const;

private:
  struct Entry
  {
    Key key;
    const void* contact = nullptr;
    Seed seed;
    std::array<Eigen::Vector3d, 2> localImpulses{
        {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()}};
    std::size_t next = npos;
    bool solved = false;
  };

  std::size_t contactIndex(const void* contact) const;

  std::vector<Entry> mPrevious;
  std::vector<Entry> mCurrent;
  std::vector<unsigned char> mConsumed;
  std::vector<std::size_t> mPairBuckets;
  std::vector<std::size_t> mContactBuckets;
  double mTimeStep = 0.0;
  const collision::CollisionGroup* mGroup = nullptr;
  std::size_t mContentVersion = 0;
};

/// Side storage preserves existing constraint-solver layouts. Solver owners
/// erase their entry at destruction; concurrent solves only use find and seed.
ContactWarmStartCache* findContactWarmStartCache(const ConstraintSolver* owner);
ContactWarmStartCache& getOrCreateContactWarmStartCache(
    const ConstraintSolver* owner);
void eraseContactWarmStartCache(const ConstraintSolver* owner);

} // namespace dart::constraint::detail

#endif // DART_CONSTRAINT_DETAIL_CONTACTWARMSTARTCACHE_HPP_
