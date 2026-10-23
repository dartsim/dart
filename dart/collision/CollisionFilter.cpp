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

#include "dart/collision/CollisionFilter.hpp"

#include "dart/collision/CollisionObject.hpp"
#include "dart/common/Macros.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/Skeleton.hpp"

#include <stdexcept>

namespace dart {
namespace collision {

namespace {

thread_local bool gSolverRestingContactFilterActive = false;
thread_local bool gPreserveRestingIslandContacts = false;

// Composite filters querying their children on this thread, innermost first,
// with the child each one is visiting.
struct CompositeIteration
{
  const CompositeCollisionFilter* mFilter;
  const CollisionFilter* mChild;
  const CompositeIteration* mOuter;
  const CompositeIteration** mSlot;
};
thread_local const CompositeIteration* gCompositeIterations = nullptr;

// Whether `filter` is querying `child` on this thread, or any child when
// `child` is null.
bool isQuerying(
    const CompositeCollisionFilter* filter, const CollisionFilter* child)
{
  for (auto* entry = gCompositeIterations; entry; entry = entry->mOuter) {
    if (entry->mFilter == filter && (!child || entry->mChild == child))
      return true;
  }
  return false;
}

// Edits that invalidate a running query's iterator were undefined in DART 6,
// so they are rejected before the set changes. Erasing another child or
// inserting without a rehash keeps that iterator valid and stays allowed.
[[noreturn]] void throwQueryInvalidated()
{
  throw std::runtime_error(
      "CompositeCollisionFilter: edit would invalidate its running query");
}

} // namespace

//==============================================================================
CollisionFilter::~CollisionFilter()
{
  // Do nothing
}

//==============================================================================
bool CollisionFilter::needCollision(
    const CollisionObject* object1, const CollisionObject* object2) const
{
  return !ignoresCollision(object1, object2);
}

//==============================================================================
void CompositeCollisionFilter::addCollisionFilter(const CollisionFilter* filter)
{
  // nullptr is not an allowed filter
  if (!filter)
    return;

  // Only an insert that rehashes invalidates a running query's iterator.
  if (mFilters.size() + 1u
          > mFilters.max_load_factor() * mFilters.bucket_count()
      && isQuerying(this, nullptr) && mFilters.count(filter) == 0u)
    throwQueryInvalidated();
  mFilters.insert(filter);
}

//==============================================================================
void CompositeCollisionFilter::removeCollisionFilter(
    const CollisionFilter* filter)
{
  if (filter && isQuerying(this, filter))
    throwQueryInvalidated();
  mFilters.erase(filter);
}

//==============================================================================
void CompositeCollisionFilter::removeAllCollisionFilters()
{
  if (isQuerying(this, nullptr))
    throwQueryInvalidated();
  mFilters.clear();
}

//==============================================================================
bool CompositeCollisionFilter::ignoresCollision(
    const CollisionObject* object1, const CollisionObject* object2) const
{
  auto& head = gCompositeIterations;
  CompositeIteration iteration{this, nullptr, head, &head};
  head = &iteration;
  // Restores the outer entry when a child returns or throws.
  struct Pop
  {
    const CompositeIteration& mIteration;
    ~Pop()
    {
      *mIteration.mSlot = mIteration.mOuter;
    }
  } pop{iteration};

  for (const auto* filter : mFilters) {
    iteration.mChild = filter;
    if (filter->ignoresCollision(object1, object2))
      return true;
  }

  return false;
}

//==============================================================================
void BodyNodeCollisionFilter::addBodyNodePairToBlackList(
    const dynamics::BodyNode* bodyNode1, const dynamics::BodyNode* bodyNode2)
{
  mBodyNodeBlackList.addPair(bodyNode1, bodyNode2);
  ++mRevision;
}

//==============================================================================
void BodyNodeCollisionFilter::removeBodyNodePairFromBlackList(
    const dynamics::BodyNode* bodyNode1, const dynamics::BodyNode* bodyNode2)
{
  mBodyNodeBlackList.removePair(bodyNode1, bodyNode2);
  ++mRevision;
}

//==============================================================================
void BodyNodeCollisionFilter::removeAllBodyNodePairsFromBlackList()
{
  mBodyNodeBlackList.removeAllPairs();
  ++mRevision;
}

//==============================================================================
void BodyNodeCollisionFilter::setSolverRestingContactFilterActive(
    bool active, bool preserve)
{
  gSolverRestingContactFilterActive = active;
  gPreserveRestingIslandContacts = active && preserve;
}

//==============================================================================
std::size_t BodyNodeCollisionFilter::getRevision() const
{
  std::size_t seed = mRevision;
  auto mix = [](std::size_t& value, std::size_t input) {
    value ^= input + 0x9e3779b97f4a7c15ULL + (value << 6) + (value >> 2);
  };

  mix(seed, dynamics::Skeleton::getGlobalStructuralVersion());
  mix(seed, dynamics::Skeleton::getGlobalDeactivationStateVersion());
  mix(seed, gSolverRestingContactFilterActive ? 1u : 0u);
  mix(seed, gPreserveRestingIslandContacts ? 1u : 0u);
  return seed;
}

//==============================================================================
bool BodyNodeCollisionFilter::isSolverRestingContactFilterActive() const
{
  return gSolverRestingContactFilterActive;
}

//==============================================================================
bool BodyNodeCollisionFilter::hasBodyNodePairBlackList() const
{
  return !mBodyNodeBlackList.empty();
}

//==============================================================================
std::size_t BodyNodeCollisionFilter::getBodyNodePairBlackListRevision() const
{
  return mRevision;
}

//==============================================================================
bool BodyNodeCollisionFilter::ignoresCollision(
    const collision::CollisionObject* object1,
    const collision::CollisionObject* object2) const
{
  if (object1 == object2)
    return true;

  // We don't filter out for non-ShapeNode because this class shouldn't have the
  // authority to make decisions about filtering any ShapeFrames that aren't
  // attached to a BodyNode. So here we just return false. In order to decide
  // whether the non-ShapeNode should be ignored, please use other collision
  // filters.
  const auto* bodyNode1 = object1->getBodyNode();
  const auto* bodyNode2 = object2->getBodyNode();
  if (!bodyNode1 || !bodyNode2)
    return false;

  if (bodyNode1 == bodyNode2)
    return true;

  if (!bodyNode1->isCollidable() || !bodyNode2->isCollidable())
    return true;

  const auto* skel1 = bodyNode1->getSkeletonRawPtr();
  const auto* skel2 = bodyNode2->getSkeletonRawPtr();
  if (!skel1 || !skel2)
    return false;

  if (!skel1->isMobile() && !skel2->isMobile())
    return true;

  if (skel1 == skel2) {
    if (!skel1->isEnabledSelfCollisionCheck())
      return true;

    if (!skel1->isEnabledAdjacentBodyCheck()) {
      if (areAdjacentBodies(bodyNode1, bodyNode2))
        return true;
    }
  }

  if (!mBodyNodeBlackList.empty()
      && mBodyNodeBlackList.contains(bodyNode1, bodyNode2)) {
    return true;
  }

  const bool skel1Inactive = !skel1->isMobile() || skel1->isResting();
  const bool skel2Inactive = !skel2->isMobile() || skel2->isResting();
  if (gSolverRestingContactFilterActive && skel1Inactive && skel2Inactive) {
    const bool sameFrozenMobileIsland
        = gPreserveRestingIslandContacts && skel1->isMobile()
          && skel2->isMobile() && skel1->isResting() && skel2->isResting()
          && skel1->getIslandIndex() >= 0
          && skel1->getIslandIndex() == skel2->getIslandIndex();
    const bool restingMobileOnStaticSupport
        = gPreserveRestingIslandContacts
          && ((skel1->isMobile() && skel1->isResting()
               && skel1->getIslandIndex() >= 0 && !skel2->isMobile())
              || (skel2->isMobile() && skel2->isResting()
                  && skel2->getIslandIndex() >= 0 && !skel1->isMobile()));
    if (sameFrozenMobileIsland || restingMobileOnStaticSupport)
      return false;

    return true;
  }

  return false;
}

//==============================================================================
bool BodyNodeCollisionFilter::areAdjacentBodies(
    const dynamics::BodyNode* bodyNode1,
    const dynamics::BodyNode* bodyNode2) const
{
  if ((bodyNode1->getParentBodyNode() == bodyNode2)
      || (bodyNode2->getParentBodyNode() == bodyNode1)) {
    DART_ASSERT(bodyNode1->getSkeleton() == bodyNode2->getSkeleton());
    return true;
  }

  return false;
}

} // namespace collision
} // namespace dart
