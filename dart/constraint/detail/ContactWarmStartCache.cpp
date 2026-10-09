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

#include <dart/constraint/detail/ContactWarmStartCache.hpp>

#include <algorithm>
#include <functional>
#include <memory>
#include <mutex>
#include <unordered_map>

#include <cassert>
#include <cmath>

namespace dart::constraint::detail {
namespace {

std::size_t warmStartPointerHash(const void* pointer)
{
  auto value = std::hash<const void*>{}(pointer);
  value ^= value >> 16;
  value *= std::size_t(0x9e3779b9);
  return value ^ (value >> 16);
}

std::size_t warmStartPairHash(const ContactWarmStartCache::Key& key)
{
  return warmStartPointerHash(key.frames[0])
         ^ warmStartPointerHash(key.frames[1]);
}

bool warmStartSamePair(
    const ContactWarmStartCache::Key& a, const ContactWarmStartCache::Key& b)
{
  return a.frames == b.frames
         || (a.frames[0] == b.frames[1] && a.frames[1] == b.frames[0]);
}

std::mutex& warmStartRegistryMutex()
{
  static auto* mutex = new std::mutex;
  return *mutex;
}

auto& warmStartRegistry()
{
  static auto* registry = new std::unordered_map<
      const ConstraintSolver*,
      std::unique_ptr<ContactWarmStartCache>>;
  return *registry;
}

} // namespace

//==============================================================================
void ContactWarmStartCache::reserve(std::size_t count)
{
  count = std::max(count, mPrevious.size());
  mPrevious.reserve(count);
  mCurrent.reserve(count);
  mConsumed.reserve(count);
  std::size_t buckets = 1;
  while (buckets < 2 * count)
    buckets *= 2;
  if (mPairBuckets.size() < buckets) {
    mPairBuckets.resize(buckets, npos);
    mContactBuckets.resize(buckets, npos);
  }
}

//==============================================================================
void ContactWarmStartCache::begin(
    double timeStep,
    std::size_t count,
    const collision::CollisionGroup* group,
    std::size_t contentVersion)
{
  if (timeStep != mTimeStep || !std::isfinite(timeStep) || timeStep <= 0.0
      || group != mGroup || contentVersion != mContentVersion)
    mPrevious.clear();
  mTimeStep = timeStep;
  mGroup = group;
  mContentVersion = contentVersion;
  reserve(count);
  mCurrent.clear();
  mConsumed.assign(mPrevious.size(), 0);
  std::fill(mPairBuckets.begin(), mPairBuckets.end(), npos);
  std::fill(mContactBuckets.begin(), mContactBuckets.end(), npos);
  for (std::size_t i = 0; i < mPrevious.size(); ++i) {
    auto& entry = mPrevious[i];
    std::size_t slot = warmStartPairHash(entry.key) & (mPairBuckets.size() - 1);
    while (mPairBuckets[slot] != npos
           && !warmStartSamePair(entry.key, mPrevious[mPairBuckets[slot]].key))
      slot = (slot + 1) & (mPairBuckets.size() - 1);
    entry.next = mPairBuckets[slot];
    mPairBuckets[slot] = i;
  }
}

//==============================================================================
std::size_t ContactWarmStartCache::add(const void* contact, const Key& key)
{
  if (!contact || !key.frames[0] || !key.frames[1]
      || key.frames[0] == key.frames[1] || !std::isfinite(mTimeStep)
      || mTimeStep <= 0.0 || contactIndex(contact) != npos)
    return npos;
  assert(mCurrent.size() < mCurrent.capacity());
  assert(mCurrent.size() < mContactBuckets.size() / 2);
  if (mCurrent.size() >= mCurrent.capacity()
      || mCurrent.size() >= mContactBuckets.size() / 2)
    return npos;
  Entry entry;
  entry.key = key;
  entry.contact = contact;
  for (int side = 0; side < 2; ++side) {
    const double norm = key.normals[side].norm();
    if (!key.points[side].allFinite() || !key.normals[side].allFinite()
        || !std::isfinite(norm) || norm == 0.0)
      return npos;
    entry.key.normals[side] /= norm;
  }
  std::size_t slot = warmStartPairHash(key) & (mPairBuckets.size() - 1);
  while (mPairBuckets[slot] != npos
         && !warmStartSamePair(key, mPrevious[mPairBuckets[slot]].key))
    slot = (slot + 1) & (mPairBuckets.size() - 1);
  std::size_t match = npos;
  double nearest = std::numeric_limits<double>::infinity();
  bool reversedMatch = false;
  for (std::size_t i = mPairBuckets[slot]; i != npos; i = mPrevious[i].next) {
    if (mConsumed[i])
      continue;
    const auto& previous = mPrevious[i];
    const bool reversed = key.frames != previous.key.frames;
    double distance = 0.0;
    bool compatible = true;
    for (int side = 0; side < 2; ++side) {
      const int oldSide = reversed ? 1 - side : side;
      const double squaredDistance
          = (key.points[side] - previous.key.points[oldSide]).squaredNorm();
      const double normalDot = entry.key.normals[side].dot(
          previous.key.normals[oldSide] * (reversed ? -1.0 : 1.0));
      compatible &= squaredDistance <= 1e-6 && normalDot >= 0.999;
      distance += squaredDistance;
    }
    if (compatible
        && (distance < nearest || (distance == nearest && i < match))) {
      nearest = distance;
      match = i;
      reversedMatch = reversed;
    }
  }
  if (match != npos) {
    mConsumed[match] = 1;
    entry.seed.matched = true;
    entry.seed.localImpulse = reversedMatch ? -mPrevious[match].localImpulses[1]
                                            : mPrevious[match].localImpulses[0];
  }
  const std::size_t index = mCurrent.size();
  mCurrent.push_back(entry);
  slot = warmStartPointerHash(contact) & (mContactBuckets.size() - 1);
  while (mContactBuckets[slot] != npos)
    slot = (slot + 1) & (mContactBuckets.size() - 1);
  mContactBuckets[slot] = index;
  return index;
}

//==============================================================================
std::size_t ContactWarmStartCache::contactIndex(const void* contact) const
{
  if (!contact || mContactBuckets.empty())
    return npos;
  std::size_t slot
      = warmStartPointerHash(contact) & (mContactBuckets.size() - 1);
  while (mContactBuckets[slot] != npos) {
    const auto index = mContactBuckets[slot];
    if (mCurrent[index].contact == contact)
      return index;
    slot = (slot + 1) & (mContactBuckets.size() - 1);
  }
  return npos;
}

//==============================================================================
const ContactWarmStartCache::Seed* ContactWarmStartCache::seed(
    const void* contact) const
{
  const auto index = contactIndex(contact);
  return index == npos ? nullptr : &mCurrent[index].seed;
}

//==============================================================================
void ContactWarmStartCache::update(
    const void* contact, const std::array<Eigen::Vector3d, 2>& localImpulses)
{
  const auto index = contactIndex(contact);
  if (index == npos)
    return;
  auto& entry = mCurrent[index];
  entry.localImpulses = localImpulses;
  entry.solved = localImpulses[0].allFinite() && localImpulses[1].allFinite();
}

//==============================================================================
void ContactWarmStartCache::finish()
{
  mCurrent.erase(
      std::remove_if(
          mCurrent.begin(),
          mCurrent.end(),
          [](const Entry& entry) { return !entry.solved; }),
      mCurrent.end());
  mPrevious.swap(mCurrent);
  mCurrent.clear();
  std::fill(mContactBuckets.begin(), mContactBuckets.end(), npos);
}

//==============================================================================
void ContactWarmStartCache::clear()
{
  mPrevious.clear();
  mCurrent.clear();
  mConsumed.clear();
  std::fill(mPairBuckets.begin(), mPairBuckets.end(), npos);
  std::fill(mContactBuckets.begin(), mContactBuckets.end(), npos);
  mTimeStep = 0.0;
  mGroup = nullptr;
  mContentVersion = 0;
}

//==============================================================================
std::size_t ContactWarmStartCache::size() const
{
  return mPrevious.size();
}

//==============================================================================
ContactWarmStartCache* findContactWarmStartCache(const ConstraintSolver* owner)
{
  std::lock_guard<std::mutex> lock(warmStartRegistryMutex());
  auto& registry = warmStartRegistry();
  const auto found = registry.find(owner);
  return found == registry.end() ? nullptr : found->second.get();
}

//==============================================================================
ContactWarmStartCache& getOrCreateContactWarmStartCache(
    const ConstraintSolver* owner)
{
  std::lock_guard<std::mutex> lock(warmStartRegistryMutex());
  auto& cache = warmStartRegistry()[owner];
  if (!cache)
    cache = std::make_unique<ContactWarmStartCache>();
  return *cache;
}

//==============================================================================
void eraseContactWarmStartCache(const ConstraintSolver* owner)
{
  std::lock_guard<std::mutex> lock(warmStartRegistryMutex());
  warmStartRegistry().erase(owner);
}

} // namespace dart::constraint::detail
