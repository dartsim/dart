/*
 * Copyright (c) 2011-2019, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#ifndef DART_COLLISION_DETAIL_UNORDEREDPAIRS_HPP_
#define DART_COLLISION_DETAIL_UNORDEREDPAIRS_HPP_

#include <unordered_map>
#include <unordered_set>

namespace dart {
namespace collision {
namespace detail {

template <class T>
class UnorderedPairs
{
public:
  /// Adds a pair to this container.
  void addPair(const T* left, const T* right);

  /// Removes a pair from this container.
  void removePair(const T* left, const T* right);

  /// Removes all the pairs from this container.
  void removeAllPairs();

  /// Returns true if this container contains the pair.
  bool contains(const T* left, const T* right) const;

private:
  /// The actual container to store pairs.
  ///
  /// Each pair is stored so that the key of the std::unordered_map always has
  /// a value less than every element in the std::unordered_set that is
  /// associated with it.
  std::unordered_map<const T*, std::unordered_set<const T*>> mList;
};

//==============================================================================
template <class T>
void UnorderedPairs<T>::addPair(const T* left, const T* right)
{
  if (!left || !right)
    return;

  const auto* less = left;
  const auto* greater = right;

  if (less > greater)
    std::swap(less, greater);

  // Call insert in case an entry for bodyNodeLess doesn't exist. If it doesn't
  // exist, it will be initialized with an empty set. If it does already exist,
  // we will just get an iterator to the existing entry.
  const auto itLess = mList.insert(
      std::make_pair(less, std::unordered_set<const T*>())).first;

  // Insert bodyNodeGreater into the set corresponding to bodyNodeLess. If the
  // pair already existed, this will do nothing.
  itLess->second.insert(greater);
}

//==============================================================================
template<class T>
void UnorderedPairs<T>::removePair(const T* left, const T* right)
{
  if (!left || !right)
    return;

  const auto* bodyNodeLess = left;
  const auto* bodyNodeGreater = right;

  if (bodyNodeLess > bodyNodeGreater)
    std::swap(bodyNodeLess, bodyNodeGreater);

  // Remove the pair only when it already exists
  const auto resultLeft = mList.find(bodyNodeLess);
  const bool foundLeft = (resultLeft != mList.end());
  if (foundLeft)
  {
    auto& associatedRights = resultLeft->second;
    associatedRights.erase(bodyNodeGreater);

    if (associatedRights.empty())
      mList.erase(resultLeft);
  }
}

//==============================================================================
template<class T>
void UnorderedPairs<T>::removeAllPairs()
{
  mList.clear();
}

//==============================================================================
template<class T>
bool UnorderedPairs<T>::contains(const T* left, const T* right) const
{
  const auto* less = left;
  const auto* greater = right;

  if (less > greater)
    std::swap(less, greater);

  const auto resultLeft = mList.find(less);
  const bool foundLeft = (resultLeft != mList.end());
  if (foundLeft)
  {
    auto& associatedRights = resultLeft->second;

    const auto resultRight = associatedRights.find(greater);
    const bool foundRight = (resultRight != associatedRights.end());
    if (foundRight)
      return true;
  }

  return false;
}

} // namespace detail
} // namespace collision
} // namespace dart

#endif // DART_COLLISION_DETAIL_UNORDEREDPAIRS_HPP_
