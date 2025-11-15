/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#ifndef DART_UTILS_FILECONTENTS_HPP_
#define DART_UTILS_FILECONTENTS_HPP_

#include <dart/simulation/SmartPointer.hpp>

#include <dart/dynamics/SmartPointer.hpp>

#include <vector>

#include <cstddef>

namespace dart {
namespace utils {

/// Aggregates every object a parsed file may produce.
struct FileContents
{
  /// Worlds declared in the file.
  std::vector<simulation::WorldPtr> worlds;

  /// Skeletons declared in the file. Skeletons that belong to a world are
  /// shared with that world.
  std::vector<dynamics::SkeletonPtr> skeletons;

  /// ReferentialSkeleton instances declared in the file.
  std::vector<dynamics::ReferentialSkeletonPtr> referentialSkeletons;

  /// Linkage instances declared in the file.
  std::vector<dynamics::LinkagePtr> linkages;

  /// Branch instances declared in the file.
  std::vector<dynamics::BranchPtr> branches;

  /// Chain instances declared in the file.
  std::vector<dynamics::ChainPtr> chains;

  /// Returns true when the file did not yield any supported type.
  bool empty() const;

  /// Returns the total number of objects stored in the struct.
  std::size_t count() const;

  /// Appends \p other to this instance without altering ownership.
  void merge(const FileContents& other);
};

//==============================================================================
inline bool FileContents::empty() const
{
  return worlds.empty() && skeletons.empty() && referentialSkeletons.empty()
         && linkages.empty() && branches.empty() && chains.empty();
}

//==============================================================================
inline std::size_t FileContents::count() const
{
  return worlds.size() + skeletons.size() + referentialSkeletons.size()
         + linkages.size() + branches.size() + chains.size();
}

//==============================================================================
inline void FileContents::merge(const FileContents& other)
{
  worlds.insert(worlds.end(), other.worlds.begin(), other.worlds.end());
  skeletons.insert(
      skeletons.end(), other.skeletons.begin(), other.skeletons.end());
  referentialSkeletons.insert(
      referentialSkeletons.end(),
      other.referentialSkeletons.begin(),
      other.referentialSkeletons.end());
  linkages.insert(linkages.end(), other.linkages.begin(), other.linkages.end());
  branches.insert(branches.end(), other.branches.begin(), other.branches.end());
  chains.insert(chains.end(), other.chains.begin(), other.chains.end());
}

} // namespace utils
} // namespace dart

#endif // DART_UTILS_FILECONTENTS_HPP_
