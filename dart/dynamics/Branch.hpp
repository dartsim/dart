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

#ifndef DART_DYNAMICS_BRANCH_HPP_
#define DART_DYNAMICS_BRANCH_HPP_

#include "dart/dynamics/Linkage.hpp"

namespace dart {
namespace dynamics {

/// Branch is a specialized type of Linkage that represents a complete subtree
/// of a Skeleton. The Branch will start at a specific BodyNode and will include
/// every BodyNode that descends from it, all the way to the leaves.
class Branch : public Linkage
{
public:

  struct Criteria
  {
    /// Constructor. Requires a starting BodyNode.
    Criteria(BodyNode* _start);

    /// Return a vector of BodyNodes that form a full subtree, starting from
    /// mStart
    std::vector<BodyNode*> satisfy() const;

    /// The BodyNode where the Branch starts
    WeakBodyNodePtr mStart;

    /// Convert this Criteria into Linkage criteria
    Linkage::Criteria convert() const;

    /// Operator for implicit conversion to a Linkage::Criteria
    operator Linkage::Criteria() const;

    /// Converts Linkage::Criteria to Branch::Criteria
    static Criteria convert(const Linkage::Criteria& criteria);
  };

  /// Create a Branch
  static BranchPtr create(const Branch::Criteria& _criteria,
                          const std::string& _name = "Branch");

  /// Creates and returns a clone of this Branch.
  BranchPtr cloneBranch() const;

  /// Creates and returns a clone of this Branch.
  BranchPtr cloneBranch(const std::string& cloneName) const;

  // To expose MetaSkeleton::cloneMetaSkeleton(), which takes no cloneName.
  using MetaSkeleton::cloneMetaSkeleton;

  // Documentation inherited
  MetaSkeletonPtr cloneMetaSkeleton(const std::string& cloneName) const override;

  /// Returns false if a new BodyNode has been attached to any BodyNode of this
  /// Branch, or if a BodyNode of this Branch has been detached.
  bool isStillBranch() const;

protected:

  /// Constructor for the Branch class. Note that you can simply pass a BodyNode
  /// pointer into this constructor, and it will be implicitly converted into a
  /// Branch::Criteria.
  Branch(const Branch::Criteria& _criteria,
         const std::string& _name = "Branch");

  // Documentation inherited
  void update() override;

  /// The original number of child nodes for each BodyNode of this Branch
  std::vector<std::size_t> mNumChildNodes;
};

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_BRANCH_HPP_
