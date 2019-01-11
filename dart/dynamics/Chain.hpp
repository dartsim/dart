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

#ifndef DART_DYNAMICS_CHAIN_HPP_
#define DART_DYNAMICS_CHAIN_HPP_

#include "dart/dynamics/Linkage.hpp"

namespace dart {
namespace dynamics {

/// Chain is a specialized type of Linkage that represents a single unbranching
/// and fully connected sequence of BodyNodes. A chain will start from a
/// specified BodyNode and include every BodyNode on the way to a target
/// BodyNode, except it will stop if it encounters a branching (BodyNode with
/// multiple child BodyNodes) or a FreeJoint.
class Chain : public Linkage
{
public:

  struct Criteria
  {
    /// Constructor for Chain::Criteria
    Criteria(BodyNode* _start, BodyNode* _target, bool _includeBoth = false);

    /// Return a vector of BodyNodes that form a chain
    std::vector<BodyNode*> satisfy() const;

    /// mStart will be the first BodyNode in the chain
    WeakBodyNodePtr mStart;

    /// mTarget will be the final BodyNode in the chain, unless there is a
    /// branching or a FreeJoint along the way
    WeakBodyNodePtr mTarget;

    /// Set this to true if both the start and the target BodyNode should be
    /// included. Otherwise, whichever is upstream of the other will be left out
    /// of the chain.
    bool mIncludeBoth;

    /// Convert this Criteria into Linkage::Criteria
    Linkage::Criteria convert() const;

    /// Operator for implicit conversion to a Linkage::Criteria
    operator Linkage::Criteria() const;

    /// Converts Linkage::Criteria to Chain::Criteria
    static Criteria convert(const Linkage::Criteria& criteria);
  };

  /// This enum is used to specify to the create() function that both the start
  /// and the target BodyNodes should be included in the Chain that gets
  /// generated.
  enum IncludeBothTag { IncludeBoth };

  /// Create a Chain given some Chain::Criteria
  static ChainPtr create(const Chain::Criteria& _criteria,
                         const std::string& _name = "Chain");

  /// Create a Chain given a start and a target BodyNode. Note that whichever
  /// BodyNode is upstream of the other will be excluded from the Chain.
  static ChainPtr create(BodyNode* _start, BodyNode* _target,
                         const std::string& _name = "Chain");

  /// Create a Chain given a start and a target BodyNode. In this version, both
  /// BodyNodes will be included in the Chain that gets created.
  static ChainPtr create(BodyNode* _start, BodyNode* _target,
                         IncludeBothTag, const std::string& _name = "Chain");

  /// Creates and returns a clone of this Chain.
  ChainPtr cloneChain() const;

  /// Creates and returns a clone of this Chain.
  ChainPtr cloneChain(const std::string& cloneName) const;

  // To expose MetaSkeleton::cloneMetaSkeleton(), which takes no cloneName.
  using MetaSkeleton::cloneMetaSkeleton;

  // Documentation inherited
  MetaSkeletonPtr cloneMetaSkeleton(const std::string& cloneName) const override;

  /// Returns false if this Chain has been broken, or some new Branching has
  /// been added.
  bool isStillChain() const;

protected:

  /// Constructor for the Chain class
  Chain(const Chain::Criteria& _criteria, const std::string& _name = "Chain");

  /// Alternative constructor for the Chain class
  Chain(BodyNode* _start, BodyNode* _target,
        const std::string& _name = "Chain");

  /// Alternative constructor for the Chain class
  Chain(BodyNode* _start, BodyNode* _target,
        IncludeBothTag, const std::string& _name = "Chain");

};

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_CHAIN_HPP_
