/*
 * Copyright (c) 2015-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2015-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#ifndef DART_DYNAMICS_LINKAGE_HPP_
#define DART_DYNAMICS_LINKAGE_HPP_

#include "dart/dynamics/ReferentialSkeleton.hpp"
#include <unordered_set>

namespace dart {
namespace dynamics {

/// A Linkage is a ReferentialSkeleton with the special property that all the
/// BodyNodes included in it form a contiguous graph. This property is only
/// guaranteed during construction of the Linkage. After the Linkage has been
/// constructed, a user might alter how the BodyNodes in a Linkage are
/// assembled. The function Linkage::isAssembled() can be used to check whether
/// the Linkage is still assembled. The function Linkage::satisfyCriteria() can
/// be used to redefine the Linkage so that the original Linkage::Criteria is
/// met again. The function Linkage::reassemble() will reassemble the BodyNodes
/// so that they match whatever assembly they had the last time
/// Linkage::reassemble() was called (or the assembly that they had when the
/// Linkage was constructed, if Linkage::reassemble has never been called).
class Linkage : public ReferentialSkeleton
{
public:

  /// The Criteria class is used to specify how a Linkage should be constructed
  struct Criteria
  {
    /// The ExpansionPolicy indicates how the collection of BodyNodes should
    /// expand from the starting BodyNode (mStart)
    enum ExpansionPolicy {
      INCLUDE = 0,  ///< Do not expand from the target. Include everything up to the target and then stop.
      EXCLUDE,      ///< Do not expand from the target. Include everything up to the target, but NOT the target, and then stop.
      DOWNSTREAM,   ///< Include the target, and then expand downstream, toward the leaves of the tree.
      UPSTREAM      ///< Include the target, and then expand upstream, toward the root of the tree.
    };

    /// Return a vector of BodyNodes that satisfy the parameters of the Criteria
    std::vector<BodyNode*> satisfy() const;

    /// This structure defines targets for the expansion criteria and the
    /// desired behavior for those targets
    struct Target
    {
      /// Default constructor for Target
      Target(BodyNode* _target = nullptr,
             ExpansionPolicy _policy = INCLUDE,
             bool _chain = false);

      /// The Linkage will expand from the starting BodyNode up to this node
      WeakBodyNodePtr mNode;

      /// After the target has been reached (if it is reached), the Linkage will
      /// start to follow this expansion policy.
      ExpansionPolicy mPolicy;

      /// If this is set to true, the expansion towards this target will
      /// terminate if (1) a fork/split in the kinematics is reached or (2) a
      /// FreeJoint is reached.
      bool mChain;
    };

    /// This Target will serve as the starting point for the criteria
    /// satisfaction
    Target mStart;

    /// The Linkage will extend from mStart to each of these targets. Each
    /// BodyNode along the way will be included in the Linkage, including the
    /// mTarget. However, if a terminal BodyNode is reached along the way, then
    /// nothing past the terminal BodyNode will be included. Therefore, If you
    /// want to expand towards a target but not include the target, you can set
    /// the BodyNode as both an mTarget and an mTerminal, and set the mInclusive
    /// flag in mTerminal to false.
    std::vector<Target> mTargets;

    /// Any expansion performed by the criteria will be halted if mTerminal is
    /// reached. If mInclusive is set to true, then mTerminal will be included
    /// in the Linkage. If mInclusive is set to false, then mTerminal will not
    /// be included in the Linkage. Note that the BodyNode of mStart may be
    /// included as an inclusive terminal, but NOT as an exclusive terminal.
    struct Terminal
    {
      /// Default constructor for Terminal
      Terminal(BodyNode* _terminal = nullptr, bool _inclusive = true);

      /// BodyNode that should halt any expansion
      WeakBodyNodePtr mTerminal;

      /// Whether or not the BodyNode should be included after expansion has
      /// halted
      bool mInclusive;
    };

    /// Any expansion (whether from an ExpansionPolicy or an attempt to reach an
    /// entry in mTargets) will be halted if it reaches any entry in mTerminal
    std::vector<Terminal> mTerminals;

  protected:

    /// Refresh the content of mMapOfTerminals
    void refreshTerminalMap() const;

    /// Satisfy the expansion policy of a target
    void expansionPolicy(BodyNode* _start, ExpansionPolicy _policy,
                         std::vector<BodyNode*>& _bns) const;

    /// Expand downstream
    void expandDownstream(BodyNode* _start, std::vector<BodyNode*>& _bns,
                          bool _includeStart) const;

    /// Expand upstream
    void expandUpstream(BodyNode* _start, std::vector<BodyNode*>& _bns,
                        bool _includeStart) const;

    /// Construct a path from start to target
    void expandToTarget(const Target& _start, const Target& _target,
                        std::vector<BodyNode*>& _bns) const;

    /// Expand upwards from the _start BodyNode to the _target BodyNode
    std::vector<BodyNode*> climbToTarget(BodyNode* _start, BodyNode* _target) const;

    /// Expand upwards from both BodyNodes to a common root
    std::vector<BodyNode*> climbToCommonRoot(
        const Target& _start, const Target& _target, bool _chain) const;

    /// Crawl through the list and cut it off anywhere that the criteria is
    /// violated
    void trimBodyNodes(std::vector<BodyNode*>& _bns, bool _chain,
                       bool _movingUpstream) const;

    /// Hashed set for terminals to allow quick lookup
    mutable std::unordered_map<BodyNode*, bool> mMapOfTerminals;
  };

  /// Create a Linkage with the given Criteria
  static LinkagePtr create(const Criteria& _criteria,
                           const std::string& _name = "Linkage");

  /// Returns false if the original assembly of this Linkage has been broken in
  /// some way
  bool isAssembled() const;

  /// Revert the assembly of this Linkage to its original structure
  void reassemble();

  /// Redefine this Linkage so that its Criteria is satisfied
  void satisfyCriteria();

protected:

  /// Constructor for the Linkage class. satisfyCriteria() will be called during
  /// construction.
  Linkage(const Criteria& _criteria, const std::string& _name = "Linkage");

  /// Update any metadata needed by the Linkage or its derived classes
  virtual void update();

  /// Criteria that defines the structure of this Linkage
  Criteria mCriteria;

  /// Recording of the parent BodyNodes that were held by each of the BodyNodes
  /// when the Linkage was constructed
  std::vector<WeakBodyNodePtr> mParentBodyNodes;
};

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_LINKAGE_HPP_
