/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <mxgrey@gatech.edu>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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

#include "dart/dynamics/ReferentialSkeleton.h"

#ifndef DART_DYNAMICS_GROUP_H_
#define DART_DYNAMICS_GROUP_H_

namespace dart {
namespace dynamics {

class Group : public ReferentialSkeleton
{
public:

  /// Create a Group out of a set of BodyNodes
  static GroupPtr create(
      const std::string& _name = "Group",
      const std::vector<BodyNode*>& _bodyNodes = std::vector<BodyNode*>());

  /// Create a Group out of a set of DegreesOfFreedom
  static GroupPtr create(
      const std::string& _name,
      const std::vector<DegreeOfFreedom*>& _dofs);

  /// Destructor
  virtual ~Group() = default;

  /// Swap the index of BodyNode _index1 with _index2
  void swapBodyNodeIndices(size_t _index1, size_t _index2);

  /// Swap the index of DegreeOfFreedom _index1 with _index2
  void swapDofIndices(size_t _index1, size_t _index2);

  /// Add a BodyNode to this Group. If _warning is true, you will be warned when
  /// you attempt to add the same BodyNode twice, and assertion will be thrown.
  void addBodyNode(BodyNode* _bn, bool _warning=true);

  /// Add a set of BodyNodes to this Group. If _warning is true, you will be
  /// warned when you attempt to add the same BodyNode twice, and an assertion
  /// will be thrown.
  void addBodyNodes(const std::vector<BodyNode*>& _bodyNodes,
                    bool _warning=true);

  /// Remove a BodyNode from this Group. Note: All DegreesOfFreedom belonging to
  /// this BodyNode will also be removed. If _warning is true, you will be
  /// warned when you attempt to remove BodyNode that is not in this Group, and
  /// an assertion will be thrown.
  ///
  /// The function will return false if the BodyNode was not already in this
  /// Group.
  bool removeBodyNode(BodyNode* _bn, bool _warning=true);

  /// Remove a set of BodyNodes from this Group. Note: All DegreesOfFreedom
  /// belonging to each BodyNode will also be removed. If _warning is true, you
  /// will be warned when you attempt to remove a BodyNode that is not in this
  /// Group, and an assertion will be thrown.
  ///
  /// The function will return false if one of the BodyNodes was not already in
  /// this Group.
  bool removeBodyNodes(const std::vector<BodyNode*>& _bodyNodes,
                       bool _warning=true);

  /// Add a DegreeOfFreedom to this Group. Note: The BodyNode of this
  /// DegreeOfFreedom will also be added. If _warning is true, you will be
  /// warned when you attempt to add the same DegreeOfFreedom twice, and an
  /// assertion will be thrown.
  void addDof(DegreeOfFreedom* _dof, bool _warning=true);

  /// Add a set of DegreesOfFreedom to this Group. Note: The BodyNodes of these
  /// DegreesOfFreedom will also be added. If _warning is true, you will be
  /// warned when you attempt to add the same DegreeOfFreedom twice, and an
  /// assertion will be thrown.
  void addDofs(const std::vector<DegreeOfFreedom*>& _dofs, bool _warning=true);

  /// Remove a DegreeOfFreedom from this Group. If _warning is true, you will be
  /// warned when you attempt to remove a DegreeOfFreedom that is not in this
  /// Group, and an assertion will be thrown.
  ///
  /// This function will return false if the DegreeOfFreedom was not already in
  /// this Group.
  bool removeDof(DegreeOfFreedom* _dof, bool _warning=true);

  /// Remove a set of DegreesOfFreedom from this Group. If _warning is true, you
  /// will be warned when you attempt to remove a DegreeOfFreedom that is not
  /// in this Group, and an assertion will be thrown.
  ///
  /// This function will return false if the DegreeOfFreedom was not alraedy in
  /// this Group.
  bool removeDofs(const std::vector<DegreeOfFreedom*>& _dofs,
                  bool _warning=true);

protected:
  /// Default constructor
  Group(const std::string& _name = "Group",
        const std::vector<BodyNode*>& _bodyNodes = std::vector<BodyNode*>());

  /// Alternative constructor
  Group(const std::string& _name,
        const std::vector<DegreeOfFreedom*>& _dofs);
};

} // dynamics
} // dart

#endif // DART_DYNAMICS_GROUP_H_
