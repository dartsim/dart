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

#include <dart/dynamics/referential_skeleton.hpp>

#include <dart/export.hpp>

#include <span>

#ifndef DART_DYNAMICS_GROUP_HPP_
  #define DART_DYNAMICS_GROUP_HPP_

namespace dart {
namespace dynamics {

class DART_API Group : public ReferentialSkeleton
{
public:
  /// Create a Group out of a set of BodyNodes. If includeJoints is true, the
  /// parent Joint of each BodyNode will also be added to the Group. If
  /// includeDofs is true, then the parent DegreesOfFreedom of each BodyNode
  /// will also be added to the Group.
  [[nodiscard]] static GroupPtr create(
      const std::string& name = "Group",
      std::span<BodyNode* const> bodyNodes = {},
      bool includeJoints = true,
      bool includeDofs = true);

  /// Create a Group out of a set of DegreesOfFreedom. If includeBodyNodes is
  /// true, then the child BodyNode of each DegreeOfFreedom will also be added
  /// to the Group. If includeJoints is true, then the Joint of each
  /// DegreeOfFreedom will also be added to the Group.
  [[nodiscard]] static GroupPtr create(
      const std::string& name,
      std::span<DegreeOfFreedom* const> dofs,
      bool includeBodyNodes = true,
      bool includeJoints = true);

  /// Create a Group that mimics the given MetaSkeleton
  [[nodiscard]] static GroupPtr create(
      const std::string& _name, const MetaSkeletonPtr& _metaSkeleton);

  /// Destructor
  virtual ~Group() = default;

  /// Creates and returns a clone of this Group.
  GroupPtr cloneGroup() const;

  /// Creates and returns a clone of this Group.
  GroupPtr cloneGroup(const std::string& cloneName) const;

  // To expose MetaSkeleton::cloneMetaSkeleton(), which takes no cloneName.
  using MetaSkeleton::cloneMetaSkeleton;

  // Documentation inherited
  MetaSkeletonPtr cloneMetaSkeleton(
      const std::string& cloneName) const override;

  /// Swap the index of BodyNode _index1 with _index2
  void swapBodyNodeIndices(std::size_t _index1, std::size_t _index2);

  /// Swap the index of DegreeOfFreedom _index1 with _index2
  void swapDofIndices(std::size_t _index1, std::size_t _index2);

  /// Add a BodyNode and its parent DegreesOfFreedom to this Group. If _warning
  /// is true, you will be warned when the BodyNode and all its DegreesOfFreedom
  /// were already in the Group, and an assertion will be thrown.
  ///
  /// This function will return false if the BodyNode and all its
  /// DegreesOfFreedom were already in the Group.
  bool addComponent(BodyNode* _bn, bool _warning = true);

  /// Add set of BodyNodes and their parent DegreesOfFreedom to this Group. If
  /// warning is true, you will be warned when an entire component was already
  /// in the Group, and an assertion will be thrown.
  ///
  /// This function will return false if all of the components in the set were
  /// already in this Group.
  bool addComponents(std::span<BodyNode* const> bodyNodes, bool warning = true);

  /// Remove a BodyNode and its parent DegreesOfFreedom from this Group. If
  /// _warning is true, you will be warned if this Group does not have the
  /// BodyNode or any of its DegreesOfFreedom, and an assertion will be thrown.
  ///
  /// This function will return false if the Group did not include the BodyNode
  /// or any of its parent DegreesOfFreedom.
  bool removeComponent(BodyNode* _bn, bool _warning = true);

  /// Remove a set of BodyNodes and their parent DegreesOfFreedom from this
  /// Group. If warning is true, you will be warned if any of the components
  /// were completely missing from this Group, and an assertion will be thrown.
  ///
  /// This function will return false if none of the components in this set were
  /// in the Group.
  bool removeComponents(
      std::span<BodyNode* const> bodyNodes, bool warning = true);

  /// Add a BodyNode to this Group. If _warning is true, you will be warned when
  /// you attempt to add the same BodyNode twice, and an assertion will be
  /// thrown.
  ///
  /// This function will return false if the BodyNode was already in the Group.
  bool addBodyNode(BodyNode* _bn, bool _warning = true);

  /// Add a set of BodyNodes to this Group. If warning is true, you will be
  /// warned when you attempt to add a BodyNode that is already in the Group,
  /// and an assertion will be thrown.
  ///
  /// This function will return false if all of the BodyNodes were already in
  /// the Group.
  bool addBodyNodes(std::span<BodyNode* const> bodyNodes, bool warning = true);

  /// Remove a BodyNode from this Group. If _warning is true, you will be warned
  /// when you attempt to remove a BodyNode that is not in this Group, and an
  /// assertion will be thrown.
  ///
  /// The function will return false if the BodyNode was not in this Group.
  bool removeBodyNode(BodyNode* _bn, bool _warning = true);

  /// Remove a set of BodyNodes from this Group. If warning is true, you will
  /// be warned when you attempt to remove a BodyNode that is not in this Group,
  /// and an assertion will be thrown.
  ///
  /// The function will return false if none of the BodyNodes were in this
  /// Group.
  bool removeBodyNodes(
      std::span<BodyNode* const> bodyNodes, bool warning = true);

  /// Add a Joint to this Group. If _addDofs is true, it will also add all the
  /// DegreesOfFreedom of the Joint. If _warning is true, you will be warned
  /// if the Joint (and all its DOFs if _addDofs is set to true) was already in
  /// the Group, and an assertion will be thrown.
  ///
  /// This function will return false if the Joint (and all its DOFs, if
  /// _addDofs is true) was already in the Group.
  bool addJoint(Joint* _joint, bool _addDofs = true, bool _warning = true);

  /// Add a set of Joints to this Group. If addDofs is true, it will also add
  /// all the DOFs of each Joint. If warning is true, you will be warned when
  /// you attempt to add a Joint that is already in the Group (and all its DOFs
  /// are in the Group, if addDofs is set to true), and an assertion will be
  /// thrown.
  ///
  /// This function will return false if all the Joints (and their DOFs if
  /// addDofs is set to true) were already in the Group.
  bool addJoints(
      std::span<Joint* const> joints, bool addDofs = true, bool warning = true);

  /// Remove a Joint from this Group. If _removeDofs is true, it will also
  /// remove any DOFs belonging to this Joint. If _warning is true, you will
  /// be warned if you attempt to remove a Joint which is not in this Group (and
  /// none of its DOFs are in the Group if _removeDofs is set to true), and an
  /// assertion will be thrown.
  ///
  /// This function will return false if the Joint was not in the Group (and
  /// neither were any of its DOFs, if _removeDofs is set to true).
  bool removeJoint(
      Joint* _joint, bool _removeDofs = true, bool _warning = true);

  /// Remove a set of Joints from this Group. If removeDofs is true, it will
  /// also remove any DOFs belonging to any of the Joints. If warning is true,
  /// you will be warned if you attempt to remove a Joint which is not in this
  /// Group (and none of its DOFs are in the Group if removeDofs is set to
  /// true), and an assertion will be thrown.
  ///
  /// This function will return false if none of the Joints (nor any of their
  /// DOFs if removeDofs is set to true) were in the Group.
  bool removeJoints(
      std::span<Joint* const> joints,
      bool removeDofs = true,
      bool warning = true);

  /// Add a DegreeOfFreedom to this Group. If _addJoint is true, the Joint of
  /// this DOF will also be added to the Group. If _warning is true, you will be
  /// warned when you attempt to add the same DegreeOfFreedom twice, and an
  /// assertion will be thrown.
  ///
  /// This function will return false if the DegreeOfFreedom was already in the
  /// Group.
  bool addDof(
      DegreeOfFreedom* _dof, bool _addJoint = true, bool _warning = true);

  /// Add a set of DegreesOfFreedom to this Group. If addJoint is true, the
  /// Joint of each DOF will also be added to the Group. If warning is true,
  /// you will be warned when you attempt to add the same DegreeOfFreedom twice,
  /// and an assertion will be thrown.
  ///
  /// This function will return false if all of the DegreesOfFreedom was already
  /// in the Group.
  bool addDofs(
      std::span<DegreeOfFreedom* const> dofs,
      bool addJoint = true,
      bool warning = true);

  /// Remove a DegreeOfFreedom from this Group. If _cleanupJoint is true, the
  /// Joint of this DOF will be removed, but only if no other DOFs belonging to
  /// the Joint are remaining in the Group. If _warning is true, you will be
  /// warned when you attempt to remove a DegreeOfFreedom that is not in this
  /// Group, and an assertion will be thrown.
  ///
  /// This function will return false if the DegreeOfFreedom was not in this
  /// Group.
  bool removeDof(
      DegreeOfFreedom* _dof, bool _cleanupJoint = true, bool _warning = true);

  /// Remove a set of DegreesOfFreedom from this Group. If cleanupJoint is
  /// true, the Joint of this DOF will be removed, but only if no other DOFs
  /// belonging to the Joint are remaining in the Group. If warning is true,
  /// you will be warned when you attempt to remove a DegreeOfFreedom that is
  /// not in this Group, and an assertion will be thrown.
  ///
  /// This function will return false if none of the DegreesOfFreedom were in
  /// this Group.
  bool removeDofs(
      std::span<DegreeOfFreedom* const> dofs,
      bool cleanupJoint = true,
      bool warning = true);

protected:
  /// Default constructor
  Group(
      const std::string& name,
      std::span<BodyNode* const> bodyNodes,
      bool includeJoints,
      bool includeDofs);

  /// Alternative constructor
  Group(
      const std::string& name,
      std::span<DegreeOfFreedom* const> dofs,
      bool includeBodyNodes,
      bool includeJoints);

  /// MetaSkeleton-based constructor
  Group(const std::string& _name, const MetaSkeletonPtr& _metaSkeleton);
};

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_GROUP_HPP_
