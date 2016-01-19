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

#ifndef KIDO_DYNAMICS_DETAIL_SKELETON_HPP_
#define KIDO_DYNAMICS_DETAIL_SKELETON_HPP_

//==============================================================================
template <class JointType>
JointType* Skeleton::moveBodyNodeTree(
    BodyNode* _bodyNode,
    const SkeletonPtr& _newSkeleton,
    BodyNode* _parentNode,
    const typename JointType::Properties& _joint)
{
  JointType* parentJoint = new JointType(_joint);

  if(moveBodyNodeTree(parentJoint, _bodyNode, _newSkeleton, _parentNode))
    return parentJoint;

  // If the move failed, we should delete the Joint that we created and return
  // a nullptr.
  delete parentJoint;
  return nullptr;
}

//==============================================================================
template <class JointType>
std::pair<JointType*, BodyNode*> Skeleton::cloneBodyNodeTree(
    const BodyNode* _bodyNode,
    const SkeletonPtr& _newSkeleton,
    BodyNode* _parentNode,
    const typename JointType::Properties& _joint, bool _recursive) const
{
  JointType* parentJoint = new JointType(_joint);
  std::pair<Joint*, BodyNode*> root =
      cloneBodyNodeTree(parentJoint, _bodyNode, _newSkeleton, _parentNode,
                        _recursive);
  return std::pair<JointType*, BodyNode*>(parentJoint, root.second);
}

//==============================================================================
template <class JointType, class NodeType>
std::pair<JointType*, NodeType*> Skeleton::createJointAndBodyNodePair(
    BodyNode* _parent,
    const typename JointType::Properties& _jointProperties,
    const typename NodeType::Properties& _bodyProperties)
{
  JointType* joint = new JointType(_jointProperties);
  NodeType* node = new NodeType(_parent, joint, _bodyProperties);
  registerBodyNode(node);

  return std::pair<JointType*, NodeType*>(joint, node);
}

#endif // KIDO_DYNAMICS_DETAIL_SKELETON_HPP_
