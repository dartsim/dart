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

#include "dart/dynamics/JacobianNode.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/InverseKinematics.h"

namespace kido {
namespace dynamics {

//==============================================================================
JacobianNode::JacobianNode()
  : Entity(nullptr, "", false),
    Frame(nullptr, ""),
    Node(Node::ConstructAbstract),
    mIsBodyJacobianDirty(true),
    mIsWorldJacobianDirty(true),
    mIsBodyJacobianSpatialDerivDirty(true),
    mIsWorldJacobianClassicDerivDirty(true)
{
  // Do nothing
}

//==============================================================================
// This destructor needs to be defined somewhere that the definition of
// InverseKinematics is visible, because it's needed by the
// std::unique_ptr<InverseKinematics> class member
JacobianNode::~JacobianNode() = default;

//==============================================================================
const std::shared_ptr<InverseKinematics>&
JacobianNode::getIK(bool _createIfNull)
{
  if(nullptr == mIK && _createIfNull)
    createIK();

  return mIK;
}

//==============================================================================
const std::shared_ptr<InverseKinematics>& JacobianNode::getOrCreateIK()
{
  return getIK(true);
}

//==============================================================================
std::shared_ptr<const InverseKinematics> JacobianNode::getIK() const
{
  return const_cast<JacobianNode*>(this)->getIK(false);
}

//==============================================================================
const std::shared_ptr<InverseKinematics>& JacobianNode::createIK()
{
  mIK = InverseKinematics::create(this);
  return mIK;
}

//==============================================================================
void JacobianNode::clearIK()
{
  mIK = nullptr;
}

//==============================================================================
void JacobianNode::notifyJacobianUpdate()
{
  // mIsWorldJacobianDirty depends on mIsBodyJacobianDirty, so we only need to
  // check mIsBodyJacobianDirty if we want to terminate.
  if(mIsBodyJacobianDirty)
    return;

  mIsBodyJacobianDirty = true;
  mIsWorldJacobianDirty = true;

  for(JacobianNode* child : mChildJacobianNodes)
    child->notifyJacobianUpdate();
}

//==============================================================================
void JacobianNode::notifyJacobianDerivUpdate()
{
  // These two flags are independent of each other, so we must check that both
  // are true if we want to terminate early.
  if(mIsBodyJacobianSpatialDerivDirty && mIsWorldJacobianClassicDerivDirty)
    return;

  mIsBodyJacobianSpatialDerivDirty = true;
  mIsWorldJacobianClassicDerivDirty = true;

  for(JacobianNode* child : mChildJacobianNodes)
    child->notifyJacobianDerivUpdate();
}

} // namespace dynamics
} // namespace kido
