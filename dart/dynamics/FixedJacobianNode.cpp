/*
 * Copyright (c) 2016, Georgia Tech Research Corporation
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

#include "dart/dynamics/FixedJacobianNode.h"
#include "dart/dynamics/BodyNode.h"

namespace dart {
namespace dynamics {

//==============================================================================
void FixedJacobianNode::setRelativeTransform(
    const Eigen::Isometry3d& newRelativeTf)
{
  if(newRelativeTf.matrix() ==
     FixedFrame::mAspectProperties.mRelativeTf.matrix())
    return;

  FixedFrame::setRelativeTransform(newRelativeTf);
  notifyJacobianUpdate();
  notifyJacobianDerivUpdate();
}

//==============================================================================
bool FixedJacobianNode::dependsOn(size_t _genCoordIndex) const
{
  return mBodyNode->dependsOn(_genCoordIndex);
}

//==============================================================================
size_t FixedJacobianNode::getNumDependentGenCoords() const
{
  return mBodyNode->getNumDependentGenCoords();
}

//==============================================================================
size_t FixedJacobianNode::getDependentGenCoordIndex(size_t _arrayIndex) const
{
  return mBodyNode->getDependentGenCoordIndex(_arrayIndex);
}

//==============================================================================
const std::vector<size_t>& FixedJacobianNode::getDependentGenCoordIndices() const
{
  return mBodyNode->getDependentGenCoordIndices();
}

//==============================================================================
size_t FixedJacobianNode::getNumDependentDofs() const
{
  return mBodyNode->getNumDependentDofs();
}

//==============================================================================
DegreeOfFreedom* FixedJacobianNode::getDependentDof(size_t _index)
{
  return mBodyNode->getDependentDof(_index);
}

//==============================================================================
const DegreeOfFreedom* FixedJacobianNode::getDependentDof(size_t _index) const
{
  return mBodyNode->getDependentDof(_index);
}

//==============================================================================
const std::vector<DegreeOfFreedom*>& FixedJacobianNode::getDependentDofs()
{
  return mBodyNode->getDependentDofs();
}

//==============================================================================
const std::vector<const DegreeOfFreedom*>& FixedJacobianNode::getDependentDofs() const
{
  return static_cast<const BodyNode*>(mBodyNode)->getDependentDofs();
}

//==============================================================================
const std::vector<const DegreeOfFreedom*> FixedJacobianNode::getChainDofs() const
{
  return mBodyNode->getChainDofs();
}

//==============================================================================
const math::Jacobian& FixedJacobianNode::getJacobian() const
{
  if (mIsBodyJacobianDirty)
    updateBodyJacobian();

  return mCache.mBodyJacobian;
}

//==============================================================================
const math::Jacobian& FixedJacobianNode::getWorldJacobian() const
{
  if(mIsWorldJacobianDirty)
    updateWorldJacobian();

  return mCache.mWorldJacobian;
}

//==============================================================================
const math::Jacobian& FixedJacobianNode::getJacobianSpatialDeriv() const
{
  if(mIsBodyJacobianSpatialDerivDirty)
    updateBodyJacobianSpatialDeriv();

  return mCache.mBodyJacobianSpatialDeriv;
}

//==============================================================================
const math::Jacobian& FixedJacobianNode::getJacobianClassicDeriv() const
{
  if(mIsWorldJacobianClassicDerivDirty)
    updateWorldJacobianClassicDeriv();

  return mCache.mWorldJacobianClassicDeriv;
}

//==============================================================================
FixedJacobianNode::FixedJacobianNode(
    BodyNode* parent, const Eigen::Isometry3d& transform)
  : FixedFrame(parent, transform),
    detail::FixedJacobianNodeCompositeBase(parent)
{
  // Do nothing
}

//==============================================================================
FixedJacobianNode::FixedJacobianNode(
    const std::tuple<BodyNode *, Eigen::Isometry3d>& args)
  : FixedJacobianNode(std::get<0>(args), std::get<1>(args))
{
  // Delegating constructor
}

//==============================================================================
void FixedJacobianNode::updateBodyJacobian() const
{
  mCache.mBodyJacobian = math::AdInvTJac(getRelativeTransform(),
                                         mBodyNode->getJacobian());
  mIsBodyJacobianDirty = false;
}

//==============================================================================
void FixedJacobianNode::updateWorldJacobian() const
{
  mCache.mWorldJacobian = math::AdRJac(getWorldTransform(), getJacobian());

  mIsWorldJacobianDirty = false;
}

//==============================================================================
void FixedJacobianNode::updateBodyJacobianSpatialDeriv() const
{
  mCache.mBodyJacobianSpatialDeriv =
      math::AdInvTJac(getRelativeTransform(),
                      mBodyNode->getJacobianSpatialDeriv());

  mIsBodyJacobianSpatialDerivDirty = false;
}

//==============================================================================
void FixedJacobianNode::updateWorldJacobianClassicDeriv() const
{
  const math::Jacobian& dJ_parent = mBodyNode->getJacobianClassicDeriv();
  const math::Jacobian& J_parent = mBodyNode->getWorldJacobian();

  const Eigen::Vector3d& v_local =
      getLinearVelocity(mBodyNode, Frame::World());

  const Eigen::Vector3d& w_parent = mBodyNode->getAngularVelocity();
  const Eigen::Vector3d& p = (getWorldTransform().translation()
                  - mBodyNode->getWorldTransform().translation()).eval();

  mCache.mWorldJacobianClassicDeriv = dJ_parent;
  mCache.mWorldJacobianClassicDeriv.bottomRows<3>().noalias() +=
      J_parent.topRows<3>().colwise().cross(v_local + w_parent.cross(p))
      + dJ_parent.topRows<3>().colwise().cross(p);

  mIsWorldJacobianClassicDerivDirty = false;
}


} // namespace dynamics
} // namespace dart
