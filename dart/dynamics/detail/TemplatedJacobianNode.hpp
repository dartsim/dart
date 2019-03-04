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

#ifndef DART_DYNAMICS_DETAIL_TEMPLATEDJACOBIAN_HPP_
#define DART_DYNAMICS_DETAIL_TEMPLATEDJACOBIAN_HPP_

#include "dart/dynamics/TemplatedJacobianNode.hpp"

namespace dart {
namespace dynamics {

//==============================================================================
template<class NodeType>
math::Jacobian
TemplatedJacobianNode<NodeType>::getJacobian(
    const Frame* _inCoordinatesOf) const
{
  if(this == _inCoordinatesOf)
    return static_cast<const NodeType*>(this)->getJacobian();
  else if(_inCoordinatesOf->isWorld())
    return static_cast<const NodeType*>(this)->getWorldJacobian();

  return math::AdRJac(getTransform(_inCoordinatesOf),
                      static_cast<const NodeType*>(this)->getJacobian());
}

//==============================================================================
template<class NodeType>
math::Jacobian
TemplatedJacobianNode<NodeType>::getJacobian(
    const Eigen::Vector3d& _offset) const
{
  math::Jacobian J = static_cast<const NodeType*>(this)->getJacobian();
  J.bottomRows<3>() += J.topRows<3>().colwise().cross(_offset);

  return J;
}

//==============================================================================
template<class NodeType>
math::Jacobian
TemplatedJacobianNode<NodeType>::getJacobian(
    const Eigen::Vector3d& _offset,
    const Frame* _inCoordinatesOf) const
{
  if(this == _inCoordinatesOf)
    return getJacobian(_offset);
  else if(_inCoordinatesOf->isWorld())
    return getWorldJacobian(_offset);

  Eigen::Isometry3d T = getTransform(_inCoordinatesOf);
  T.translation() = - T.linear() * _offset;

  return math::AdTJac(T, static_cast<const NodeType*>(this)->getJacobian());
}

//==============================================================================
template<class NodeType>
math::Jacobian
TemplatedJacobianNode<NodeType>::getWorldJacobian(
    const Eigen::Vector3d& _offset) const
{
  math::Jacobian J = static_cast<const NodeType*>(this)->getWorldJacobian();
  J.bottomRows<3>() += J.topRows<3>().colwise().cross(
                                      getWorldTransform().linear() * _offset);

  return J;
}

//==============================================================================
template<class NodeType>
math::LinearJacobian
TemplatedJacobianNode<NodeType>::getLinearJacobian(
    const Frame* _inCoordinatesOf) const
{
  if(this == _inCoordinatesOf)
  {
    const math::Jacobian& J =
        static_cast<const NodeType*>(this)->getJacobian();

    return J.bottomRows<3>();
  }
  else if(_inCoordinatesOf->isWorld())
  {
    const math::Jacobian& JWorld =
        static_cast<const NodeType*>(this)->getWorldJacobian();

    return JWorld.bottomRows<3>();
  }

  const math::Jacobian& J =
      static_cast<const NodeType*>(this)->getJacobian();

  return getTransform(_inCoordinatesOf).linear() * J.bottomRows<3>();
}

//==============================================================================
template<class NodeType>
math::LinearJacobian
TemplatedJacobianNode<NodeType>::getLinearJacobian(
    const Eigen::Vector3d& _offset,
    const Frame* _inCoordinatesOf) const
{
  const math::Jacobian& J =
      static_cast<const NodeType*>(this)->getJacobian();

  math::LinearJacobian JLinear;
  JLinear = J.bottomRows<3>() + J.topRows<3>().colwise().cross(_offset);

  if(this == _inCoordinatesOf)
    return JLinear;

  return getTransform(_inCoordinatesOf).linear() * JLinear;
}

//==============================================================================
template<class NodeType>
math::AngularJacobian
TemplatedJacobianNode<NodeType>::getAngularJacobian(
    const Frame* _inCoordinatesOf) const
{
  if(this == _inCoordinatesOf)
  {
    const math::Jacobian& J =
        static_cast<const NodeType*>(this)->getJacobian();
    return J.topRows<3>();
  }
  else if(_inCoordinatesOf->isWorld())
  {
    const math::Jacobian& JWorld =
        static_cast<const NodeType*>(this)->getWorldJacobian();
    return JWorld.topRows<3>();
  }

  const math::Jacobian& J =
      static_cast<const NodeType*>(this)->getJacobian();

  return getTransform(_inCoordinatesOf).linear() * J.topRows<3>();
}

//==============================================================================
template<class NodeType>
math::Jacobian
TemplatedJacobianNode<NodeType>::getJacobianSpatialDeriv(
    const Frame* _inCoordinatesOf) const
{
  if(this == _inCoordinatesOf)
    return static_cast<const NodeType*>(this)->getJacobianSpatialDeriv();

  return math::AdRJac(getTransform(_inCoordinatesOf),
        static_cast<const NodeType*>(this)->getJacobianSpatialDeriv());
}

//==============================================================================
template<class NodeType>
math::Jacobian
TemplatedJacobianNode<NodeType>::getJacobianSpatialDeriv(
    const Eigen::Vector3d& _offset) const
{
  math::Jacobian J_d =
      static_cast<const NodeType*>(this)->getJacobianSpatialDeriv();

  J_d.bottomRows<3>() += J_d.topRows<3>().colwise().cross(_offset);

  return J_d;
}

//==============================================================================
template<class NodeType>
math::Jacobian
TemplatedJacobianNode<NodeType>::getJacobianSpatialDeriv(
    const Eigen::Vector3d& _offset,
    const Frame* _inCoordinatesOf) const
{
  if(this == _inCoordinatesOf)
    return getJacobianSpatialDeriv(_offset);

  Eigen::Isometry3d T = getTransform(_inCoordinatesOf);
  T.translation() = T.linear() * -_offset;

  return math::AdTJac(
        T, static_cast<const NodeType*>(this)->getJacobianSpatialDeriv());
}

//==============================================================================
template<class NodeType>
math::Jacobian
TemplatedJacobianNode<NodeType>::getJacobianClassicDeriv(
    const Frame* _inCoordinatesOf) const
{
  if(_inCoordinatesOf->isWorld())
    return static_cast<const NodeType*>(this)->getJacobianClassicDeriv();

  return math::AdRInvJac(_inCoordinatesOf->getWorldTransform(),
        static_cast<const NodeType*>(this)->getJacobianClassicDeriv());
}

//==============================================================================
template<class NodeType>
math::Jacobian
TemplatedJacobianNode<NodeType>::getJacobianClassicDeriv(
    const Eigen::Vector3d& _offset,
    const Frame* _inCoordinatesOf) const
{
  math::Jacobian J_d =
      static_cast<const NodeType*>(this)->getJacobianClassicDeriv();

  const math::Jacobian& J =
      static_cast<const NodeType*>(this)->getWorldJacobian();

  const Eigen::Vector3d& w = getAngularVelocity();
  const Eigen::Vector3d& p = (getWorldTransform().linear() * _offset).eval();

  J_d.bottomRows<3>() += J_d.topRows<3>().colwise().cross(p)
                         + J.topRows<3>().colwise().cross(w.cross(p));

  if(_inCoordinatesOf->isWorld())
    return J_d;

  return math::AdRInvJac(_inCoordinatesOf->getWorldTransform(), J_d);
}

//==============================================================================
template<class NodeType>
math::LinearJacobian
TemplatedJacobianNode<NodeType>::getLinearJacobianDeriv(
    const Frame* _inCoordinatesOf) const
{
  const math::Jacobian& J_d =
      static_cast<const NodeType*>(this)->getJacobianClassicDeriv();

  if(_inCoordinatesOf->isWorld())
    return J_d.bottomRows<3>();

  return _inCoordinatesOf->getWorldTransform().linear().transpose()
          * J_d.bottomRows<3>();
}

//==============================================================================
template<class NodeType>
math::LinearJacobian
TemplatedJacobianNode<NodeType>::getLinearJacobianDeriv(
    const Eigen::Vector3d& _offset,
    const Frame* _inCoordinatesOf) const
{
  const math::Jacobian& J_d =
      static_cast<const NodeType*>(this)->getJacobianClassicDeriv();

  const math::Jacobian& J =
      static_cast<const NodeType*>(this)->getWorldJacobian();

  const Eigen::Vector3d& w = getAngularVelocity();
  const Eigen::Vector3d& p = (getWorldTransform().linear() * _offset).eval();

  if(_inCoordinatesOf->isWorld())
    return J_d.bottomRows<3>() + J_d.topRows<3>().colwise().cross(p)
           + J.topRows<3>().colwise().cross(w.cross(p));

  return _inCoordinatesOf->getWorldTransform().linear().transpose()
         * (J_d.bottomRows<3>() + J_d.topRows<3>().colwise().cross(p)
            + J.topRows<3>().colwise().cross(w.cross(p)));
}

//==============================================================================
template<class NodeType>
math::AngularJacobian
TemplatedJacobianNode<NodeType>::getAngularJacobianDeriv(
    const Frame* _inCoordinatesOf) const
{
  const math::Jacobian& J_d =
      static_cast<const NodeType*>(this)->getJacobianClassicDeriv();

  if(_inCoordinatesOf->isWorld())
    return J_d.topRows<3>();

  return _inCoordinatesOf->getWorldTransform().linear().transpose()
         * J_d.topRows<3>();
}

//==============================================================================
template<class NodeType>
TemplatedJacobianNode<NodeType>::TemplatedJacobianNode(BodyNode* bn)
  : Entity(Entity::ConstructAbstract),
    Frame(Frame::ConstructAbstract),
    JacobianNode(bn)
{
  // Do nothing
}

} // namespace dynamics
} // namespace dart


#endif // DART_DYNAMICS_DETAIL_TEMPLATEDJACOBIAN_HPP_
