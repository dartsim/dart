/*
 * Copyright (c) 2011-2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>
 *            Jeongseok Lee <jslee02@gmail.com>
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

#include "dart/dynamics/BodyNode.h"

#include <algorithm>
#include <vector>
#include <string>

#include "dart/common/Console.h"
#include "dart/math/Helpers.h"
#include "dart/renderer/RenderInterface.h"
#include "dart/dynamics/Joint.h"
#include "dart/dynamics/Shape.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/Marker.h"
#include "dart/dynamics/SoftBodyNode.h"

#define DART_DEFAULT_FRICTION_COEFF 1.0
#define DART_DEFAULT_RESTITUTION_COEFF 0.0

namespace dart {
namespace dynamics {

//==============================================================================
typedef std::set<Entity*> EntityPtrSet;

//==============================================================================
template <typename T>
static T getVectorObjectIfAvailable(size_t _index, const std::vector<T>& _vec)
{
  assert(_index < _vec.size());
  if(_index < _vec.size())
    return _vec[_index];

  return NULL;
}

//==============================================================================
int BodyNode::msBodyNodeCount = 0;

//==============================================================================
BodyNode::BodyNode(const std::string& _name)
  : Entity(Frame::World(), _name, false),
    Frame(Frame::World(), _name),
    mID(BodyNode::msBodyNodeCount++),
    mName(_name),
    mGravityMode(true),
    mI(Eigen::Matrix6d::Identity()),
    mCenterOfMass(Eigen::Vector3d::Zero()),
    mIxx(1.0),
    mIyy(1.0),
    mIzz(1.0),
    mIxy(0.0),
    mIxz(0.0),
    mIyz(0.0),
    mMass(1.0),
    mFrictionCoeff(DART_DEFAULT_FRICTION_COEFF),
    mRestitutionCoeff(DART_DEFAULT_RESTITUTION_COEFF),
    mIsCollidable(true),
    mIsColliding(false),
    mSkeleton(NULL),
    mParentJoint(NULL),
    mParentBodyNode(NULL),
    mChildBodyNodes(std::vector<BodyNode*>(0)),
    mIsBodyJacobianDirty(true),
    mIsWorldJacobianDirty(true),
    mIsBodyJacobianSpatialDerivDirty(true),
    mIsWorldJacobianClassicDerivDirty(true),
    mPartialAcceleration(Eigen::Vector6d::Zero()),
    mIsPartialAccelerationDirty(true),
    mF(Eigen::Vector6d::Zero()),
    mFext(Eigen::Vector6d::Zero()),
    mFgravity(Eigen::Vector6d::Zero()),
    mArtInertia(Eigen::Matrix6d::Identity()),
    mArtInertiaImplicit(Eigen::Matrix6d::Identity()),
    mBiasForce(Eigen::Vector6d::Zero()),
    mCg_dV(Eigen::Vector6d::Zero()),
    mCg_F(Eigen::Vector6d::Zero()),
    mG_F(Eigen::Vector6d::Zero()),
    mFext_F(Eigen::Vector6d::Zero()),
    mM_dV(Eigen::Vector6d::Zero()),
    mM_F(Eigen::Vector6d::Zero()),
    mInvM_c(Eigen::Vector6d::Zero()),
    mInvM_U(Eigen::Vector6d::Zero()),
    mArbitrarySpatial(Eigen::Vector6d::Zero()),
    mDelV(Eigen::Vector6d::Zero()),
    mBiasImpulse(Eigen::Vector6d::Zero()),
    mConstraintImpulse(Eigen::Vector6d::Zero()),
    mImpF(Eigen::Vector6d::Zero())
{
}

//==============================================================================
BodyNode::~BodyNode()
{
  // Release shapes for collisions
  for (std::vector<Shape*>::const_iterator itColShape = mColShapes.begin();
       itColShape != mColShapes.end(); ++itColShape)
  {
    // If the collision shapes are not in the list of visualization shapes
    if (mVizShapes.end() == std::find(mVizShapes.begin(), mVizShapes.end(),
                                      *itColShape))
    {
      // Delete it
      delete (*itColShape);
    }
  }

  // Release markers
  for (std::vector<Marker*>::const_iterator it = mMarkers.begin();
       it != mMarkers.end(); ++it)
  {
    delete (*it);
  }

  // Release parent joint
  delete mParentJoint;
}

//==============================================================================
const std::string& BodyNode::setName(const std::string& _name)
{
  // If it already has the requested name, do nothing
  if(mName == _name)
    return mName;

  // If the BodyNode belongs to a Skeleton, consult the Skeleton's NameManager
  if(mSkeleton)
  {
    mSkeleton->mNameMgrForBodyNodes.removeName(mName);
    SoftBodyNode* softnode = dynamic_cast<SoftBodyNode*>(this);
    if(softnode)
      mSkeleton->mNameMgrForSoftBodyNodes.removeName(mName);

    mName = _name;
    mSkeleton->addEntryToBodyNodeNameMgr(this);

    if(softnode)
      mSkeleton->addEntryToSoftBodyNodeNameMgr(softnode);
  }
  else
  {
    mName = _name;
  }

  // Return the final name (which might have been altered by the Skeleton's
  // NameManager)
  return mName;
}

//==============================================================================
const std::string& BodyNode::getName() const
{
  return mName;
}

//==============================================================================
void BodyNode::setGravityMode(bool _gravityMode)
{
  if (mGravityMode == _gravityMode)
    return;

  mGravityMode = _gravityMode;

  if (mSkeleton)
    mSkeleton->mIsGravityForcesDirty = true;
}

//==============================================================================
bool BodyNode::getGravityMode() const
{
  return mGravityMode;
}

//==============================================================================
bool BodyNode::isCollidable() const
{
  return mIsCollidable;
}

//==============================================================================
void BodyNode::setCollidable(bool _isCollidable)
{
  mIsCollidable = _isCollidable;
}

//==============================================================================
void BodyNode::setMass(double _mass)
{
  assert(_mass >= 0.0 && "Negative mass is not allowable.");

  mMass = _mass;

  _updateSpatialInertia();
}

//==============================================================================
double BodyNode::getMass() const
{
  return mMass;
}

//==============================================================================
void BodyNode::setMomentOfInertia(double _Ixx, double _Iyy, double _Izz,
                                  double _Ixy, double _Ixz, double _Iyz)
{
  assert(_Ixx >= 0.0);
  assert(_Iyy >= 0.0);
  assert(_Izz >= 0.0);

  mIxx = _Ixx;
  mIyy = _Iyy;
  mIzz = _Izz;

  mIxy = _Ixy;
  mIxz = _Ixz;
  mIyz = _Iyz;

  _updateSpatialInertia();
}

//==============================================================================
void BodyNode::getMomentOfInertia(double& _Ixx, double& _Iyy, double& _Izz,
                                  double& _Ixy, double& _Ixz, double& _Iyz)
{
  _Ixx = mIxx;
  _Iyy = mIyy;
  _Izz = mIzz;

  _Ixy = mIxy;
  _Ixz = mIxz;
  _Iyz = mIyz;
}

//==============================================================================
const Eigen::Matrix6d& BodyNode::getSpatialInertia() const
{
  return mI;
}

//==============================================================================
const math::Inertia& BodyNode::getArticulatedInertia() const
{
  if(mSkeleton && mSkeleton->mIsArticulatedInertiaDirty)
    mSkeleton->updateArticulatedInertia();

  return mArtInertia;
}

//==============================================================================
const math::Inertia& BodyNode::getArticulatedInertiaImplicit() const
{
  if(mSkeleton && mSkeleton->mIsArticulatedInertiaDirty)
    mSkeleton->updateArticulatedInertia();

  return mArtInertiaImplicit;
}

//==============================================================================
void BodyNode::setLocalCOM(const Eigen::Vector3d& _com)
{
  mCenterOfMass = _com;

  _updateSpatialInertia();
}

//==============================================================================
const Eigen::Vector3d& BodyNode::getLocalCOM() const
{
  return mCenterOfMass;
}

//==============================================================================
Eigen::Vector3d BodyNode::getWorldCOM() const
{
  return getWorldTransform() * mCenterOfMass;
}

//==============================================================================
Eigen::Vector3d BodyNode::getWorldCOMVelocity() const
{
  return getSpatialVelocity(mCenterOfMass, Frame::World(), Frame::World()).tail<3>();
}

//==============================================================================
Eigen::Vector3d BodyNode::getWorldCOMAcceleration() const
{
  return getSpatialAcceleration(mCenterOfMass, Frame::World(), Frame::World()).tail<3>();
}

//==============================================================================
Eigen::Vector3d BodyNode::getCOM(const Frame* _withRespectTo) const
{
  return getTransform(_withRespectTo) * mCenterOfMass;
}

//==============================================================================
Eigen::Vector3d BodyNode::getCOMLinearVelocity(const Frame* _relativeTo,
                                            const Frame* _inCoordinatesOf) const
{
  return getLinearVelocity(mCenterOfMass, _relativeTo, _inCoordinatesOf);
}

//==============================================================================
Eigen::Vector6d BodyNode::getCOMSpatialVelocity() const
{
  return getSpatialVelocity(mCenterOfMass);
}

//==============================================================================
Eigen::Vector6d BodyNode::getCOMSpatialVelocity(const Frame* _relativeTo,
                                            const Frame* _inCoordinatesOf) const
{
  return getSpatialVelocity(mCenterOfMass, _relativeTo, _inCoordinatesOf);
}

//==============================================================================
Eigen::Vector3d BodyNode::getCOMLinearAcceleration(const Frame* _relativeTo,
                                            const Frame* _inCoordinatesOf) const
{
  return getLinearAcceleration(mCenterOfMass, _relativeTo, _inCoordinatesOf);
}

//==============================================================================
Eigen::Vector6d BodyNode::getCOMSpatialAcceleration() const
{
  return getSpatialAcceleration(mCenterOfMass);
}

//==============================================================================
Eigen::Vector6d BodyNode::getCOMSpatialAcceleration(const Frame* _relativeTo,
                                            const Frame* _inCoordinatesOf) const
{
  return getSpatialAcceleration(mCenterOfMass, _relativeTo, _inCoordinatesOf);
}

//==============================================================================
void BodyNode::setFrictionCoeff(double _coeff)
{
  assert(0.0 <= _coeff
         && "Coefficient of friction should be non-negative value.");
  mFrictionCoeff = _coeff;
}

//==============================================================================
double BodyNode::getFrictionCoeff() const
{
  return mFrictionCoeff;
}

//==============================================================================
void BodyNode::setRestitutionCoeff(double _coeff)
{
  assert(0.0 <= _coeff && _coeff <= 1.0
         && "Coefficient of restitution should be in range of [0, 1].");
  mRestitutionCoeff = _coeff;
}

//==============================================================================
double BodyNode::getRestitutionCoeff() const
{
  return mRestitutionCoeff;
}

//==============================================================================
void BodyNode::addVisualizationShape(Shape* _p)
{
  mVizShapes.push_back(_p);
}

//==============================================================================
size_t BodyNode::getNumVisualizationShapes() const
{
  return mVizShapes.size();
}

//==============================================================================
Shape* BodyNode::getVisualizationShape(int _index)
{
  return getVectorObjectIfAvailable<Shape*>(_index, mVizShapes);
}

//==============================================================================
const Shape* BodyNode::getVisualizationShape(int _index) const
{
  return getVectorObjectIfAvailable<Shape*>(_index, mVizShapes);
}

//==============================================================================
void BodyNode::addCollisionShape(Shape* _p)
{
  mColShapes.push_back(_p);
}

//==============================================================================
size_t BodyNode::getNumCollisionShapes() const
{
  return mColShapes.size();
}

//==============================================================================
Shape* BodyNode::getCollisionShape(int _index)
{
  return getVectorObjectIfAvailable<Shape*>(_index, mColShapes);
}

//==============================================================================
const Shape* BodyNode::getCollisionShape(int _index) const
{
  return getVectorObjectIfAvailable<Shape*>(_index, mColShapes);
}

//==============================================================================
Skeleton* BodyNode::getSkeleton()
{
  return mSkeleton;
}

//==============================================================================
const Skeleton* BodyNode::getSkeleton() const
{
  return mSkeleton;
}

//==============================================================================
void BodyNode::setParentJoint(Joint* _joint)
{
  if (_joint->getChildBodyNode())
  {
    assert(_joint->getChildBodyNode() != this);
    // TODO: How should we handle this scenario? (The Joint that was passed in
    // already has some other child BodyNode!)
  }

  if (mSkeleton)
  {
    mSkeleton->unregisterJoint(mParentJoint);
    mSkeleton->registerJoint(_joint);
  }

  if (mParentJoint)
    mParentJoint->mChildBodyNode = NULL;

  mParentJoint = _joint;
  mParentJoint->mChildBodyNode = this;
  // TODO: Should we delete the original mParentJoint? Seems like the BodyNode
  // should be responsible for its parent joint
}

//==============================================================================
Joint* BodyNode::getParentJoint()
{
  return mParentJoint;
}

//==============================================================================
const Joint* BodyNode::getParentJoint() const
{
  return mParentJoint;
}

//==============================================================================
BodyNode* BodyNode::getParentBodyNode()
{
  return mParentBodyNode;
}

//==============================================================================
const BodyNode* BodyNode::getParentBodyNode() const
{
  return mParentBodyNode;
}

//==============================================================================
void BodyNode::addChildBodyNode(BodyNode* _body)
{
  assert(_body != NULL);

  if(std::find(mChildBodyNodes.begin(), mChildBodyNodes.end(), _body) !=
     mChildBodyNodes.end())
  {
    dtwarn << "[BodyNode::addChildBodyNode(~)] Attempting to add a BodyNode '"
           << _body->getName() << "' as a child BodyNode of '" << getName()
           << "', which is already its parent." << std::endl;
    return;
  }

  mChildBodyNodes.push_back(_body);
  _body->mParentBodyNode = this;
  _body->changeParentFrame(this);
}

//==============================================================================
BodyNode* BodyNode::getChildBodyNode(size_t _index)
{
  return getVectorObjectIfAvailable<BodyNode*>(_index, mChildBodyNodes);
}

//==============================================================================
const BodyNode* BodyNode::getChildBodyNode(size_t _index) const
{
  return getVectorObjectIfAvailable<BodyNode*>(_index, mChildBodyNodes);
}

//==============================================================================
size_t BodyNode::getNumChildBodyNodes() const
{
  return mChildBodyNodes.size();
}

//==============================================================================
void BodyNode::addMarker(Marker* _marker)
{
  mMarkers.push_back(_marker);
  if(mSkeleton)
    mSkeleton->addEntryToMarkerNameMgr(_marker);
}

//==============================================================================
size_t BodyNode::getNumMarkers() const
{
  return mMarkers.size();
}

//==============================================================================
Marker* BodyNode::getMarker(size_t _index)
{
  return getVectorObjectIfAvailable<Marker*>(_index, mMarkers);
}

//==============================================================================
const Marker* BodyNode::getMarker(size_t _index) const
{
  return getVectorObjectIfAvailable<Marker*>(_index, mMarkers);
}

//==============================================================================
bool BodyNode::dependsOn(int _genCoordIndex) const
{
  return std::binary_search(mDependentGenCoordIndices.begin(),
                            mDependentGenCoordIndices.end(),
                            _genCoordIndex);
}

//==============================================================================
size_t BodyNode::getNumDependentGenCoords() const
{
  return mDependentGenCoordIndices.size();
}

//==============================================================================
size_t BodyNode::getDependentGenCoordIndex(size_t _arrayIndex) const
{
  assert(0 <= _arrayIndex && _arrayIndex < mDependentGenCoordIndices.size());

  return mDependentGenCoordIndices[_arrayIndex];
}

//==============================================================================
const std::vector<size_t>& BodyNode::getDependentGenCoordIndices() const
{
  return mDependentGenCoordIndices;
}

//==============================================================================
template <typename DofType, typename BnType, typename SkelType, typename JType>
std::vector<DofType*> getDependentDofsTemplate(BnType* bn)
{
  std::vector<DofType*> dofs;
  SkelType* skel = bn->getSkeleton();
  if(!skel)
  {
    JType* joint = bn->getParentJoint();
    if(!joint)
      return dofs;

    size_t nDofs = joint->getNumDofs();
    dofs.reserve(nDofs);
    for(size_t i=0; i<nDofs; ++i)
      dofs.push_back(joint->getDof(i));

    return dofs;
  }

  const std::vector<size_t>& coords = bn->getDependentGenCoordIndices();
  size_t nDofs = coords.size();
  dofs.reserve(nDofs);
  for(size_t i=0; i<nDofs; ++i)
    dofs.push_back(skel->getDof(coords[i]));

  return dofs;
}

//==============================================================================
std::vector<DegreeOfFreedom*> BodyNode::getDependentDofs()
{
  return getDependentDofsTemplate<
      DegreeOfFreedom, BodyNode, Skeleton, Joint>(this);
}

//==============================================================================
std::vector<const DegreeOfFreedom*> BodyNode::getDependentDofs() const
{
  return getDependentDofsTemplate<
      const DegreeOfFreedom, const BodyNode, const Skeleton, const Joint>(this);
}

//==============================================================================
const Eigen::Isometry3d& BodyNode::getRelativeTransform() const
{
  return mParentJoint->getLocalTransform();
}

//==============================================================================
const Eigen::Vector6d& BodyNode::getRelativeSpatialVelocity() const
{
  return mParentJoint->getLocalSpatialVelocity();
}

//==============================================================================
const Eigen::Vector6d& BodyNode::getBodyVelocity() const
{
  return getSpatialVelocity();
}

//==============================================================================
Eigen::Vector3d BodyNode::getBodyLinearVelocity() const
{
  return getSpatialVelocity().tail<3>();
}

//==============================================================================
Eigen::Vector3d BodyNode::getBodyLinearVelocity(
    const Eigen::Vector3d& _offset) const
{
  const Eigen::Vector6d& V = getSpatialVelocity();
  return V.tail<3>() + V.head<3>().cross(_offset);
}

//==============================================================================
Eigen::Vector3d BodyNode::getBodyAngularVelocity() const
{
  return getSpatialVelocity().head<3>();
}

//==============================================================================
Eigen::Vector3d BodyNode::getWorldLinearVelocity() const
{
  return getLinearVelocity();
}

//==============================================================================
Eigen::Vector3d BodyNode::getWorldLinearVelocity(
    const Eigen::Vector3d& _offset) const
{
  return getLinearVelocity(_offset);
}

//==============================================================================
Eigen::Vector3d BodyNode::getWorldAngularVelocity() const
{
  return getAngularVelocity();
}

//==============================================================================
const Eigen::Vector6d& BodyNode::getRelativeSpatialAcceleration() const
{
  return mParentJoint->getLocalSpatialAcceleration();
}

//==============================================================================
const Eigen::Vector6d& BodyNode::getPrimaryRelativeAcceleration() const
{
  return mParentJoint->getLocalPrimaryAcceleration();
}

//==============================================================================
const Eigen::Vector6d& BodyNode::getPartialAcceleration() const
{
  if(mIsPartialAccelerationDirty)
    updatePartialAcceleration();

  return mPartialAcceleration;
}

//==============================================================================
const math::Jacobian& BodyNode::getJacobian() const
{
  if (mIsBodyJacobianDirty)
    _updateBodyJacobian();

  return mBodyJacobian;
}

//==============================================================================
math::Jacobian BodyNode::getJacobian(const Frame* _inCoordinatesOf) const
{
  if(this == _inCoordinatesOf)
    return getJacobian();
  else if(_inCoordinatesOf->isWorld())
    return getWorldJacobian();

  return math::AdRJac(getTransform(_inCoordinatesOf), getJacobian());
}

//==============================================================================
math::Jacobian BodyNode::getJacobian(const Eigen::Vector3d& _offset) const
{
  math::Jacobian J = getJacobian();
  J.bottomRows<3>() += J.topRows<3>().colwise().cross(_offset);

  return J;
}

//==============================================================================
math::Jacobian BodyNode::getJacobian(const Eigen::Vector3d& _offset,
                                     const Frame* _inCoordinatesOf) const
{
  if(this == _inCoordinatesOf)
    return getJacobian(_offset);
  if(_inCoordinatesOf->isWorld())
    return getWorldJacobian(_offset);

  Eigen::Isometry3d T = getTransform(_inCoordinatesOf);
  T.translation() = - T.linear() * _offset;

  return math::AdTJac(T, getJacobian());
}

//==============================================================================
const math::Jacobian& BodyNode::getWorldJacobian() const
{
  if(mIsWorldJacobianDirty)
    _updateWorldJacobian();

  return mWorldJacobian;
}

//==============================================================================
math::Jacobian BodyNode::getWorldJacobian(const Eigen::Vector3d& _offset) const
{
  math::Jacobian J = getWorldJacobian();
  J.bottomRows<3>() += J.topRows<3>().colwise().cross(
                                        getWorldTransform().linear() * _offset);

  return J;
}

//==============================================================================
math::LinearJacobian BodyNode::getLinearJacobian(
    const Frame* _inCoordinatesOf) const
{
  if(this == _inCoordinatesOf)
    return getJacobian().bottomRows<3>();

  return getTransform(_inCoordinatesOf).linear() * getJacobian().bottomRows<3>();
}

//==============================================================================
math::LinearJacobian BodyNode::getLinearJacobian(const Eigen::Vector3d& _offset,
                                            const Frame* _inCoordinatesOf) const
{
  const math::Jacobian& J = getJacobian();
  math::LinearJacobian JLinear;
  JLinear = J.bottomRows<3>() + J.topRows<3>().colwise().cross(_offset);

  if(this == _inCoordinatesOf)
    return JLinear;

  return getTransform(_inCoordinatesOf).linear() * JLinear;
}

//==============================================================================
math::AngularJacobian BodyNode::getAngularJacobian(
                                            const Frame* _inCoordinatesOf) const
{
  if(this == _inCoordinatesOf)
    return getJacobian().topRows<3>();

  return getTransform(_inCoordinatesOf).linear() * getJacobian().topRows<3>();
}

//==============================================================================
const math::Jacobian& BodyNode::getJacobianSpatialDeriv() const
{
  if(mIsBodyJacobianSpatialDerivDirty)
    _updateBodyJacobianSpatialDeriv();

  return mBodyJacobianSpatialDeriv;
}

//==============================================================================
math::Jacobian BodyNode::getJacobianSpatialDeriv(const Frame* _inCoordinatesOf) const
{
  if(this == _inCoordinatesOf)
    return getJacobianSpatialDeriv();

  return math::AdRJac(getTransform(_inCoordinatesOf), getJacobianSpatialDeriv());
}

//==============================================================================
math::Jacobian BodyNode::getJacobianSpatialDeriv(const Eigen::Vector3d& _offset) const
{
  math::Jacobian J_d = getJacobianSpatialDeriv();
  J_d.bottomRows<3>() += J_d.topRows<3>().colwise().cross(_offset);

  return J_d;
}

//==============================================================================
math::Jacobian BodyNode::getJacobianSpatialDeriv(const Eigen::Vector3d& _offset,
                                            const Frame* _inCoordinatesOf) const
{
  if(this == _inCoordinatesOf)
    return getJacobianSpatialDeriv(_offset);

  Eigen::Isometry3d T = getTransform(_inCoordinatesOf);
  T.translation() = T.linear() * -_offset;

  return math::AdTJac(T, getJacobianSpatialDeriv());
}

//==============================================================================
const math::Jacobian& BodyNode::getJacobianClassicDeriv() const
{
  if(mIsWorldJacobianClassicDerivDirty)
    _updateWorldJacobianClassicDeriv();

  return mWorldJacobianClassicDeriv;
}

//==============================================================================
math::Jacobian BodyNode::getJacobianClassicDeriv(const Frame* _inCoordinatesOf) const
{
  if(_inCoordinatesOf->isWorld())
    return getJacobianClassicDeriv();

  return math::AdRInvJac(_inCoordinatesOf->getWorldTransform(),
                         getJacobianClassicDeriv());
}

//==============================================================================
math::Jacobian BodyNode::getJacobianClassicDeriv(const Eigen::Vector3d& _offset,
                                            const Frame* _inCoordinatesOf) const
{
  math::Jacobian J_d = getJacobianClassicDeriv();
  const math::Jacobian& J = getWorldJacobian();
  const Eigen::Vector3d& w = getAngularVelocity();
  const Eigen::Vector3d& p = (getWorldTransform().linear() * _offset).eval();

  J_d.bottomRows<3>() += J_d.topRows<3>().colwise().cross(p)
                         + J.topRows<3>().colwise().cross(w.cross(p));

  if(_inCoordinatesOf->isWorld())
    return J_d;

  return math::AdRInvJac(_inCoordinatesOf->getWorldTransform(), J_d);
}

//==============================================================================
math::LinearJacobian BodyNode::getLinearJacobianDeriv(
    const Frame* _inCoordinatesOf) const
{
  const math::Jacobian& J_d = getJacobianClassicDeriv();
  if(_inCoordinatesOf->isWorld())
    return J_d.bottomRows<3>();

  return _inCoordinatesOf->getWorldTransform().linear().transpose()
          * J_d.bottomRows<3>();
}

//==============================================================================
math::LinearJacobian BodyNode::getLinearJacobianDeriv(
    const Eigen::Vector3d& _offset, const Frame* _inCoordinatesOf) const
{
  const math::Jacobian& J_d = getJacobianClassicDeriv();
  const math::Jacobian& J = getWorldJacobian();
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
math::AngularJacobian BodyNode::getAngularJacobianDeriv(
    const Frame* _inCoordinatesOf) const
{
  const math::Jacobian& J_d = getJacobianClassicDeriv();

  if(_inCoordinatesOf->isWorld())
    return J_d.topRows<3>();

  return _inCoordinatesOf->getWorldTransform().linear().transpose()
         * J_d.topRows<3>();
}

//==============================================================================
const Eigen::Vector6d& BodyNode::getBodyAcceleration() const
{
  return getSpatialAcceleration();
}

//==============================================================================
Eigen::Vector3d BodyNode::getBodyLinearAcceleration() const
{
  return getSpatialAcceleration().tail<3>();
}

//==============================================================================
Eigen::Vector3d BodyNode::getBodyLinearAcceleration(
    const Eigen::Vector3d& _offset) const
{
  return getSpatialAcceleration(_offset).tail<3>();
}

//==============================================================================
Eigen::Vector3d BodyNode::getBodyAngularAcceleration() const
{
  return getSpatialAcceleration().head<3>();
}

//==============================================================================
Eigen::Vector3d BodyNode::getWorldLinearAcceleration() const
{
  return getSpatialAcceleration(Frame::World(), Frame::World()).tail<3>();
}

//==============================================================================
Eigen::Vector3d BodyNode::getWorldLinearAcceleration(
    const Eigen::Vector3d& _offset) const
{
  return getSpatialAcceleration(_offset, Frame::World(), Frame::World()).tail<3>();
}

//==============================================================================
Eigen::Vector3d BodyNode::getWorldAngularAcceleration() const
{
  return getSpatialAcceleration(Frame::World(), Frame::World()).head<3>();
}

//==============================================================================
const math::Jacobian& BodyNode::getBodyJacobian()
{
  return getJacobian();
}

//==============================================================================
math::LinearJacobian BodyNode::getBodyLinearJacobian()
{
  return getJacobian().bottomRows<3>();
}

//==============================================================================
math::LinearJacobian BodyNode::getBodyLinearJacobian(
    const Eigen::Vector3d& _offset)
{
  return getJacobian(_offset).bottomRows<3>();
}

//==============================================================================
math::AngularJacobian BodyNode::getBodyAngularJacobian()
{
  return getJacobian().topRows<3>();
}

//==============================================================================
math::LinearJacobian BodyNode::getWorldLinearJacobian()
{
  return getJacobian(Frame::World()).bottomRows<3>();
}

//==============================================================================
math::LinearJacobian BodyNode::getWorldLinearJacobian(
    const Eigen::Vector3d& _offset)
{
  return getJacobian(_offset, Frame::World()).bottomRows<3>();
}

//==============================================================================
math::AngularJacobian BodyNode::getWorldAngularJacobian()
{
  return getJacobian(Frame::World()).topRows<3>();
}

//==============================================================================
const math::Jacobian& BodyNode::getBodyJacobianDeriv()
{
  return getJacobianSpatialDeriv();
}

//==============================================================================
math::LinearJacobian BodyNode::getBodyLinearJacobianDeriv()
{
  return getJacobianSpatialDeriv().bottomRows<3>();
}

//==============================================================================
math::LinearJacobian BodyNode::getBodyLinearJacobianDeriv(
    const Eigen::Vector3d& _offset)
{
  return getJacobianSpatialDeriv(_offset).bottomRows<3>();
}

//==============================================================================
math::AngularJacobian BodyNode::getBodyAngularJacobianDeriv()
{
  return getJacobianSpatialDeriv().topRows<3>();
}

//==============================================================================
math::LinearJacobian BodyNode::getWorldLinearJacobianDeriv()
{
  return getJacobianSpatialDeriv(Frame::World()).bottomRows<3>();
}

//==============================================================================
math::LinearJacobian BodyNode::getWorldLinearJacobianDeriv(
    const Eigen::Vector3d& _offset)
{
  return getJacobianSpatialDeriv(_offset, Frame::World()).bottomRows<3>();
}

//==============================================================================
math::AngularJacobian BodyNode::getWorldAngularJacobianDeriv()
{
  return getJacobianSpatialDeriv(Frame::World()).topRows<3>();
}

//==============================================================================
const Eigen::Vector6d& BodyNode::getBodyVelocityChange() const
{
  return mDelV;
}

//==============================================================================
void BodyNode::setColliding(bool _isColliding)
{
  mIsColliding = _isColliding;
}

//==============================================================================
bool BodyNode::isColliding()
{
  return mIsColliding;
}

//==============================================================================
void BodyNode::addExtForce(const Eigen::Vector3d& _force,
                           const Eigen::Vector3d& _offset,
                           bool _isForceLocal,
                           bool _isOffsetLocal)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  Eigen::Vector6d F = Eigen::Vector6d::Zero();
  const Eigen::Isometry3d& W = getWorldTransform();

  if (_isOffsetLocal)
    T.translation() = _offset;
  else
    T.translation() = W.inverse() * _offset;

  if (_isForceLocal)
    F.tail<3>() = _force;
  else
    F.tail<3>() = W.linear().transpose() * _force;

  mFext += math::dAdInvT(T, F);

  if(mSkeleton)
    mSkeleton->mIsExternalForcesDirty = true;
}

//==============================================================================
void BodyNode::setExtForce(const Eigen::Vector3d& _force,
                           const Eigen::Vector3d& _offset,
                           bool _isForceLocal, bool _isOffsetLocal)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  Eigen::Vector6d F = Eigen::Vector6d::Zero();
  const Eigen::Isometry3d& W = getWorldTransform();

  if (_isOffsetLocal)
    T.translation() = _offset;
  else
    T.translation() = W.inverse() * _offset;

  if (_isForceLocal)
    F.tail<3>() = _force;
  else
    F.tail<3>() = W.linear().transpose() * _force;

  mFext = math::dAdInvT(T, F);

  if(mSkeleton)
    mSkeleton->mIsExternalForcesDirty = true;
}

//==============================================================================
void BodyNode::addExtTorque(const Eigen::Vector3d& _torque, bool _isLocal)
{
  if (_isLocal)
    mFext.head<3>() += _torque;
  else
    mFext.head<3>() += getWorldTransform().linear().transpose() * _torque;

  if(mSkeleton)
    mSkeleton->mIsExternalForcesDirty = true;
}

//==============================================================================
void BodyNode::setExtTorque(const Eigen::Vector3d& _torque, bool _isLocal)
{
  if (_isLocal)
    mFext.head<3>() = _torque;
  else
    mFext.head<3>() = getWorldTransform().linear().transpose() * _torque;

  if(mSkeleton)
    mSkeleton->mIsExternalForcesDirty = true;
}

//==============================================================================
void BodyNode::init(Skeleton* _skeleton)
{
  assert(_skeleton);

  mSkeleton = _skeleton;
  mParentJoint->init(_skeleton);

  //--------------------------------------------------------------------------
  // Fill the list of generalized coordinates this node depends on, and sort
  // it.
  //--------------------------------------------------------------------------
  if (mParentBodyNode)
    mDependentGenCoordIndices = mParentBodyNode->mDependentGenCoordIndices;
  else
    mDependentGenCoordIndices.clear();

  for (size_t i = 0; i < mParentJoint->getNumDofs(); i++)
    mDependentGenCoordIndices.push_back(mParentJoint->getIndexInSkeleton(i));

  // Sort
  std::sort(mDependentGenCoordIndices.begin(), mDependentGenCoordIndices.end());

#ifndef NDEBUG
  // Check whether there is duplicated indices.
  int nDepGenCoordIndices = mDependentGenCoordIndices.size();
  for (int i = 0; i < nDepGenCoordIndices - 1; i++)
  {
    for (int j = i + 1; j < nDepGenCoordIndices; j++)
    {
      assert(mDependentGenCoordIndices[i] !=
          mDependentGenCoordIndices[j] &&
          "Duplicated index is found in mDependentGenCoordIndices.");
    }
  }
#endif

  //--------------------------------------------------------------------------
  // Set dimensions of dynamics matrices and vectors.
  //--------------------------------------------------------------------------
  int numDepGenCoords = getNumDependentGenCoords();
  mBodyJacobian.setZero(6, numDepGenCoords);
  mWorldJacobian.setZero(6, numDepGenCoords);
  mBodyJacobianSpatialDeriv.setZero(6, numDepGenCoords);
  mWorldJacobianClassicDeriv.setZero(6, numDepGenCoords);
}

//==============================================================================
void BodyNode::processNewEntity(Entity* _newChildEntity)
{
  // Here we want to sort out whether the Entity that has been added is a child
  // BodyNode or not

  // Check if it's a child BodyNode (if not, then it's just some other arbitrary
  // type of Entity)
  if(find(mChildBodyNodes.begin(), mChildBodyNodes.end(), _newChildEntity) !=
     mChildBodyNodes.end())
    return;
  // TODO: Looks redundant

  // Check if it's already accounted for in our Non-BodyNode Entities
  if(mNonBodyNodeEntities.find(_newChildEntity) != mNonBodyNodeEntities.end())
  {
    dtwarn << "[BodyNode::processNewEntity(~)] Attempting to add an Entity '"
           << _newChildEntity->getName() << "' as a child Entity of '"
           << getName() << "', which is already its parent." << std::endl;
    return;
  }

  // Add it to the Non-BodyNode Entities
  mNonBodyNodeEntities.insert(_newChildEntity);
}

//==============================================================================
void BodyNode::processRemovedEntity(Entity* _oldChildEntity)
{
  std::vector<BodyNode*>::iterator it = find(mChildBodyNodes.begin(),
                                             mChildBodyNodes.end(),
                                             _oldChildEntity);
  if(it != mChildBodyNodes.end())
    mChildBodyNodes.erase(it);

  if(find(mNonBodyNodeEntities.begin(), mNonBodyNodeEntities.end(),
          _oldChildEntity) != mNonBodyNodeEntities.end())
    mNonBodyNodeEntities.erase(_oldChildEntity);
}

//==============================================================================
void BodyNode::drawMarkers(renderer::RenderInterface* _ri,
                           const Eigen::Vector4d& _color,
                           bool _useDefaultColor) const
{
  if (!_ri)
    return;

  _ri->pushMatrix();

  mParentJoint->applyGLTransform(_ri);

  // render the corresponding mMarkerss
  for (unsigned int i = 0; i < mMarkers.size(); i++)
    mMarkers[i]->draw(_ri, true, _color, _useDefaultColor);

  for (unsigned int i = 0; i < mChildBodyNodes.size(); i++)
    mChildBodyNodes[i]->drawMarkers(_ri, _color, _useDefaultColor);

  _ri->popMatrix();
}

//==============================================================================
void BodyNode::notifyTransformUpdate()
{
  notifyVelocityUpdate(); // Global Velocity depends on the Global Transform

  if(mNeedTransformUpdate)
    return;

  mNeedTransformUpdate = true;

  if(mSkeleton)
  {
    mSkeleton->mIsCoriolisForcesDirty = true;
    mSkeleton->mIsGravityForcesDirty = true;
    mSkeleton->mIsCoriolisAndGravityForcesDirty = true;
    mSkeleton->mIsExternalForcesDirty = true;
  }

  // Child BodyNodes and other generic Entities are notified separately to allow
  // some optimizations
  for(size_t i=0; i<mChildBodyNodes.size(); ++i)
    mChildBodyNodes[i]->notifyTransformUpdate();

  for(Entity* entity : mNonBodyNodeEntities)
    entity->notifyTransformUpdate();
}

//==============================================================================
void BodyNode::notifyVelocityUpdate()
{
  notifyAccelerationUpdate(); // Global Acceleration depends on Global Velocity

  if(mNeedVelocityUpdate)
    return;

  mNeedVelocityUpdate = true;
  mIsBodyJacobianSpatialDerivDirty = true;
  mIsWorldJacobianClassicDerivDirty = true;
  mIsPartialAccelerationDirty = true;

  if(mSkeleton)
  {
    mSkeleton->mIsCoriolisForcesDirty = true;
    mSkeleton->mIsCoriolisAndGravityForcesDirty = true;
  }

  // Child BodyNodes and other generic Entities are notified separately to allow
  // some optimizations
  for(size_t i=0; i<mChildBodyNodes.size(); ++i)
    mChildBodyNodes[i]->notifyVelocityUpdate();

  for(Entity* entity : mNonBodyNodeEntities)
    entity->notifyVelocityUpdate();
}

//==============================================================================
void BodyNode::notifyAccelerationUpdate()
{
  // If we already know we need to update, just quit
  if(mNeedAccelerationUpdate)
    return;

  mNeedAccelerationUpdate = true;

  for(size_t i=0; i<mChildBodyNodes.size(); ++i)
    mChildBodyNodes[i]->notifyAccelerationUpdate();

  for(Entity* entity : mNonBodyNodeEntities)
    entity->notifyAccelerationUpdate();
}

//==============================================================================
void BodyNode::updateTransform()
{
  // Calling getWorldTransform will update the transform if an update is needed
  getWorldTransform();
  assert(math::verifyTransform(mWorldTransform));
}

//==============================================================================
void BodyNode::updateVelocity()
{
  // Calling getSpatialVelocity will update the velocity if an update is needed
  getSpatialVelocity();
  assert(!math::isNan(mVelocity));
}

//==============================================================================
void BodyNode::updatePartialAcceleration() const
{
  // Compute partial acceleration
  mParentJoint->setPartialAccelerationTo(mPartialAcceleration,
                                         getSpatialVelocity());
  mIsPartialAccelerationDirty = false;
}

//==============================================================================
void BodyNode::updateAcceleration()
{
  updateAccelerationID();
}

//==============================================================================
void BodyNode::updateAccelerationID()
{
  // Note: auto-updating has replaced this function
  getSpatialAcceleration();
  // Verification
  assert(!math::isNan(mAcceleration));
}

//==============================================================================
void BodyNode::updateBodyWrench(const Eigen::Vector3d& _gravity,
                                bool _withExternalForces)
{
  updateTransmittedForceID(_gravity, _withExternalForces);
}

//==============================================================================
void BodyNode::updateTransmittedForceID(const Eigen::Vector3d& _gravity,
                                        bool _withExternalForces)
{
  // Gravity force
  if (mGravityMode == true)
    mFgravity.noalias() = mI * math::AdInvRLinear(getWorldTransform(),_gravity);
  else
    mFgravity.setZero();

  // Inertial force
  mF.noalias() = mI * getSpatialAcceleration();

  // External force
  if (_withExternalForces)
    mF -= mFext;

  // Verification
  assert(!math::isNan(mF));

  // Gravity force
  mF -= mFgravity;

  // Coriolis force
  const Eigen::Vector6d& V = getSpatialVelocity();
  mF -= math::dad(V, mI * V);

  //
  for (const auto& childBodyNode : mChildBodyNodes)
  {
    Joint* childJoint = childBodyNode->getParentJoint();
    assert(childJoint != NULL);

    mF += math::dAdInvT(childJoint->getLocalTransform(),
                        childBodyNode->getBodyForce());
  }

  // Verification
  assert(!math::isNan(mF));
}

//==============================================================================
void BodyNode::updateGeneralizedForce(bool _withDampingForces)
{
  updateJointForceID(0.001, _withDampingForces, false);
}

//==============================================================================
void BodyNode::updateArtInertia(double _timeStep) const
{
  // Set spatial inertia to the articulated body inertia
  mArtInertia = mI;
  mArtInertiaImplicit = mI;

  // and add child articulated body inertia
  for (const auto& child : mChildBodyNodes)
  {
    Joint* childJoint = child->getParentJoint();

    childJoint->addChildArtInertiaTo(mArtInertia, child->mArtInertia);
    childJoint->addChildArtInertiaImplicitTo(mArtInertiaImplicit,
                                             child->mArtInertiaImplicit);
  }

  // Verification
//  assert(!math::isNan(mArtInertia));
  assert(!math::isNan(mArtInertiaImplicit));

  // Update parent joint's inverse of projected articulated body inertia
  mParentJoint->updateInvProjArtInertia(mArtInertia);
  mParentJoint->updateInvProjArtInertiaImplicit(mArtInertiaImplicit, _timeStep);

  // Verification
//  assert(!math::isNan(mArtInertia));
  assert(!math::isNan(mArtInertiaImplicit));
}

//==============================================================================
void BodyNode::updateBiasForce(const Eigen::Vector3d& _gravity,
                               double _timeStep)
{
  // Gravity force
  if (mGravityMode == true)
    mFgravity.noalias() = mI * math::AdInvRLinear(getWorldTransform(),_gravity);
  else
    mFgravity.setZero();

  // Set bias force
  const Eigen::Vector6d& V = getSpatialVelocity();
  mBiasForce = -math::dad(V, mI * V) - mFext - mFgravity;

  // Verification
  assert(!math::isNan(mBiasForce));

  // And add child bias force
  for (const auto& childBodyNode : mChildBodyNodes)
  {
    Joint* childJoint = childBodyNode->getParentJoint();

    childJoint->addChildBiasForceTo(mBiasForce,
                                childBodyNode->getArticulatedInertiaImplicit(),
                                childBodyNode->mBiasForce,
                                childBodyNode->getPartialAcceleration());
  }

  // Verification
  assert(!math::isNan(mBiasForce));

  // Update parent joint's total force with implicit joint damping and spring
  // forces
  mParentJoint->updateTotalForce( getArticulatedInertiaImplicit()
                                  * getPartialAcceleration() + mBiasForce,
                                  _timeStep);
}

//==============================================================================
void BodyNode::updateBiasImpulse()
{
  // Update impulsive bias force
  mBiasImpulse = -mConstraintImpulse;

  // And add child bias impulse
  for (auto& childBodyNode : mChildBodyNodes)
  {
    Joint* childJoint = childBodyNode->getParentJoint();

    childJoint->addChildBiasImpulseTo(mBiasImpulse,
                                      childBodyNode->getArticulatedInertia(),
                                      childBodyNode->mBiasImpulse);
  }

  // Verification
  assert(!math::isNan(mBiasImpulse));

  // Update parent joint's total force
  mParentJoint->updateTotalImpulse(mBiasImpulse);
}

//==============================================================================
void BodyNode::updateJointAndBodyAcceleration()
{
  updateAccelerationFD();
}

//==============================================================================
void BodyNode::updateJointVelocityChange()
{
  updateVelocityChangeFD();
}

//==============================================================================
void BodyNode::updateTransmittedWrench()
{
  updateTransmittedForceFD();
}

//==============================================================================
void BodyNode::updateTransmittedForceFD()
{
  mF = mBiasForce;
  mF.noalias() += getArticulatedInertiaImplicit() * getSpatialAcceleration();

  assert(!math::isNan(mF));
}

//==============================================================================
void BodyNode::updateBodyImpForceFwdDyn()
{
  updateTransmittedImpulse();
}

//==============================================================================
void BodyNode::updateTransmittedImpulse()
{
  mImpF = mBiasImpulse;
  mImpF.noalias() += getArticulatedInertia() * mDelV;

  assert(!math::isNan(mImpF));
}

//==============================================================================
void BodyNode::updateAccelerationFD()
{
  if (mParentBodyNode)
  {
    // Update joint acceleration
    mParentJoint->updateAcceleration(getArticulatedInertiaImplicit(),
                                     mParentBodyNode->getSpatialAcceleration());
  }
  else
  {
    // Update joint acceleration
    mParentJoint->updateAcceleration(getArticulatedInertiaImplicit(),
                                     Eigen::Vector6d::Zero());
  }

  // Verify the spatial acceleration of this body
  assert(!math::isNan(mAcceleration));
}

//==============================================================================
void BodyNode::updateVelocityChangeFD()
{
  if (mParentBodyNode)
  {
    // Update joint velocity change
    mParentJoint->updateVelocityChange(getArticulatedInertia(),
                                       mParentBodyNode->mDelV);

    // Transmit spatial acceleration of parent body to this body
    mDelV = math::AdInvT(mParentJoint->getLocalTransform(),
                         mParentBodyNode->mDelV);
  }
  else
  {
    // Update joint velocity change
    mParentJoint->updateVelocityChange(getArticulatedInertia(),
                                       Eigen::Vector6d::Zero());

    // Transmit spatial acceleration of parent body to this body
    mDelV.setZero();
  }

  // Add parent joint's acceleration to this body
  mParentJoint->addVelocityChangeTo(mDelV);

  // Verify the spatial velocity change of this body
  assert(!math::isNan(mDelV));
}

//==============================================================================
void BodyNode::updateJointForceID(double _timeStep,
                                  double _withDampingForces,
                                  double _withSpringForces)
{
  assert(mParentJoint != NULL);
  mParentJoint->updateForceID(mF, _timeStep,
                              _withDampingForces, _withSpringForces);
}

//==============================================================================
void BodyNode::updateJointForceFD(double _timeStep,
                                  double _withDampingForces,
                                  double _withSpringForces)
{
  assert(mParentJoint != NULL);
  mParentJoint->updateForceFD(mF, _timeStep,
                              _withDampingForces, _withSpringForces);
}

//==============================================================================
void BodyNode::updateJointImpulseFD()
{
  assert(mParentJoint != NULL);
  mParentJoint->updateImpulseFD(mF);
}

//==============================================================================
void BodyNode::updateConstrainedTerms(double _timeStep)
{
  // 1. dq = dq + del_dq
  // 2. ddq = ddq + del_dq / dt
  // 3. tau = tau + imp / dt
  mParentJoint->updateConstrainedTerms(_timeStep);

  //
  mF += mImpF / _timeStep;
}

//==============================================================================
const Eigen::Vector6d& BodyNode::getExternalForceLocal() const
{
  return mFext;
}

//==============================================================================
Eigen::Vector6d BodyNode::getExternalForceGlobal() const
{
  return math::dAdInvT(getWorldTransform(), mFext);
}

//==============================================================================
void BodyNode::addConstraintImpulse(const Eigen::Vector3d& _constImp,
                                    const Eigen::Vector3d& _offset,
                                    bool _isImpulseLocal,
                                    bool _isOffsetLocal)
{
  // TODO(JS): Add contact sensor data here (DART 4.1)

  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  Eigen::Vector6d F = Eigen::Vector6d::Zero();
  const Eigen::Isometry3d& W = getWorldTransform();

  if (_isOffsetLocal)
    T.translation() = _offset;
  else
    T.translation() = W.inverse() * _offset;

  if (_isImpulseLocal)
    F.tail<3>() = _constImp;
  else
    F.tail<3>() = W.linear().transpose() * _constImp;

  mConstraintImpulse += math::dAdInvT(T, F);
}

//==============================================================================
void BodyNode::clearConstraintImpulse()
{
  mDelV.setZero();
  mBiasImpulse.setZero();
  mConstraintImpulse.setZero();
  mImpF.setZero();

  mParentJoint->resetConstraintImpulses();
  mParentJoint->resetTotalImpulses();
  mParentJoint->resetVelocityChanges();
}

//==============================================================================
const Eigen::Vector6d& BodyNode::getBodyForce() const
{
  return mF;
}

//==============================================================================
void BodyNode::setConstraintImpulse(const Eigen::Vector6d& _constImp)
{
  assert(!math::isNan(_constImp));
  mConstraintImpulse = _constImp;
}

//==============================================================================
void BodyNode::addConstraintImpulse(const Eigen::Vector6d& _constImp)
{
  assert(!math::isNan(_constImp));
  mConstraintImpulse += _constImp;
}

//==============================================================================
const Eigen::Vector6d& BodyNode::getConstraintImpulse() const
{
  return mConstraintImpulse;
}

//==============================================================================
double BodyNode::getKineticEnergy() const
{
  const Eigen::Vector6d& V = getSpatialVelocity();
  return 0.5 * V.dot(mI * V);
}

//==============================================================================
double BodyNode::getPotentialEnergy(const Eigen::Vector3d& _gravity) const
{
  return -mMass * getWorldTransform().translation().dot(_gravity);
}

//==============================================================================
Eigen::Vector3d BodyNode::getLinearMomentum() const
{
  return (mI * getSpatialVelocity()).tail<3>();
}

//==============================================================================
Eigen::Vector3d BodyNode::getAngularMomentum(const Eigen::Vector3d& _pivot)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = _pivot;
  return math::dAdT(T, mI * getSpatialVelocity()).head<3>();
}

//==============================================================================
bool BodyNode::isImpulseReponsible() const
{
  return isReactive();
}

//==============================================================================
bool BodyNode::isReactive() const
{
  if (mSkeleton->isMobile() && getNumDependentGenCoords() > 0)
  {
    // Check if all the ancestor joints are motion prescribed.
    const BodyNode* body = this;
    while (body != NULL)
    {
      if (body->mParentJoint->isDynamic())
        return true;

      body = body->mParentBodyNode;
    }
    // TODO: Checking if all the ancestor joints are motion prescribed is
    // expensive. It would be good to evaluate this in advance and update only
    // when necessary.

    return false;
  }
  else
  {
    return false;
  }
}

//==============================================================================
void BodyNode::updateConstrainedJointAndBodyAcceleration(double /*_timeStep*/)
{
  // 1. dq = dq + del_dq
  // mParentJoint->updateVelocityWithVelocityChange();

  // 2. ddq = ddq + del_dq / dt
  // mParentJoint->updateAccelerationWithVelocityChange(_timeStep);

  // 3. tau = tau + imp / dt
  // mParentJoint->updateForceWithImpulse(_timeStep);
}

//==============================================================================
void BodyNode::updateConstrainedTransmittedForce(double _timeStep)
{
  ///
  mAcceleration += mDelV / _timeStep;

  ///
  mF += mImpF / _timeStep;
}

//==============================================================================
void BodyNode::aggregateCoriolisForceVector(Eigen::VectorXd* _C)
{
  aggregateCombinedVector(_C, Eigen::Vector3d::Zero());
}

//==============================================================================
void BodyNode::aggregateGravityForceVector(Eigen::VectorXd* _g,
                                           const Eigen::Vector3d& _gravity)
{
  if (mGravityMode == true)
    mG_F = mI * math::AdInvRLinear(getWorldTransform(), _gravity);
  else
    mG_F.setZero();

  for (std::vector<BodyNode*>::const_iterator it = mChildBodyNodes.begin();
       it != mChildBodyNodes.end(); ++it)
  {
    mG_F += math::dAdInvT((*it)->mParentJoint->getLocalTransform(),
                          (*it)->mG_F);
  }

  int nGenCoords = mParentJoint->getNumDofs();
  if (nGenCoords > 0)
  {
    Eigen::VectorXd g = -(mParentJoint->getLocalJacobian().transpose() * mG_F);
    int iStart = mParentJoint->getIndexInSkeleton(0);
    _g->segment(iStart, nGenCoords) = g;
  }
}

//==============================================================================
void BodyNode::updateCombinedVector()
{
  if (mParentBodyNode)
  {
    mCg_dV = math::AdInvT(mParentJoint->getLocalTransform(),
                          mParentBodyNode->mCg_dV) + getPartialAcceleration();
  }
  else
  {
    mCg_dV = getPartialAcceleration();
  }
}

//==============================================================================
void BodyNode::aggregateCombinedVector(Eigen::VectorXd* _Cg,
                                       const Eigen::Vector3d& _gravity)
{
  // H(i) = I(i) * W(i) -
  //        dad{V}(I(i) * V(i)) + sum(k \in children) dAd_{T(i,j)^{-1}}(H(k))
  if (mGravityMode == true)
    mFgravity = mI * math::AdInvRLinear(getWorldTransform(), _gravity);
  else
    mFgravity.setZero();

  const Eigen::Vector6d& V = getSpatialVelocity();
  mCg_F = mI * mCg_dV;
  mCg_F -= mFgravity;
  mCg_F -= math::dad(V, mI * V);

  for (std::vector<BodyNode*>::iterator it = mChildBodyNodes.begin();
       it != mChildBodyNodes.end(); ++it)
  {
    mCg_F += math::dAdInvT((*it)->getParentJoint()->mT, (*it)->mCg_F);
  }

  int nGenCoords = mParentJoint->getNumDofs();
  if (nGenCoords > 0)
  {
    Eigen::VectorXd Cg
        = mParentJoint->getLocalJacobian().transpose() * mCg_F;
    int iStart = mParentJoint->getIndexInSkeleton(0);
    _Cg->segment(iStart, nGenCoords) = Cg;
  }
}

//==============================================================================
void BodyNode::aggregateExternalForces(Eigen::VectorXd* _Fext)
{
  mFext_F = mFext;

  for (std::vector<BodyNode*>::const_iterator it = mChildBodyNodes.begin();
       it != mChildBodyNodes.end(); ++it)
  {
    mFext_F += math::dAdInvT((*it)->mParentJoint->getLocalTransform(),
                             (*it)->mFext_F);
  }

  int nGenCoords = mParentJoint->getNumDofs();
  if (nGenCoords > 0)
  {
    Eigen::VectorXd Fext = mParentJoint->getLocalJacobian().transpose()*mFext_F;
    int iStart = mParentJoint->getIndexInSkeleton(0);
    _Fext->segment(iStart, nGenCoords) = Fext;
  }
}

//==============================================================================
void BodyNode::aggregateSpatialToGeneralized(Eigen::VectorXd* _generalized,
                                             const Eigen::Vector6d& _spatial)
{
  //
  mArbitrarySpatial = _spatial;

  //
  for (std::vector<BodyNode*>::const_iterator it = mChildBodyNodes.begin();
       it != mChildBodyNodes.end(); ++it)
  {
    mArbitrarySpatial += math::dAdInvT((*it)->mParentJoint->getLocalTransform(),
                                       (*it)->mArbitrarySpatial);
  }

  // Project the spatial quantity to generalized coordinates
  size_t iStart = mParentJoint->getIndexInSkeleton(0);
  _generalized->segment(iStart, mParentJoint->getNumDofs())
      = mParentJoint->getSpatialToGeneralized(mArbitrarySpatial);
}

//==============================================================================
void BodyNode::updateMassMatrix()
{
  mM_dV.setZero();
  int dof = mParentJoint->getNumDofs();
  if (dof > 0)
  {
    mM_dV.noalias() += mParentJoint->getLocalJacobian()
                       * mParentJoint->getAccelerations();
    assert(!math::isNan(mM_dV));
  }
  if (mParentBodyNode)
    mM_dV += math::AdInvT(mParentJoint->getLocalTransform(),
                          mParentBodyNode->mM_dV);
  assert(!math::isNan(mM_dV));
}

//==============================================================================
void BodyNode::aggregateMassMatrix(Eigen::MatrixXd* _MCol, int _col)
{
  //
  mM_F.noalias() = mI * mM_dV;

  // Verification
  assert(!math::isNan(mM_F));

  //
  for (std::vector<BodyNode*>::const_iterator it = mChildBodyNodes.begin();
       it != mChildBodyNodes.end(); ++it)
  {
    mM_F += math::dAdInvT((*it)->getParentJoint()->getLocalTransform(),
                          (*it)->mM_F);
  }

  // Verification
  assert(!math::isNan(mM_F));

  //
  int dof = mParentJoint->getNumDofs();
  if (dof > 0)
  {
    int iStart = mParentJoint->getIndexInSkeleton(0);
    _MCol->block(iStart, _col, dof, 1).noalias() =
        mParentJoint->getLocalJacobian().transpose() * mM_F;
  }
}

//==============================================================================
void BodyNode::aggregateAugMassMatrix(Eigen::MatrixXd* _MCol, int _col,
                                      double _timeStep)
{
  // TODO(JS): Need to be reimplemented

  //
  mM_F.noalias() = mI * mM_dV;

  // Verification
  assert(!math::isNan(mM_F));

  //
  for (std::vector<BodyNode*>::const_iterator it = mChildBodyNodes.begin();
       it != mChildBodyNodes.end(); ++it)
  {
    mM_F += math::dAdInvT((*it)->getParentJoint()->getLocalTransform(),
                          (*it)->mM_F);
  }

  // Verification
  assert(!math::isNan(mM_F));

  //
  int dof = mParentJoint->getNumDofs();
  if (dof > 0)
  {
    Eigen::MatrixXd K = Eigen::MatrixXd::Zero(dof, dof);
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(dof, dof);
    for (int i = 0; i < dof; ++i)
    {
      K(i, i) = mParentJoint->getSpringStiffness(i);
      D(i, i) = mParentJoint->getDampingCoefficient(i);
    }

    int iStart = mParentJoint->getIndexInSkeleton(0);

    _MCol->block(iStart, _col, dof, 1).noalias()
        = mParentJoint->getLocalJacobian().transpose() * mM_F
          + D * (_timeStep * mParentJoint->getAccelerations())
          + K * (_timeStep * _timeStep * mParentJoint->getAccelerations());
  }
}

//==============================================================================
void BodyNode::updateInvMassMatrix()
{
  //
  mInvM_c.setZero();

  //
  for (std::vector<BodyNode*>::const_iterator it = mChildBodyNodes.begin();
       it != mChildBodyNodes.end(); ++it)
  {
    (*it)->getParentJoint()->addChildBiasForceForInvMassMatrix(
          mInvM_c, (*it)->getArticulatedInertia(), (*it)->mInvM_c);
  }

  // Verification
  assert(!math::isNan(mInvM_c));

  // Update parent joint's total force for inverse mass matrix
  mParentJoint->updateTotalForceForInvMassMatrix(mInvM_c);
}

//==============================================================================
void BodyNode::updateInvAugMassMatrix()
{
  //
  mInvM_c.setZero();

  //
  for (std::vector<BodyNode*>::const_iterator it = mChildBodyNodes.begin();
       it != mChildBodyNodes.end(); ++it)
  {
    (*it)->getParentJoint()->addChildBiasForceForInvAugMassMatrix(
          mInvM_c, (*it)->getArticulatedInertiaImplicit(), (*it)->mInvM_c);
  }

  // Verification
  assert(!math::isNan(mInvM_c));

  // Update parent joint's total force for inverse mass matrix
  mParentJoint->updateTotalForceForInvMassMatrix(mInvM_c);
}

//==============================================================================
void BodyNode::aggregateInvMassMatrix(Eigen::MatrixXd* _InvMCol, int _col)
{
  if (mParentBodyNode)
  {
    //
    mParentJoint->getInvMassMatrixSegment(
          *_InvMCol, _col, getArticulatedInertia(), mParentBodyNode->mInvM_U);

    //
    mInvM_U = math::AdInvT(mParentJoint->getLocalTransform(),
                           mParentBodyNode->mInvM_U);
  }
  else
  {
    //
    mParentJoint->getInvMassMatrixSegment(
          *_InvMCol, _col, getArticulatedInertia(), Eigen::Vector6d::Zero());

    //
    mInvM_U.setZero();
  }

  //
  mParentJoint->addInvMassMatrixSegmentTo(mInvM_U);
}

//==============================================================================
void BodyNode::aggregateInvAugMassMatrix(Eigen::MatrixXd* _InvMCol, int _col,
                                         double /*_timeStep*/)
{
  if (mParentBodyNode)
  {
    //
    mParentJoint->getInvAugMassMatrixSegment(
          *_InvMCol, _col, getArticulatedInertiaImplicit(),
          mParentBodyNode->mInvM_U);

    //
    mInvM_U = math::AdInvT(mParentJoint->getLocalTransform(),
                           mParentBodyNode->mInvM_U);
  }
  else
  {
    //
    mParentJoint->getInvAugMassMatrixSegment(
          *_InvMCol, _col, getArticulatedInertiaImplicit(),
          Eigen::Vector6d::Zero());

    //
    mInvM_U.setZero();
  }

  //
  mParentJoint->addInvMassMatrixSegmentTo(mInvM_U);
}

//==============================================================================
void BodyNode::_updateBodyJacobian() const
{
  //--------------------------------------------------------------------------
  // Jacobian update
  //
  // J = | J1 J2 ... Jn |
  //   = | Ad(T(i,i-1), J_parent) J_local |
  //
  //   J_parent: (6 x parentDOF)
  //    J_local: (6 x localDOF)
  //         Ji: (6 x 1) se3
  //          n: number of dependent coordinates
  //--------------------------------------------------------------------------

  if(NULL == mParentJoint)
    return;

  const int localDof     = mParentJoint->getNumDofs();
  const int ascendantDof = getNumDependentGenCoords() - localDof;

  // Parent Jacobian
  if (mParentBodyNode)
  {
    assert(static_cast<size_t>(mParentBodyNode->getJacobian().cols())
           + mParentJoint->getNumDofs()
           == static_cast<size_t>(mBodyJacobian.cols()));

    assert(mParentJoint);
    mBodyJacobian.leftCols(ascendantDof) =
        math::AdInvTJac(mParentJoint->getLocalTransform(),
                        mParentBodyNode->getJacobian());
  }

  // Local Jacobian
  mBodyJacobian.rightCols(localDof) = mParentJoint->getLocalJacobian();

  mIsBodyJacobianDirty = false;
}

//==============================================================================
void BodyNode::_updateWorldJacobian() const
{
  mWorldJacobian = math::AdRJac(getWorldTransform(), getJacobian());

  mIsWorldJacobianDirty = false;
}

//==============================================================================
void BodyNode::_updateBodyJacobianSpatialDeriv() const
{
  //--------------------------------------------------------------------------
  // Body Jacobian first spatial derivative update
  //
  // dJ = | dJ1 dJ2 ... dJn |
  //   = | Ad(T(i,i-1), dJ_parent) dJ_local |
  //
  //   dJ_parent: (6 x parentDOF)
  //    dJ_local: (6 x localDOF)
  //         dJi: (6 x 1) se3
  //          n: number of dependent coordinates
  //--------------------------------------------------------------------------

  if(NULL == mParentJoint)
    return;

  const size_t numLocalDOFs = mParentJoint->getNumDofs();
  const size_t numParentDOFs = getNumDependentGenCoords() - numLocalDOFs;

  // Parent Jacobian
  if (mParentBodyNode)
  {
    const math::Jacobian& dJ_parent = mParentBodyNode->getJacobianSpatialDeriv();

    assert(static_cast<size_t>(dJ_parent.cols()) + mParentJoint->getNumDofs()
           == static_cast<size_t>(mBodyJacobianSpatialDeriv.cols()));

    mBodyJacobianSpatialDeriv.leftCols(numParentDOFs)
        = math::AdInvTJac(mParentJoint->getLocalTransform(), dJ_parent);

    mBodyJacobianSpatialDeriv -= math::adJac(getSpatialVelocity(),
                                             getJacobian());
  }

  // Local Jacobian
  mBodyJacobianSpatialDeriv.rightCols(numLocalDOFs)
      = mParentJoint->getLocalJacobianTimeDeriv();

  mIsBodyJacobianSpatialDerivDirty = false;
}

//==============================================================================
void BodyNode::_updateWorldJacobianClassicDeriv() const
{
  //----------------------------------------------------------------------------
  // World Jacobian first classic deriv update
  //
  // dJr = |                   dJr_parent                                           dJr_local - Jr_local x w |
  //
  // dJl = | dJl_parent + Jr_parent x (v_local + w_parent x p) + dJr_parent x p     dJl_local - Jl_local x w |
  //
  // dJr: Rotational portion of Jacobian derivative
  // dJl: Linear portion of Jacobian derivative
  // dJr_parent: Parent rotational Jacobian derivative
  // dJl_parent: Parent linear Jacobian derivative
  // dJr_local: Local rotational Jacobian derivative (in World coordinates)
  // dJl_local: Local linear Jacobian derivative (in World coordinates)
  // v_local: Linear velocity relative to parent Frame
  // w_parent: Total angular velocity of the parent Frame
  // w: Total angular velocity of this Frame
  // p: Offset from origin of parent Frame

  if(NULL == mParentJoint)
    return;

  const size_t numLocalDOFs = mParentJoint->getNumDofs();
  const size_t numParentDOFs = getNumDependentGenCoords() - numLocalDOFs;


  if(mParentBodyNode)
  {
    const math::Jacobian& dJ_parent = mParentBodyNode->getJacobianClassicDeriv();
    const math::Jacobian& J_parent = mParentBodyNode->getWorldJacobian();

    const Eigen::Vector3d& v_local = getLinearVelocity(mParentBodyNode,
                                                       Frame::World());
    const Eigen::Vector3d& w_parent = mParentFrame->getAngularVelocity();
    const Eigen::Vector3d& p = (getWorldTransform().translation()
                  - mParentBodyNode->getWorldTransform().translation()).eval();

    assert(static_cast<size_t>(dJ_parent.cols()) + mParentJoint->getNumDofs()
           == static_cast<size_t>(mWorldJacobianClassicDeriv.cols()));

    // dJr
    mWorldJacobianClassicDeriv.block(0,0,3,numParentDOFs)
        = dJ_parent.topRows<3>();
    mWorldJacobianClassicDeriv.block(3,0,3,numParentDOFs)
        = dJ_parent.bottomRows<3>()
          + J_parent.topRows<3>().colwise().cross(v_local + w_parent.cross(p))
          + dJ_parent.topRows<3>().colwise().cross(p);
  }

  const math::Jacobian& dJ_local = mParentJoint->getLocalJacobianTimeDeriv();
  const math::Jacobian& J_local = mParentJoint->getLocalJacobian();
  const Eigen::Isometry3d& T = getWorldTransform();
  const Eigen::Vector3d& w = getAngularVelocity();

  mWorldJacobianClassicDeriv.block(0,numParentDOFs,3,numLocalDOFs)
      = T.linear()*dJ_local.topRows<3>()
        - (T.linear()*J_local.topRows<3>()).colwise().cross(w);

  mWorldJacobianClassicDeriv.block(3,numParentDOFs,3,numLocalDOFs)
      = T.linear()*dJ_local.bottomRows<3>()
        - (T.linear()*J_local.bottomRows<3>()).colwise().cross(w);

  mIsWorldJacobianClassicDerivDirty = false;
}

//==============================================================================
void BodyNode::_updateSpatialInertia()
{
  // G = | I - m*[r]*[r]   m*[r] |
  //     |        -m*[r]     m*I |

  // m*r
  double mr0 = mMass * mCenterOfMass[0];
  double mr1 = mMass * mCenterOfMass[1];
  double mr2 = mMass * mCenterOfMass[2];

  // m*[r]*[r]
  double mr0r0 = mr0 * mCenterOfMass[0];
  double mr1r1 = mr1 * mCenterOfMass[1];
  double mr2r2 = mr2 * mCenterOfMass[2];
  double mr0r1 = mr0 * mCenterOfMass[1];
  double mr1r2 = mr1 * mCenterOfMass[2];
  double mr2r0 = mr2 * mCenterOfMass[0];

  // Top left corner (3x3)
  mI(0, 0) =  mIxx + mr1r1 + mr2r2;
  mI(1, 1) =  mIyy + mr2r2 + mr0r0;
  mI(2, 2) =  mIzz + mr0r0 + mr1r1;
  mI(0, 1) =  mIxy - mr0r1;
  mI(0, 2) =  mIxz - mr2r0;
  mI(1, 2) =  mIyz - mr1r2;

  // Top right corner (3x3)
  mI(1, 5) = -mr0;
  mI(0, 5) =  mr1;
  mI(0, 4) = -mr2;
  mI(2, 4) =  mr0;
  mI(2, 3) = -mr1;
  mI(1, 3) =  mr2;
  assert(mI(0, 3) == 0.0);
  assert(mI(1, 4) == 0.0);
  assert(mI(2, 5) == 0.0);

  // Bottom right corner (3x3)
  mI(3, 3) =  mMass;
  mI(4, 4) =  mMass;
  mI(5, 5) =  mMass;
  assert(mI(3, 4) == 0.0);
  assert(mI(3, 5) == 0.0);
  assert(mI(4, 5) == 0.0);

  mI.triangularView<Eigen::StrictlyLower>() = mI.transpose();

  if(mSkeleton)
    mSkeleton->notifyArticulatedInertiaUpdate();
}

//==============================================================================
void BodyNode::clearExternalForces()
{
  mFext.setZero();
  if(mSkeleton)
    mSkeleton->mIsExternalForcesDirty = true;
}

}  // namespace dynamics
}  // namespace dart
