/*
 * Copyright (c) 2013-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2013-2016, Humanoid Lab, Georgia Tech Research Corporation
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

#include "dart/dynamics/SoftBodyNode.hpp"

#include <map>
#include <string>
#include <vector>

#include "dart/common/Console.hpp"
#include "dart/math/Helpers.hpp"
#include "dart/dynamics/Joint.hpp"
#include "dart/dynamics/Shape.hpp"
#include "dart/dynamics/Skeleton.hpp"

#include "dart/dynamics/PointMass.hpp"
#include "dart/dynamics/SoftMeshShape.hpp"

namespace dart {
namespace dynamics {

namespace detail {

//==============================================================================
SoftBodyNodeUniqueProperties::SoftBodyNodeUniqueProperties(
    double _Kv, double _Ke, double _DampCoeff,
    const std::vector<PointMass::Properties>& _points,
    const std::vector<Eigen::Vector3i>& _faces)
  : mKv(_Kv),
    mKe(_Ke),
    mDampCoeff(_DampCoeff),
    mPointProps(_points),
    mFaces(_faces)
{
  // Do nothing
}

//==============================================================================
void SoftBodyNodeUniqueProperties::addPointMass(
    const PointMass::Properties& _properties)
{
  mPointProps.push_back(_properties);
}

//==============================================================================
bool SoftBodyNodeUniqueProperties::connectPointMasses(std::size_t i1, std::size_t i2)
{
  if(i1 >= mPointProps.size() || i2 >= mPointProps.size())
  {
    if(mPointProps.size() == 0)
      dtwarn << "[SoftBodyNode::Properties::addConnection] Attempting to "
             << "add a connection between indices " << i1 << " and " << i2
             << ", but there are currently no entries in mPointProps!\n";
    else
      dtwarn << "[SoftBodyNode::Properties::addConnection] Attempting to "
             << "add a connection between indices " << i1 << " and " << i2
             << ", but the entries in mPointProps only go up to "
             << mPointProps.size()-1 << "!\n";
    return false;
  }

  mPointProps[i1].mConnectedPointMassIndices.push_back(i2);
  mPointProps[i2].mConnectedPointMassIndices.push_back(i1);

  return true;
}

//==============================================================================
void SoftBodyNodeUniqueProperties::addFace(const Eigen::Vector3i& _newFace)
{
  assert(_newFace[0] != _newFace[1]);
  assert(_newFace[1] != _newFace[2]);
  assert(_newFace[2] != _newFace[0]);
  assert(0 <= _newFace[0] && static_cast<std::size_t>(_newFace[0]) < mPointProps.size());
  assert(0 <= _newFace[1] && static_cast<std::size_t>(_newFace[1]) < mPointProps.size());
  assert(0 <= _newFace[2] && static_cast<std::size_t>(_newFace[2]) < mPointProps.size());
  mFaces.push_back(_newFace);
}

//==============================================================================
SoftBodyNodeProperties::SoftBodyNodeProperties(
    const BodyNode::Properties& _bodyProperties,
    const SoftBodyNodeUniqueProperties& _softProperties)
  : BodyNode::Properties(_bodyProperties),
    SoftBodyNodeUniqueProperties(_softProperties)
{
  // Do nothing
}

} // namespace detail

//==============================================================================
SoftBodyNode::~SoftBodyNode()
{
  for (std::size_t i = 0; i < mPointMasses.size(); ++i)
    delete mPointMasses[i];

  delete mNotifier;
}

//==============================================================================
SoftBodyNode* SoftBodyNode::asSoftBodyNode()
{
  return this;
}

//==============================================================================
const SoftBodyNode* SoftBodyNode::asSoftBodyNode() const
{
  return this;
}

//==============================================================================
void SoftBodyNode::setProperties(const Properties& _properties)
{
  BodyNode::setProperties(
        static_cast<const BodyNode::Properties&>(_properties));

  setProperties(static_cast<const UniqueProperties&>(_properties));
}

//==============================================================================
void SoftBodyNode::setProperties(const UniqueProperties& _properties)
{
  setAspectProperties(_properties);
}

//==============================================================================
void SoftBodyNode::setAspectState(const AspectState& state)
{
  if(mAspectState.mPointStates == state.mPointStates)
    return;

  mAspectState = state;
  mNotifier->notifyTransformUpdate();
}

//==============================================================================
void SoftBodyNode::setAspectProperties(const AspectProperties& properties)
{
  setVertexSpringStiffness(properties.mKv);
  setEdgeSpringStiffness(properties.mKe);
  setDampingCoefficient(properties.mDampCoeff);

  if(properties.mPointProps != mAspectProperties.mPointProps
     || properties.mFaces != mAspectProperties.mFaces)
  {
    mAspectProperties.mPointProps = properties.mPointProps;
    mAspectProperties.mFaces = properties.mFaces;
    configurePointMasses(mSoftShapeNode.lock());
  }
}

//==============================================================================
SoftBodyNode::Properties SoftBodyNode::getSoftBodyNodeProperties() const
{
  return Properties(getBodyNodeProperties(), mAspectProperties);
}

//==============================================================================
void SoftBodyNode::copy(const SoftBodyNode& _otherSoftBodyNode)
{
  if(this == &_otherSoftBodyNode)
    return;

  setProperties(_otherSoftBodyNode.getSoftBodyNodeProperties());
}

//==============================================================================
void SoftBodyNode::copy(const SoftBodyNode* _otherSoftBodyNode)
{
  if(nullptr == _otherSoftBodyNode)
    return;

  copy(*_otherSoftBodyNode);
}

//==============================================================================
SoftBodyNode& SoftBodyNode::operator=(const SoftBodyNode& _otherSoftBodyNode)
{
  copy(_otherSoftBodyNode);
  return *this;
}

//==============================================================================
std::size_t SoftBodyNode::getNumPointMasses() const
{
  return mPointMasses.size();
}

//==============================================================================
PointMass* SoftBodyNode::getPointMass(std::size_t _idx)
{
  assert(_idx < mPointMasses.size());
  if(_idx < mPointMasses.size())
    return mPointMasses[_idx];

  return nullptr;
}

//==============================================================================
const PointMass* SoftBodyNode::getPointMass(std::size_t _idx) const
{
  return const_cast<SoftBodyNode*>(this)->getPointMass(_idx);
}

//==============================================================================
const std::vector<PointMass*>& SoftBodyNode::getPointMasses() const
{
  return mPointMasses;
}

//==============================================================================
SoftBodyNode::SoftBodyNode(BodyNode* _parentBodyNode,
                           Joint* _parentJoint,
                           const Properties& _properties)
  : Entity(Frame::World(), false),
    Frame(Frame::World()),
    Base(std::make_tuple(_parentBodyNode, _parentJoint, _properties)),
    mSoftShapeNode(nullptr)
{
  createSoftBodyAspect();
  mNotifier = new PointMassNotifier(this, getName()+"_PointMassNotifier");
  ShapeNode* softNode = createShapeNodeWith<
      VisualAspect, CollisionAspect, DynamicsAspect>(
        std::make_shared<SoftMeshShape>(this), getName()+"_SoftMeshShape");
  mSoftShapeNode = softNode;

  // Dev's Note: We do this workaround (instead of just using setProperties(~))
  // because mSoftShapeNode cannot be used until init(SkeletonPtr) has been
  // called on this BodyNode, but that happens after construction is finished.
  mAspectProperties = _properties;
  configurePointMasses(softNode);
  mNotifier->notifyTransformUpdate();
}

//==============================================================================
BodyNode* SoftBodyNode::clone(BodyNode* _parentBodyNode,
                              Joint* _parentJoint, bool cloneNodes) const
{
  SoftBodyNode* clonedBn = new SoftBodyNode(
        _parentBodyNode, _parentJoint, getSoftBodyNodeProperties());

  clonedBn->matchAspects(this);

  if(cloneNodes)
    clonedBn->matchNodes(this);

  return clonedBn;
}

//==============================================================================
void SoftBodyNode::configurePointMasses(ShapeNode* softNode)
{
  const UniqueProperties& softProperties = mAspectProperties;

  std::size_t newCount = softProperties.mPointProps.size();
  std::size_t oldCount = mPointMasses.size();

  if(newCount == oldCount)
    return;

  // Adjust the number of PointMass objects since that has changed
  if(newCount < oldCount)
  {
    for(std::size_t i = newCount; i < oldCount; ++i)
      delete mPointMasses[i];
    mPointMasses.resize(newCount);
  }
  else if(oldCount < newCount)
  {
    mPointMasses.resize(newCount);
    for(std::size_t i = oldCount; i < newCount; ++i)
    {
      mPointMasses[i] = new PointMass(this);
      mPointMasses[i]->mIndex = i;
      mPointMasses[i]->init();
    }
  }

  // Resize the number of States in the Aspect
  mAspectState.mPointStates.resize(
        softProperties.mPointProps.size(), PointMass::State());

  // Access the SoftMeshShape and reallocate its meshes
  if(softNode)
  {
    std::shared_ptr<SoftMeshShape> softShape =
        std::dynamic_pointer_cast<SoftMeshShape>(softNode->getShape());

    if(softShape)
      softShape->_buildMesh();
  }
  else
  {
    dtwarn << "[SoftBodyNode::configurePointMasses] The ShapeNode containing "
           << "the SoftMeshShape for the SoftBodyNode named [" << getName()
           << "] (" << this << ") has been removed. The soft body features for "
           << "this SoftBodyNode cannot be used unless you recreate the "
           << "SoftMeshShape.\n";

    std::cout << "ShapeNodes: " << std::endl;
    for(std::size_t i=0; i < getNumShapeNodes(); ++i)
    {
      std::cout << "- " << i << ") " << getShapeNode(i)->getName() << std::endl;
    }
  }

  incrementVersion();
  mNotifier->notifyTransformUpdate();
}

//==============================================================================
void SoftBodyNode::init(const SkeletonPtr& _skeleton)
{
  BodyNode::init(_skeleton);

  for (std::size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses[i]->init();

//  //----------------------------------------------------------------------------
//  // Visualization shape
//  //----------------------------------------------------------------------------
//  assert(mSoftVisualShape == nullptr);
//  mSoftVisualShape = new SoftMeshShape(this);
//  BodyNode::addVisualizationShape(mSoftVisualShape);

//  //----------------------------------------------------------------------------
//  // Collision shape
//  //----------------------------------------------------------------------------
//  assert(mSoftCollShape == nullptr);
//  mSoftCollShape = new SoftMeshShape(this);
//  BodyNode::addCollisionShape(mSoftCollShape);
}

//==============================================================================
//void SoftBodyNode::aggregateGenCoords(std::vector<GenCoord*>* _genCoords)
//{
//  BodyNode::aggregateGenCoords(_genCoords);
//  aggregatePointMassGenCoords(_genCoords);
//}

//==============================================================================
//void SoftBodyNode::aggregatePointMassGenCoords(
//    std::vector<GenCoord*>* _genCoords)
//{
//  for (std::size_t i = 0; i < getNumPointMasses(); ++i)
//  {
//    PointMass* pointMass = getPointMass(i);
//    for (int j = 0; j < pointMass->getNumDofs(); ++j)
//    {
//      GenCoord* genCoord = pointMass->getGenCoord(j);
//      genCoord->setSkeletonIndex(_genCoords->size());
//      _genCoords->push_back(genCoord);
//    }
//  }
//}

//==============================================================================
PointMassNotifier* SoftBodyNode::getNotifier()
{
  return mNotifier;
}

//==============================================================================
const PointMassNotifier* SoftBodyNode::getNotifier() const
{
  return mNotifier;
}

//==============================================================================
double SoftBodyNode::getMass() const
{
  double totalMass = BodyNode::getMass();

  for (std::size_t i = 0; i < mPointMasses.size(); ++i)
    totalMass += mPointMasses.at(i)->getMass();

  return totalMass;
}

//==============================================================================
void SoftBodyNode::setVertexSpringStiffness(double _kv)
{
  assert(0.0 <= _kv);

  if(_kv == mAspectProperties.mKv)
    return;

  mAspectProperties.mKv = _kv;
  incrementVersion();
}

//==============================================================================
double SoftBodyNode::getVertexSpringStiffness() const
{
  return mAspectProperties.mKv;
}

//==============================================================================
void SoftBodyNode::setEdgeSpringStiffness(double _ke)
{
  assert(0.0 <= _ke);

  if(_ke == mAspectProperties.mKe)
    return;

  mAspectProperties.mKe = _ke;
  incrementVersion();
}

//==============================================================================
double SoftBodyNode::getEdgeSpringStiffness() const
{
  return mAspectProperties.mKe;
}

//==============================================================================
void SoftBodyNode::setDampingCoefficient(double _damp)
{
  assert(_damp >= 0.0);

  if(_damp == mAspectProperties.mDampCoeff)
    return;

  mAspectProperties.mDampCoeff = _damp;
  incrementVersion();
}

//==============================================================================
double SoftBodyNode::getDampingCoefficient() const
{
  return mAspectProperties.mDampCoeff;
}

//==============================================================================
void SoftBodyNode::removeAllPointMasses()
{
  mPointMasses.clear();
  mAspectProperties.mPointProps.clear();
  mAspectProperties.mFaces.clear();
  configurePointMasses(mSoftShapeNode.lock());
}

//==============================================================================
PointMass* SoftBodyNode::addPointMass(const PointMass::Properties& _properties)
{
  mPointMasses.push_back(new PointMass(this));
  mPointMasses.back()->mIndex = mPointMasses.size()-1;
  mAspectProperties.mPointProps.push_back(_properties);
  configurePointMasses(mSoftShapeNode.lock());

  return mPointMasses.back();
}

//==============================================================================
void SoftBodyNode::connectPointMasses(std::size_t _idx1, std::size_t _idx2)
{
  assert(_idx1 != _idx2);
  assert(_idx1 < mPointMasses.size());
  assert(_idx2 < mPointMasses.size());
  mPointMasses[_idx1]->addConnectedPointMass(mPointMasses[_idx2]);
  mPointMasses[_idx2]->addConnectedPointMass(mPointMasses[_idx1]);
  // Version incremented in addConnectedPointMass
}

//==============================================================================
void SoftBodyNode::addFace(const Eigen::Vector3i& _face)
{
  mAspectProperties.addFace(_face);
  configurePointMasses(mSoftShapeNode.lock());
}

//==============================================================================
const Eigen::Vector3i& SoftBodyNode::getFace(std::size_t _idx) const
{
  assert(_idx < mAspectProperties.mFaces.size());
  return mAspectProperties.mFaces[_idx];
}

//==============================================================================
std::size_t SoftBodyNode::getNumFaces() const
{
  return mAspectProperties.mFaces.size();
}

//==============================================================================
void SoftBodyNode::clearConstraintImpulse()
{
  BodyNode::clearConstraintImpulse();

  for (std::size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->clearConstraintImpulse();
}

//==============================================================================
void SoftBodyNode::checkArticulatedInertiaUpdate() const
{
  ConstSkeletonPtr skel = getSkeleton();
  if(skel && skel->mTreeCache[mTreeIndex].mDirty.mArticulatedInertia)
    skel->updateArticulatedInertia(mTreeIndex);
}

//==============================================================================
void SoftBodyNode::updateTransform()
{
  BodyNode::updateTransform();

  for (std::size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->updateTransform();

  mNotifier->clearTransformNotice();
}

//==============================================================================
void SoftBodyNode::updateVelocity()
{
  BodyNode::updateVelocity();

  for (std::size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->updateVelocity();

  mNotifier->clearVelocityNotice();
}

//==============================================================================
void SoftBodyNode::updatePartialAcceleration() const
{
  BodyNode::updatePartialAcceleration();

  for (std::size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->updatePartialAcceleration();

  mNotifier->clearPartialAccelerationNotice();
}

//==============================================================================
void SoftBodyNode::updateAccelerationID()
{
  BodyNode::updateAccelerationID();

  for (std::size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->updateAccelerationID();

  mNotifier->clearAccelerationNotice();
}

//==============================================================================
void SoftBodyNode::updateTransmittedForceID(const Eigen::Vector3d& _gravity,
                                            bool _withExternalForces)
{
  const Eigen::Matrix6d& mI =
      BodyNode::mAspectProperties.mInertia.getSpatialTensor();
  for (auto& pointMass : mPointMasses)
    pointMass->updateTransmittedForceID(_gravity, _withExternalForces);

  // Gravity force
  if (BodyNode::mAspectProperties.mGravityMode == true)
    mFgravity.noalias() = mI * math::AdInvRLinear(getWorldTransform(),_gravity);
  else
    mFgravity.setZero();

  // Inertial force
  mF.noalias() = mI * getSpatialAcceleration();

  // External force
  if (_withExternalForces)
    mF -= BodyNode::mAspectState.mFext;

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
    assert(childJoint != nullptr);

    mF += math::dAdInvT(childJoint->getRelativeTransform(),
                        childBodyNode->getBodyForce());
  }
  for (auto& pointMass : mPointMasses)
  {
    mF.head<3>() += pointMass->getLocalPosition().cross(pointMass->mF);
    mF.tail<3>() += pointMass->mF;
  }

  // Verification
  assert(!math::isNan(mF));
}

//==============================================================================
void SoftBodyNode::updateJointForceID(double _timeStep,
                                      bool _withDampingForces,
                                      bool _withSpringForces)
{
  for (std::size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->updateJointForceID(_timeStep,
                                           _withDampingForces,
                                           _withSpringForces);

  BodyNode::updateJointForceID(_timeStep,
                               _withDampingForces,
                               _withSpringForces);
}

//==============================================================================
void SoftBodyNode::updateJointForceFD(double _timeStep,
                                      bool _withDampingForces,
                                      bool _withSpringForces)
{
  BodyNode::updateJointForceFD(_timeStep,
                               _withDampingForces,
                               _withSpringForces);
}

//==============================================================================
void SoftBodyNode::updateJointImpulseFD()
{
  BodyNode::updateJointImpulseFD();
}

//==============================================================================
void SoftBodyNode::updateArtInertia(double _timeStep) const
{
  const Eigen::Matrix6d& mI =
      BodyNode::mAspectProperties.mInertia.getSpatialTensor();
  for (auto& pointMass : mPointMasses)
    pointMass->updateArtInertiaFD(_timeStep);

  assert(mParentJoint != nullptr);

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

  //
  for (const auto& pointMass : mPointMasses)
  {
    _addPiToArtInertia(pointMass->getLocalPosition(), pointMass->mPi);
    _addPiToArtInertiaImplicit(pointMass->getLocalPosition(),
                               pointMass->mImplicitPi);
  }

  // Verification
  assert(!math::isNan(mArtInertia));
  assert(!math::isNan(mArtInertiaImplicit));

  // Update parent joint's inverse of projected articulated body inertia
  mParentJoint->updateInvProjArtInertia(mArtInertia);
  mParentJoint->updateInvProjArtInertiaImplicit(mArtInertiaImplicit, _timeStep);

  // Verification
  assert(!math::isNan(mArtInertia));
  assert(!math::isNan(mArtInertiaImplicit));
}

//==============================================================================
void SoftBodyNode::updateBiasForce(const Eigen::Vector3d& _gravity,
                                   double _timeStep)
{
  const Eigen::Matrix6d& mI =
      BodyNode::mAspectProperties.mInertia.getSpatialTensor();
  for (auto& pointMass : mPointMasses)
    pointMass->updateBiasForceFD(_timeStep, _gravity);

  // Gravity force
  if (BodyNode::mAspectProperties.mGravityMode == true)
    mFgravity.noalias() = mI * math::AdInvRLinear(getWorldTransform(),_gravity);
  else
    mFgravity.setZero();

  // Set bias force
  const Eigen::Vector6d& V = getSpatialVelocity();
  mBiasForce = -math::dad(V, mI * V) - BodyNode::mAspectState.mFext - mFgravity;

  // Verifycation
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

  //
  for (const auto& pointMass : mPointMasses)
  {
    mBiasForce.head<3>() += pointMass->getLocalPosition().cross(pointMass->mBeta);
    mBiasForce.tail<3>() += pointMass->mBeta;
  }

  // Verifycation
  assert(!math::isNan(mBiasForce));

  // Update parent joint's total force with implicit joint damping and spring
  // forces
  mParentJoint->updateTotalForce( getArticulatedInertiaImplicit()
                                  * getPartialAcceleration() + mBiasForce,
                                  _timeStep );
}

//==============================================================================
void SoftBodyNode::updateAccelerationFD()
{
  BodyNode::updateAccelerationFD();

  for (auto& pointMass : mPointMasses)
    pointMass->updateAccelerationFD();

  mNotifier->clearAccelerationNotice();
}

//==============================================================================
void SoftBodyNode::updateTransmittedForceFD()
{
  BodyNode::updateTransmittedForceFD();

  for (auto& pointMass : mPointMasses)
    pointMass->updateTransmittedForce();
}

//==============================================================================
void SoftBodyNode::updateBiasImpulse()
{
  for (auto& pointMass : mPointMasses)
    pointMass->updateBiasImpulseFD();

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

  for (auto& pointMass : mPointMasses)
  {
    mBiasImpulse.head<3>() += pointMass->getLocalPosition().cross(pointMass->mImpBeta);
    mBiasImpulse.tail<3>() += pointMass->mImpBeta;
  }

  // Verification
  assert(!math::isNan(mBiasImpulse));

  // Update parent joint's total force
  mParentJoint->updateTotalImpulse(mBiasImpulse);
}

//==============================================================================
void SoftBodyNode::updateVelocityChangeFD()
{
  BodyNode::updateVelocityChangeFD();

  for (auto& pointMass : mPointMasses)
    pointMass->updateVelocityChangeFD();
}

//==============================================================================
void SoftBodyNode::updateTransmittedImpulse()
{
  BodyNode::updateTransmittedImpulse();

  for (auto& pointMass : mPointMasses)
    pointMass->updateTransmittedImpulse();
}

//==============================================================================
void SoftBodyNode::updateConstrainedTerms(double _timeStep)
{
  BodyNode::updateConstrainedTerms(_timeStep);

  for (auto& pointMass : mPointMasses)
    pointMass->updateConstrainedTermsFD(_timeStep);
}

//==============================================================================
void SoftBodyNode::updateMassMatrix()
{
  BodyNode::updateMassMatrix();

//  for (std::size_t i = 0; i < mPointMasses.size(); ++i)
//    mPointMasses.at(i)->updateMassMatrix();
}

//==============================================================================
void SoftBodyNode::aggregateMassMatrix(Eigen::MatrixXd& _MCol, std::size_t _col)
{
  BodyNode::aggregateMassMatrix(_MCol, _col);
//  //------------------------ PointMass Part ------------------------------------
//  for (std::size_t i = 0; i < mPointMasses.size(); ++i)
//    mPointMasses.at(i)->aggregateMassMatrix(_MCol, _col);

//  //----------------------- SoftBodyNode Part ----------------------------------
//  //
//  mM_F.noalias() = mI * mM_dV;

//  // Verification
//  assert(!math::isNan(mM_F));

//  //
//  for (std::vector<BodyNode*>::const_iterator it = mChildBodyNodes.begin();
//       it != mChildBodyNodes.end(); ++it)
//  {
//    mM_F += math::dAdInvT((*it)->getParentJoint()->getRelativeTransform(),
//                          (*it)->mM_F);
//  }

//  //
//  for (std::vector<PointMass*>::iterator it = mPointMasses.begin();
//       it != mPointMasses.end(); ++it)
//  {
//    mM_F.head<3>() += (*it)->getLocalPosition().cross((*it)->mM_F);
//    mM_F.tail<3>() += (*it)->mM_F;
//  }

//  // Verification
//  assert(!math::isNan(mM_F));

//  //
//  int dof = mParentJoint->getNumDofs();
//  if (dof > 0)
//  {
//    int iStart = mParentJoint->getIndexInTree(0);
//    _MCol->block(iStart, _col, dof, 1).noalias()
//        = mParentJoint->getRelativeJacobian().transpose() * mM_F;
//  }
}

//==============================================================================
void SoftBodyNode::aggregateAugMassMatrix(Eigen::MatrixXd& _MCol, std::size_t _col,
                                          double _timeStep)
{
  // TODO(JS): Need to be reimplemented

  const Eigen::Matrix6d& mI =
      BodyNode::mAspectProperties.mInertia.getSpatialTensor();
  //------------------------ PointMass Part ------------------------------------
  for (std::size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->aggregateAugMassMatrix(_MCol, _col, _timeStep);

  //----------------------- SoftBodyNode Part ----------------------------------
  mM_F.noalias() = mI * mM_dV;
  assert(!math::isNan(mM_F));

  for (std::vector<BodyNode*>::const_iterator it = mChildBodyNodes.begin();
       it != mChildBodyNodes.end(); ++it)
  {
    mM_F += math::dAdInvT((*it)->getParentJoint()->getRelativeTransform(),
                          (*it)->mM_F);
  }
  for (std::vector<PointMass*>::iterator it = mPointMasses.begin();
       it != mPointMasses.end(); ++it)
  {
    mM_F.head<3>() += (*it)->getLocalPosition().cross((*it)->mM_F);
    mM_F.tail<3>() += (*it)->mM_F;
  }
  assert(!math::isNan(mM_F));

  std::size_t dof = mParentJoint->getNumDofs();
  if (dof > 0)
  {
    Eigen::MatrixXd K = Eigen::MatrixXd::Zero(dof, dof);
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(dof, dof);
    for (std::size_t i = 0; i < dof; ++i)
    {
      K(i, i) = mParentJoint->getSpringStiffness(i);
      D(i, i) = mParentJoint->getDampingCoefficient(i);
    }
    int iStart = mParentJoint->getIndexInTree(0);

    // TODO(JS): Not recommended to use Joint::getAccelerations
    _MCol.block(iStart, _col, dof, 1).noalias()
        = mParentJoint->getRelativeJacobian().transpose() * mM_F
          + D * (_timeStep * mParentJoint->getAccelerations())
          + K * (_timeStep * _timeStep * mParentJoint->getAccelerations());
  }
}

//==============================================================================
void SoftBodyNode::updateInvMassMatrix()
{
  //------------------------ PointMass Part ------------------------------------
  for (std::size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->updateInvMassMatrix();

  //----------------------- SoftBodyNode Part ----------------------------------
  //
  mInvM_c.setZero();

  //
  for (std::vector<BodyNode*>::const_iterator it = mChildBodyNodes.begin();
       it != mChildBodyNodes.end(); ++it)
  {
    (*it)->getParentJoint()->addChildBiasForceForInvMassMatrix(
          mInvM_c, (*it)->getArticulatedInertia(), (*it)->mInvM_c);
  }

  //
  for (std::vector<PointMass*>::iterator it = mPointMasses.begin();
       it != mPointMasses.end(); ++it)
  {
    mInvM_c.head<3>() += (*it)->getLocalPosition().cross((*it)->mBiasForceForInvMeta);
    mInvM_c.tail<3>() += (*it)->mBiasForceForInvMeta;
  }

  // Verification
  assert(!math::isNan(mInvM_c));

  // Update parent joint's total force for inverse mass matrix
  mParentJoint->updateTotalForceForInvMassMatrix(mInvM_c);
}

//==============================================================================
void SoftBodyNode::updateInvAugMassMatrix()
{
  BodyNode::updateInvAugMassMatrix();

//  //------------------------ PointMass Part ------------------------------------
////  for (std::size_t i = 0; i < mPointMasses.size(); ++i)
////    mPointMasses.at(i)->updateInvAugMassMatrix();

//  //----------------------- SoftBodyNode Part ----------------------------------
//  //
//  mInvM_c.setZero();

//  //
//  for (std::vector<BodyNode*>::const_iterator it = mChildBodyNodes.begin();
//       it != mChildBodyNodes.end(); ++it)
//  {
//    (*it)->getParentJoint()->addChildBiasForceForInvAugMassMatrix(
//          mInvM_c, (*it)->getArticulatedInertiaImplicit(), (*it)->mInvM_c);
//  }

//  //
////  for (std::vector<PointMass*>::iterator it = mPointMasses.begin();
////       it != mPointMasses.end(); ++it)
////  {
////    mInvM_c.head<3>() += (*it)->getLocalPosition().cross((*it)->mBiasForceForInvMeta);
////    mInvM_c.tail<3>() += (*it)->mBiasForceForInvMeta;
////  }

//  // Verification
//  assert(!math::isNan(mInvM_c));

//  // Update parent joint's total force for inverse mass matrix
//  mParentJoint->updateTotalForceForInvMassMatrix(mInvM_c);
}

//==============================================================================
void SoftBodyNode::aggregateInvMassMatrix(Eigen::MatrixXd& _InvMCol, std::size_t _col)
{
  if (mParentBodyNode)
  {
    //
    mParentJoint->getInvMassMatrixSegment(
          _InvMCol, _col, getArticulatedInertia(), mParentBodyNode->mInvM_U);

    //
    mInvM_U = math::AdInvT(mParentJoint->getRelativeTransform(),
                           mParentBodyNode->mInvM_U);
  }
  else
  {
    //
    mParentJoint->getInvMassMatrixSegment(
          _InvMCol, _col, getArticulatedInertia(), Eigen::Vector6d::Zero());

    //
    mInvM_U.setZero();
  }

  //
  mParentJoint->addInvMassMatrixSegmentTo(mInvM_U);

  //
  for (std::size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->aggregateInvMassMatrix(_InvMCol, _col);
}

//==============================================================================
void SoftBodyNode::aggregateInvAugMassMatrix(Eigen::MatrixXd& _InvMCol,
                                             std::size_t _col,
                                             double _timeStep)
{
  BodyNode::aggregateInvAugMassMatrix(_InvMCol, _col, _timeStep);

//  if (mParentBodyNode)
//  {
//    //
//    mParentJoint->getInvAugMassMatrixSegment(
//          *_InvMCol, _col, getArticulatedInertiaImplicit(), mParentBodyNode->mInvM_U);

//    //
//    mInvM_U = math::AdInvT(mParentJoint->mT, mParentBodyNode->mInvM_U);
//  }
//  else
//  {
//    //
//    mParentJoint->getInvAugMassMatrixSegment(
//          *_InvMCol, _col, getArticulatedInertiaImplicit(), Eigen::Vector6d::Zero());

//    //
//    mInvM_U.setZero();
//  }

//  //
//  mParentJoint->addInvMassMatrixSegmentTo(mInvM_U);

//  //
////  for (std::size_t i = 0; i < mPointMasses.size(); ++i)
////    mPointMasses.at(i)->aggregateInvAugMassMatrix(_InvMCol, _col, _timeStep);
}

//==============================================================================
void SoftBodyNode::aggregateCoriolisForceVector(Eigen::VectorXd& _C)
{
  BodyNode::aggregateCoriolisForceVector(_C);
}

//==============================================================================
void SoftBodyNode::aggregateGravityForceVector(Eigen::VectorXd& _g,
                                               const Eigen::Vector3d& _gravity)
{
  const Eigen::Matrix6d& mI =
      BodyNode::mAspectProperties.mInertia.getSpatialTensor();
  //------------------------ PointMass Part ------------------------------------
  for (std::size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->aggregateGravityForceVector(_g, _gravity);

  //----------------------- SoftBodyNode Part ----------------------------------
  if (BodyNode::mAspectProperties.mGravityMode == true)
    mG_F = mI * math::AdInvRLinear(getWorldTransform(), _gravity);
  else
    mG_F.setZero();

  for (std::vector<BodyNode*>::const_iterator it = mChildBodyNodes.begin();
       it != mChildBodyNodes.end(); ++it)
  {
    mG_F += math::dAdInvT((*it)->mParentJoint->getRelativeTransform(),
                          (*it)->mG_F);
  }

  for (std::vector<PointMass*>::iterator it = mPointMasses.begin();
       it != mPointMasses.end(); ++it)
  {
    mG_F.head<3>() += (*it)->getLocalPosition().cross((*it)->mG_F);
    mG_F.tail<3>() += (*it)->mG_F;
  }

  int nGenCoords = mParentJoint->getNumDofs();
  if (nGenCoords > 0)
  {
    Eigen::VectorXd g = -(mParentJoint->getRelativeJacobian().transpose() * mG_F);
    int iStart = mParentJoint->getIndexInTree(0);
    _g.segment(iStart, nGenCoords) = g;
  }
}

//==============================================================================
void SoftBodyNode::updateCombinedVector()
{
  BodyNode::updateCombinedVector();

//  for (std::size_t i = 0; i < mPointMasses.size(); ++i)
//    mPointMasses.at(i)->updateCombinedVector();
}

//==============================================================================
void SoftBodyNode::aggregateCombinedVector(Eigen::VectorXd& _Cg,
                                           const Eigen::Vector3d& _gravity)
{
  BodyNode::aggregateCombinedVector(_Cg, _gravity);
//  //------------------------ PointMass Part ------------------------------------
//  for (std::size_t i = 0; i < mPointMasses.size(); ++i)
//    mPointMasses.at(i)->aggregateCombinedVector(_Cg, _gravity);

//  //----------------------- SoftBodyNode Part ----------------------------------
//  // H(i) = I(i) * W(i) -
//  //        dad{V}(I(i) * V(i)) + sum(k \in children) dAd_{T(i,j)^{-1}}(H(k))
//  if (mGravityMode == true)
//    mFgravity = mI * math::AdInvRLinear(mW, _gravity);
//  else
//    mFgravity.setZero();

//  mCg_F = mI * mCg_dV;
//  mCg_F -= mFgravity;
//  mCg_F -= math::dad(mV, mI * mV);

//  for (std::vector<BodyNode*>::iterator it = mChildBodyNodes.begin();
//       it != mChildBodyNodes.end(); ++it)
//  {
//    mCg_F += math::dAdInvT((*it)->getParentJoint()->getRelativeTransform(),
//                           (*it)->mCg_F);
//  }

//  for (std::vector<PointMass*>::iterator it = mPointMasses.begin();
//       it != mPointMasses.end(); ++it)
//  {
//    mCg_F.head<3>() += (*it)->getLocalPosition().cross((*it)->mCg_F);
//    mCg_F.tail<3>() += (*it)->mCg_F;
//  }

//  int nGenCoords = mParentJoint->getNumDofs();
//  if (nGenCoords > 0)
//  {
//    Eigen::VectorXd Cg = mParentJoint->getRelativeJacobian().transpose() * mCg_F;
//    int iStart = mParentJoint->getIndexInTree(0);
//    _Cg->segment(iStart, nGenCoords) = Cg;
//  }
}

//==============================================================================
void SoftBodyNode::aggregateExternalForces(Eigen::VectorXd& _Fext)
{
  //------------------------ PointMass Part ------------------------------------
  for (std::size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->aggregateExternalForces(_Fext);

  //----------------------- SoftBodyNode Part ----------------------------------
  mFext_F = BodyNode::mAspectState.mFext;

  for (std::vector<BodyNode*>::const_iterator it = mChildBodyNodes.begin();
       it != mChildBodyNodes.end(); ++it)
  {
    mFext_F += math::dAdInvT((*it)->mParentJoint->getRelativeTransform(),
                             (*it)->mFext_F);
  }

  for (std::vector<PointMass*>::iterator it = mPointMasses.begin();
       it != mPointMasses.end(); ++it)
  {
    mFext_F.head<3>() += (*it)->getLocalPosition().cross((*it)->mFext);
    mFext_F.tail<3>() += (*it)->mFext;
  }

  int nGenCoords = mParentJoint->getNumDofs();
  if (nGenCoords > 0)
  {
    Eigen::VectorXd Fext
        = mParentJoint->getRelativeJacobian().transpose() * mFext_F;
    int iStart = mParentJoint->getIndexInTree(0);
    _Fext.segment(iStart, nGenCoords) = Fext;
  }
}

//==============================================================================
void SoftBodyNode::clearExternalForces()
{
  BodyNode::clearExternalForces();

  for (std::size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses.at(i)->clearExtForce();
}

//==============================================================================
void SoftBodyNode::clearInternalForces()
{
  BodyNode::clearInternalForces();

  for (std::size_t i = 0; i < mPointMasses.size(); ++i)
    mPointMasses[i]->resetForces();
}

//==============================================================================
void SoftBodyNode::_addPiToArtInertia(const Eigen::Vector3d& _p, double _Pi) const
{
  Eigen::Matrix3d tmp = math::makeSkewSymmetric(_p);

  mArtInertia.topLeftCorner<3, 3>()    -= _Pi * tmp * tmp;
  mArtInertia.topRightCorner<3, 3>()   += _Pi * tmp;
  mArtInertia.bottomLeftCorner<3, 3>() -= _Pi * tmp;

  mArtInertia(3, 3) += _Pi;
  mArtInertia(4, 4) += _Pi;
  mArtInertia(5, 5) += _Pi;
}

//==============================================================================
void SoftBodyNode::_addPiToArtInertiaImplicit(const Eigen::Vector3d& _p,
                                              double _ImplicitPi) const
{
  Eigen::Matrix3d tmp = math::makeSkewSymmetric(_p);

  mArtInertiaImplicit.topLeftCorner<3, 3>()    -= _ImplicitPi * tmp * tmp;
  mArtInertiaImplicit.topRightCorner<3, 3>()   += _ImplicitPi * tmp;
  mArtInertiaImplicit.bottomLeftCorner<3, 3>() -= _ImplicitPi * tmp;

  mArtInertiaImplicit(3, 3) += _ImplicitPi;
  mArtInertiaImplicit(4, 4) += _ImplicitPi;
  mArtInertiaImplicit(5, 5) += _ImplicitPi;
}

//==============================================================================
void SoftBodyNode::updateInertiaWithPointMass()
{
  // TODO(JS): Not implemented

  const Eigen::Matrix6d& mI =
      BodyNode::mAspectProperties.mInertia.getSpatialTensor();
  mI2 = mI;

  for (std::size_t i = 0; i < mPointMasses.size(); ++i)
  {

  }
}

//==============================================================================
SoftBodyNode::UniqueProperties SoftBodyNodeHelper::makeBoxProperties(
    const Eigen::Vector3d& _size,
    const Eigen::Isometry3d& _localTransform,
    double _totalMass,
    double _vertexStiffness,
    double _edgeStiffness,
    double _dampingCoeff)
{
  SoftBodyNode::UniqueProperties properties(
        _vertexStiffness, _edgeStiffness, _dampingCoeff);

  //----------------------------------------------------------------------------
  // Point masses
  //----------------------------------------------------------------------------
  // Number of point masses
  std::size_t nPointMasses = 8;
  properties.mPointProps.resize(8);

  // Mass per vertices
  double mass = _totalMass / nPointMasses;

  // Resting positions for each point mass
  std::vector<Eigen::Vector3d> restingPos(nPointMasses,
                                          Eigen::Vector3d::Zero());
  restingPos[0] = _size.cwiseProduct(Eigen::Vector3d(-1.0, -1.0, -1.0)) * 0.5;
  restingPos[1] = _size.cwiseProduct(Eigen::Vector3d(+1.0, -1.0, -1.0)) * 0.5;
  restingPos[2] = _size.cwiseProduct(Eigen::Vector3d(-1.0, +1.0, -1.0)) * 0.5;
  restingPos[3] = _size.cwiseProduct(Eigen::Vector3d(+1.0, +1.0, -1.0)) * 0.5;
  restingPos[4] = _size.cwiseProduct(Eigen::Vector3d(-1.0, -1.0, +1.0)) * 0.5;
  restingPos[5] = _size.cwiseProduct(Eigen::Vector3d(+1.0, -1.0, +1.0)) * 0.5;
  restingPos[6] = _size.cwiseProduct(Eigen::Vector3d(-1.0, +1.0, +1.0)) * 0.5;
  restingPos[7] = _size.cwiseProduct(Eigen::Vector3d(+1.0, +1.0, +1.0)) * 0.5;

  // Point masses
  for (std::size_t i = 0; i < nPointMasses; ++i)
  {
    properties.mPointProps[i].mX0 = _localTransform * restingPos[i];
    properties.mPointProps[i].mMass = mass;
  }

  //----------------------------------------------------------------------------
  // Edges
  //----------------------------------------------------------------------------
  // -- Bottoms
  properties.connectPointMasses(0, 1);
  properties.connectPointMasses(1, 3);
  properties.connectPointMasses(3, 2);
  properties.connectPointMasses(2, 0);

  // -- Tops
  properties.connectPointMasses(4, 5);
  properties.connectPointMasses(5, 7);
  properties.connectPointMasses(7, 6);
  properties.connectPointMasses(6, 4);

  // -- Sides
  properties.connectPointMasses(0, 4);
  properties.connectPointMasses(1, 5);
  properties.connectPointMasses(2, 6);
  properties.connectPointMasses(3, 7);

  //----------------------------------------------------------------------------
  // Faces
  //----------------------------------------------------------------------------
  // -- -Z
  properties.addFace(Eigen::Vector3i(1, 0, 2));  // 0
  properties.addFace(Eigen::Vector3i(1, 2, 3));  // 1

  // -- +Z
  properties.addFace(Eigen::Vector3i(5, 6, 4));  // 2
  properties.addFace(Eigen::Vector3i(5, 7, 6));  // 3

  // -- -Y
  properties.addFace(Eigen::Vector3i(0, 5, 4));  // 4
  properties.addFace(Eigen::Vector3i(0, 1, 5));  // 5

  // -- +Y
  properties.addFace(Eigen::Vector3i(1, 3, 7));  // 6
  properties.addFace(Eigen::Vector3i(1, 7, 5));  // 7

  // -- -X
  properties.addFace(Eigen::Vector3i(3, 2, 6));  // 8
  properties.addFace(Eigen::Vector3i(3, 6, 7));  // 9

  // -- +X
  properties.addFace(Eigen::Vector3i(2, 0, 4));  // 10
  properties.addFace(Eigen::Vector3i(2, 4, 6));  // 11

  return properties;
}

//==============================================================================
void SoftBodyNodeHelper::setBox(SoftBodyNode*            _softBodyNode,
                                const Eigen::Vector3d&   _size,
                                const Eigen::Isometry3d& _localTransform,
                                double                   _totalMass,
                                double                   _vertexStiffness,
                                double                   _edgeStiffness,
                                double                   _dampingCoeff)
{
  assert(_softBodyNode != nullptr);
  _softBodyNode->setProperties(makeBoxProperties(_size,
                                                 _localTransform,
                                                 _totalMass,
                                                 _vertexStiffness,
                                                 _edgeStiffness,
                                                 _dampingCoeff));
}

//==============================================================================
SoftBodyNode::UniqueProperties SoftBodyNodeHelper::makeBoxProperties(
    const Eigen::Vector3d& _size,
    const Eigen::Isometry3d& _localTransform,
    const Eigen::Vector3i& _frags,
    double _totalMass,
    double _vertexStiffness,
    double _edgeStiffness,
    double _dampingCoeff)
{
  Eigen::Vector3i frags = _frags;

  for (int i = 0; i < 3; ++i)
  {
    if (frags[i] <= 2)
    {
      dtwarn << "[SoftBodyNodeHelper::makeBoxProperties] Invalid argument for "
             << "_frags. The number of vertices assigned to soft box edge #"
             << i << " is " << frags[i] << ", but it must be greater than or "
             << "equal to 3. We will set it to 3.\n";
      frags[i] = 3;
    }
  }

  std::size_t id = 0;

  SoftBodyNode::UniqueProperties properties(
        _vertexStiffness, _edgeStiffness, _dampingCoeff);

  //----------------------------------------------------------------------------
  // Point masses
  //----------------------------------------------------------------------------
  // Number of point masses
  assert(frags[0] > 1 && frags[1] > 1 && frags[2] > 1);

  std::size_t nCorners = 8;
  std::size_t nVerticesAtEdgeX = frags[0] - 2;
  std::size_t nVerticesAtEdgeY = frags[1] - 2;
  std::size_t nVerticesAtEdgeZ = frags[2] - 2;
  std::size_t nVerticesAtSideX = nVerticesAtEdgeY*nVerticesAtEdgeZ;
  std::size_t nVerticesAtSideY = nVerticesAtEdgeZ*nVerticesAtEdgeX;
  std::size_t nVerticesAtSideZ = nVerticesAtEdgeX*nVerticesAtEdgeY;
  std::size_t nVertices
      = nCorners
        + 4*(nVerticesAtEdgeX + nVerticesAtEdgeY + nVerticesAtEdgeZ)
        + 2*(nVerticesAtSideX + nVerticesAtSideY + nVerticesAtSideZ);

  // Mass per vertices
  double mass = _totalMass / nVertices;

  Eigen::Vector3d segLength(_size[0]/(frags[0] - 1),
                            _size[1]/(frags[1] - 1),
                            _size[2]/(frags[2] - 1));

  typedef std::pair<PointMass::Properties, std::size_t> PointPair;

  std::vector<PointPair> corners(nCorners);

  std::vector< std::vector<PointPair> >
      edgeX(4, std::vector<PointPair>(nVerticesAtEdgeX));
  std::vector< std::vector<PointPair> >
      edgeY(4, std::vector<PointPair>(nVerticesAtEdgeY));
  std::vector< std::vector<PointPair> >
      edgeZ(4, std::vector<PointPair>(nVerticesAtEdgeZ));

  std::vector< std::vector<PointPair> >
      sideXNeg(nVerticesAtEdgeY, std::vector<PointPair>(nVerticesAtEdgeZ));
  std::vector< std::vector<PointPair> >
      sideXPos(nVerticesAtEdgeY, std::vector<PointPair>(nVerticesAtEdgeZ));

  std::vector< std::vector<PointPair> >
      sideYNeg(nVerticesAtEdgeZ, std::vector<PointPair>(nVerticesAtEdgeX));
  std::vector< std::vector<PointPair> >
      sideYPos(nVerticesAtEdgeZ, std::vector<PointPair>(nVerticesAtEdgeX));

  std::vector< std::vector<PointPair> >
      sideZNeg(nVerticesAtEdgeX, std::vector<PointPair>(nVerticesAtEdgeY));
  std::vector< std::vector<PointPair> >
      sideZPos(nVerticesAtEdgeX, std::vector<PointPair>(nVerticesAtEdgeY));

  Eigen::Vector3d x0y0z0 = _size.cwiseProduct(Eigen::Vector3d(-1.0, -1.0, -1.0)) * 0.5;
  Eigen::Vector3d x1y0z0 = _size.cwiseProduct(Eigen::Vector3d(+1.0, -1.0, -1.0)) * 0.5;
  Eigen::Vector3d x1y1z0 = _size.cwiseProduct(Eigen::Vector3d(+1.0, +1.0, -1.0)) * 0.5;
  Eigen::Vector3d x0y1z0 = _size.cwiseProduct(Eigen::Vector3d(-1.0, +1.0, -1.0)) * 0.5;
  Eigen::Vector3d x0y0z1 = _size.cwiseProduct(Eigen::Vector3d(-1.0, -1.0, +1.0)) * 0.5;
  Eigen::Vector3d x1y0z1 = _size.cwiseProduct(Eigen::Vector3d(+1.0, -1.0, +1.0)) * 0.5;
  Eigen::Vector3d x1y1z1 = _size.cwiseProduct(Eigen::Vector3d(+1.0, +1.0, +1.0)) * 0.5;
  Eigen::Vector3d x0y1z1 = _size.cwiseProduct(Eigen::Vector3d(-1.0, +1.0, +1.0)) * 0.5;

  std::vector<Eigen::Vector3d> beginPts;
  Eigen::Vector3d restPos;

  // Corners
  beginPts.resize(nCorners);
  beginPts[0] = x0y0z0;
  beginPts[1] = x1y0z0;
  beginPts[2] = x1y1z0;
  beginPts[3] = x0y1z0;
  beginPts[4] = x0y0z1;
  beginPts[5] = x1y0z1;
  beginPts[6] = x1y1z1;
  beginPts[7] = x0y1z1;

  for (std::size_t i = 0; i < nCorners; ++i)
  {
    corners[i].first.setRestingPosition(_localTransform * beginPts[i]);
    corners[i].first.setMass(mass);
    properties.addPointMass(corners[i].first);

    corners[i].second = id++;
  }

  // Edges (along X-axis)
  beginPts.resize(4);
  beginPts[0] = x0y0z0;
  beginPts[1] = x0y1z0;
  beginPts[2] = x0y1z1;
  beginPts[3] = x0y0z1;

  for (std::size_t i = 0; i < 4; ++i)
  {
    restPos = beginPts[i];

    for (std::size_t j = 0; j < nVerticesAtEdgeX; ++j)
    {
      restPos[0] += segLength[0];

      edgeX[i][j].first.setRestingPosition(_localTransform * restPos);
      edgeX[i][j].first.setMass(mass);
      properties.addPointMass(edgeX[i][j].first);

      edgeX[i][j].second = id++;
    }
  }

  // Edges (along Y-axis)
  beginPts[0] = x0y0z0;
  beginPts[1] = x0y0z1;
  beginPts[2] = x1y0z1;
  beginPts[3] = x1y0z0;

  for (std::size_t i = 0; i < 4; ++i)
  {
    restPos = beginPts[i];

    for (std::size_t j = 0; j < nVerticesAtEdgeY; ++j)
    {
      restPos[1] += segLength[1];

      edgeY[i][j].first.setRestingPosition(_localTransform * restPos);
      edgeY[i][j].first.setMass(mass);
      properties.addPointMass(edgeY[i][j].first);

      edgeY[i][j].second = id++;
    }
  }

  // Edges (along Z-axis)
  beginPts[0] = x0y0z0;
  beginPts[1] = x1y0z0;
  beginPts[2] = x1y1z0;
  beginPts[3] = x0y1z0;

  for (std::size_t i = 0; i < 4; ++i)
  {
    restPos = beginPts[i];

    for (std::size_t j = 0; j < nVerticesAtEdgeZ; ++j)
    {
      restPos[2] += segLength[2];

      edgeZ[i][j].first.setRestingPosition(_localTransform * restPos);
      edgeZ[i][j].first.setMass(mass);
      properties.addPointMass(edgeZ[i][j].first);

      edgeZ[i][j].second = id++;
    }
  }

  // Negative X side
  restPos = x0y0z0;

  for (std::size_t i = 0; i < nVerticesAtEdgeY; ++i)
  {
    restPos[2] = x0y0z0[2];
    restPos[1] += segLength[1];

    for (std::size_t j = 0; j < nVerticesAtEdgeZ; ++j)
    {
      restPos[2] += segLength[2];

      sideXNeg[i][j].first.setRestingPosition(_localTransform * restPos);
      sideXNeg[i][j].first.setMass(mass);
      properties.addPointMass(sideXNeg[i][j].first);

      sideXNeg[i][j].second = id++;
    }
  }

  // Positive X side
  restPos = x1y0z0;

  for (std::size_t i = 0; i < nVerticesAtEdgeY; ++i)
  {
    restPos[2] = x1y0z0[2];
    restPos[1] += segLength[1];

    for (std::size_t j = 0; j < nVerticesAtEdgeZ; ++j)
    {
      restPos[2] += segLength[2];

      sideXPos[i][j].first.setRestingPosition(_localTransform * restPos);
      sideXPos[i][j].first.setMass(mass);
      properties.addPointMass(sideXPos[i][j].first);

      sideXPos[i][j].second = id++;
    }
  }

  // Negative Y side
  restPos = x0y0z0;

  for (std::size_t i = 0; i < nVerticesAtEdgeZ; ++i)
  {
    restPos[0] = x0y0z0[0];
    restPos[2] += segLength[2];

    for (std::size_t j = 0; j < nVerticesAtEdgeX; ++j)
    {
      restPos[0] += segLength[0];

      sideYNeg[i][j].first.setRestingPosition(_localTransform * restPos);
      sideYNeg[i][j].first.setMass(mass);
      properties.addPointMass(sideYNeg[i][j].first);

      sideYNeg[i][j].second = id++;
    }
  }

  // Positive Y side
  restPos = x0y1z0;

  for (std::size_t i = 0; i < nVerticesAtEdgeZ; ++i)
  {
    restPos[0] = x0y1z0[0];
    restPos[2] += segLength[2];

    for (std::size_t j = 0; j < nVerticesAtEdgeX; ++j)
    {
      restPos[0] += segLength[0];

      sideYPos[i][j].first.setRestingPosition(_localTransform * restPos);
      sideYPos[i][j].first.setMass(mass);
      properties.addPointMass(sideYPos[i][j].first);

      sideYPos[i][j].second = id++;
    }
  }

  // Negative Z side
  restPos = x0y0z0;

  for (std::size_t i = 0; i < nVerticesAtEdgeX; ++i)
  {
    restPos[1] = x0y0z0[1];
    restPos[0] += segLength[0];

    for (std::size_t j = 0; j < nVerticesAtEdgeY; ++j)
    {
      restPos[1] += segLength[1];

      sideZNeg[i][j].first.setRestingPosition(_localTransform * restPos);
      sideZNeg[i][j].first.setMass(mass);
      properties.addPointMass(sideZNeg[i][j].first);

      sideZNeg[i][j].second = id++;
    }
  }

  // Positive Z side
  restPos = x0y0z1;

  for (std::size_t i = 0; i < nVerticesAtEdgeX; ++i)
  {
    restPos[1] = x0y0z1[1];
    restPos[0] += segLength[0];

    for (std::size_t j = 0; j < nVerticesAtEdgeY; ++j)
    {
      restPos[1] += segLength[1];

      sideZPos[i][j].first.setRestingPosition(_localTransform * restPos);
      sideZPos[i][j].first.setMass(mass);
      properties.addPointMass(sideZPos[i][j].first);

      sideZPos[i][j].second = id++;
    }
  }

  //----------------------------------------------------------------------------
  // Faces
  //----------------------------------------------------------------------------
  std::size_t nFacesX = 2*(frags[1] - 1)*(frags[2] - 1);
  std::size_t nFacesY = 2*(frags[2] - 1)*(frags[0] - 1);
  std::size_t nFacesZ = 2*(frags[0] - 1)*(frags[1] - 1);
  std::size_t nFaces  = 2*nFacesX + 2*nFacesY + 2*nFacesZ;

  std::vector<Eigen::Vector3i> faces(nFaces, Eigen::Vector3i::Zero());

  int fIdx = 0;

  // Corners[0] faces
  faces[fIdx][0] = corners[0].second;
  faces[fIdx][1] = edgeZ[0][0].second;
  faces[fIdx][2] = edgeY[0][0].second;
  fIdx++;

  faces[fIdx][0] = sideXNeg[0][0].second;
  faces[fIdx][1] = edgeY[0][0].second;
  faces[fIdx][2] = edgeZ[0][0].second;
  fIdx++;

  faces[fIdx][0] = corners[0].second;
  faces[fIdx][1] = edgeX[0][0].second;
  faces[fIdx][2] = edgeZ[0][0].second;
  fIdx++;

  faces[fIdx][0] = sideYNeg[0][0].second;
  faces[fIdx][1] = edgeZ[0][0].second;
  faces[fIdx][2] = edgeX[0][0].second;
  fIdx++;

  faces[fIdx][0] = corners[0].second;
  faces[fIdx][1] = edgeY[0][0].second;
  faces[fIdx][2] = edgeX[0][0].second;
  fIdx++;

  faces[fIdx][0] = sideZNeg[0][0].second;
  faces[fIdx][1] = edgeX[0][0].second;
  faces[fIdx][2] = edgeY[0][0].second;
  fIdx++;

  properties.addFace(faces[0]);
  properties.addFace(faces[1]);
  properties.addFace(faces[2]);
  properties.addFace(faces[3]);
  properties.addFace(faces[4]);
  properties.addFace(faces[5]);

  // Corners[1] faces
  faces[fIdx][0] = corners[1].second;
  faces[fIdx][1] = edgeY[3][0].second;
  faces[fIdx][2] = edgeZ[1][0].second;
  fIdx++;

  faces[fIdx][0] = sideXPos[0][0].second;
  faces[fIdx][1] = edgeZ[1][0].second;
  faces[fIdx][2] = edgeY[3][0].second;
  fIdx++;

  faces[fIdx][0] = corners[1].second;
  faces[fIdx][1] = edgeZ[1][0].second;
  faces[fIdx][2] = edgeX[0].back().second;
  fIdx++;

  faces[fIdx][0] = sideYNeg[0].back().second;
  faces[fIdx][1] = edgeX[0].back().second;
  faces[fIdx][2] = edgeZ[1][0].second;
  fIdx++;

  faces[fIdx][0] = corners[1].second;
  faces[fIdx][1] = edgeX[0].back().second;
  faces[fIdx][2] = edgeY[3][0].second;
  fIdx++;

  faces[fIdx][0] = sideZNeg.back()[0].second;
  faces[fIdx][1] = edgeY[3][0].second;
  faces[fIdx][2] = edgeX[0].back().second;
  fIdx++;

  properties.addFace(faces[6]);
  properties.addFace(faces[7]);
  properties.addFace(faces[8]);
  properties.addFace(faces[9]);
  properties.addFace(faces[10]);
  properties.addFace(faces[11]);

  // Corners[2] faces
  faces[fIdx][0] = corners[2].second;
  faces[fIdx][1] = edgeZ[2][0].second;
  faces[fIdx][2] = edgeY[3].back().second;
  fIdx++;

  faces[fIdx][0] = sideXPos.back()[0].second;
  faces[fIdx][1] = edgeY[3].back().second;
  faces[fIdx][2] = edgeZ[2][0].second;
  fIdx++;

  faces[fIdx][0] = corners[2].second;
  faces[fIdx][1] = edgeX[1].back().second;
  faces[fIdx][2] = edgeZ[2][0].second;
  fIdx++;

  faces[fIdx][0] = sideYPos[0].back().second;
  faces[fIdx][1] = edgeZ[2][0].second;
  faces[fIdx][2] = edgeX[1].back().second;
  fIdx++;

  faces[fIdx][0] = corners[2].second;
  faces[fIdx][1] = edgeY[3].back().second;
  faces[fIdx][2] = edgeX[1].back().second;
  fIdx++;

  faces[fIdx][0] = sideZNeg.back().back().second;
  faces[fIdx][1] = edgeX[1].back().second;
  faces[fIdx][2] = edgeY[3].back().second;
  fIdx++;

  properties.addFace(faces[12]);
  properties.addFace(faces[13]);
  properties.addFace(faces[14]);
  properties.addFace(faces[15]);
  properties.addFace(faces[16]);
  properties.addFace(faces[17]);

  // Corners[3] faces
  faces[fIdx][0] = corners[3].second;
  faces[fIdx][1] = edgeY[0].back().second;
  faces[fIdx][2] = edgeZ[3][0].second;
  fIdx++;

  faces[fIdx][0] = sideXNeg.back()[0].second;
  faces[fIdx][1] = edgeZ[3][0].second;
  faces[fIdx][2] = edgeY[0].back().second;
  fIdx++;

  faces[fIdx][0] = corners[3].second;
  faces[fIdx][1] = edgeZ[3][0].second;
  faces[fIdx][2] = edgeX[1][0].second;
  fIdx++;

  faces[fIdx][0] = sideYPos[0][0].second;
  faces[fIdx][1] = edgeX[1][0].second;
  faces[fIdx][2] = edgeZ[3][0].second;
  fIdx++;

  faces[fIdx][0] = corners[3].second;
  faces[fIdx][1] = edgeX[1][0].second;
  faces[fIdx][2] = edgeY[0].back().second;
  fIdx++;

  faces[fIdx][0] = sideZNeg[0].back().second;
  faces[fIdx][1] = edgeY[0].back().second;
  faces[fIdx][2] = edgeX[1][0].second;
  fIdx++;

  properties.addFace(faces[18]);
  properties.addFace(faces[19]);
  properties.addFace(faces[20]);
  properties.addFace(faces[21]);
  properties.addFace(faces[22]);
  properties.addFace(faces[23]);

  // Corners[4] faces
  faces[fIdx][0] = corners[4].second;
  faces[fIdx][1] = edgeY[1][0].second;
  faces[fIdx][2] = edgeZ[0].back().second;
  fIdx++;

  faces[fIdx][0] = sideXNeg[0].back().second;
  faces[fIdx][1] = edgeY[1][0].second;
  faces[fIdx][2] = edgeZ[0].back().second;
  fIdx++;

  faces[fIdx][0] = corners[4].second;
  faces[fIdx][1] = edgeZ[0].back().second;
  faces[fIdx][2] = edgeX[3][0].second;
  fIdx++;

  faces[fIdx][0] = sideYNeg.back()[0].second;
  faces[fIdx][1] = edgeZ[0].back().second;
  faces[fIdx][2] = edgeX[3][0].second;
  fIdx++;

  faces[fIdx][0] = corners[4].second;
  faces[fIdx][1] = edgeX[3][0].second;
  faces[fIdx][2] = edgeY[1][0].second;
  fIdx++;

  faces[fIdx][0] = sideZPos[0][0].second;
  faces[fIdx][1] = edgeX[3][0].second;
  faces[fIdx][2] = edgeY[1][0].second;
  fIdx++;

  properties.addFace(faces[24]);
  properties.addFace(faces[25]);
  properties.addFace(faces[26]);
  properties.addFace(faces[27]);
  properties.addFace(faces[28]);
  properties.addFace(faces[29]);

  // Corners[5] faces
  faces[fIdx][0] = corners[5].second;
  faces[fIdx][1] = edgeZ[1].back().second;
  faces[fIdx][2] = edgeY[2][0].second;
  fIdx++;

  faces[fIdx][0] = sideXPos[0].back().second;
  faces[fIdx][1] = edgeY[2][0].second;
  faces[fIdx][2] = edgeZ[1].back().second;
  fIdx++;

  faces[fIdx][0] = corners[5].second;
  faces[fIdx][1] = edgeX[3].back().second;
  faces[fIdx][2] = edgeZ[1].back().second;
  fIdx++;

  faces[fIdx][0] = sideYNeg.back().back().second;
  faces[fIdx][1] = edgeZ[1].back().second;
  faces[fIdx][2] = edgeX[3].back().second;
  fIdx++;

  faces[fIdx][0] = corners[5].second;
  faces[fIdx][1] = edgeY[2][0].second;
  faces[fIdx][2] = edgeX[3].back().second;
  fIdx++;

  faces[fIdx][0] = sideZPos.back()[0].second;
  faces[fIdx][1] = edgeX[3].back().second;
  faces[fIdx][2] = edgeY[2][0].second;
  fIdx++;

  properties.addFace(faces[30]);
  properties.addFace(faces[31]);
  properties.addFace(faces[32]);
  properties.addFace(faces[33]);
  properties.addFace(faces[34]);
  properties.addFace(faces[35]);

  // Corners[6] faces
  faces[fIdx][0] = corners[6].second;
  faces[fIdx][1] = edgeY[2].back().second;
  faces[fIdx][2] = edgeZ[2].back().second;
  fIdx++;

  faces[fIdx][0] = sideXPos.back().back().second;
  faces[fIdx][1] = edgeZ[2].back().second;
  faces[fIdx][2] = edgeY[2].back().second;
  fIdx++;

  faces[fIdx][0] = corners[6].second;
  faces[fIdx][1] = edgeZ[2].back().second;
  faces[fIdx][2] = edgeX[2].back().second;
  fIdx++;

  faces[fIdx][0] = sideYPos.back().back().second;
  faces[fIdx][1] = edgeX[2].back().second;
  faces[fIdx][2] = edgeZ[2].back().second;
  fIdx++;

  faces[fIdx][0] = corners[6].second;
  faces[fIdx][1] = edgeX[2].back().second;
  faces[fIdx][2] = edgeY[2].back().second;
  fIdx++;

  faces[fIdx][0] = sideZPos.back().back().second;
  faces[fIdx][1] = edgeY[2].back().second;
  faces[fIdx][2] = edgeX[2].back().second;
  fIdx++;

  properties.addFace(faces[36]);
  properties.addFace(faces[37]);
  properties.addFace(faces[38]);
  properties.addFace(faces[39]);
  properties.addFace(faces[40]);
  properties.addFace(faces[41]);

  // Corners[7] faces
  faces[fIdx][0] = corners[7].second;
  faces[fIdx][1] = edgeZ[3].back().second;
  faces[fIdx][2] = edgeY[1].back().second;
  fIdx++;

  faces[fIdx][0] = sideXNeg.back().back().second;
  faces[fIdx][1] = edgeY[1].back().second;
  faces[fIdx][2] = edgeZ[3].back().second;
  fIdx++;

  faces[fIdx][0] = corners[7].second;
  faces[fIdx][1] = edgeX[2][0].second;
  faces[fIdx][2] = edgeZ[3].back().second;
  fIdx++;

  faces[fIdx][0] = sideYPos.back()[0].second;
  faces[fIdx][1] = edgeZ[3].back().second;
  faces[fIdx][2] = edgeX[2][0].second;
  fIdx++;

  faces[fIdx][0] = corners[7].second;
  faces[fIdx][1] = edgeY[1].back().second;
  faces[fIdx][2] = edgeX[2][0].second;
  fIdx++;

  faces[fIdx][0] = sideZPos[0].back().second;
  faces[fIdx][1] = edgeX[2][0].second;
  faces[fIdx][2] = edgeY[1].back().second;
  fIdx++;

  properties.addFace(faces[42]);
  properties.addFace(faces[43]);
  properties.addFace(faces[44]);
  properties.addFace(faces[45]);
  properties.addFace(faces[46]);
  properties.addFace(faces[47]);

  // EdgeX[0]
  for (std::size_t i = 0; i < edgeX[0].size() - 1; ++i)
  {
    faces[fIdx][0] = edgeX[0][i].second;
    faces[fIdx][1] = edgeX[0][i + 1].second;
    faces[fIdx][2] = sideYNeg[0][i].second;
    properties.addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = sideYNeg[0][i + 1].second;
    faces[fIdx][1] = sideYNeg[0][i].second;
    faces[fIdx][2] = edgeX[0][i + 1].second;
    properties.addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = edgeX[0][i].second;
    faces[fIdx][1] = sideZNeg[i][0].second;
    faces[fIdx][2] = edgeX[0][i + 1].second;
    properties.addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = sideZNeg[i + 1][0].second;
    faces[fIdx][1] = edgeX[0][i + 1].second;
    faces[fIdx][2] = sideZNeg[i][0].second;
    properties.addFace(faces[fIdx]);
    fIdx++;
  }

  // EdgeX[1]
  for (std::size_t i = 0; i < edgeX[1].size() - 1; ++i)
  {
    faces[fIdx][0] = edgeX[1][i].second;
    faces[fIdx][1] = sideYPos[0][i].second;
    faces[fIdx][2] = edgeX[1][i + 1].second;
    properties.addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = sideYPos[0][i + 1].second;
    faces[fIdx][1] = edgeX[1][i + 1].second;
    faces[fIdx][2] = sideYPos[0][i].second;
    properties.addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = edgeX[1][i].second;
    faces[fIdx][1] = edgeX[1][i + 1].second;
    faces[fIdx][2] = sideZNeg[i].back().second;
    properties.addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = sideZNeg[i + 1].back().second;
    faces[fIdx][1] = sideZNeg[i].back().second;
    faces[fIdx][2] = edgeX[1][i + 1].second;
    properties.addFace(faces[fIdx]);
    fIdx++;
  }

  // EdgeX[2]
  for (std::size_t i = 0; i < edgeX[2].size() - 1; ++i)
  {
    faces[fIdx][0] = edgeX[2][i + 1].second;
    faces[fIdx][1] = sideYPos.back()[i + 1].second;
    faces[fIdx][2] = edgeX[2][i].second;
    properties.addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = sideYPos.back()[i].second;
    faces[fIdx][1] = edgeX[2][i].second;
    faces[fIdx][2] = sideYPos.back()[i + 1].second;
    properties.addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = edgeX[2][i + 1].second;
    faces[fIdx][1] = edgeX[2][i].second;
    faces[fIdx][2] = sideZPos[i + 1].back().second;
    properties.addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = sideZPos[i].back().second;
    faces[fIdx][1] = sideZPos[i + 1].back().second;
    faces[fIdx][2] = edgeX[2][i].second;
    properties.addFace(faces[fIdx]);
    fIdx++;
  }

  // EdgeX[3]
  for (std::size_t i = 0; i < edgeX[3].size() - 1; ++i)
  {
    faces[fIdx][0] = edgeX[3][i].second;
    faces[fIdx][1] = sideYNeg.back()[i].second;
    faces[fIdx][2] = edgeX[3][i + 1].second;
    properties.addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = sideYNeg.back()[i + 1].second;
    faces[fIdx][1] = edgeX[3][i + 1].second;
    faces[fIdx][2] = sideYNeg.back()[i].second;
    properties.addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = edgeX[3][i].second;
    faces[fIdx][1] = edgeX[3][i + 1].second;
    faces[fIdx][2] = sideZPos[i][0].second;
    properties.addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = sideZPos[i + 1][0].second;
    faces[fIdx][1] = sideZPos[i][0].second;
    faces[fIdx][2] = edgeX[3][i + 1].second;
    properties.addFace(faces[fIdx]);
    fIdx++;
  }

  // edgeY[0]
  for (std::size_t i = 0; i < edgeY[0].size() - 1; ++i)
  {
    faces[fIdx][0] = edgeY[0][i + 1].second;
    faces[fIdx][1] = sideZNeg[0][i + 1].second;
    faces[fIdx][2] = edgeY[0][i].second;
    properties.addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = sideZNeg[0][i].second;
    faces[fIdx][1] = edgeY[0][i].second;
    faces[fIdx][2] = sideZNeg[0][i + 1].second;
    properties.addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = edgeY[0][i].second;
    faces[fIdx][1] = sideXNeg[i][0].second;
    faces[fIdx][2] = edgeY[0][i + 1].second;
    properties.addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = sideXNeg[i + 1][0].second;
    faces[fIdx][1] = edgeY[0][i + 1].second;
    faces[fIdx][2] = sideXNeg[i][0].second;
    properties.addFace(faces[fIdx]);
    fIdx++;
  }

  // edgeY[1]
  for (std::size_t i = 0; i < edgeY[1].size() - 1; ++i)
  {
    faces[fIdx][0] = edgeY[1][i].second;
    faces[fIdx][1] = sideZPos[0][i].second;
    faces[fIdx][2] = edgeY[1][i + 1].second;
    properties.addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = sideZPos[0][i + 1].second;
    faces[fIdx][1] = edgeY[1][i + 1].second;
    faces[fIdx][2] = sideZPos[0][i].second;
    properties.addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = edgeY[1][i].second;
    faces[fIdx][1] = edgeY[1][i + 1].second;
    faces[fIdx][2] = sideXNeg[i].back().second;
    properties.addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = sideXNeg[i + 1].back().second;
    faces[fIdx][1] = sideXNeg[i].back().second;
    faces[fIdx][2] = edgeY[1][i + 1].second;
    properties.addFace(faces[fIdx]);
    fIdx++;
  }

  // edgeY[2]
  for (std::size_t i = 0; i < edgeY[2].size() - 1; ++i)
  {
    faces[fIdx][0] = edgeY[2][i + 1].second;
    faces[fIdx][1] = sideZPos.back()[i + 1].second;
    faces[fIdx][2] = edgeY[2][i].second;
    properties.addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = sideZPos.back()[i].second;
    faces[fIdx][1] = edgeY[2][i].second;
    faces[fIdx][2] = sideZPos.back()[i + 1].second;
    properties.addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = edgeY[2][i + 1].second;
    faces[fIdx][1] = edgeY[2][i].second;
    faces[fIdx][2] = sideXPos[i + 1].back().second;
    properties.addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = sideXPos[i].back().second;
    faces[fIdx][1] = sideXPos[i + 1].back().second;
    faces[fIdx][2] = edgeY[2][i].second;
    properties.addFace(faces[fIdx]);
    fIdx++;
  }

  // edgeY[3]
  for (std::size_t i = 0; i < edgeY[3].size() - 1; ++i)
  {
    faces[fIdx][0] = edgeY[3][i].second;
    faces[fIdx][1] = sideZNeg.back()[i].second;
    faces[fIdx][2] = edgeY[3][i + 1].second;
    properties.addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = sideZNeg.back()[i + 1].second;
    faces[fIdx][1] = edgeY[3][i + 1].second;
    faces[fIdx][2] = sideZNeg.back()[i].second;
    properties.addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = edgeY[3][i].second;
    faces[fIdx][1] = edgeY[3][i + 1].second;
    faces[fIdx][2] = sideXPos[i][0].second;
    properties.addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = sideXPos[i + 1][0].second;
    faces[fIdx][1] = sideXPos[i][0].second;
    faces[fIdx][2] = edgeY[3][i + 1].second;
    properties.addFace(faces[fIdx]);
    fIdx++;
  }

  // edgeZ[0]
  for (std::size_t i = 0; i < edgeZ[0].size() - 1; ++i)
  {
    faces[fIdx][0] = edgeZ[0][i + 1].second;
    faces[fIdx][1] = sideXNeg[0][i + 1].second;
    faces[fIdx][2] = edgeZ[0][i].second;
    properties.addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = sideXNeg[0][i].second;
    faces[fIdx][1] = edgeZ[0][i].second;
    faces[fIdx][2] = sideXNeg[0][i + 1].second;
    properties.addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = edgeZ[0][i].second;
    faces[fIdx][1] = sideYNeg[i][0].second;
    faces[fIdx][2] = edgeZ[0][i + 1].second;
    properties.addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = sideYNeg[i + 1][0].second;
    faces[fIdx][1] = edgeZ[0][i + 1].second;
    faces[fIdx][2] = sideYNeg[i][0].second;
    properties.addFace(faces[fIdx]);
    fIdx++;
  }

  // edgeZ[1]
  for (std::size_t i = 0; i < edgeZ[1].size() - 1; ++i)
  {
    faces[fIdx][0] = edgeZ[1][i].second;
    faces[fIdx][1] = sideXPos[0][i].second;
    faces[fIdx][2] = edgeZ[1][i + 1].second;
    properties.addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = sideXPos[0][i + 1].second;
    faces[fIdx][1] = edgeZ[1][i + 1].second;
    faces[fIdx][2] = sideXPos[0][i].second;
    properties.addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = edgeZ[1][i].second;
    faces[fIdx][1] = edgeZ[1][i + 1].second;
    faces[fIdx][2] = sideYNeg[i].back().second;
    properties.addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = sideYNeg[i + 1].back().second;
    faces[fIdx][1] = sideYNeg[i].back().second;
    faces[fIdx][2] = edgeZ[1][i + 1].second;
    properties.addFace(faces[fIdx]);
    fIdx++;
  }

  // edgeZ[2]
  for (std::size_t i = 0; i < edgeZ[2].size() - 1; ++i)
  {
    faces[fIdx][0] = edgeZ[2][i + 1].second;
    faces[fIdx][1] = sideXPos.back()[i + 1].second;
    faces[fIdx][2] = edgeZ[2][i].second;
    properties.addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = sideXPos.back()[i].second;
    faces[fIdx][1] = edgeZ[2][i].second;
    faces[fIdx][2] = sideXPos.back()[i + 1].second;
    properties.addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = edgeZ[2][i + 1].second;
    faces[fIdx][1] = edgeZ[2][i].second;
    faces[fIdx][2] = sideYPos[i + 1].back().second;
    properties.addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = sideYPos[i].back().second;
    faces[fIdx][1] = sideYPos[i + 1].back().second;
    faces[fIdx][2] = edgeZ[2][i].second;
    properties.addFace(faces[fIdx]);
    fIdx++;
  }

  // edgeZ[3]
  for (std::size_t i = 0; i < edgeZ[3].size() - 1; ++i)
  {
    faces[fIdx][0] = edgeZ[3][i].second;
    faces[fIdx][1] = sideXNeg.back()[i].second;
    faces[fIdx][2] = edgeZ[3][i + 1].second;
    properties.addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = sideXNeg.back()[i + 1].second;
    faces[fIdx][1] = edgeZ[3][i + 1].second;
    faces[fIdx][2] = sideXNeg.back()[i].second;
    properties.addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = edgeZ[3][i].second;
    faces[fIdx][1] = edgeZ[3][i + 1].second;
    faces[fIdx][2] = sideYPos[i][0].second;
    properties.addFace(faces[fIdx]);
    fIdx++;

    faces[fIdx][0] = sideYPos[i + 1][0].second;
    faces[fIdx][1] = sideYPos[i][0].second;
    faces[fIdx][2] = edgeZ[3][i + 1].second;
    properties.addFace(faces[fIdx]);
    fIdx++;
  }

  // -X side
  for (std::size_t i = 0; i < sideXNeg.size() - 1; ++i)
  {
    for (std::size_t j = 0; j < sideXNeg[i].size() - 1; ++j)
    {
      faces[fIdx][0] = sideXNeg[i + 0][j + 0].second;
      faces[fIdx][1] = sideXNeg[i + 0][j + 1].second;
      faces[fIdx][2] = sideXNeg[i + 1][j + 0].second;
      properties.addFace(faces[fIdx]);
      fIdx++;

      faces[fIdx][0] = sideXNeg[i + 1][j + 1].second;
      faces[fIdx][1] = sideXNeg[i + 1][j + 0].second;
      faces[fIdx][2] = sideXNeg[i + 0][j + 1].second;
      properties.addFace(faces[fIdx]);
      fIdx++;
    }
  }

  // +X side
  for (std::size_t i = 0; i < sideXPos.size() - 1; ++i)
  {
    for (std::size_t j = 0; j < sideXPos[i].size() - 1; ++j)
    {
      faces[fIdx][0] = sideXPos[i + 0][j + 0].second;
      faces[fIdx][1] = sideXPos[i + 1][j + 0].second;
      faces[fIdx][2] = sideXPos[i + 0][j + 1].second;
      properties.addFace(faces[fIdx]);
      fIdx++;

      faces[fIdx][0] = sideXPos[i + 1][j + 1].second;
      faces[fIdx][1] = sideXPos[i + 0][j + 1].second;
      faces[fIdx][2] = sideXPos[i + 1][j + 0].second;
      properties.addFace(faces[fIdx]);
      fIdx++;
    }
  }

  // -Y side
  for (std::size_t i = 0; i < sideYNeg.size() - 1; ++i)
  {
    for (std::size_t j = 0; j < sideYNeg[i].size() - 1; ++j)
    {
      faces[fIdx][0] = sideYNeg[i + 0][j + 0].second;
      faces[fIdx][1] = sideYNeg[i + 0][j + 1].second;
      faces[fIdx][2] = sideYNeg[i + 1][j + 0].second;
      properties.addFace(faces[fIdx]);
      fIdx++;

      faces[fIdx][0] = sideYNeg[i + 1][j + 1].second;
      faces[fIdx][1] = sideYNeg[i + 1][j + 0].second;
      faces[fIdx][2] = sideYNeg[i + 0][j + 1].second;
      properties.addFace(faces[fIdx]);
      fIdx++;
    }
  }

  // +Y side
  for (std::size_t i = 0; i < sideYPos.size() - 1; ++i)
  {
    for (std::size_t j = 0; j < sideYPos[i].size() - 1; ++j)
    {
      faces[fIdx][0] = sideYPos[i + 0][j + 0].second;
      faces[fIdx][1] = sideYPos[i + 1][j + 0].second;
      faces[fIdx][2] = sideYPos[i + 0][j + 1].second;
      properties.addFace(faces[fIdx]);
      fIdx++;

      faces[fIdx][0] = sideYPos[i + 1][j + 1].second;
      faces[fIdx][1] = sideYPos[i + 0][j + 1].second;
      faces[fIdx][2] = sideYPos[i + 1][j + 0].second;
      properties.addFace(faces[fIdx]);
      fIdx++;
    }
  }

  // -Z side
  for (std::size_t i = 0; i < sideZNeg.size() - 1; ++i)
  {
    for (std::size_t j = 0; j < sideZNeg[i].size() - 1; ++j)
    {
      faces[fIdx][0] = sideZNeg[i + 0][j + 0].second;
      faces[fIdx][1] = sideZNeg[i + 0][j + 1].second;
      faces[fIdx][2] = sideZNeg[i + 1][j + 0].second;
      properties.addFace(faces[fIdx]);
      fIdx++;

      faces[fIdx][0] = sideZNeg[i + 1][j + 1].second;
      faces[fIdx][1] = sideZNeg[i + 1][j + 0].second;
      faces[fIdx][2] = sideZNeg[i + 0][j + 1].second;
      properties.addFace(faces[fIdx]);
      fIdx++;
    }
  }

  // +Z side
  for (std::size_t i = 0; i < sideZPos.size() - 1; ++i)
  {
    for (std::size_t j = 0; j < sideZPos[i].size() - 1; ++j)
    {
      faces[fIdx][0] = sideZPos[i + 0][j + 0].second;
      faces[fIdx][1] = sideZPos[i + 1][j + 0].second;
      faces[fIdx][2] = sideZPos[i + 0][j + 1].second;
      properties.addFace(faces[fIdx]);
      fIdx++;

      faces[fIdx][0] = sideZPos[i + 1][j + 1].second;
      faces[fIdx][1] = sideZPos[i + 0][j + 1].second;
      faces[fIdx][2] = sideZPos[i + 1][j + 0].second;
      properties.addFace(faces[fIdx]);
      fIdx++;
    }
  }

  return properties;
}

//==============================================================================
void SoftBodyNodeHelper::setBox(SoftBodyNode*            _softBodyNode,
                                const Eigen::Vector3d&   _size,
                                const Eigen::Isometry3d& _localTransform,
                                const Eigen::Vector3i&   _frags,
                                double                   _totalMass,
                                double                   _vertexStiffness,
                                double                   _edgeStiffness,
                                double                   _dampingCoeff)
{
  assert(_softBodyNode != nullptr);
  _softBodyNode->setProperties(makeBoxProperties(_size,
                                                 _localTransform,
                                                 _frags,
                                                 _totalMass,
                                                 _vertexStiffness,
                                                 _edgeStiffness,
                                                 _dampingCoeff));
}

//==============================================================================
SoftBodyNode::UniqueProperties
SoftBodyNodeHelper::makeSinglePointMassProperties(
    double _totalMass,
    double _vertexStiffness,
    double _edgeStiffness,
    double _dampingCoeff)
{
  SoftBodyNode::UniqueProperties properties(
        _vertexStiffness, _edgeStiffness, _dampingCoeff);

  //----------------------------------------------------------------------------
  // Point masses
  //----------------------------------------------------------------------------
  // Number of point masses
  std::size_t nPointMasses = 1;

  // Mass per vertices
  double mass = _totalMass / nPointMasses;

  // Resting positions for each point mass
  std::vector<Eigen::Vector3d> restingPos(nPointMasses,
                                          Eigen::Vector3d::Zero());
  restingPos[0] = Eigen::Vector3d(+0.1, +0.1, +0.1);

  // Point masses
  for (std::size_t i = 0; i < nPointMasses; ++i)
  {
    PointMass::Properties point(restingPos[i], mass);
    properties.addPointMass(point);
  }

  return properties;
}

//==============================================================================
void SoftBodyNodeHelper::setSinglePointMass(SoftBodyNode* _softBodyNode,
                                        double _totalMass,
                                        double _vertexStiffness,
                                        double _edgeStiffness,
                                        double _dampingCoeff)
{
  assert(_softBodyNode != nullptr);
  _softBodyNode->setProperties(makeSinglePointMassProperties(
                                 _totalMass,
                                 _vertexStiffness,
                                 _edgeStiffness,
                                 _dampingCoeff));
}

//==============================================================================
SoftBodyNode::UniqueProperties SoftBodyNodeHelper::makeSphereProperties(
    double _radius,
    std::size_t _nSlices,
    std::size_t _nStacks,
    double _totalMass,
    double _vertexStiffness,
    double _edgeStiffness,
    double _dampingCoeff)
{
  return makeEllipsoidProperties(
        Eigen::Vector3d::Constant(_radius*2.0),
        _nSlices,
        _nStacks,
        _totalMass,
        _vertexStiffness,
        _edgeStiffness,
        _dampingCoeff);
}

//==============================================================================
SoftBodyNode::UniqueProperties SoftBodyNodeHelper::makeEllipsoidProperties(
    const Eigen::Vector3d& _size,
    std::size_t _nSlices,
    std::size_t _nStacks,
    double _totalMass,
    double _vertexStiffness,
    double _edgeStiffness,
    double _dampingCoeff)
{
  using namespace dart::math::suffixes;

  SoftBodyNode::UniqueProperties properties(_vertexStiffness,
                                            _edgeStiffness,
                                            _dampingCoeff);

  //----------------------------------------------------------------------------
  // Point masses
  //----------------------------------------------------------------------------
  // Number of point masses
  int nPointMasses = (_nStacks - 1) * _nSlices + 2;

  // Mass per vertices
  double mass = _totalMass / nPointMasses;

  // Resting positions for each point mass
  // -- top
  properties.addPointMass(
        PointMass::Properties(Eigen::Vector3d(0.0, 0.0, 0.5 * _size(2)), mass));

  // middle
  float drho = 1_pi / _nStacks;
  float dtheta = 2_pi / _nSlices;
  for (std::size_t i = 1; i < _nStacks; i++)
  {
    float rho = i * drho;
    float srho = (sin(rho));
    float crho = (cos(rho));

    for (std::size_t j = 0; j < _nSlices; j++)
    {
      float theta = (j == _nSlices) ? 0.0f : j * dtheta;
      float stheta = (-sin(theta));
      float ctheta = (cos(theta));

      float x = 0.5 * srho * stheta;
      float y = 0.5 * srho * ctheta;
      float z = 0.5 * crho;

      properties.addPointMass(
            PointMass::Properties(
              Eigen::Vector3d(x * _size(0), y * _size(1), z * _size(2)), mass));
    }
  }
  // bottom
  properties.addPointMass(
      PointMass::Properties(Eigen::Vector3d(0.0, 0.0, -0.5 * _size(2)), mass));


  //----------------------------------------------------------------------------
  // Edges
  //----------------------------------------------------------------------------
  // a) longitudinal
  // -- top
  for (std::size_t i = 0; i < _nSlices; i++)
    properties.connectPointMasses(0, i + 1);
  // -- middle
  for (std::size_t i = 0; i < _nStacks - 2; i++)
    for (std::size_t j = 0; j < _nSlices; j++)
      properties.connectPointMasses(i*_nSlices + j + 1,
                                        (i + 1)*_nSlices + j + 1);
  // -- bottom
  for (std::size_t i = 0; i < _nSlices; i++)
    properties.connectPointMasses((_nStacks-1)*_nSlices + 1,
                                      (_nStacks-2)*_nSlices + i + 1);

  // b) latitudinal
  for (std::size_t i = 0; i < _nStacks - 1; i++)
  {
    for (std::size_t j = 0; j < _nSlices - 1; j++)
    {
      properties.connectPointMasses(i*_nSlices + j + 1, i*_nSlices + j + 2);
    }
    properties.connectPointMasses((i+1)*_nSlices, i*_nSlices + 1);
  }

  // c) cross (shear)
  for (std::size_t i = 0; i < _nStacks - 2; i++)
  {
    for (std::size_t j = 0; j < _nSlices - 1; j++)
    {
      properties.connectPointMasses(i * _nSlices + j + 1,
                                        (i + 1) * _nSlices + j + 2);
      properties.connectPointMasses(i * _nSlices + j + 2,
                                        (i + 1) * _nSlices + j + 1);
    }
    properties.connectPointMasses((i+1)*_nSlices, (i+1)*_nSlices + 1);
    properties.connectPointMasses(i*_nSlices + 1, (i+2)*_nSlices);
  }

  //----------------------------------------------------------------------------
  // Faces
  //----------------------------------------------------------------------------
  int meshIdx1 = 0;
  int meshIdx2 = 0;
  int meshIdx3 = 0;

  // top
  meshIdx1 = 0;
  for (std::size_t i = 0; i < _nSlices - 1; i++)
  {
    meshIdx2 = i + 1;
    meshIdx3 = i + 2;
    properties.addFace(Eigen::Vector3i(meshIdx1, meshIdx2, meshIdx3));
  }
  meshIdx2 = _nSlices;
  meshIdx3 = 1;
  properties.addFace(Eigen::Vector3i(meshIdx1, meshIdx2, meshIdx3));

  // middle
  for (std::size_t i = 0; i < _nStacks - 2; i++)
  {
    for (std::size_t j = 0; j < _nSlices - 1; j++)
    {
      meshIdx1 = i*_nSlices + j + 1;
      meshIdx2 = (i + 1)*_nSlices + j + 1;
      meshIdx3 = i*_nSlices + j + 2;
      properties.addFace(Eigen::Vector3i(meshIdx1, meshIdx2, meshIdx3));

      meshIdx1 = i*_nSlices + j + 2;
      meshIdx2 = (i + 1)*_nSlices + j + 1;
      meshIdx3 = (i + 1)*_nSlices + j + 2;
      properties.addFace(Eigen::Vector3i(meshIdx1, meshIdx2, meshIdx3));
    }

    meshIdx1 = (i + 1)*_nSlices;
    meshIdx2 = (i + 2)*_nSlices;
    meshIdx3 = i*_nSlices + 1;
    properties.addFace(Eigen::Vector3i(meshIdx1, meshIdx2, meshIdx3));

    meshIdx1 = i*_nSlices + 1;
    meshIdx2 = (i + 2)*_nSlices;
    meshIdx3 = (i + 2)*_nSlices + 1;
    properties.addFace(Eigen::Vector3i(meshIdx1, meshIdx2, meshIdx3));
  }

  // bottom
  meshIdx1 = (_nStacks-1)*_nSlices + 1;
  for (std::size_t i = 0; i < _nSlices - 1; i++)
  {
    meshIdx2 = (_nStacks-2)*_nSlices + i + 2;
    meshIdx3 = (_nStacks-2)*_nSlices + i + 1;
    properties.addFace(Eigen::Vector3i(meshIdx1, meshIdx2, meshIdx3));
  }
  meshIdx2 = (_nStacks-2)*_nSlices + 2;
  meshIdx3 = (_nStacks-1)*_nSlices;
  properties.addFace(Eigen::Vector3i(meshIdx1, meshIdx2, meshIdx3));

  return properties;
}

//==============================================================================
void SoftBodyNodeHelper::setEllipsoid(SoftBodyNode*          _softBodyNode,
                                      const Eigen::Vector3d& _size,
                                      std::size_t                 _nSlices,
                                      std::size_t                 _nStacks,
                                      double                 _totalMass,
                                      double                 _vertexStiffness,
                                      double                 _edgeStiffness,
                                      double                 _dampingCoeff)
{
  assert(_softBodyNode != nullptr);
  _softBodyNode->setProperties(makeEllipsoidProperties(_size,
                                                       _nSlices,
                                                       _nStacks,
                                                       _totalMass,
                                                       _vertexStiffness,
                                                       _edgeStiffness,
                                                       _dampingCoeff));
}

//==============================================================================
SoftBodyNode::UniqueProperties SoftBodyNodeHelper::makeCylinderProperties(
    double _radius,
    double _height,
    std::size_t _nSlices,
    std::size_t _nStacks,
    std::size_t _nRings,
    double _totalMass,
    double _vertexStiffness,
    double _edgeStiffness,
    double _dampingCoeff)
{
  using namespace dart::math::suffixes;

  SoftBodyNode::UniqueProperties properties(_vertexStiffness,
                                            _edgeStiffness,
                                            _dampingCoeff);

  //----------------------------------------------------------------------------
  // Point masses
  //----------------------------------------------------------------------------
  // Number of point masses
  std::size_t nTopPointMasses = _nSlices * (_nRings - 1) + 1;
  std::size_t nDrumPointMasses = (_nStacks + 1) * _nSlices;
  std::size_t nTotalMasses = nDrumPointMasses + 2 * nTopPointMasses;

  // Mass per vertices
  double mass = _totalMass / nTotalMasses;

  // Resting positions for each point mass
  float dradius = _radius / static_cast<float>(_nRings);
  float dtheta = 2_pi / static_cast<float>(_nSlices);

  // -- top
  properties.addPointMass(PointMass::Properties(
            Eigen::Vector3d(0.0, 0.0, 0.5 * _height), mass));

  for (std::size_t i = 1; i < _nRings; ++i)
  {
    float z = 0.5;
    float radius = i * dradius;

    for (std::size_t j = 0; j < _nSlices; j++)
    {
      float theta = (j == _nSlices) ? 0.0f : j * dtheta;
      float stheta = (-sin(theta));
      float ctheta = (cos(theta));

      float x = stheta;
      float y = ctheta;

      properties.addPointMass(PointMass::Properties(
            Eigen::Vector3d(x * radius, y * radius, z * _height), mass));
    }
  }

  // -- middle
  float dz     = -1.0 / static_cast<float>(_nStacks);
  for (std::size_t i = 0; i < _nStacks + 1; i++)
  {
    float z = 0.5 + i * dz;

    for (std::size_t j = 0; j < _nSlices; j++)
    {
      float theta = (j == _nSlices) ? 0.0f : j * dtheta;
      float stheta = (-sin(theta));
      float ctheta = (cos(theta));

      float x = stheta;
      float y = ctheta;

      properties.addPointMass(PointMass::Properties(
            Eigen::Vector3d(x * _radius, y * _radius, z * _height), mass));
    }
  }

  // -- bottom
  for (std::size_t i = 1; i < _nRings; ++i)
  {
    float z = -0.5;
    float radius = _radius - i * dradius;

    for (std::size_t j = 0; j < _nSlices; j++)
    {
      float theta = (j == _nSlices) ? 0.0f : j * dtheta;
      float stheta = (-sin(theta));
      float ctheta = (cos(theta));

      float x = stheta;
      float y = ctheta;

      properties.addPointMass(PointMass::Properties(
            Eigen::Vector3d(x * radius, y * radius, z * _height), mass));
    }
  }

  properties.addPointMass(PointMass::Properties(
        Eigen::Vector3d(0.0, 0.0, -0.5 * _height), mass));

  //----------------------------------------------------------------------------
  // Edges
  //----------------------------------------------------------------------------
  // A. Drum part

  // a) longitudinal
  // -- top
  for (std::size_t i = 0; i < _nSlices; i++)
    properties.connectPointMasses(0, i + 1);
  for (std::size_t i = 0; i < _nRings - 1; i++)
  {
    for (std::size_t j = 0; j < _nSlices; j++)
    {
      properties.connectPointMasses(
            _nSlices + 1 + (i + 0) * _nSlices + j,
            _nSlices + 1 + (i + 1) * _nSlices + j);
    }
  }
  // -- middle
  for (std::size_t i = 0; i < _nStacks - 1; i++)
  {
    for (std::size_t j = 0; j < _nSlices; j++)
    {
      properties.connectPointMasses(
            nTopPointMasses + (i + 0) * _nSlices + j,
            nTopPointMasses + (i + 1) * _nSlices + j);
    }
  }
  // -- bottom
  for (std::size_t i = 0; i < _nRings - 1; i++)
  {
    for (std::size_t j = 0; j < _nSlices; j++)
    {
      properties.connectPointMasses(
            nTopPointMasses + (nDrumPointMasses - _nSlices)
            + (i + 0) * _nSlices + j,
            nTopPointMasses + (nDrumPointMasses - _nSlices)
            + (i + 1) * _nSlices + j);
    }
  }
  for (std::size_t i = 1; i < _nSlices; i++)
    properties.connectPointMasses(nTotalMasses - 1 - i,
                                      nTotalMasses - 1);

  // b) latitudinal
  for (std::size_t i = 0; i < _nStacks; i++)
  {
    for (std::size_t j = 0; j < _nSlices - 1; j++)
    {
      properties.connectPointMasses(
            nTopPointMasses + i * _nSlices + j + 0,
            nTopPointMasses + i * _nSlices + j + 1);
    }

    properties.connectPointMasses(
          nTopPointMasses + (i + 0) * _nSlices + _nSlices - 1,
          nTopPointMasses + (i + 0) * _nSlices);
  }
  // -- disk parts
  // TODO(JS): No latitudinal connections for top and bottom disks

  // c) cross (shear)
  // -- drum part
  for (std::size_t i = 0; i < _nStacks - 2; i++)
  {
    for (std::size_t j = 0; j < _nSlices - 1; j++)
    {
      properties.connectPointMasses(
            nTopPointMasses + (i + 0) * _nSlices + j + 0,
            nTopPointMasses + (i + 1) * _nSlices + j + 1);
      properties.connectPointMasses(
            nTopPointMasses + (i + 0) * _nSlices + j + 1,
            nTopPointMasses + (i + 1) * _nSlices + j + 0);
    }

    properties.connectPointMasses(
          nTopPointMasses + (i + 0) * _nSlices + _nSlices - 1,
          nTopPointMasses + (i + 1) * _nSlices);
    properties.connectPointMasses(
          nTopPointMasses + (i + 0) * _nSlices,
          nTopPointMasses + (i + 1) * _nSlices + _nSlices - 1);
  }
  // -- disk parts
  // TODO(JS): No cross connections for top and bottom disks

  //----------------------------------------------------------------------------
  // Faces
  //----------------------------------------------------------------------------
  int meshIdx1 = 0;
  int meshIdx2 = 0;
  int meshIdx3 = 0;

  // top
  std::size_t nConePointMass = 1;
  meshIdx1 = 0;
  for (std::size_t i = 0; i < _nSlices - 1; i++)
  {
    meshIdx2 = i + 1;
    meshIdx3 = i + 2;
    properties.addFace(Eigen::Vector3i(meshIdx1, meshIdx2, meshIdx3));
  }
  meshIdx2 = _nSlices;
  meshIdx3 = 1;
  properties.addFace(Eigen::Vector3i(meshIdx1, meshIdx2, meshIdx3));
  for (std::size_t i = 0; i < _nRings - 1; ++i)
  {
    for (std::size_t j = 0; j < _nSlices - 1; ++j)
    {
      meshIdx1 = (i + 0) * _nSlices + j;
      meshIdx2 = (i + 1) * _nSlices + j;
      meshIdx3 = (i + 0) * _nSlices + j + 1;
      properties.addFace(Eigen::Vector3i(nConePointMass + meshIdx1,
                                             nConePointMass + meshIdx2,
                                             nConePointMass + meshIdx3));

      meshIdx1 = (i + 0) * _nSlices + j + 1;
      meshIdx2 = (i + 1) * _nSlices + j;
      meshIdx3 = (i + 1) * _nSlices + j + 1;
      properties.addFace(Eigen::Vector3i(nConePointMass + meshIdx1,
                                             nConePointMass + meshIdx2,
                                             nConePointMass + meshIdx3));
    }

    meshIdx1 = (i + 0) * _nSlices + _nSlices - 1;
    meshIdx2 = (i + 1) * _nSlices + _nSlices - 1;
    meshIdx3 = (i + 0) * _nSlices + 0;
    properties.addFace(Eigen::Vector3i(nConePointMass + meshIdx1,
                                           nConePointMass + meshIdx2,
                                           nConePointMass + meshIdx3));

    meshIdx1 = (i + 0) * _nSlices + 0;
    meshIdx2 = (i + 1) * _nSlices + _nSlices - 1;
    meshIdx3 = (i + 1) * _nSlices + 0;
    properties.addFace(Eigen::Vector3i(nConePointMass + meshIdx1,
                                           nConePointMass + meshIdx2,
                                           nConePointMass + meshIdx3));
  }

  // middle
  for (std::size_t i = 0; i < _nStacks; i++)
  {
    for (std::size_t j = 0; j < _nSlices - 1; j++)
    {
      meshIdx1 = (i + 0) * _nSlices + j;
      meshIdx2 = (i + 1) * _nSlices + j;
      meshIdx3 = (i + 0) * _nSlices + j + 1;
      properties.addFace(Eigen::Vector3i(nTopPointMasses + meshIdx1,
                                             nTopPointMasses + meshIdx2,
                                             nTopPointMasses + meshIdx3));

      meshIdx1 = (i + 0) * _nSlices + j + 1;
      meshIdx2 = (i + 1) * _nSlices + j;
      meshIdx3 = (i + 1) * _nSlices + j + 1;
      properties.addFace(Eigen::Vector3i(nTopPointMasses + meshIdx1,
                                             nTopPointMasses + meshIdx2,
                                             nTopPointMasses + meshIdx3));
    }

    meshIdx1 = (i + 0) * _nSlices;
    meshIdx2 = (i + 0) * _nSlices + _nSlices - 1;
    meshIdx3 = (i + 1) * _nSlices;
    properties.addFace(Eigen::Vector3i(nTopPointMasses + meshIdx1,
                                           nTopPointMasses + meshIdx2,
                                           nTopPointMasses + meshIdx3));

    meshIdx1 = (i + 0) * _nSlices + _nSlices - 1;
    meshIdx2 = (i + 1) * _nSlices + _nSlices - 1;
    meshIdx3 = (i + 1) * _nSlices + 0;
    properties.addFace(Eigen::Vector3i(nTopPointMasses + meshIdx1,
                                           nTopPointMasses + meshIdx2,
                                           nTopPointMasses + meshIdx3));
  }

  // bottom
  for (std::size_t i = 0; i < _nRings - 1; ++i)
  {
    for (std::size_t j = 0; j < _nSlices - 1; ++j)
    {
      meshIdx1 = (i + 0) * _nSlices + j;
      meshIdx2 = (i + 1) * _nSlices + j;
      meshIdx3 = (i + 0) * _nSlices + j + 1;
      properties.addFace(
            Eigen::Vector3i(
              nTopPointMasses + (nDrumPointMasses - _nSlices) + meshIdx1,
              nTopPointMasses + (nDrumPointMasses - _nSlices) + meshIdx2,
              nTopPointMasses + (nDrumPointMasses - _nSlices) + meshIdx3));

      meshIdx1 = (i + 0) * _nSlices + j + 1;
      meshIdx2 = (i + 1) * _nSlices + j;
      meshIdx3 = (i + 1) * _nSlices + j + 1;
      properties.addFace(
            Eigen::Vector3i(
              nTopPointMasses + (nDrumPointMasses - _nSlices) + meshIdx1,
              nTopPointMasses + (nDrumPointMasses - _nSlices) + meshIdx2,
              nTopPointMasses + (nDrumPointMasses - _nSlices) + meshIdx3));
    }

    meshIdx1 = (i + 0) * _nSlices + _nSlices - 1;
    meshIdx2 = (i + 1) * _nSlices + _nSlices - 1;
    meshIdx3 = (i + 0) * _nSlices + 0;
    properties.addFace(
          Eigen::Vector3i(
            nTopPointMasses + (nDrumPointMasses - _nSlices) + meshIdx1,
            nTopPointMasses + (nDrumPointMasses - _nSlices) + meshIdx2,
            nTopPointMasses + (nDrumPointMasses - _nSlices) + meshIdx3));

    meshIdx1 = (i + 0) * _nSlices + 0;
    meshIdx2 = (i + 1) * _nSlices + _nSlices - 1;
    meshIdx3 = (i + 1) * _nSlices + 0;
    properties.addFace(
          Eigen::Vector3i(
            nTopPointMasses + (nDrumPointMasses - _nSlices) + meshIdx1,
            nTopPointMasses + (nDrumPointMasses - _nSlices) + meshIdx2,
            nTopPointMasses + (nDrumPointMasses - _nSlices) + meshIdx3));
  }
  meshIdx1 = 1;
  for (std::size_t i = 0; i < _nSlices - 1; i++)
  {
    meshIdx2 = i + 2;
    meshIdx3 = i + 3;
    properties.addFace(Eigen::Vector3i(
                             nTotalMasses - meshIdx1,
                             nTotalMasses - meshIdx2,
                             nTotalMasses - meshIdx3));
  }
  meshIdx2 = _nSlices + 1;
  meshIdx3 = 2;
  properties.addFace(Eigen::Vector3i(
                           nTotalMasses - meshIdx1,
                           nTotalMasses - meshIdx2,
                           nTotalMasses - meshIdx3));

  return properties;
}

//==============================================================================
void SoftBodyNodeHelper::setCylinder(SoftBodyNode* _softBodyNode,
                                     double _radius,
                                     double _height,
                                     std::size_t _nSlices,
                                     std::size_t _nStacks,
                                     std::size_t _nRings,
                                     double _totalMass,
                                     double _vertexStiffness,
                                     double _edgeStiffness,
                                     double _dampingCoeff)
{
  assert(_softBodyNode != nullptr);
  _softBodyNode->setProperties(makeCylinderProperties(_radius,
                                                      _height,
                                                      _nSlices,
                                                      _nStacks,
                                                      _nRings,
                                                      _totalMass,
                                                      _vertexStiffness,
                                                      _edgeStiffness,
                                                      _dampingCoeff));
}

}  // namespace dynamics
}  // namespace dart

