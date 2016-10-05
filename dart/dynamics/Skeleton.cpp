/*
 * Copyright (c) 2011-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2011-2016, Humanoid Lab, Georgia Tech Research Corporation
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

#include "dart/dynamics/Skeleton.hpp"

#include <algorithm>
#include <queue>
#include <string>
#include <vector>

#include "dart/common/Console.hpp"
#include "dart/common/Deprecated.hpp"
#include "dart/common/StlHelpers.hpp"
#include "dart/math/Geometry.hpp"
#include "dart/math/Helpers.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/DegreeOfFreedom.hpp"
#include "dart/dynamics/Joint.hpp"
#include "dart/dynamics/ShapeNode.hpp"
#include "dart/dynamics/EndEffector.hpp"
#include "dart/dynamics/InverseKinematics.hpp"
#include "dart/dynamics/Marker.hpp"
#include "dart/dynamics/PointMass.hpp"
#include "dart/dynamics/SoftBodyNode.hpp"

#define SET_ALL_FLAGS( X ) for(auto& cache : mTreeCache) cache.mDirty. X = true;\
                           mSkelCache.mDirty. X = true;

#define SET_FLAG( Y, X ) mTreeCache[ Y ].mDirty. X = true;                      \
                         mSkelCache.mDirty. X = true;

#define ON_ALL_TREES( X ) for(std::size_t i=0; i < mTreeCache.size(); ++i) X (i);


#define CHECK_CONFIG_VECTOR_SIZE( V )                                           \
  if( V .size() > 0 )                                                           \
  {                                                                             \
    if(nonzero_size != INVALID_INDEX                                            \
       && V .size() != static_cast<int>(nonzero_size))                          \
    {                                                                           \
      dterr << "[Skeleton::Configuration] Mismatch in size of vector [" << #V   \
            << "] (expected " << nonzero_size << " | found " << V . size()      \
            << "\n";                                                            \
      assert(false);                                                            \
    }                                                                           \
    else if(nonzero_size == INVALID_INDEX)                                      \
      nonzero_size = V .size();                                                 \
  }


namespace dart {
namespace dynamics {

namespace detail {

//==============================================================================
/// Templated function for passing each entry in a std::vector<Data> into each
/// member of an array of Objects belonging to some Owner class.
///
/// The ObjectBase argument should be the base class of Object in which the
/// setData function is defined. In many cases, ObjectBase may be the same as
/// Object, but it is not always.
//
// TODO(MXG): Consider putting this in an accessible header if it might be
// useful in other places.
template <class Owner, class Object, class ObjectBase, class Data,
          std::size_t (Owner::*getNumObjects)() const,
          Object* (Owner::*getObject)(std::size_t),
          void (ObjectBase::*setData)(const Data&)>
void setAllMemberObjectData(Owner* owner, const std::vector<Data>& data)
{
  if(!owner)
  {
    dterr << "[setAllMemberObjectData] Attempting to set ["
          << typeid(Data).name() << "] of every [" << typeid(Object).name()
          << "] in a nullptr [" << typeid(Owner).name() << "]. Please report "
          << "this as a bug!\n";
    assert(false);
    return;
  }

  std::size_t numObjects = (owner->*getNumObjects)();

  if(data.size() != numObjects)
  {
    dtwarn << "[setAllMemberObjectData] Mismatch between the number of ["
           << typeid(Object).name() << "] member objects (" << numObjects
           << ") in the [" << typeid(Owner).name() << "] named ["
           << owner->getName() << "] (" << owner << ") and the number of ["
           << typeid(Object).name() << "] which is (" << data.size()
           << ") while setting [" << typeid(Data).name() << "]\n"
           << " -- We will set (" << std::min(numObjects, data.size())
           << ") of them.\n";
    numObjects = std::min(numObjects, data.size());
  }

  for(std::size_t i=0; i < numObjects; ++i)
    ((owner->*getObject)(i)->*setData)(data[i]);
}

//==============================================================================
/// Templated function for aggregating a std::vector<Data> out of each member of
/// an array of Objects belonging to some Owner class.
///
/// The ObjectBase argument should be the base class of Object in which the
/// getData function is defined. In many cases, ObjectBase may be the same as
/// Object, but it is not always.
//
// TODO(MXG): Consider putting this in an accessible header if it might be
// useful in other places.
template <class Owner, class Object, class ObjectBase, class Data,
          std::size_t (Owner::*getNumObjects)() const,
          const Object* (Owner::*getObject)(std::size_t) const,
          Data (ObjectBase::*getData)() const>
std::vector<Data> getAllMemberObjectData(const Owner* owner)
{
  if(!owner)
  {
    dterr << "[getAllMemberObjectData] Attempting to get the ["
          << typeid(Data).name() << "] from every [" << typeid(Object).name()
          << "] in a nullptr [" << typeid(Owner).name() << "]. Please report "
          << "this as a bug!\n";
    assert(false);
    return std::vector<Data>();
  }

  const std::size_t numObjects = (owner->*getNumObjects)();
  std::vector<Data> data;
  data.reserve(numObjects);

  for(std::size_t i=0; i < numObjects; ++i)
    data.push_back(((owner->*getObject)(i)->*getData)());

  return data;
}

//==============================================================================
SkeletonAspectProperties::SkeletonAspectProperties(
    const std::string& _name,
    bool _isMobile,
    const Eigen::Vector3d& _gravity,
    double _timeStep,
    bool _enabledSelfCollisionCheck,
    bool _enableAdjacentBodyCheck)
  : mName(_name),
    mIsMobile(_isMobile),
    mGravity(_gravity),
    mTimeStep(_timeStep),
    mEnabledSelfCollisionCheck(_enabledSelfCollisionCheck),
    mEnabledAdjacentBodyCheck(_enableAdjacentBodyCheck)
{
  // Do nothing
}

//==============================================================================
void setAllBodyNodeStates(Skeleton* skel, const BodyNodeStateVector& states)
{
  setAllMemberObjectData<
      Skeleton, BodyNode, common::Composite, common::Composite::State,
      &Skeleton::getNumBodyNodes, &Skeleton::getBodyNode,
      &common::Composite::setCompositeState>(skel, states);
}

//==============================================================================
BodyNodeStateVector getAllBodyNodeStates(const Skeleton* skel)
{
  return getAllMemberObjectData<
      Skeleton, BodyNode, common::Composite, common::Composite::State,
      &Skeleton::getNumBodyNodes, &Skeleton::getBodyNode,
      &common::Composite::getCompositeState>(skel);
}

//==============================================================================
void setAllBodyNodeProperties(
    Skeleton* skel, const BodyNodePropertiesVector& properties)
{
  setAllMemberObjectData<
      Skeleton, BodyNode, common::Composite, common::Composite::Properties,
      &Skeleton::getNumBodyNodes, &Skeleton::getBodyNode,
      &common::Composite::setCompositeProperties>(skel, properties);
}

//==============================================================================
BodyNodePropertiesVector getAllBodyNodeProperties(const Skeleton* skel)
{
  return getAllMemberObjectData<
      Skeleton, BodyNode, common::Composite, common::Composite::Properties,
      &Skeleton::getNumBodyNodes, &Skeleton::getBodyNode,
      &common::Composite::getCompositeProperties>(skel);
}

//==============================================================================
void setAllJointStates(Skeleton* skel, const BodyNodeStateVector& states)
{
  setAllMemberObjectData<
      Skeleton, Joint, common::Composite, common::Composite::State,
      &Skeleton::getNumJoints, &Skeleton::getJoint,
      &common::Composite::setCompositeState>(skel, states);
}

//==============================================================================
BodyNodeStateVector getAllJointStates(const Skeleton* skel)
{
  return getAllMemberObjectData<
      Skeleton, Joint, common::Composite, common::Composite::State,
      &Skeleton::getNumJoints, &Skeleton::getJoint,
      &common::Composite::getCompositeState>(skel);
}

//==============================================================================
void setAllJointProperties(
    Skeleton* skel, const BodyNodePropertiesVector& properties)
{
  setAllMemberObjectData<
      Skeleton, Joint, common::Composite, common::Composite::Properties,
      &Skeleton::getNumJoints, &Skeleton::getJoint,
      &common::Composite::setCompositeProperties>(skel, properties);
}

//==============================================================================
BodyNodePropertiesVector getAllJointProperties(const Skeleton* skel)
{
  return getAllMemberObjectData<
      Skeleton, Joint, common::Composite, common::Composite::Properties,
      &Skeleton::getNumJoints, &Skeleton::getJoint,
      &common::Composite::getCompositeProperties>(skel);
}

} // namespace detail

//==============================================================================
Skeleton::Configuration::Configuration(
    const Eigen::VectorXd& positions,
    const Eigen::VectorXd& velocities,
    const Eigen::VectorXd& accelerations,
    const Eigen::VectorXd& forces,
    const Eigen::VectorXd& commands)
  : mPositions(positions),
    mVelocities(velocities),
    mAccelerations(accelerations),
    mForces(forces),
    mCommands(commands)
{
  std::size_t nonzero_size = INVALID_INDEX;

  CHECK_CONFIG_VECTOR_SIZE(positions);
  CHECK_CONFIG_VECTOR_SIZE(velocities);
  CHECK_CONFIG_VECTOR_SIZE(accelerations);
  CHECK_CONFIG_VECTOR_SIZE(forces);
  CHECK_CONFIG_VECTOR_SIZE(commands);

  if(nonzero_size != INVALID_INDEX)
  {
    for(std::size_t i=0; i < nonzero_size; ++i)
      mIndices.push_back(i);
  }
}

//==============================================================================
Skeleton::Configuration::Configuration(
    const std::vector<std::size_t>& indices,
    const Eigen::VectorXd& positions,
    const Eigen::VectorXd& velocities,
    const Eigen::VectorXd& accelerations,
    const Eigen::VectorXd& forces,
    const Eigen::VectorXd& commands)
  : mIndices(indices),
    mPositions(positions),
    mVelocities(velocities),
    mAccelerations(accelerations),
    mForces(forces),
    mCommands(commands)
{
  std::size_t nonzero_size = indices.size();

  CHECK_CONFIG_VECTOR_SIZE(positions);
  CHECK_CONFIG_VECTOR_SIZE(velocities);
  CHECK_CONFIG_VECTOR_SIZE(accelerations);
  CHECK_CONFIG_VECTOR_SIZE(forces);
  CHECK_CONFIG_VECTOR_SIZE(commands);
}

//==============================================================================
#define RETURN_IF_CONFIG_VECTOR_IS_INEQ( V )                                    \
  if( V .size() != other. V .size() )                                           \
    return false;                                                               \
  if( V != other. V )                                                           \
    return false;

//==============================================================================
bool Skeleton::Configuration::operator==(const Configuration& other) const
{
  if(mIndices != other.mIndices)
    return false;

  RETURN_IF_CONFIG_VECTOR_IS_INEQ(mPositions);
  RETURN_IF_CONFIG_VECTOR_IS_INEQ(mVelocities);
  RETURN_IF_CONFIG_VECTOR_IS_INEQ(mAccelerations);
  RETURN_IF_CONFIG_VECTOR_IS_INEQ(mForces);
  RETURN_IF_CONFIG_VECTOR_IS_INEQ(mCommands);

  return true;
}

//==============================================================================
bool Skeleton::Configuration::operator!=(const Configuration& other) const
{
  return !(*this == other);
}

//==============================================================================
SkeletonPtr Skeleton::create(const std::string& _name)
{
  return create(AspectPropertiesData(_name));
}

//==============================================================================
SkeletonPtr Skeleton::create(const AspectPropertiesData& properties)
{
  SkeletonPtr skel(new Skeleton(properties));
  skel->setPtr(skel);
  return skel;
}

//==============================================================================
SkeletonPtr Skeleton::getPtr()
{
  return mPtr.lock();
}

//==============================================================================
ConstSkeletonPtr Skeleton::getPtr() const
{
  return mPtr.lock();
}

//==============================================================================
SkeletonPtr Skeleton::getSkeleton()
{
  return mPtr.lock();
}

//==============================================================================
ConstSkeletonPtr Skeleton::getSkeleton() const
{
  return mPtr.lock();
}

//==============================================================================
std::mutex& Skeleton::getMutex() const
{
  return mMutex;
}

//==============================================================================
Skeleton::~Skeleton()
{
  for (BodyNode* bn : mSkelCache.mBodyNodes)
    delete bn;
}

//==============================================================================
SkeletonPtr Skeleton::clone() const
{
  return clone(getName());
}

//==============================================================================
SkeletonPtr Skeleton::clone(const std::string& cloneName) const
{
  SkeletonPtr skelClone = Skeleton::create(cloneName);

  for(std::size_t i=0; i<getNumBodyNodes(); ++i)
  {
    // Create a clone of the parent Joint
    Joint* joint = getJoint(i)->clone();

    // Identify the original parent BodyNode
    const BodyNode* originalParent = getBodyNode(i)->getParentBodyNode();

    // Grab the parent BodyNode clone (using its name, which is guaranteed to be
    // unique), or use nullptr if this is a root BodyNode
    BodyNode* parentClone = (originalParent == nullptr)? nullptr :
          skelClone->getBodyNode(originalParent->getName());

    if( (nullptr != originalParent) && (nullptr == parentClone) )
    {
      dterr << "[Skeleton::clone] Failed to find a clone of BodyNode named ["
            << originalParent->getName() << "] which is needed as the parent "
            << "of the BodyNode named [" << getBodyNode(i)->getName()
            << "] and should already have been created. Please report this as "
            << "a bug!\n";
    }

    BodyNode* newBody = getBodyNode(i)->clone(parentClone, joint, false);

    // The IK module gets cloned by the Skeleton and not by the BodyNode,
    // because IK modules rely on the Skeleton's structure and indexing. If the
    // IK was cloned by the BodyNode into a Skeleton that has a different
    // structure, then there is no guarantee that it will continue to work
    // correctly.
    if(getBodyNode(i)->getIK())
      newBody->mIK = getBodyNode(i)->getIK()->clone(newBody);

    skelClone->registerBodyNode(newBody);
  }

  // Clone over the nodes in such a way that their indexing will match up with
  // the original
  for(const auto& nodeType : mNodeMap)
  {
    for(const auto& node : nodeType.second)
    {
      const BodyNode* originalBn = node->getBodyNodePtr();
      BodyNode* newBn = skelClone->getBodyNode(originalBn->getName());
      node->cloneNode(newBn)->attach();
    }
  }

  skelClone->setProperties(getAspectProperties());
  skelClone->setName(cloneName);
  skelClone->setState(getState());

  return skelClone;
}

//==============================================================================
#define SET_CONFIG_VECTOR( V )                                                  \
  if(configuration.m ## V . size() > 0)                                         \
  {                                                                             \
    if(static_cast<int>(configuration.mIndices.size()) !=                       \
        configuration.m ## V .size())                                           \
    {                                                                           \
      dterr << "[Skeleton::setConfiguration] Mismatch in size of vector ["      \
            << #V << "] (expected " << configuration.mIndices.size()            \
            << " | found " << configuration.m ## V .size() << "\n";             \
      assert(false);                                                            \
    }                                                                           \
    else                                                                        \
      set ## V ( configuration.mIndices, configuration.m ## V );                \
  }

//==============================================================================
void Skeleton::setConfiguration(const Configuration& configuration)
{
  SET_CONFIG_VECTOR(Positions);
  SET_CONFIG_VECTOR(Velocities);
  SET_CONFIG_VECTOR(Accelerations);
  SET_CONFIG_VECTOR(Forces);
  SET_CONFIG_VECTOR(Commands);
}

//==============================================================================
Skeleton::Configuration Skeleton::getConfiguration(int flags) const
{
  std::vector<std::size_t> indices;
  for(std::size_t i=0; i < getNumDofs(); ++i)
    indices.push_back(i);

  return getConfiguration(indices, flags);
}

//==============================================================================
Skeleton::Configuration Skeleton::getConfiguration(
    const std::vector<std::size_t>& indices, int flags) const
{
  Configuration config(indices);
  if(flags == CONFIG_NOTHING)
    return config;

  if( (flags & CONFIG_POSITIONS) == CONFIG_POSITIONS )
    config.mPositions = getPositions(indices);

  if( (flags & CONFIG_VELOCITIES) == CONFIG_VELOCITIES )
    config.mVelocities = getVelocities(indices);

  if( (flags & CONFIG_ACCELERATIONS) == CONFIG_ACCELERATIONS )
    config.mAccelerations = getAccelerations(indices);

  if( (flags & CONFIG_FORCES) == CONFIG_FORCES )
    config.mForces = getForces(indices);

  if( (flags & CONFIG_COMMANDS) == CONFIG_COMMANDS )
    config.mCommands = getCommands(indices);

  return config;
}

//==============================================================================
void Skeleton::setState(const State& state)
{
  setCompositeState(state);
}

//==============================================================================
Skeleton::State Skeleton::getState() const
{
  return getCompositeState();
}

//==============================================================================
void Skeleton::setProperties(const Properties& properties)
{
  setCompositeProperties(properties);
}

//==============================================================================
Skeleton::Properties Skeleton::getProperties() const
{
  return getCompositeProperties();
}

//==============================================================================
void Skeleton::setProperties(const AspectProperties& properties)
{
  setAspectProperties(properties);
}

//==============================================================================
void Skeleton::setAspectProperties(const AspectProperties& properties)
{
  setName(properties.mName);
  setMobile(properties.mIsMobile);
  setGravity(properties.mGravity);
  setTimeStep(properties.mTimeStep);
  setSelfCollisionCheck(properties.mEnabledSelfCollisionCheck);
  setAdjacentBodyCheck(properties.mEnabledAdjacentBodyCheck);
}

//==============================================================================
const Skeleton::AspectProperties& Skeleton::getSkeletonProperties() const
{
  return mAspectProperties;
}

//==============================================================================
const std::string& Skeleton::setName(const std::string& _name)
{
  if(_name == mAspectProperties.mName && !_name.empty())
    return mAspectProperties.mName;

  const std::string oldName = mAspectProperties.mName;
  mAspectProperties.mName = _name;

  mNameMgrForBodyNodes.setManagerName(
        "Skeleton::BodyNode | "+mAspectProperties.mName);
  mNameMgrForSoftBodyNodes.setManagerName(
        "Skeleton::SoftBodyNode | "+mAspectProperties.mName);
  mNameMgrForJoints.setManagerName(
        "Skeleton::Joint | "+mAspectProperties.mName);
  mNameMgrForDofs.setManagerName(
        "Skeleton::DegreeOfFreedom | "+mAspectProperties.mName);

  for(auto& mgr : mNodeNameMgrMap)
    mgr.second.setManagerName( std::string("Skeleton::") + mgr.first.name()
                               + " | " + mAspectProperties.mName );

  ConstMetaSkeletonPtr me = mPtr.lock();
  mNameChangedSignal.raise(me, oldName, mAspectProperties.mName);

  return mAspectProperties.mName;
}

//==============================================================================
const std::string& Skeleton::getName() const
{
  return mAspectProperties.mName;
}

//==============================================================================
const std::string& Skeleton::addEntryToBodyNodeNameMgr(BodyNode* _newNode)
{
  _newNode->BodyNode::mAspectProperties.mName =
      mNameMgrForBodyNodes.issueNewNameAndAdd(_newNode->getName(), _newNode);

  return _newNode->BodyNode::mAspectProperties.mName;
}

//==============================================================================
const std::string& Skeleton::addEntryToJointNameMgr(Joint* _newJoint,
                                                    bool _updateDofNames)
{
  _newJoint->mAspectProperties.mName =
      mNameMgrForJoints.issueNewNameAndAdd(_newJoint->getName(), _newJoint);

  if(_updateDofNames)
    _newJoint->updateDegreeOfFreedomNames();

  return _newJoint->mAspectProperties.mName;
}

//==============================================================================
void Skeleton::addEntryToSoftBodyNodeNameMgr(SoftBodyNode* _newNode)
{
  // Note: This doesn't need the same checks as BodyNode and Joint, because
  // its name has already been resolved against all the BodyNodes, which includes
  // all SoftBodyNodes.
  mNameMgrForSoftBodyNodes.addName(_newNode->getName(), _newNode);
}

//==============================================================================
void Skeleton::enableSelfCollision(bool enableAdjacentBodyCheck)
{
  enableSelfCollisionCheck();
  setAdjacentBodyCheck(enableAdjacentBodyCheck);
}

//==============================================================================
void Skeleton::disableSelfCollision()
{
  disableSelfCollisionCheck();
  setAdjacentBodyCheck(false);
}

//==============================================================================
void Skeleton::setSelfCollisionCheck(bool enable)
{
  mAspectProperties.mEnabledSelfCollisionCheck = enable;
}

//==============================================================================
bool Skeleton::getSelfCollisionCheck() const
{
  return mAspectProperties.mEnabledSelfCollisionCheck;
}

//==============================================================================
void Skeleton::enableSelfCollisionCheck()
{
  setSelfCollisionCheck(true);
}

//==============================================================================
void Skeleton::disableSelfCollisionCheck()
{
  setSelfCollisionCheck(false);
}

//==============================================================================
bool Skeleton::isEnabledSelfCollisionCheck() const
{
  return getSelfCollisionCheck();
}

//==============================================================================
void Skeleton::setAdjacentBodyCheck(bool enable)
{
  mAspectProperties.mEnabledAdjacentBodyCheck = enable;
}

//==============================================================================
bool Skeleton::getAdjacentBodyCheck() const
{
  return mAspectProperties.mEnabledAdjacentBodyCheck;
}

//==============================================================================
void Skeleton::enableAdjacentBodyCheck()
{
  setAdjacentBodyCheck(true);
}

//==============================================================================
void Skeleton::disableAdjacentBodyCheck()
{
  setAdjacentBodyCheck(false);
}

//==============================================================================
bool Skeleton::isEnabledAdjacentBodyCheck() const
{
  return getAdjacentBodyCheck();
}

//==============================================================================
void Skeleton::setMobile(bool _isMobile)
{
  mAspectProperties.mIsMobile = _isMobile;
}

//==============================================================================
bool Skeleton::isMobile() const
{
  return mAspectProperties.mIsMobile;
}

//==============================================================================
void Skeleton::setTimeStep(double _timeStep)
{
  assert(_timeStep > 0.0);
  mAspectProperties.mTimeStep = _timeStep;

  for(std::size_t i=0; i<mTreeCache.size(); ++i)
    notifyArticulatedInertiaUpdate(i);
}

//==============================================================================
double Skeleton::getTimeStep() const
{
  return mAspectProperties.mTimeStep;
}

//==============================================================================
void Skeleton::setGravity(const Eigen::Vector3d& _gravity)
{
  mAspectProperties.mGravity = _gravity;
  SET_ALL_FLAGS(mGravityForces);
  SET_ALL_FLAGS(mCoriolisAndGravityForces);
  ON_ALL_TREES(notifySupportUpdate);
}

//==============================================================================
const Eigen::Vector3d& Skeleton::getGravity() const
{
  return mAspectProperties.mGravity;
}

//==============================================================================
std::size_t Skeleton::getNumBodyNodes() const
{
  return mSkelCache.mBodyNodes.size();
}

//==============================================================================
std::size_t Skeleton::getNumRigidBodyNodes() const
{
  return mSkelCache.mBodyNodes.size() - mSoftBodyNodes.size();
}

//==============================================================================
std::size_t Skeleton::getNumSoftBodyNodes() const
{
  return mSoftBodyNodes.size();
}

//==============================================================================
std::size_t Skeleton::getNumTrees() const
{
  return mTreeCache.size();
}

//==============================================================================
BodyNode* Skeleton::getRootBodyNode(std::size_t _treeIdx)
{
  if( mTreeCache.size() > _treeIdx)
    return mTreeCache[_treeIdx].mBodyNodes[0];

  if(mTreeCache.size() == 0)
  {
    dterr << "[Skeleton::getRootBodyNode] Requested a root BodyNode from a "
          << "Skeleton with no BodyNodes!\n";
    assert(false);
  }
  else
  {
    dterr << "[Skeleton::getRootBodyNode] Requested invalid root BodyNode "
          << "index (" << _treeIdx << ")! Must be less than "
          << mTreeCache.size() << ".\n";
    assert(false);
  }

  return nullptr;
}

//==============================================================================
const BodyNode* Skeleton::getRootBodyNode(std::size_t _treeIdx) const
{
  return const_cast<Skeleton*>(this)->getRootBodyNode(_treeIdx);
}

//==============================================================================
BodyNode* Skeleton::getBodyNode(std::size_t _idx)
{
  return common::getVectorObjectIfAvailable<BodyNode*>(
        _idx, mSkelCache.mBodyNodes);
}

//==============================================================================
const BodyNode* Skeleton::getBodyNode(std::size_t _idx) const
{
  return common::getVectorObjectIfAvailable<BodyNode*>(
        _idx, mSkelCache.mBodyNodes);
}

//==============================================================================
SoftBodyNode* Skeleton::getSoftBodyNode(std::size_t _idx)
{
  return common::getVectorObjectIfAvailable<SoftBodyNode*>(
        _idx, mSoftBodyNodes);
}

//==============================================================================
const SoftBodyNode* Skeleton::getSoftBodyNode(std::size_t _idx) const
{
  return common::getVectorObjectIfAvailable<SoftBodyNode*>(
        _idx, mSoftBodyNodes);
}

//==============================================================================
BodyNode* Skeleton::getBodyNode(const std::string& _name)
{
  return mNameMgrForBodyNodes.getObject(_name);
}

//==============================================================================
const BodyNode* Skeleton::getBodyNode(const std::string& _name) const
{
  return mNameMgrForBodyNodes.getObject(_name);
}

//==============================================================================
SoftBodyNode* Skeleton::getSoftBodyNode(const std::string& _name)
{
  return mNameMgrForSoftBodyNodes.getObject(_name);
}

//==============================================================================
const SoftBodyNode* Skeleton::getSoftBodyNode(const std::string& _name) const
{
  return mNameMgrForSoftBodyNodes.getObject(_name);
}

//==============================================================================
template <class T>
static std::vector<const T*>& convertToConstPtrVector(
    const std::vector<T*>& vec, std::vector<const T*>& const_vec)
{
  const_vec.resize(vec.size());
  for(std::size_t i=0; i<vec.size(); ++i)
    const_vec[i] = vec[i];
  return const_vec;
}

//==============================================================================
const std::vector<BodyNode*>& Skeleton::getBodyNodes()
{
  return mSkelCache.mBodyNodes;
}

//==============================================================================
const std::vector<const BodyNode*>& Skeleton::getBodyNodes() const
{
  return convertToConstPtrVector<BodyNode>(
        mSkelCache.mBodyNodes, mSkelCache.mConstBodyNodes);
}

//==============================================================================
template <class ObjectT, std::size_t (ObjectT::*getIndexInSkeleton)() const>
static std::size_t templatedGetIndexOf(const Skeleton* _skel, const ObjectT* _obj,
                                  const std::string& _type, bool _warning)
{
  if(nullptr == _obj)
  {
    if(_warning)
    {
      dterr << "[Skeleton::getIndexOf] Requesting the index of a nullptr "
            << _type << " within the Skeleton [" << _skel->getName() << "] ("
            << _skel << ")!\n";
      assert(false);
    }
    return INVALID_INDEX;
  }

  if(_skel == _obj->getSkeleton().get())
    return (_obj->*getIndexInSkeleton)();

  if(_warning)
  {
    dterr << "[Skeleton::getIndexOf] Requesting the index of a " << _type << " ["
          << _obj->getName() << "] (" << _obj << ") from a Skeleton that it does "
          << "not belong to!\n";
    assert(false);
  }

  return INVALID_INDEX;
}

//==============================================================================
std::size_t Skeleton::getIndexOf(const BodyNode* _bn, bool _warning) const
{
  return templatedGetIndexOf<BodyNode, &BodyNode::getIndexInSkeleton>(
        this, _bn, "BodyNode", _warning);
}

//==============================================================================
const std::vector<BodyNode*>& Skeleton::getTreeBodyNodes(std::size_t _treeIdx)
{
  if(_treeIdx >= mTreeCache.size())
  {
    std::size_t count = mTreeCache.size();
    dterr << "[Skeleton::getTreeBodyNodes] Requesting an invalid tree ("
          << _treeIdx << ") "
          << (count > 0? (std::string("when the max tree index is (") + std::to_string(count-1) + ")\n" ) :
                         std::string("when there are no trees in this Skeleton\n") );
    assert(false);
  }

  return mTreeCache[_treeIdx].mBodyNodes;
}

//==============================================================================
std::vector<const BodyNode*> Skeleton::getTreeBodyNodes(std::size_t _treeIdx) const
{
  return convertToConstPtrVector<BodyNode>(
        mTreeCache[_treeIdx].mBodyNodes, mTreeCache[_treeIdx].mConstBodyNodes);
}

//==============================================================================
std::size_t Skeleton::getNumJoints() const
{
  // The number of joints and body nodes are identical
  return getNumBodyNodes();
}

//==============================================================================
Joint* Skeleton::getJoint(std::size_t _idx)
{
  BodyNode* bn = common::getVectorObjectIfAvailable<BodyNode*>(
                   _idx, mSkelCache.mBodyNodes);
  if (bn)
    return bn->getParentJoint();

  return nullptr;
}

//==============================================================================
const Joint* Skeleton::getJoint(std::size_t _idx) const
{
  return const_cast<Skeleton*>(this)->getJoint(_idx);
}

//==============================================================================
Joint* Skeleton::getJoint(const std::string& _name)
{
  return mNameMgrForJoints.getObject(_name);
}

//==============================================================================
const Joint* Skeleton::getJoint(const std::string& _name) const
{
  return mNameMgrForJoints.getObject(_name);
}

//==============================================================================
std::size_t Skeleton::getIndexOf(const Joint* _joint, bool _warning) const
{
  return templatedGetIndexOf<Joint, &Joint::getJointIndexInSkeleton>(
        this, _joint, "Joint", _warning);
}

//==============================================================================
std::size_t Skeleton::getNumDofs() const
{
  return mSkelCache.mDofs.size();
}

//==============================================================================
DegreeOfFreedom* Skeleton::getDof(std::size_t _idx)
{
  return common::getVectorObjectIfAvailable<DegreeOfFreedom*>(
        _idx, mSkelCache.mDofs);
}

//==============================================================================
const DegreeOfFreedom* Skeleton::getDof(std::size_t _idx) const
{
  return common::getVectorObjectIfAvailable<DegreeOfFreedom*>(
        _idx, mSkelCache.mDofs);
}

//==============================================================================
DegreeOfFreedom* Skeleton::getDof(const std::string& _name)
{
  return mNameMgrForDofs.getObject(_name);
}

//==============================================================================
const DegreeOfFreedom* Skeleton::getDof(const std::string& _name) const
{
  return mNameMgrForDofs.getObject(_name);
}

//==============================================================================
const std::vector<DegreeOfFreedom*>& Skeleton::getDofs()
{
  return mSkelCache.mDofs;
}

//==============================================================================
std::vector<const DegreeOfFreedom*> Skeleton::getDofs() const
{
  return convertToConstPtrVector<DegreeOfFreedom>(
        mSkelCache.mDofs, mSkelCache.mConstDofs);
}

//==============================================================================
std::size_t Skeleton::getIndexOf(const DegreeOfFreedom* _dof, bool _warning) const
{
  return templatedGetIndexOf<DegreeOfFreedom,
      &DegreeOfFreedom::getIndexInSkeleton>(
        this, _dof, "DegreeOfFreedom", _warning);
}

//==============================================================================
const std::vector<DegreeOfFreedom*>& Skeleton::getTreeDofs(std::size_t _treeIdx)
{
  return mTreeCache[_treeIdx].mDofs;
}

//==============================================================================
const std::vector<const DegreeOfFreedom*>& Skeleton::getTreeDofs(
    std::size_t _treeIdx) const
{
  return convertToConstPtrVector<DegreeOfFreedom>(
        mTreeCache[_treeIdx].mDofs, mTreeCache[_treeIdx].mConstDofs);
}

//==============================================================================
bool Skeleton::checkIndexingConsistency() const
{
  bool consistent = true;

  // Check each BodyNode in the Skeleton cache
  for(std::size_t i=0; i<mSkelCache.mBodyNodes.size(); ++i)
  {
    const BodyNode* bn = mSkelCache.mBodyNodes[i];
    if(bn->mIndexInSkeleton != i)
    {
      dterr << "[Skeleton::checkIndexingConsistency] BodyNode named ["
            << bn->getName() << "] in Skeleton [" << getName() << "] is "
            << "mistaken about its index in the Skeleton (" << i << " | "
            << bn->mIndexInSkeleton << "). Please report this as a bug!"
            << std::endl;
      consistent = false;
      assert(false);
    }

    const BodyNode* nameEntryForBodyNode = getBodyNode(bn->getName());
    if(nameEntryForBodyNode != bn)
    {
      dterr << "[Skeleton::checkIndexingConsistency] Skeleton named ["
            << getName() << "] (" << this << ") is mistaken about the name of "
            << "BodyNode [" << bn->getName() << "] (" << bn << "). The name "
            << "instead maps to [" << nameEntryForBodyNode->getName() << "] ("
            << nameEntryForBodyNode << "). Please report this as a bug!"
            << std::endl;
      consistent = false;
      assert(false);
    }

    const Joint* joint = bn->getParentJoint();
    const Joint* nameEntryForJoint = getJoint(joint->getName());
    if(nameEntryForJoint != joint)
    {
      dterr << "[Skeleton::checkIndexingConsistency] Skeleton named ["
            << getName() << "] (" << this << ") is mistaken about the name of "
            << "Joint [" << joint->getName() << "] (" << joint << "). The name "
            << "instead maps to [" << nameEntryForJoint->getName() << "] ("
            << nameEntryForJoint << "). Please report this as a bug!"
            << std::endl;
      consistent = false;
      assert(false);
    }

    const BodyNode::NodeMap& nodeMap = bn->mNodeMap;
    for(const auto& nodeType : nodeMap)
    {
      const std::vector<Node*>& nodes = nodeType.second;
      for(std::size_t k=0; k < nodes.size(); ++k)
      {
        const Node* node = nodes[k];
        if(node->getBodyNodePtr() != bn)
        {
          dterr << "[Skeleton::checkIndexingConsistency] Node named ["
                << node->getName() << "] (" << node << ") in Skeleton ["
                << getName() << "] (" << this << ") is mistaken about its "
                << "BodyNode [" << node->getBodyNodePtr()->getName() << "] ("
                << node->getBodyNodePtr() << "). Please report this as a bug!"
                << std::endl;
          consistent = false;
          assert(false);
        }

        if(node->mIndexInBodyNode != k)
        {
          dterr << "[Skeleton::checkIndexingConsistency] Node named ["
                << node->getName() << "] (" << node << ") in Skeleton ["
                << getName() << "] (" << this << ") is mistaken about its "
                << "index in its BodyNode (" << k << "|"
                << node->mIndexInBodyNode << "). Please report this as a bug!"
                << std::endl;
          consistent = false;
          assert(false);
        }

        // TODO(MXG): Consider checking Node names here
      }
    }
  }

  // Check DegreesOfFreedom indexing
  for(std::size_t i=0; i < getNumDofs(); ++i)
  {
    const DegreeOfFreedom* dof = getDof(i);
    if(dof->getIndexInSkeleton() != i)
    {
      dterr << "[Skeleton::checkIndexingConsistency] DegreeOfFreedom named ["
            << dof->getName() << "] (" << dof << ") in Skeleton ["
            << getName() << "] (" << this << ") is mistaken about its index "
            << "in its Skeleton (" << i << "|" << dof->getIndexInSkeleton()
            << "). Please report this as a bug!" << std::endl;
      consistent = false;
      assert(false);
    }

    const DegreeOfFreedom* nameEntryForDof = getDof(dof->getName());
    if(nameEntryForDof != dof)
    {
      dterr << "[Skeleton::checkIndexingConsistency] Skeleton named ["
            << getName() << "] (" << this << ") is mistaken about the name of "
            << "DegreeOfFreedom [" << dof->getName() << "] (" << dof << "). "
            << "The name instead maps to [" << nameEntryForDof->getName()
            << "] (" << nameEntryForDof << "). Please report this as a bug!"
            << std::endl;
      consistent = false;
      assert(false);
    }
  }

  // Check each Node in the Skeleton-scope NodeMap
  {
    const Skeleton::NodeMap& nodeMap = mNodeMap;
    for(const auto& nodeType : nodeMap)
    {
      const std::vector<Node*>& nodes = nodeType.second;
      for(std::size_t k=0; k < nodes.size(); ++k)
      {
        const Node* node = nodes[k];
        if(node->getSkeleton().get() != this)
        {
          dterr << "[Skeleton::checkIndexingConsistency] Node named ["
                << node->getName() << "] (" << node << ") in Skeleton ["
                << getName() << "] (" << this << ") is mistaken about its "
                << "Skeleton [" << node->getSkeleton()->getName() << "] ("
                << node->getSkeleton()<< "). Please report this as a bug!"
                << std::endl;
          consistent = false;
          assert(false);
        }

        if(node->mIndexInSkeleton != k)
        {
          dterr << "[Skeleton::checkIndexingConsistency] Node named ["
                << node->getName() << "] (" << node << ") in Skeleton ["
                << getName() << "] (" << this << ") is mistaken about its "
                << "index in its Skeleton (" << k << "|"
                << node->mIndexInSkeleton << "). Please report this as a bug!"
                << std::endl;
          consistent = false;
          assert(false);
        }
      }
    }
  }

  // Check each BodyNode in each Tree cache
  for(std::size_t i=0; i<mTreeCache.size(); ++i)
  {
    const DataCache& cache = mTreeCache[i];
    for(std::size_t j=0; j<cache.mBodyNodes.size(); ++j)
    {
      const BodyNode* bn = cache.mBodyNodes[j];
      if(bn->mTreeIndex != i)
      {
        dterr << "[Skeleton::checkIndexingConsistency] BodyNode named ["
              << bn->getName() << "] in Skeleton [" << getName() << "] is "
              << "mistaken about its tree's index (" << i << "|"
              << bn->mTreeIndex << "). Please report this as a bug!"
              << std::endl;
        consistent = false;
        assert(false);
      }

      if(bn->mIndexInTree != j)
      {
        dterr << "[Skeleton::checkIndexingConsistency] BodyNode named ["
              << bn->getName() << "] (" << bn << ") in Skeleton ["
              << getName() << "] (" << this << ") is mistaken about its index "
              << "in the tree (" << j << "|" << bn->mIndexInTree << "). Please "
              << "report this as a bug!" << std::endl;
        consistent = false;
        assert(false);
      }
    }

    for(std::size_t j=0; j < cache.mDofs.size(); ++j)
    {
      const DegreeOfFreedom* dof = cache.mDofs[j];
      if(dof->getTreeIndex() != i)
      {
        dterr << "[Skeleton::checkIndexingConsistency] DegreeOfFreedom named ["
              << dof->getName() << "] (" << dof << ") in Skeleton ["
              << getName() << "] (" << this << ") is mistaken about its tree's "
              << "index (" << i << "|" << dof->getTreeIndex() << "). Please "
              << "report this as a bug!" << std::endl;
        consistent = false;
        assert(false);
      }
    }
  }

  // Check that the Tree cache and the number of Tree NodeMaps match up
  if(mTreeCache.size() != mTreeNodeMaps.size())
  {
    consistent = false;
    dterr << "[Skeleton::checkIndexingConsistency] Skeleton named ["
              << getName() << "] (" << this << ") has inconsistent tree cache "
              << " and tree Node map sizes (" << mTreeCache.size() << "|"
              << mTreeNodeMaps.size() << "). Please report this as a bug!"
              << std::endl;
    assert(false);
  }

  // Check each Node in the NodeMap of each Tree
  for(std::size_t i=0; i < mTreeNodeMaps.size(); ++i)
  {
    const NodeMap& nodeMap = mTreeNodeMaps[i];

    for(const auto& nodeType : nodeMap)
    {
      const std::vector<Node*>& nodes = nodeType.second;
      for(std::size_t k=0; k < nodes.size(); ++k)
      {
        const Node* node = nodes[k];
        if(node->getBodyNodePtr()->mTreeIndex != i)
        {
          dterr << "[Skeleton::checkIndexingConsistency] Node named ["
                << node->getName() << "] (" << node << ") in Skeleton ["
                << getName() << "] (" << this << ") is mistaken about its "
                << "Tree Index (" << i << "|"
                << node->getBodyNodePtr()->mTreeIndex << "). Please report "
                << "this as a bug!" << std::endl;
          consistent = false;
          assert(false);
        }

        if(node->mIndexInTree != k)
        {
          dterr << "[Skeleton::checkIndexingConsistency] Node named ["
                << node->getName() << "] (" << node << ") in Skeleton ["
                << getName() << "] (" << this << ") is mistaken about its "
                << "index in its tree (" << k << "|"
                << node->mIndexInTree << "). Please report this as a bug!"
                << std::endl;
          consistent = false;
          assert(false);
        }
      }
    }
  }

  return consistent;
}

//==============================================================================
const std::shared_ptr<WholeBodyIK>& Skeleton::getIK(bool _createIfNull)
{
  if(nullptr == mWholeBodyIK && _createIfNull)
    createIK();

  return mWholeBodyIK;
}

//==============================================================================
const std::shared_ptr<WholeBodyIK>& Skeleton::getOrCreateIK()
{
  return getIK(true);
}

//==============================================================================
std::shared_ptr<const WholeBodyIK> Skeleton::getIK() const
{
  return mWholeBodyIK;
}

//==============================================================================
const std::shared_ptr<WholeBodyIK>& Skeleton::createIK()
{
  mWholeBodyIK = WholeBodyIK::create(mPtr.lock());
  return mWholeBodyIK;
}

//==============================================================================
void Skeleton::clearIK()
{
  mWholeBodyIK = nullptr;
}

//==============================================================================
DART_BAKE_SPECIALIZED_NODE_SKEL_DEFINITIONS( Skeleton, Marker )

//==============================================================================
DART_BAKE_SPECIALIZED_NODE_SKEL_DEFINITIONS( Skeleton, ShapeNode )

//==============================================================================
DART_BAKE_SPECIALIZED_NODE_SKEL_DEFINITIONS( Skeleton, EndEffector )

//==============================================================================
void Skeleton::integratePositions(double _dt)
{
  for (std::size_t i = 0; i < mSkelCache.mBodyNodes.size(); ++i)
    mSkelCache.mBodyNodes[i]->getParentJoint()->integratePositions(_dt);

  for (std::size_t i = 0; i < mSoftBodyNodes.size(); ++i)
  {
    for (std::size_t j = 0; j < mSoftBodyNodes[i]->getNumPointMasses(); ++j)
      mSoftBodyNodes[i]->getPointMass(j)->integratePositions(_dt);
  }
}

//==============================================================================
void Skeleton::integrateVelocities(double _dt)
{
  for (std::size_t i = 0; i < mSkelCache.mBodyNodes.size(); ++i)
    mSkelCache.mBodyNodes[i]->getParentJoint()->integrateVelocities(_dt);

  for (std::size_t i = 0; i < mSoftBodyNodes.size(); ++i)
  {
    for (std::size_t j = 0; j < mSoftBodyNodes[i]->getNumPointMasses(); ++j)
      mSoftBodyNodes[i]->getPointMass(j)->integrateVelocities(_dt);
  }
}

//==============================================================================
Eigen::VectorXd Skeleton::getPositionDifferences(
    const Eigen::VectorXd& _q2, const Eigen::VectorXd& _q1) const
{
  if (static_cast<std::size_t>(_q2.size()) != getNumDofs()
      || static_cast<std::size_t>(_q1.size()) != getNumDofs())
  {
    dterr << "Skeleton::getPositionsDifference: q1's size[" << _q1.size()
          << "] or q2's size[" << _q2.size() << "is different with the dof ["
          << getNumDofs() << "]." << std::endl;
    return Eigen::VectorXd::Zero(getNumDofs());
  }

  Eigen::VectorXd dq(getNumDofs());

  for (const auto& bodyNode : mSkelCache.mBodyNodes)
  {
    const Joint* joint = bodyNode->getParentJoint();
    const std::size_t dof   = joint->getNumDofs();

    if (dof)
    {
      std::size_t index = joint->getDof(0)->getIndexInSkeleton();
      const Eigen::VectorXd& q2Seg = _q2.segment(index, dof);
      const Eigen::VectorXd& q1Seg = _q1.segment(index, dof);
      dq.segment(index, dof) = joint->getPositionDifferences(q2Seg, q1Seg);
    }
  }

  return dq;
}

//==============================================================================
Eigen::VectorXd Skeleton::getVelocityDifferences(
    const Eigen::VectorXd& _dq2, const Eigen::VectorXd& _dq1) const
{
  if (static_cast<std::size_t>(_dq2.size()) != getNumDofs()
      || static_cast<std::size_t>(_dq1.size()) != getNumDofs())
  {
    dterr << "Skeleton::getPositionsDifference: dq1's size[" << _dq1.size()
          << "] or dq2's size[" << _dq2.size() << "is different with the dof ["
          << getNumDofs() << "]." << std::endl;
    return Eigen::VectorXd::Zero(getNumDofs());
  }

  // All the tangent spaces of Joint's configuration spaces are vector spaces.
  return _dq2 - _dq1;
}

//==============================================================================
static bool isValidBodyNode(const Skeleton* _skeleton,
                            const JacobianNode* _node,
                            const std::string& _fname)
{
  if (nullptr == _node)
  {
    dtwarn << "[Skeleton::" << _fname << "] Invalid BodyNode pointer: "
           << "nullptr. Returning zero Jacobian.\n";
    assert(false);
    return false;
  }

  // The given BodyNode should be in the Skeleton
  if (_node->getSkeleton().get() != _skeleton)
  {
    dtwarn << "[Skeleton::" << _fname << "] Attempting to get a Jacobian for a "
           "BodyNode [" << _node->getName() << "] (" << _node
           << ") that is not in this Skeleton [" << _skeleton->getName()
           << "] (" << _skeleton << "). Returning zero Jacobian.\n";
    assert(false);
    return false;
  }

  return true;
}

//==============================================================================
template <typename JacobianType>
void assignJacobian(JacobianType& _J,
                    const JacobianNode* _node,
                    const JacobianType& _JBodyNode)
{
  // Assign the BodyNode's Jacobian to the result Jacobian.
  std::size_t localIndex = 0;
  const auto& indices = _node->getDependentGenCoordIndices();
  for (const auto& index : indices)
  {
    // Each index should be less than the number of dofs of this Skeleton.
    assert(index < _node->getSkeleton()->getNumDofs());

    _J.col(index) = _JBodyNode.col(localIndex++);
  }
}

//==============================================================================
template <typename ...Args>
math::Jacobian variadicGetJacobian(
    const Skeleton* _skel, const JacobianNode* _node, Args... args)
{
  math::Jacobian J = math::Jacobian::Zero(6, _skel->getNumDofs());

  if ( !isValidBodyNode(_skel, _node, "getJacobian") )
    return J;

  const math::Jacobian JBodyNode = _node->getJacobian(args...);

  assignJacobian<math::Jacobian>(J, _node, JBodyNode);

  return J;
}

//==============================================================================
math::Jacobian Skeleton::getJacobian(const JacobianNode* _node) const
{
  return variadicGetJacobian(this, _node);
}

//==============================================================================
math::Jacobian Skeleton::getJacobian(const JacobianNode* _node,
                                     const Frame* _inCoordinatesOf) const
{
  return variadicGetJacobian(this, _node, _inCoordinatesOf);
}

//==============================================================================
math::Jacobian Skeleton::getJacobian(const JacobianNode* _node,
                                     const Eigen::Vector3d& _localOffset) const
{
  return variadicGetJacobian(this, _node, _localOffset);
}

//==============================================================================
math::Jacobian Skeleton::getJacobian(const JacobianNode* _node,
                                     const Eigen::Vector3d& _localOffset,
                                     const Frame* _inCoordinatesOf) const
{
  return variadicGetJacobian(this, _node, _localOffset, _inCoordinatesOf);
}

//==============================================================================
template <typename ...Args>
math::Jacobian variadicGetWorldJacobian(
    const Skeleton* _skel, const JacobianNode* _node, Args... args)
{
  math::Jacobian J = math::Jacobian::Zero(6, _skel->getNumDofs());

  if( !isValidBodyNode(_skel, _node, "getWorldJacobian") )
    return J;

  const math::Jacobian JBodyNode = _node->getWorldJacobian(args...);

  assignJacobian<math::Jacobian>(J, _node, JBodyNode);

  return J;
}

//==============================================================================
math::Jacobian Skeleton::getWorldJacobian(const JacobianNode* _node) const
{
  return variadicGetWorldJacobian(this, _node);
}

//==============================================================================
math::Jacobian Skeleton::getWorldJacobian(
    const JacobianNode* _node,
    const Eigen::Vector3d& _localOffset) const
{
  return variadicGetWorldJacobian(this, _node, _localOffset);
}

//==============================================================================
template <typename ...Args>
math::LinearJacobian variadicGetLinearJacobian(
    const Skeleton* _skel, const JacobianNode* _node, Args... args)
{
  math::LinearJacobian J =
      math::LinearJacobian::Zero(3, _skel->getNumDofs());

  if( !isValidBodyNode(_skel, _node, "getLinearJacobian") )
    return J;

  const math::LinearJacobian JBodyNode = _node->getLinearJacobian(args...);

  assignJacobian<math::LinearJacobian>(J, _node, JBodyNode);

  return J;
}


//==============================================================================
math::LinearJacobian Skeleton::getLinearJacobian(
    const JacobianNode* _node,
    const Frame* _inCoordinatesOf) const
{
  return variadicGetLinearJacobian(this, _node, _inCoordinatesOf);
}

//==============================================================================
math::LinearJacobian Skeleton::getLinearJacobian(
    const JacobianNode* _node,
    const Eigen::Vector3d& _localOffset,
    const Frame* _inCoordinatesOf) const
{
  return variadicGetLinearJacobian(
        this, _node, _localOffset, _inCoordinatesOf);
}

//==============================================================================
template <typename ...Args>
math::AngularJacobian variadicGetAngularJacobian(
    const Skeleton* _skel, const JacobianNode* _node, Args... args)
{
  math::AngularJacobian J =
      math::AngularJacobian::Zero(3, _skel->getNumDofs());

  if( !isValidBodyNode(_skel, _node, "getAngularJacobian") )
    return J;

  const math::AngularJacobian JBodyNode =
      _node->getAngularJacobian(args...);

  assignJacobian<math::AngularJacobian>(J, _node, JBodyNode);

  return J;
}

//==============================================================================
math::AngularJacobian Skeleton::getAngularJacobian(
    const JacobianNode* _node,
    const Frame* _inCoordinatesOf) const
{
  return variadicGetAngularJacobian(this, _node, _inCoordinatesOf);
}

//==============================================================================
template <typename ...Args>
math::Jacobian variadicGetJacobianSpatialDeriv(
    const Skeleton* _skel, const JacobianNode* _node, Args... args)
{
  math::Jacobian dJ = math::Jacobian::Zero(6, _skel->getNumDofs());

  if( !isValidBodyNode(_skel, _node, "getJacobianSpatialDeriv") )
    return dJ;

  const math::Jacobian dJBodyNode = _node->getJacobianSpatialDeriv(args...);

  assignJacobian<math::Jacobian>(dJ, _node, dJBodyNode);

  return dJ;
}

//==============================================================================
math::Jacobian Skeleton::getJacobianSpatialDeriv(
    const JacobianNode* _node) const
{
  return variadicGetJacobianSpatialDeriv(this, _node);
}

//==============================================================================
math::Jacobian Skeleton::getJacobianSpatialDeriv(
    const JacobianNode* _node,
    const Frame* _inCoordinatesOf) const
{
  return variadicGetJacobianSpatialDeriv(this, _node, _inCoordinatesOf);
}

//==============================================================================
math::Jacobian Skeleton::getJacobianSpatialDeriv(
    const JacobianNode* _node,
    const Eigen::Vector3d& _localOffset) const
{
  return variadicGetJacobianSpatialDeriv(this, _node, _localOffset);
}

//==============================================================================
math::Jacobian Skeleton::getJacobianSpatialDeriv(
    const JacobianNode* _node,
    const Eigen::Vector3d& _localOffset,
    const Frame* _inCoordinatesOf) const
{
  return variadicGetJacobianSpatialDeriv(
        this, _node, _localOffset, _inCoordinatesOf);
}

//==============================================================================
template <typename ...Args>
math::Jacobian variadicGetJacobianClassicDeriv(
    const Skeleton* _skel, const JacobianNode* _node, Args... args)
{
  math::Jacobian dJ = math::Jacobian::Zero(6, _skel->getNumDofs());

  if( !isValidBodyNode(_skel, _node, "getJacobianClassicDeriv") )
    return dJ;

  const math::Jacobian dJBodyNode = _node->getJacobianClassicDeriv(args...);

  assignJacobian<math::Jacobian>(dJ, _node, dJBodyNode);

  return dJ;
}

//==============================================================================
math::Jacobian Skeleton::getJacobianClassicDeriv(
    const JacobianNode* _node) const
{
  return variadicGetJacobianClassicDeriv(this, _node);
}

//==============================================================================
math::Jacobian Skeleton::getJacobianClassicDeriv(
    const JacobianNode* _node,
    const Frame* _inCoordinatesOf) const
{
  return variadicGetJacobianClassicDeriv(this, _node, _inCoordinatesOf);
}

//==============================================================================
math::Jacobian Skeleton::getJacobianClassicDeriv(
    const JacobianNode* _node,
    const Eigen::Vector3d& _localOffset,
    const Frame* _inCoordinatesOf) const
{
  return variadicGetJacobianClassicDeriv(
        this, _node, _localOffset, _inCoordinatesOf);
}

//==============================================================================
template <typename ...Args>
math::LinearJacobian variadicGetLinearJacobianDeriv(
    const Skeleton* _skel, const JacobianNode* _node, Args... args)
{
  math::LinearJacobian dJv =
      math::LinearJacobian::Zero(3, _skel->getNumDofs());

  if ( !isValidBodyNode(_skel, _node, "getLinearJacobianDeriv") )
    return dJv;

  const math::LinearJacobian dJvBodyNode =
      _node->getLinearJacobianDeriv(args...);

  assignJacobian<math::LinearJacobian>(dJv, _node, dJvBodyNode);

  return dJv;
}

//==============================================================================
math::LinearJacobian Skeleton::getLinearJacobianDeriv(
    const JacobianNode* _node,
    const Frame* _inCoordinatesOf) const
{
  return variadicGetLinearJacobianDeriv(this, _node, _inCoordinatesOf);
}

//==============================================================================
math::LinearJacobian Skeleton::getLinearJacobianDeriv(
    const JacobianNode* _node,
    const Eigen::Vector3d& _localOffset,
    const Frame* _inCoordinatesOf) const
{
  return variadicGetLinearJacobianDeriv(
        this, _node, _localOffset, _inCoordinatesOf);
}

//==============================================================================
template <typename ...Args>
math::AngularJacobian variadicGetAngularJacobianDeriv(
    const Skeleton* _skel, const JacobianNode* _node, Args... args)
{
  math::AngularJacobian dJw =
      math::AngularJacobian::Zero(3, _skel->getNumDofs());

  if ( !isValidBodyNode(_skel, _node, "getAngularJacobianDeriv") )
    return dJw;

  const math::AngularJacobian dJwBodyNode =
      _node->getAngularJacobianDeriv(args...);

  assignJacobian<math::AngularJacobian>(dJw, _node, dJwBodyNode);

  return dJw;
}

//==============================================================================
math::AngularJacobian Skeleton::getAngularJacobianDeriv(
    const JacobianNode* _node, const Frame* _inCoordinatesOf) const
{
  return variadicGetAngularJacobianDeriv(this, _node, _inCoordinatesOf);
}

//==============================================================================
double Skeleton::getMass() const
{
  return mTotalMass;
}

//==============================================================================
const Eigen::MatrixXd& Skeleton::getMassMatrix(std::size_t _treeIdx) const
{
  if (mTreeCache[_treeIdx].mDirty.mMassMatrix)
    updateMassMatrix(_treeIdx);
  return mTreeCache[_treeIdx].mM;
}

//==============================================================================
const Eigen::MatrixXd& Skeleton::getMassMatrix() const
{
  if (mSkelCache.mDirty.mMassMatrix)
    updateMassMatrix();
  return mSkelCache.mM;
}

//==============================================================================
const Eigen::MatrixXd& Skeleton::getAugMassMatrix(std::size_t _treeIdx) const
{
  if (mTreeCache[_treeIdx].mDirty.mAugMassMatrix)
    updateAugMassMatrix(_treeIdx);

  return mTreeCache[_treeIdx].mAugM;
}

//==============================================================================
const Eigen::MatrixXd& Skeleton::getAugMassMatrix() const
{
  if (mSkelCache.mDirty.mAugMassMatrix)
    updateAugMassMatrix();

  return mSkelCache.mAugM;
}

//==============================================================================
const Eigen::MatrixXd& Skeleton::getInvMassMatrix(std::size_t _treeIdx) const
{
  if (mTreeCache[_treeIdx].mDirty.mInvMassMatrix)
    updateInvMassMatrix(_treeIdx);

  return mTreeCache[_treeIdx].mInvM;
}

//==============================================================================
const Eigen::MatrixXd& Skeleton::getInvMassMatrix() const
{
  if (mSkelCache.mDirty.mInvMassMatrix)
    updateInvMassMatrix();

  return mSkelCache.mInvM;
}

//==============================================================================
const Eigen::MatrixXd& Skeleton::getInvAugMassMatrix(std::size_t _treeIdx) const
{
  if (mTreeCache[_treeIdx].mDirty.mInvAugMassMatrix)
    updateInvAugMassMatrix(_treeIdx);

  return mTreeCache[_treeIdx].mInvAugM;
}

//==============================================================================
const Eigen::MatrixXd& Skeleton::getInvAugMassMatrix() const
{
  if (mSkelCache.mDirty.mInvAugMassMatrix)
    updateInvAugMassMatrix();

  return mSkelCache.mInvAugM;
}

//==============================================================================
const Eigen::VectorXd& Skeleton::getCoriolisForces(std::size_t _treeIdx) const
{
  if (mTreeCache[_treeIdx].mDirty.mCoriolisForces)
    updateCoriolisForces(_treeIdx);

  return mTreeCache[_treeIdx].mCvec;
}

//==============================================================================
const Eigen::VectorXd& Skeleton::getCoriolisForces() const
{
  if (mSkelCache.mDirty.mCoriolisForces)
    updateCoriolisForces();

  return mSkelCache.mCvec;
}

//==============================================================================
const Eigen::VectorXd& Skeleton::getGravityForces(std::size_t _treeIdx) const
{
  if (mTreeCache[_treeIdx].mDirty.mGravityForces)
    updateGravityForces(_treeIdx);

  return mTreeCache[_treeIdx].mG;
}

//==============================================================================
const Eigen::VectorXd& Skeleton::getGravityForces() const
{
  if (mSkelCache.mDirty.mGravityForces)
    updateGravityForces();

  return mSkelCache.mG;
}

//==============================================================================
const Eigen::VectorXd& Skeleton::getCoriolisAndGravityForces(std::size_t _treeIdx) const
{
  if (mTreeCache[_treeIdx].mDirty.mCoriolisAndGravityForces)
    updateCoriolisAndGravityForces(_treeIdx);

  return mTreeCache[_treeIdx].mCg;
}

//==============================================================================
const Eigen::VectorXd& Skeleton::getCoriolisAndGravityForces() const
{
  if (mSkelCache.mDirty.mCoriolisAndGravityForces)
    updateCoriolisAndGravityForces();

  return mSkelCache.mCg;
}

//==============================================================================
const Eigen::VectorXd& Skeleton::getExternalForces(std::size_t _treeIdx) const
{
  if (mTreeCache[_treeIdx].mDirty.mExternalForces)
    updateExternalForces(_treeIdx);

  return mTreeCache[_treeIdx].mFext;
}

//==============================================================================
const Eigen::VectorXd& Skeleton::getExternalForces() const
{
  if (mSkelCache.mDirty.mExternalForces)
    updateExternalForces();

  return mSkelCache.mFext;
}

//==============================================================================
const Eigen::VectorXd& Skeleton::getConstraintForces(std::size_t _treeIdx) const
{
  return computeConstraintForces(mTreeCache[_treeIdx]);
}

//==============================================================================
const Eigen::VectorXd& Skeleton::getConstraintForces() const
{
  return computeConstraintForces(mSkelCache);
}

//==============================================================================
//const Eigen::VectorXd& Skeleton::getDampingForceVector() {
//  if (mIsDampingForceVectorDirty)
//    updateDampingForceVector();
//  return mFd;
//}

//==============================================================================
Skeleton::Skeleton(const AspectPropertiesData& properties)
  : mTotalMass(0.0),
    mIsImpulseApplied(false),
    mUnionSize(1)
{
  createAspect<Aspect>(properties);
  createAspect<detail::BodyNodeVectorProxyAspect>();
  createAspect<detail::JointVectorProxyAspect>();
}

//==============================================================================
void Skeleton::setPtr(const SkeletonPtr& _ptr)
{
  mPtr = _ptr;
  resetUnion();
}

//==============================================================================
void Skeleton::constructNewTree()
{
  mTreeCache.push_back(DataCache());

  mTreeNodeMaps.push_back(NodeMap());
  NodeMap& nodeMap = mTreeNodeMaps.back();

  // Create the machinery needed to directly call on specialized node types
  for(auto& nodeType : mSpecializedTreeNodes)
  {
    const std::type_index& index = nodeType.first;
    nodeMap[index] = std::vector<Node*>();

    std::vector<NodeMap::iterator>* nodeVec = nodeType.second;
    nodeVec->push_back(nodeMap.find(index));

    assert(nodeVec->size() == mTreeCache.size());
  }
}

//==============================================================================
void Skeleton::registerBodyNode(BodyNode* _newBodyNode)
{
#ifndef NDEBUG  // Debug mode
  std::vector<BodyNode*>::iterator repeat =
      std::find(mSkelCache.mBodyNodes.begin(), mSkelCache.mBodyNodes.end(),
                _newBodyNode);
  if(repeat != mSkelCache.mBodyNodes.end())
  {
    dterr << "[Skeleton::registerBodyNode] Attempting to double-register the "
          << "BodyNode named [" << _newBodyNode->getName() << "] in the "
          << "Skeleton named [" << getName() << "]. Please report this as a "
          << "bug!\n";
    assert(false);
    return;
  }
#endif // -------- Debug mode

  mSkelCache.mBodyNodes.push_back(_newBodyNode);
  if(nullptr == _newBodyNode->getParentBodyNode())
  {
    // Create a new tree and add the new BodyNode to it
    _newBodyNode->mIndexInTree = 0;
    constructNewTree();
    mTreeCache.back().mBodyNodes.push_back(_newBodyNode);
    _newBodyNode->mTreeIndex = mTreeCache.size()-1;
  }
  else
  {
    std::size_t tree = _newBodyNode->getParentBodyNode()->getTreeIndex();
    _newBodyNode->mTreeIndex = tree;
    DataCache& cache = mTreeCache[tree];
    cache.mBodyNodes.push_back(_newBodyNode);
    _newBodyNode->mIndexInTree = cache.mBodyNodes.size()-1;
  }

  _newBodyNode->mSkeleton = getPtr();
  _newBodyNode->mIndexInSkeleton = mSkelCache.mBodyNodes.size()-1;
  addEntryToBodyNodeNameMgr(_newBodyNode);
  registerJoint(_newBodyNode->getParentJoint());

  SoftBodyNode* softBodyNode = dynamic_cast<SoftBodyNode*>(_newBodyNode);
  if (softBodyNode)
  {
    mSoftBodyNodes.push_back(softBodyNode);
    addEntryToSoftBodyNodeNameMgr(softBodyNode);
  }

  _newBodyNode->init(getPtr());

  BodyNode::NodeMap& nodeMap = _newBodyNode->mNodeMap;
  for(auto& nodeType : nodeMap)
    for(auto& node : nodeType.second)
      registerNode(node);

  updateTotalMass();
  updateCacheDimensions(_newBodyNode->mTreeIndex);

#ifndef NDEBUG // Debug mode
  for(std::size_t i=0; i<mSkelCache.mBodyNodes.size(); ++i)
  {
    if(mSkelCache.mBodyNodes[i]->mIndexInSkeleton != i)
    {
      dterr << "[Skeleton::registerBodyNode] BodyNode named ["
            << mSkelCache.mBodyNodes[i]->getName() << "] in Skeleton ["
            << getName() << "] is mistaken about its index in the Skeleton ( "
            << i << " : " << mSkelCache.mBodyNodes[i]->mIndexInSkeleton
            << "). Please report this as a bug!\n";
      assert(false);
    }
  }

  for(std::size_t i=0; i<mTreeCache.size(); ++i)
  {
    const DataCache& cache = mTreeCache[i];
    for(std::size_t j=0; j<cache.mBodyNodes.size(); ++j)
    {
      BodyNode* bn = cache.mBodyNodes[j];
      if(bn->mTreeIndex != i)
      {
        dterr << "[Skeleton::registerBodyNode] BodyNode named ["
              << bn->getName() << "] in Skeleton [" << getName() << "] is "
              << "mistaken about its tree's index (" << i << " : "
              << bn->mTreeIndex << "). Please report this as a bug!\n";
        assert(false);
      }

      if(bn->mIndexInTree != j)
      {
        dterr << "[Skeleton::registerBodyNode] BodyNode named ["
              << bn->getName() << "] in Skeleton [" << getName() << "] is "
              << "mistaken about its index in the tree (" << j << " : "
              << bn->mIndexInTree << "). Please report this as a bug!\n";
        assert(false);
      }
    }
  }
#endif // ------- Debug mode

  _newBodyNode->mStructuralChangeSignal.raise(_newBodyNode);
}

//==============================================================================
void Skeleton::registerJoint(Joint* _newJoint)
{
  if (nullptr == _newJoint)
  {
    dterr << "[Skeleton::registerJoint] Error: Attempting to add a nullptr "
             "Joint to the Skeleton named [" << mAspectProperties.mName << "]. Report "
             "this as a bug!\n";
    assert(false);
    return;
  }

  addEntryToJointNameMgr(_newJoint);
  _newJoint->registerDofs();

  std::size_t tree = _newJoint->getChildBodyNode()->getTreeIndex();
  std::vector<DegreeOfFreedom*>& treeDofs = mTreeCache[tree].mDofs;
  for(std::size_t i = 0; i < _newJoint->getNumDofs(); ++i)
  {
    mSkelCache.mDofs.push_back(_newJoint->getDof(i));
    _newJoint->getDof(i)->mIndexInSkeleton = mSkelCache.mDofs.size()-1;

    treeDofs.push_back(_newJoint->getDof(i));
    _newJoint->getDof(i)->mIndexInTree = treeDofs.size()-1;
  }
}

//==============================================================================
void Skeleton::registerNode(NodeMap& nodeMap, Node* _newNode, std::size_t& _index)
{
  NodeMap::iterator it = nodeMap.find(typeid(*_newNode));

  if(nodeMap.end() == it)
  {
    nodeMap[typeid(*_newNode)] = std::vector<Node*>();
    it = nodeMap.find(typeid(*_newNode));
  }

  std::vector<Node*>& nodes = it->second;

  if(INVALID_INDEX == _index)
  {
    // If this Node believes its index is invalid, then it should not exist
    // anywhere in the vector
    assert(std::find(nodes.begin(), nodes.end(), _newNode) == nodes.end());

    nodes.push_back(_newNode);
    _index = nodes.size()-1;
  }

  assert(std::find(nodes.begin(), nodes.end(), _newNode) != nodes.end());
}

//==============================================================================
void Skeleton::registerNode(Node* _newNode)
{
  registerNode(mNodeMap, _newNode, _newNode->mIndexInSkeleton);

  registerNode(mTreeNodeMaps[_newNode->getBodyNodePtr()->getTreeIndex()],
      _newNode, _newNode->mIndexInTree);

  const std::type_info& info = typeid(*_newNode);
  NodeNameMgrMap::iterator it = mNodeNameMgrMap.find(info);
  if(mNodeNameMgrMap.end() == it)
  {
    mNodeNameMgrMap[info] = common::NameManager<Node*>(
          std::string("Skeleton::") + info.name() + " | " + mAspectProperties.mName,
          info.name() );

    it = mNodeNameMgrMap.find(info);
  }

  common::NameManager<Node*>& mgr = it->second;
  _newNode->setName(mgr.issueNewNameAndAdd(_newNode->getName(), _newNode));
}

//==============================================================================
void Skeleton::destructOldTree(std::size_t tree)
{
  mTreeCache.erase(mTreeCache.begin() + tree);
  mTreeNodeMaps.erase(mTreeNodeMaps.begin() + tree);

  // Decrease the tree index of every BodyNode whose tree index is higher than
  // the one which is being removed. None of the BodyNodes that predate the
  // current one can have a higher tree index, so they can be ignored.
  for(std::size_t i=tree; i < mTreeCache.size(); ++i)
  {
    DataCache& loweredTree = mTreeCache[i];
    for(std::size_t j=0; j < loweredTree.mBodyNodes.size(); ++j)
      loweredTree.mBodyNodes[j]->mTreeIndex = i;
  }

  for(auto& nodeType : mSpecializedTreeNodes)
  {
    std::vector<NodeMap::iterator>* nodeRepo = nodeType.second;
    nodeRepo->erase(nodeRepo->begin() + tree);
  }
}

//==============================================================================
void Skeleton::unregisterBodyNode(BodyNode* _oldBodyNode)
{
  unregisterJoint(_oldBodyNode->getParentJoint());

  BodyNode::NodeMap& nodeMap = _oldBodyNode->mNodeMap;
  for(auto& nodeType : nodeMap)
    for(auto& node : nodeType.second)
      unregisterNode(node);

  mNameMgrForBodyNodes.removeName(_oldBodyNode->getName());

  std::size_t index = _oldBodyNode->getIndexInSkeleton();
  assert(mSkelCache.mBodyNodes[index] == _oldBodyNode);
  mSkelCache.mBodyNodes.erase(mSkelCache.mBodyNodes.begin()+index);
  for(std::size_t i=index; i < mSkelCache.mBodyNodes.size(); ++i)
  {
    BodyNode* bn = mSkelCache.mBodyNodes[i];
    bn->mIndexInSkeleton = i;
  }

  if(nullptr == _oldBodyNode->getParentBodyNode())
  {
    // If the parent of this BodyNode is a nullptr, then this is the root of its
    // tree. If the root of the tree is being removed, then the tree itself
    // should be destroyed.

    // There is no way that any child BodyNodes of this root BodyNode are still
    // registered, because the BodyNodes always get unregistered from leaf to
    // root.

    std::size_t tree = _oldBodyNode->getTreeIndex();
    assert(mTreeCache[tree].mBodyNodes.size() == 1);
    assert(mTreeCache[tree].mBodyNodes[0] == _oldBodyNode);

    destructOldTree(tree);
    updateCacheDimensions(mSkelCache);
  }
  else
  {
    std::size_t tree = _oldBodyNode->getTreeIndex();
    std::size_t indexInTree = _oldBodyNode->getIndexInTree();
    assert(mTreeCache[tree].mBodyNodes[indexInTree] == _oldBodyNode);
    mTreeCache[tree].mBodyNodes.erase(
          mTreeCache[tree].mBodyNodes.begin() + indexInTree);

    for(std::size_t i=indexInTree; i<mTreeCache[tree].mBodyNodes.size(); ++i)
      mTreeCache[tree].mBodyNodes[i]->mIndexInTree = i;

    updateCacheDimensions(tree);
  }

  SoftBodyNode* soft = dynamic_cast<SoftBodyNode*>(_oldBodyNode);
  if(soft)
  {
    mNameMgrForSoftBodyNodes.removeName(soft->getName());

    mSoftBodyNodes.erase(std::remove(mSoftBodyNodes.begin(),
                                     mSoftBodyNodes.end(), soft),
                         mSoftBodyNodes.end());
  }

  updateTotalMass();
}

//==============================================================================
void Skeleton::unregisterJoint(Joint* _oldJoint)
{
  if (nullptr == _oldJoint)
  {
    dterr << "[Skeleton::unregisterJoint] Attempting to unregister nullptr "
          << "Joint from Skeleton named [" << getName() << "]. Report this as "
          << "a bug!\n";
    assert(false);
    return;
  }

  mNameMgrForJoints.removeName(_oldJoint->getName());

  std::size_t tree = _oldJoint->getChildBodyNode()->getTreeIndex();
  std::vector<DegreeOfFreedom*>& treeDofs = mTreeCache[tree].mDofs;
  std::vector<DegreeOfFreedom*>& skelDofs = mSkelCache.mDofs;

  std::size_t firstSkelIndex = INVALID_INDEX;
  std::size_t firstTreeIndex = INVALID_INDEX;
  for (std::size_t i = 0; i < _oldJoint->getNumDofs(); ++i)
  {
    DegreeOfFreedom* dof = _oldJoint->getDof(i);
    mNameMgrForDofs.removeObject(dof);

    firstSkelIndex = std::min(firstSkelIndex, dof->getIndexInSkeleton());
    skelDofs.erase(
          std::remove(skelDofs.begin(), skelDofs.end(), dof), skelDofs.end());

    firstTreeIndex = std::min(firstTreeIndex, dof->getIndexInTree());
    treeDofs.erase(
          std::remove(treeDofs.begin(), treeDofs.end(), dof), treeDofs.end());
  }

  for (std::size_t i = firstSkelIndex; i < skelDofs.size(); ++i)
  {
    DegreeOfFreedom* dof = skelDofs[i];
    dof->mIndexInSkeleton = i;
  }

  for (std::size_t i = firstTreeIndex; i < treeDofs.size(); ++i)
  {
    DegreeOfFreedom* dof = treeDofs[i];
    dof->mIndexInTree = i;
  }
}

//==============================================================================
void Skeleton::unregisterNode(NodeMap& nodeMap, Node* _oldNode, std::size_t& _index)
{
  NodeMap::iterator it = nodeMap.find(typeid(*_oldNode));

  if(nodeMap.end() == it)
  {
    // If the Node was not in the map, then its index should be invalid
    assert(INVALID_INDEX == _index);
    return;
  }

  std::vector<Node*>& nodes = it->second;

  // This Node's index in the vector should be referring to this Node
  assert(nodes[_index] == _oldNode);
  nodes.erase(nodes.begin() + _index);

  _index = INVALID_INDEX;
}

//==============================================================================
void Skeleton::unregisterNode(Node* _oldNode)
{
  const std::size_t indexInSkel = _oldNode->mIndexInSkeleton;
  unregisterNode(mNodeMap, _oldNode, _oldNode->mIndexInSkeleton);

  NodeMap::iterator node_it = mNodeMap.find(typeid(*_oldNode));
  assert(mNodeMap.end() != node_it);

  const std::vector<Node*>& skelNodes = node_it->second;
  for(std::size_t i=indexInSkel; i < skelNodes.size(); ++i)
    skelNodes[i]->mIndexInSkeleton = i;

  const std::size_t indexInTree = _oldNode->mIndexInTree;
  const std::size_t treeIndex = _oldNode->getBodyNodePtr()->getTreeIndex();
  NodeMap& treeNodeMap = mTreeNodeMaps[treeIndex];
  unregisterNode(treeNodeMap, _oldNode, _oldNode->mIndexInTree);

  node_it = treeNodeMap.find(typeid(*_oldNode));
  assert(treeNodeMap.end() != node_it);

  const std::vector<Node*>& treeNodes = node_it->second;
  for(std::size_t i=indexInTree; i < treeNodes.size(); ++i)
    treeNodes[i]->mIndexInTree = i;

  // Remove it from the NameManager, if a NameManager is being used for this
  // type.
  NodeNameMgrMap::iterator name_it = mNodeNameMgrMap.find(typeid(*_oldNode));
  if(mNodeNameMgrMap.end() != name_it)
  {
    common::NameManager<Node*>& mgr = name_it->second;
    mgr.removeObject(_oldNode);
  }
}

//==============================================================================
bool Skeleton::moveBodyNodeTree(Joint* _parentJoint, BodyNode* _bodyNode,
                                SkeletonPtr _newSkeleton,
                                BodyNode* _parentNode)
{
  if(nullptr == _bodyNode)
  {
    dterr << "[Skeleton::moveBodyNodeTree] Skeleton named [" << getName()
          << "] (" << this << ") is attempting to move a nullptr BodyNode. "
          << "Please report this as a bug!\n";
    assert(false);
    return false;
  }

  if(this != _bodyNode->getSkeleton().get())
  {
    dterr << "[Skeleton::moveBodyNodeTree] Skeleton named [" << getName()
          << "] (" << this << ") is attempting to move a BodyNode named ["
          << _bodyNode->getName() << "] even though it belongs to another "
          << "Skeleton [" << _bodyNode->getSkeleton()->getName() << "] ("
          << _bodyNode->getSkeleton() << "). Please report this as a bug!\n";
    assert(false);
    return false;
  }

  if( (nullptr == _parentJoint)
      && (_bodyNode->getParentBodyNode() == _parentNode)
      && (this == _newSkeleton.get()) )
  {
    // Short-circuit if the BodyNode is already in the requested place, and its
    // Joint does not need to be changed
    return false;
  }

  if(_bodyNode == _parentNode)
  {
    dterr << "[Skeleton::moveBodyNodeTree] Attempting to move BodyNode named ["
          << _bodyNode->getName() << "] (" << _bodyNode << ") to be its own "
          << "parent. This is not permitted!\n";
    return false;
  }

  if(_parentNode && _parentNode->descendsFrom(_bodyNode))
  {
    dterr << "[Skeleton::moveBodyNodeTree] Attempting to move BodyNode named ["
          << _bodyNode->getName() << "] of Skeleton [" << getName() << "] ("
          << this << ") to be a child of BodyNode [" << _parentNode->getName()
          << "] in Skeleton [" << _newSkeleton->getName() << "] ("
          << _newSkeleton << "), but that would create a closed kinematic "
          << "chain, which is not permitted! Nothing will be moved.\n";
    return false;
  }

  if(nullptr == _newSkeleton)
  {
    if(nullptr == _parentNode)
    {
      dterr << "[Skeleton::moveBodyNodeTree] Attempting to move a BodyNode "
            << "tree starting from [" << _bodyNode->getName() << "] in "
            << "Skeleton [" << getName() << "] into a nullptr Skeleton. This "
            << "is not permitted!\n";
      return false;
    }

    _newSkeleton = _parentNode->getSkeleton();
  }

  if(_parentNode && _newSkeleton != _parentNode->getSkeleton())
  {
    dterr << "[Skeleton::moveBodyNodeTree] Mismatch between the specified "
          << "Skeleton [" << _newSkeleton->getName() << "] (" << _newSkeleton
          << ") and the specified new parent BodyNode ["
          << _parentNode->getName() << "] whose actual Skeleton is named ["
          << _parentNode->getSkeleton()->getName() << "] ("
          << _parentNode->getSkeleton() << ") while attempting to move a "
          << "BodyNode tree starting from [" << _bodyNode->getName() << "] in "
          << "Skeleton [" << getName() << "] (" << this << ")\n";
    return false;
  }

  std::vector<BodyNode*> tree = extractBodyNodeTree(_bodyNode);

  Joint* originalParent = _bodyNode->getParentJoint();
  if(originalParent != _parentJoint)
  {
    _bodyNode->mParentJoint = _parentJoint;
    _parentJoint->mChildBodyNode = _bodyNode;
    delete originalParent;
  }

  if(_parentNode != _bodyNode->getParentBodyNode())
  {
    _bodyNode->mParentBodyNode = _parentNode;
    if(_parentNode)
    {
      _parentNode->mChildBodyNodes.push_back(_bodyNode);
      _bodyNode->changeParentFrame(_parentNode);
    }
    else
    {
      _bodyNode->changeParentFrame(Frame::World());
    }
  }
  _newSkeleton->receiveBodyNodeTree(tree);

  return true;
}

//==============================================================================
std::pair<Joint*, BodyNode*> Skeleton::cloneBodyNodeTree(
    Joint* _parentJoint, const BodyNode* _bodyNode,
    const SkeletonPtr& _newSkeleton, BodyNode* _parentNode,
    bool _recursive) const
{
  std::pair<Joint*, BodyNode*> root(nullptr, nullptr);
  std::vector<const BodyNode*> tree;
  if(_recursive)
    tree = constructBodyNodeTree(_bodyNode);
  else
    tree.push_back(_bodyNode);

  std::map<std::string, BodyNode*> nameMap;
  std::vector<BodyNode*> clones;
  clones.reserve(tree.size());

  for(std::size_t i=0; i<tree.size(); ++i)
  {
    const BodyNode* original = tree[i];
    // If this is the root of the tree, and the user has requested a change in
    // its parent Joint, use the specified parent Joint instead of created a
    // clone
    Joint* joint = (i==0 && _parentJoint != nullptr) ? _parentJoint :
        original->getParentJoint()->clone();

    BodyNode* newParent = i==0 ? _parentNode :
        nameMap[original->getParentBodyNode()->getName()];

    BodyNode* clone = original->clone(newParent, joint, true);
    clones.push_back(clone);
    nameMap[clone->getName()] = clone;

    if(0==i)
    {
      root.first = joint;
      root.second = clone;
    }
  }

  _newSkeleton->receiveBodyNodeTree(clones);
  return root;
}

//==============================================================================
template <typename BodyNodeT>
static void recursiveConstructBodyNodeTree(
    std::vector<BodyNodeT*>& tree, BodyNodeT* _currentBodyNode)
{
  tree.push_back(_currentBodyNode);
  for(std::size_t i=0; i<_currentBodyNode->getNumChildBodyNodes(); ++i)
    recursiveConstructBodyNodeTree(tree, _currentBodyNode->getChildBodyNode(i));
}

//==============================================================================
std::vector<const BodyNode*> Skeleton::constructBodyNodeTree(
    const BodyNode* _bodyNode) const
{
  std::vector<const BodyNode*> tree;
  recursiveConstructBodyNodeTree<const BodyNode>(tree, _bodyNode);

  return tree;
}

//==============================================================================
std::vector<BodyNode*> Skeleton::constructBodyNodeTree(BodyNode *_bodyNode)
{
  std::vector<BodyNode*> tree;
  recursiveConstructBodyNodeTree<BodyNode>(tree, _bodyNode);

  return tree;
}

//==============================================================================
std::vector<BodyNode*> Skeleton::extractBodyNodeTree(BodyNode* _bodyNode)
{
  std::vector<BodyNode*> tree = constructBodyNodeTree(_bodyNode);

  // Go backwards to minimize the number of shifts needed
  std::vector<BodyNode*>::reverse_iterator rit;
  // Go backwards to minimize the amount of element shifting in the vectors
  for(rit = tree.rbegin(); rit != tree.rend(); ++rit)
    unregisterBodyNode(*rit);

  for(std::size_t i=0; i<mSkelCache.mBodyNodes.size(); ++i)
    mSkelCache.mBodyNodes[i]->init(getPtr());

  return tree;
}

//==============================================================================
void Skeleton::receiveBodyNodeTree(const std::vector<BodyNode*>& _tree)
{
  for(BodyNode* bn : _tree)
    registerBodyNode(bn);
}

//==============================================================================
void Skeleton::updateTotalMass()
{
  mTotalMass = 0.0;
  for(std::size_t i=0; i<getNumBodyNodes(); ++i)
    mTotalMass += getBodyNode(i)->getMass();
}

//==============================================================================
void Skeleton::updateCacheDimensions(Skeleton::DataCache& _cache)
{
  std::size_t dof = _cache.mDofs.size();
  _cache.mM        = Eigen::MatrixXd::Zero(dof, dof);
  _cache.mAugM     = Eigen::MatrixXd::Zero(dof, dof);
  _cache.mInvM     = Eigen::MatrixXd::Zero(dof, dof);
  _cache.mInvAugM  = Eigen::MatrixXd::Zero(dof, dof);
  _cache.mCvec     = Eigen::VectorXd::Zero(dof);
  _cache.mG        = Eigen::VectorXd::Zero(dof);
  _cache.mCg       = Eigen::VectorXd::Zero(dof);
  _cache.mFext     = Eigen::VectorXd::Zero(dof);
  _cache.mFc       = Eigen::VectorXd::Zero(dof);
}

//==============================================================================
void Skeleton::updateCacheDimensions(std::size_t _treeIdx)
{
  updateCacheDimensions(mTreeCache[_treeIdx]);
  updateCacheDimensions(mSkelCache);

  notifyArticulatedInertiaUpdate(_treeIdx);
}

//==============================================================================
void Skeleton::updateArticulatedInertia(std::size_t _tree) const
{
  DataCache& cache = mTreeCache[_tree];
  for (std::vector<BodyNode*>::const_reverse_iterator it = cache.mBodyNodes.rbegin();
       it != cache.mBodyNodes.rend(); ++it)
  {
    (*it)->updateArtInertia(mAspectProperties.mTimeStep);
  }

  cache.mDirty.mArticulatedInertia = false;
}

//==============================================================================
void Skeleton::updateArticulatedInertia() const
{
  for(std::size_t i=0; i<mTreeCache.size(); ++i)
  {
    DataCache& cache = mTreeCache[i];
    if(cache.mDirty.mArticulatedInertia)
      updateArticulatedInertia(i);
  }

  mSkelCache.mDirty.mArticulatedInertia = false;
}

//==============================================================================
void Skeleton::updateMassMatrix(std::size_t _treeIdx) const
{
  DataCache& cache = mTreeCache[_treeIdx];
  std::size_t dof = cache.mDofs.size();
  assert(static_cast<std::size_t>(cache.mM.cols()) == dof
         && static_cast<std::size_t>(cache.mM.rows()) == dof);
  if (dof == 0)
  {
    cache.mDirty.mMassMatrix = false;
    return;
  }

  cache.mM.setZero();

  // Backup the original internal force
  Eigen::VectorXd originalGenAcceleration = getAccelerations();

  // Clear out the accelerations of the dofs in this tree so that we can set
  // them to 1.0 one at a time to build up the mass matrix
  for (std::size_t i = 0; i < dof; ++i)
    cache.mDofs[i]->setAcceleration(0.0);

  for (std::size_t j = 0; j < dof; ++j)
  {
    // Set the acceleration of this DOF to 1.0 while all the rest are 0.0
    cache.mDofs[j]->setAcceleration(1.0);

    // Prepare cache data
    for (std::vector<BodyNode*>::const_iterator it = cache.mBodyNodes.begin();
         it != cache.mBodyNodes.end(); ++it)
    {
      (*it)->updateMassMatrix();
    }

    // Mass matrix
    for (std::vector<BodyNode*>::const_reverse_iterator it =
         cache.mBodyNodes.rbegin(); it != cache.mBodyNodes.rend(); ++it)
    {
      (*it)->aggregateMassMatrix(cache.mM, j);
      std::size_t localDof = (*it)->mParentJoint->getNumDofs();
      if (localDof > 0)
      {
        std::size_t iStart = (*it)->mParentJoint->getIndexInTree(0);

        if (iStart + localDof < j)
          break;
      }
    }

    // Set the acceleration of this DOF back to 0.0
    cache.mDofs[j]->setAcceleration(0.0);
  }
  cache.mM.triangularView<Eigen::StrictlyUpper>() = cache.mM.transpose();

  // Restore the original generalized accelerations
  const_cast<Skeleton*>(this)->setAccelerations(originalGenAcceleration);

  cache.mDirty.mMassMatrix = false;
}

//==============================================================================
void Skeleton::updateMassMatrix() const
{
  std::size_t dof = mSkelCache.mDofs.size();
  assert(static_cast<std::size_t>(mSkelCache.mM.cols()) == dof
         && static_cast<std::size_t>(mSkelCache.mM.rows()) == dof);
  if(dof == 0)
  {
    mSkelCache.mDirty.mMassMatrix = false;
    return;
  }

  mSkelCache.mM.setZero();

  for(std::size_t tree = 0; tree < mTreeCache.size(); ++tree)
  {
    const Eigen::MatrixXd& treeM = getMassMatrix(tree);
    const std::vector<DegreeOfFreedom*>& treeDofs = mTreeCache[tree].mDofs;
    std::size_t nTreeDofs = treeDofs.size();
    for(std::size_t i=0; i<nTreeDofs; ++i)
    {
      for(std::size_t j=0; j<nTreeDofs; ++j)
      {
        std::size_t ki = treeDofs[i]->getIndexInSkeleton();
        std::size_t kj = treeDofs[j]->getIndexInSkeleton();

        mSkelCache.mM(ki,kj) = treeM(i,j);
      }
    }
  }

  mSkelCache.mDirty.mMassMatrix = false;
}

//==============================================================================
void Skeleton::updateAugMassMatrix(std::size_t _treeIdx) const
{
  DataCache& cache = mTreeCache[_treeIdx];
  std::size_t dof = cache.mDofs.size();
  assert(static_cast<std::size_t>(cache.mAugM.cols()) == dof
         && static_cast<std::size_t>(cache.mAugM.rows()) == dof);
  if (dof == 0)
  {
    cache.mDirty.mAugMassMatrix = false;
    return;
  }

  cache.mAugM.setZero();

  // Backup the origianl internal force
  Eigen::VectorXd originalGenAcceleration = getAccelerations();

  // Clear out the accelerations of the DOFs in this tree so that we can set
  // them to 1.0 one at a time to build up the augmented mass matrix
  for (std::size_t i = 0; i < dof; ++i)
    cache.mDofs[i]->setAcceleration(0.0);

  for (std::size_t j = 0; j < dof; ++j)
  {
    // Set the acceleration of this DOF to 1.0 while all the rest are 0.0
    cache.mDofs[j]->setAcceleration(1.0);

    // Prepare cache data
    for (std::vector<BodyNode*>::const_iterator it = cache.mBodyNodes.begin();
         it != cache.mBodyNodes.end(); ++it)
    {
      (*it)->updateMassMatrix();
    }

    // Augmented Mass matrix
    for (std::vector<BodyNode*>::const_reverse_iterator it =
         cache.mBodyNodes.rbegin(); it != cache.mBodyNodes.rend(); ++it)
    {
      (*it)->aggregateAugMassMatrix(cache.mAugM, j, mAspectProperties.mTimeStep);
      std::size_t localDof = (*it)->mParentJoint->getNumDofs();
      if (localDof > 0)
      {
        std::size_t iStart = (*it)->mParentJoint->getIndexInTree(0);

        if (iStart + localDof < j)
          break;
      }
    }

    // Set the acceleration of this DOF back to 0.0
    cache.mDofs[j]->setAcceleration(0.0);
  }
  cache.mAugM.triangularView<Eigen::StrictlyUpper>() = cache.mAugM.transpose();

  // Restore the origianl internal force
  const_cast<Skeleton*>(this)->setAccelerations(originalGenAcceleration);

  cache.mDirty.mAugMassMatrix = false;
}

//==============================================================================
void Skeleton::updateAugMassMatrix() const
{
  std::size_t dof = mSkelCache.mDofs.size();
  assert(static_cast<std::size_t>(mSkelCache.mAugM.cols()) == dof
         && static_cast<std::size_t>(mSkelCache.mAugM.rows()) == dof);
  if(dof == 0)
  {
    mSkelCache.mDirty.mMassMatrix = false;
    return;
  }

  mSkelCache.mAugM.setZero();

  for(std::size_t tree = 0; tree < mTreeCache.size(); ++tree)
  {
    const Eigen::MatrixXd& treeAugM = getAugMassMatrix(tree);
    const std::vector<DegreeOfFreedom*>& treeDofs = mTreeCache[tree].mDofs;
    std::size_t nTreeDofs = treeDofs.size();
    for(std::size_t i=0; i<nTreeDofs; ++i)
    {
      for(std::size_t j=0; j<nTreeDofs; ++j)
      {
        std::size_t ki = treeDofs[i]->getIndexInSkeleton();
        std::size_t kj = treeDofs[j]->getIndexInSkeleton();

        mSkelCache.mAugM(ki,kj) = treeAugM(i,j);
      }
    }
  }

  mSkelCache.mDirty.mAugMassMatrix = false;
}

//==============================================================================
void Skeleton::updateInvMassMatrix(std::size_t _treeIdx) const
{
  DataCache& cache = mTreeCache[_treeIdx];
  std::size_t dof = cache.mDofs.size();
  assert(static_cast<std::size_t>(cache.mInvM.cols()) == dof
         && static_cast<std::size_t>(cache.mInvM.rows()) == dof);
  if (dof == 0)
  {
    cache.mDirty.mInvMassMatrix = false;
    return;
  }

  // We don't need to set mInvM as zero matrix as long as the below is correct
  // cache.mInvM.setZero();

  // Backup the origianl internal force
  Eigen::VectorXd originalInternalForce = getForces();

  // Clear out the forces of the dofs in this tree so that we can set them to
  // 1.0 one at a time to build up the inverse mass matrix
  for (std::size_t i = 0; i < dof; ++i)
    cache.mDofs[i]->setForce(0.0);

  for (std::size_t j = 0; j < dof; ++j)
  {
    // Set the force of this DOF to 1.0 while all the rest are 0.0
    cache.mDofs[j]->setForce(1.0);

    // Prepare cache data
    for (std::vector<BodyNode*>::const_reverse_iterator it =
         cache.mBodyNodes.rbegin(); it != cache.mBodyNodes.rend(); ++it)
    {
      (*it)->updateInvMassMatrix();
    }

    // Inverse of mass matrix
    for (std::vector<BodyNode*>::const_iterator it = cache.mBodyNodes.begin();
         it != cache.mBodyNodes.end(); ++it)
    {
      (*it)->aggregateInvMassMatrix(cache.mInvM, j);
      std::size_t localDof = (*it)->mParentJoint->getNumDofs();
      if (localDof > 0)
      {
        std::size_t iStart = (*it)->mParentJoint->getIndexInTree(0);

        if (iStart + localDof > j)
          break;
      }
    }

    // Set the force of this DOF back to 0.0
    cache.mDofs[j]->setForce(0.0);
  }
  cache.mInvM.triangularView<Eigen::StrictlyLower>() = cache.mInvM.transpose();

  // Restore the original internal force
  const_cast<Skeleton*>(this)->setForces(originalInternalForce);

  cache.mDirty.mInvMassMatrix = false;
}

//==============================================================================
void Skeleton::updateInvMassMatrix() const
{
  std::size_t dof = mSkelCache.mDofs.size();
  assert(static_cast<std::size_t>(mSkelCache.mInvM.cols()) == dof
         && static_cast<std::size_t>(mSkelCache.mInvM.rows()) == dof);
  if(dof == 0)
  {
    mSkelCache.mDirty.mInvMassMatrix = false;
    return;
  }

  mSkelCache.mInvM.setZero();

  for(std::size_t tree = 0; tree < mTreeCache.size(); ++tree)
  {
    const Eigen::MatrixXd& treeInvM = getInvMassMatrix(tree);
    const std::vector<DegreeOfFreedom*>& treeDofs = mTreeCache[tree].mDofs;
    std::size_t nTreeDofs = treeDofs.size();
    for(std::size_t i=0; i<nTreeDofs; ++i)
    {
      for(std::size_t j=0; j<nTreeDofs; ++j)
      {
        std::size_t ki = treeDofs[i]->getIndexInSkeleton();
        std::size_t kj = treeDofs[j]->getIndexInSkeleton();

        mSkelCache.mInvM(ki,kj) = treeInvM(i,j);
      }
    }
  }

  mSkelCache.mDirty.mInvMassMatrix = false;
}

//==============================================================================
void Skeleton::updateInvAugMassMatrix(std::size_t _treeIdx) const
{
  DataCache& cache = mTreeCache[_treeIdx];
  std::size_t dof = cache.mDofs.size();
  assert(static_cast<std::size_t>(cache.mInvAugM.cols()) == dof
         && static_cast<std::size_t>(cache.mInvAugM.rows()) == dof);
  if (dof == 0)
  {
    cache.mDirty.mInvAugMassMatrix = false;
    return;
  }

  // We don't need to set mInvM as zero matrix as long as the below is correct
  // mInvM.setZero();

  // Backup the origianl internal force
  Eigen::VectorXd originalInternalForce = getForces();

  // Clear out the forces of the dofs in this tree so that we can set them to
  // 1.0 one at a time to build up the inverse augmented mass matrix
  for (std::size_t i = 0; i < dof; ++i)
    cache.mDofs[i]->setForce(0.0);

  for (std::size_t j = 0; j < dof; ++j)
  {
    // Set the force of this DOF to 1.0 while all the rest are 0.0
    cache.mDofs[j]->setForce(1.0);

    // Prepare cache data
    for (std::vector<BodyNode*>::const_reverse_iterator it =
         cache.mBodyNodes.rbegin(); it != cache.mBodyNodes.rend(); ++it)
    {
      (*it)->updateInvAugMassMatrix();
    }

    // Inverse of augmented mass matrix
    for (std::vector<BodyNode*>::const_iterator it = cache.mBodyNodes.begin();
         it != cache.mBodyNodes.end(); ++it)
    {
      (*it)->aggregateInvAugMassMatrix(cache.mInvAugM, j, mAspectProperties.mTimeStep);
      std::size_t localDof = (*it)->mParentJoint->getNumDofs();
      if (localDof > 0)
      {
        std::size_t iStart = (*it)->mParentJoint->getIndexInTree(0);

        if (iStart + localDof > j)
          break;
      }
    }

    // Set the force of this DOF back to 0.0
    cache.mDofs[j]->setForce(0.0);
  }
  cache.mInvAugM.triangularView<Eigen::StrictlyLower>() =
      cache.mInvAugM.transpose();

  // Restore the original internal force
  const_cast<Skeleton*>(this)->setForces(originalInternalForce);

  cache.mDirty.mInvAugMassMatrix = false;
}

//==============================================================================
void Skeleton::updateInvAugMassMatrix() const
{
  std::size_t dof = mSkelCache.mDofs.size();
  assert(static_cast<std::size_t>(mSkelCache.mInvAugM.cols()) == dof
         && static_cast<std::size_t>(mSkelCache.mInvAugM.rows()) == dof);
  if(dof == 0)
  {
    mSkelCache.mDirty.mInvAugMassMatrix = false;
    return;
  }

  mSkelCache.mInvAugM.setZero();

  for(std::size_t tree = 0; tree < mTreeCache.size(); ++tree)
  {
    const Eigen::MatrixXd& treeInvAugM = getInvAugMassMatrix(tree);
    const std::vector<DegreeOfFreedom*>& treeDofs = mTreeCache[tree].mDofs;
    std::size_t nTreeDofs = treeDofs.size();
    for(std::size_t i=0; i<nTreeDofs; ++i)
    {
      for(std::size_t j=0; j<nTreeDofs; ++j)
      {
        std::size_t ki = treeDofs[i]->getIndexInSkeleton();
        std::size_t kj = treeDofs[j]->getIndexInSkeleton();

        mSkelCache.mInvAugM(ki,kj) = treeInvAugM(i,j);
      }
    }
  }

  mSkelCache.mDirty.mInvAugMassMatrix = false;
}

//==============================================================================
void Skeleton::updateCoriolisForces(std::size_t _treeIdx) const
{
  DataCache& cache = mTreeCache[_treeIdx];
  std::size_t dof = cache.mDofs.size();
  assert(static_cast<std::size_t>(cache.mCvec.size()) == dof);
  if (dof == 0)
  {
    cache.mDirty.mCoriolisForces = false;
    return;
  }

  cache.mCvec.setZero();

  for (std::vector<BodyNode*>::const_iterator it = cache.mBodyNodes.begin();
       it != cache.mBodyNodes.end(); ++it)
  {
    (*it)->updateCombinedVector();
  }

  for (std::vector<BodyNode*>::const_reverse_iterator it =
       cache.mBodyNodes.rbegin(); it != cache.mBodyNodes.rend(); ++it)
  {
    (*it)->aggregateCoriolisForceVector(cache.mCvec);
  }

  cache.mDirty.mCoriolisForces = false;
}

//==============================================================================
void Skeleton::updateCoriolisForces() const
{
  std::size_t dof = mSkelCache.mDofs.size();
  assert(static_cast<std::size_t>(mSkelCache.mCvec.size()) == dof);
  if(dof == 0)
  {
    mSkelCache.mDirty.mCoriolisForces = false;
    return;
  }

  mSkelCache.mCvec.setZero();

  for(std::size_t tree = 0; tree < mTreeCache.size(); ++tree)
  {
    const Eigen::VectorXd& treeCvec = getCoriolisForces(tree);
    const std::vector<DegreeOfFreedom*>& treeDofs = mTreeCache[tree].mDofs;
    std::size_t nTreeDofs = treeDofs.size();
    for(std::size_t i=0; i<nTreeDofs; ++i)
    {
      std::size_t k = treeDofs[i]->getIndexInSkeleton();
      mSkelCache.mCvec[k] = treeCvec[i];
    }
  }

  mSkelCache.mDirty.mCoriolisForces = false;
}

//==============================================================================
void Skeleton::updateGravityForces(std::size_t _treeIdx) const
{
  DataCache& cache = mTreeCache[_treeIdx];
  std::size_t dof = cache.mDofs.size();
  assert(static_cast<std::size_t>(cache.mG.size()) == dof);
  if (dof == 0)
  {
    cache.mDirty.mGravityForces = false;
    return;
  }

  cache.mG.setZero();

  for (std::vector<BodyNode*>::const_reverse_iterator it =
       cache.mBodyNodes.rbegin(); it != cache.mBodyNodes.rend(); ++it)
  {
    (*it)->aggregateGravityForceVector(cache.mG, mAspectProperties.mGravity);
  }

  cache.mDirty.mGravityForces = false;
}

//==============================================================================
void Skeleton::updateGravityForces() const
{
  std::size_t dof = mSkelCache.mDofs.size();
  assert(static_cast<std::size_t>(mSkelCache.mG.size()) == dof);
  if(dof == 0)
  {
    mSkelCache.mDirty.mGravityForces = false;
    return;
  }

  mSkelCache.mG.setZero();

  for(std::size_t tree = 0; tree < mTreeCache.size(); ++tree)
  {
    const Eigen::VectorXd& treeG = getGravityForces(tree);
    std::vector<DegreeOfFreedom*>& treeDofs = mTreeCache[tree].mDofs;
    std::size_t nTreeDofs = treeDofs.size();
    for(std::size_t i=0; i<nTreeDofs; ++i)
    {
      std::size_t k = treeDofs[i]->getIndexInSkeleton();
      mSkelCache.mG[k] = treeG[i];
    }
  }

  mSkelCache.mDirty.mGravityForces = false;
}

//==============================================================================
void Skeleton::updateCoriolisAndGravityForces(std::size_t _treeIdx) const
{
  DataCache& cache = mTreeCache[_treeIdx];
  std::size_t dof = cache.mDofs.size();
  assert(static_cast<std::size_t>(cache.mCg.size()) == dof);
  if (dof == 0)
  {
    cache.mDirty.mCoriolisAndGravityForces = false;
    return;
  }

  cache.mCg.setZero();

  for (std::vector<BodyNode*>::const_iterator it = cache.mBodyNodes.begin();
       it != cache.mBodyNodes.end(); ++it)
  {
    (*it)->updateCombinedVector();
  }

  for (std::vector<BodyNode*>::const_reverse_iterator it =
       cache.mBodyNodes.rbegin(); it != cache.mBodyNodes.rend(); ++it)
  {
    (*it)->aggregateCombinedVector(cache.mCg, mAspectProperties.mGravity);
  }

  cache.mDirty.mCoriolisAndGravityForces = false;
}

//==============================================================================
void Skeleton::updateCoriolisAndGravityForces() const
{
  std::size_t dof = mSkelCache.mDofs.size();
  assert(static_cast<std::size_t>(mSkelCache.mCg.size()) == dof);
  if (dof == 0)
  {
    mSkelCache.mDirty.mCoriolisAndGravityForces = false;
    return;
  }

  mSkelCache.mCg.setZero();

  for(std::size_t tree = 0; tree < mTreeCache.size(); ++tree)
  {
    const Eigen::VectorXd& treeCg = getCoriolisAndGravityForces(tree);
    const std::vector<DegreeOfFreedom*>& treeDofs = mTreeCache[tree].mDofs;
    std::size_t nTreeDofs = treeDofs.size();
    for(std::size_t i=0; i<nTreeDofs; ++i)
    {
      std::size_t k = treeDofs[i]->getIndexInSkeleton();
      mSkelCache.mCg[k] = treeCg[i];
    }
  }

  mSkelCache.mDirty.mCoriolisAndGravityForces = false;
}

//==============================================================================
void Skeleton::updateExternalForces(std::size_t _treeIdx) const
{
  DataCache& cache = mTreeCache[_treeIdx];
  std::size_t dof = cache.mDofs.size();
  assert(static_cast<std::size_t>(cache.mFext.size()) == dof);
  if (dof == 0)
  {
    cache.mDirty.mExternalForces = false;
    return;
  }

  // Clear external force.
  cache.mFext.setZero();

  for (std::vector<BodyNode*>::const_reverse_iterator itr =
       cache.mBodyNodes.rbegin(); itr != cache.mBodyNodes.rend(); ++itr)
  {
    (*itr)->aggregateExternalForces(cache.mFext);
  }

  // TODO(JS): Not implemented yet
//  for (std::vector<SoftBodyNode*>::iterator it = mSoftBodyNodes.begin();
//       it != mSoftBodyNodes.end(); ++it)
//  {
//    double kv = (*it)->getVertexSpringStiffness();
//    double ke = (*it)->getEdgeSpringStiffness();

//    for (int i = 0; i < (*it)->getNumPointMasses(); ++i)
//    {
//      PointMass* pm = (*it)->getPointMass(i);
//      int nN = pm->getNumConnectedPointMasses();

//      // Vertex restoring force
//      Eigen::Vector3d Fext = -(kv + nN * ke) * pm->getPositions()
//                             - (mTimeStep * (kv + nN*ke)) * pm->getVelocities();

//      // Edge restoring force
//      for (int j = 0; j < nN; ++j)
//      {
//        Fext += ke * (pm->getConnectedPointMass(j)->getPositions()
//                      + mTimeStep
//                        * pm->getConnectedPointMass(j)->getVelocities());
//      }

//      // Assign
//      int iStart = pm->getIndexInSkeleton(0);
//      mFext.segment<3>(iStart) = Fext;
//    }
//  }

  cache.mDirty.mExternalForces = false;
}

//==============================================================================
void Skeleton::updateExternalForces() const
{
  std::size_t dof = mSkelCache.mDofs.size();
  assert(static_cast<std::size_t>(mSkelCache.mFext.size()) == dof);
  if(dof == 0)
  {
    mSkelCache.mDirty.mExternalForces = false;
    return;
  }

  mSkelCache.mFext.setZero();

  for(std::size_t tree = 0; tree < mTreeCache.size(); ++tree)
  {
    const Eigen::VectorXd& treeFext = getExternalForces(tree);
    const std::vector<DegreeOfFreedom*>& treeDofs = mTreeCache[tree].mDofs;
    std::size_t nTreeDofs = treeDofs.size();
    for(std::size_t i=0; i<nTreeDofs; ++i)
    {
      std::size_t k = treeDofs[i]->getIndexInSkeleton();
      mSkelCache.mFext[k] = treeFext[i];
    }
  }

  mSkelCache.mDirty.mExternalForces = false;
}

//==============================================================================
const Eigen::VectorXd& Skeleton::computeConstraintForces(DataCache& cache) const
{
  const std::size_t dof = cache.mDofs.size();
  assert(static_cast<std::size_t>(cache.mFc.size()) == dof);

  // Body constraint impulses
  for (std::vector<BodyNode*>::reverse_iterator it =
       cache.mBodyNodes.rbegin();
       it != cache.mBodyNodes.rend(); ++it)
  {
    (*it)->aggregateSpatialToGeneralized(
          cache.mFc, (*it)->getConstraintImpulse());
  }

  // Joint constraint impulses
  for (std::size_t i = 0; i < dof; ++i)
    cache.mFc[i] += cache.mDofs[i]->getConstraintImpulse();

  // Get force by dividing the impulse by the time step
  cache.mFc = cache.mFc / mAspectProperties.mTimeStep;

  return cache.mFc;
}

//==============================================================================
static void computeSupportPolygon(
    const Skeleton* skel, math::SupportPolygon& polygon,
    math::SupportGeometry& geometry,  std::vector<std::size_t>& ee_indices,
    Eigen::Vector3d& axis1, Eigen::Vector3d& axis2, Eigen::Vector2d& centroid,
    std::size_t treeIndex)
{
  polygon.clear();
  geometry.clear();
  ee_indices.clear();

  const Eigen::Vector3d& up = -skel->getGravity();
  if(up.norm() == 0.0)
  {
    dtwarn << "[computeSupportPolygon] Requesting support polygon of a "
           << "Skeleton with no gravity. The result will only be an empty "
           << "set!\n";
    axis1.setZero();
    axis2.setZero();
    centroid = Eigen::Vector2d::Constant(std::nan(""));
    return;
  }

  std::vector<std::size_t> originalEE_map;
  originalEE_map.reserve(skel->getNumEndEffectors());
  for(std::size_t i=0; i < skel->getNumEndEffectors(); ++i)
  {
    const EndEffector* ee = skel->getEndEffector(i);
    if(ee->getSupport() && ee->getSupport()->isActive()
       && (INVALID_INDEX == treeIndex || ee->getTreeIndex() == treeIndex))
    {
      const math::SupportGeometry& eeGeom = ee->getSupport()->getGeometry();
      for(const Eigen::Vector3d& v : eeGeom)
      {
        geometry.push_back(ee->getWorldTransform()*v);
        originalEE_map.push_back(ee->getIndexInSkeleton());
      }
    }
  }

  axis1 = (up-Eigen::Vector3d::UnitX()).norm() > 1e-6 ?
        Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();

  axis1 = axis1 - up.dot(axis1)*up/up.dot(up);
  axis1.normalize();

  axis2 = up.normalized().cross(axis1);

  std::vector<std::size_t> vertex_indices;
  polygon = math::computeSupportPolgyon(vertex_indices, geometry, axis1, axis2);

  ee_indices.reserve(vertex_indices.size());
  for(std::size_t i=0; i < vertex_indices.size(); ++i)
    ee_indices[i] = originalEE_map[vertex_indices[i]];

  if(polygon.size() > 0)
    centroid = math::computeCentroidOfHull(polygon);
  else
    centroid = Eigen::Vector2d::Constant(std::nan(""));
}

//==============================================================================
const math::SupportPolygon& Skeleton::getSupportPolygon() const
{
  math::SupportPolygon& polygon = mSkelCache.mSupportPolygon;

  if(!mSkelCache.mDirty.mSupport)
    return polygon;

  computeSupportPolygon(this, polygon, mSkelCache.mSupportGeometry,
                        mSkelCache.mSupportIndices,
                        mSkelCache.mSupportAxes.first,
                        mSkelCache.mSupportAxes.second,
                        mSkelCache.mSupportCentroid, INVALID_INDEX);

  mSkelCache.mDirty.mSupport = false;
  ++mSkelCache.mDirty.mSupportVersion;
  return polygon;
}

//==============================================================================
const math::SupportPolygon& Skeleton::getSupportPolygon(std::size_t _treeIdx) const
{
  math::SupportPolygon& polygon = mTreeCache[_treeIdx].mSupportPolygon;

  if(!mTreeCache[_treeIdx].mDirty.mSupport)
    return polygon;

  computeSupportPolygon(this, polygon, mTreeCache[_treeIdx].mSupportGeometry,
                        mTreeCache[_treeIdx].mSupportIndices,
                        mTreeCache[_treeIdx].mSupportAxes.first,
                        mTreeCache[_treeIdx].mSupportAxes.second,
                        mTreeCache[_treeIdx].mSupportCentroid, _treeIdx);

  mTreeCache[_treeIdx].mDirty.mSupport = false;
  ++mTreeCache[_treeIdx].mDirty.mSupportVersion;
  return polygon;
}

//==============================================================================
const std::vector<std::size_t>& Skeleton::getSupportIndices() const
{
  getSupportPolygon();
  return mSkelCache.mSupportIndices;
}

//==============================================================================
const std::vector<std::size_t>& Skeleton::getSupportIndices(std::size_t _treeIdx) const
{
  getSupportPolygon(_treeIdx);
  return mTreeCache[_treeIdx].mSupportIndices;
}

//==============================================================================
const std::pair<Eigen::Vector3d, Eigen::Vector3d>&
Skeleton::getSupportAxes() const
{
  getSupportPolygon();
  return mSkelCache.mSupportAxes;
}

//==============================================================================
const std::pair<Eigen::Vector3d, Eigen::Vector3d>&
Skeleton::getSupportAxes(std::size_t _treeIdx) const
{
  getSupportPolygon(_treeIdx);
  return mTreeCache[_treeIdx].mSupportAxes;
}

//==============================================================================
const Eigen::Vector2d& Skeleton::getSupportCentroid() const
{
  getSupportPolygon();
  return mSkelCache.mSupportCentroid;
}

//==============================================================================
const Eigen::Vector2d& Skeleton::getSupportCentroid(std::size_t _treeIdx) const
{
  getSupportPolygon(_treeIdx);
  return mTreeCache[_treeIdx].mSupportCentroid;
}

//==============================================================================
std::size_t Skeleton::getSupportVersion() const
{
  if(mSkelCache.mDirty.mSupport)
    return mSkelCache.mDirty.mSupportVersion + 1;

  return mSkelCache.mDirty.mSupportVersion;
}

//==============================================================================
std::size_t Skeleton::getSupportVersion(std::size_t _treeIdx) const
{
  if(mTreeCache[_treeIdx].mDirty.mSupport)
    return mTreeCache[_treeIdx].mDirty.mSupportVersion + 1;

  return mTreeCache[_treeIdx].mDirty.mSupportVersion;
}

//==============================================================================
void Skeleton::computeForwardKinematics(bool _updateTransforms,
                                        bool _updateVels,
                                        bool _updateAccs)
{
  if (_updateTransforms)
  {
    for (std::vector<BodyNode*>::iterator it = mSkelCache.mBodyNodes.begin();
         it != mSkelCache.mBodyNodes.end(); ++it)
    {
      (*it)->updateTransform();
    }
  }

  if (_updateVels)
  {
    for (std::vector<BodyNode*>::iterator it = mSkelCache.mBodyNodes.begin();
         it != mSkelCache.mBodyNodes.end(); ++it)
    {
      (*it)->updateVelocity();
      (*it)->updatePartialAcceleration();
    }
  }

  if (_updateAccs)
  {
    for (std::vector<BodyNode*>::iterator it = mSkelCache.mBodyNodes.begin();
         it != mSkelCache.mBodyNodes.end(); ++it)
    {
      (*it)->updateAccelerationID();
    }
  }
}

//==============================================================================
void Skeleton::computeForwardDynamics()
{
  // Note: Articulated Inertias will be updated automatically when
  // getArtInertiaImplicit() is called in BodyNode::updateBiasForce()

  for (auto it = mSkelCache.mBodyNodes.rbegin();
       it != mSkelCache.mBodyNodes.rend(); ++it)
    (*it)->updateBiasForce(mAspectProperties.mGravity, mAspectProperties.mTimeStep);

  // Forward recursion
  for (auto& bodyNode : mSkelCache.mBodyNodes)
  {
    bodyNode->updateAccelerationFD();
    bodyNode->updateTransmittedForceFD();
    bodyNode->updateJointForceFD(mAspectProperties.mTimeStep, true, true);
  }
}

//==============================================================================
void Skeleton::computeInverseDynamics(bool _withExternalForces,
                                      bool _withDampingForces,
                                      bool _withSpringForces)
{
  // Skip immobile or 0-dof skeleton
  if (getNumDofs() == 0)
    return;

  // Backward recursion
  for (auto it = mSkelCache.mBodyNodes.rbegin();
       it != mSkelCache.mBodyNodes.rend(); ++it)
  {
    (*it)->updateTransmittedForceID(mAspectProperties.mGravity, _withExternalForces);
    (*it)->updateJointForceID(mAspectProperties.mTimeStep,
                              _withDampingForces,
                              _withSpringForces);
  }
}

//==============================================================================
void Skeleton::clearExternalForces()
{
  for (auto& bodyNode : mSkelCache.mBodyNodes)
    bodyNode->clearExternalForces();
}

//==============================================================================
void Skeleton::clearInternalForces()
{
  for (auto& bodyNode : mSkelCache.mBodyNodes)
    bodyNode->clearInternalForces();
}

//==============================================================================
void Skeleton::notifyArticulatedInertiaUpdate(std::size_t _treeIdx)
{
  SET_FLAG(_treeIdx, mArticulatedInertia);
  SET_FLAG(_treeIdx, mMassMatrix);
  SET_FLAG(_treeIdx, mAugMassMatrix);
  SET_FLAG(_treeIdx, mInvMassMatrix);
  SET_FLAG(_treeIdx, mInvAugMassMatrix);
  SET_FLAG(_treeIdx, mCoriolisForces);
  SET_FLAG(_treeIdx, mGravityForces);
  SET_FLAG(_treeIdx, mCoriolisAndGravityForces);
}

//==============================================================================
void Skeleton::notifySupportUpdate(std::size_t _treeIdx)
{
  SET_FLAG(_treeIdx, mSupport);
}

//==============================================================================
void Skeleton::clearConstraintImpulses()
{
  for (auto& bodyNode : mSkelCache.mBodyNodes)
    bodyNode->clearConstraintImpulse();
}

//==============================================================================
void Skeleton::updateBiasImpulse(BodyNode* _bodyNode)
{
  if(nullptr == _bodyNode)
  {
    dterr << "[Skeleton::updateBiasImpulse] Passed in a nullptr!\n";
    assert(false);
    return;
  }

  assert(getNumDofs() > 0);

  // This skeleton should contain _bodyNode
  assert(_bodyNode->getSkeleton().get() == this);

#ifndef NDEBUG
  // All the constraint impulse should be zero
  for (std::size_t i = 0; i < mSkelCache.mBodyNodes.size(); ++i)
    assert(mSkelCache.mBodyNodes[i]->mConstraintImpulse == Eigen::Vector6d::Zero());
#endif

  // Prepare cache data
  BodyNode* it = _bodyNode;
  while (it != nullptr)
  {
    it->updateBiasImpulse();
    it = it->getParentBodyNode();
  }
}

//==============================================================================
void Skeleton::updateBiasImpulse(BodyNode* _bodyNode,
                                 const Eigen::Vector6d& _imp)
{
  if(nullptr == _bodyNode)
  {
    dterr << "[Skeleton::updateBiasImpulse] Passed in a nullptr!\n";
    assert(false);
    return;
  }

  assert(getNumDofs() > 0);

  // This skeleton should contain _bodyNode
  assert(_bodyNode->getSkeleton().get() == this);

#ifndef NDEBUG
  // All the constraint impulse should be zero
  for (std::size_t i = 0; i < mSkelCache.mBodyNodes.size(); ++i)
    assert(mSkelCache.mBodyNodes[i]->mConstraintImpulse == Eigen::Vector6d::Zero());
#endif

  // Set impulse of _bodyNode
  _bodyNode->mConstraintImpulse = _imp;

  // Prepare cache data
  BodyNode* it = _bodyNode;
  while (it != nullptr)
  {
    it->updateBiasImpulse();
    it = it->getParentBodyNode();
  }

  _bodyNode->mConstraintImpulse.setZero();
}

//==============================================================================
void Skeleton::updateBiasImpulse(BodyNode* _bodyNode1,
                                 const Eigen::Vector6d& _imp1,
                                 BodyNode* _bodyNode2,
                                 const Eigen::Vector6d& _imp2)
{
  // Assertions
  if(nullptr == _bodyNode1)
  {
    dterr << "[Skeleton::updateBiasImpulse] Passed in nullptr for BodyNode1!\n";
    assert(false);
    return;
  }

  if(nullptr == _bodyNode2)
  {
    dterr << "[Skeleton::updateBiasImpulse] Passed in nullptr for BodyNode2!\n";
    assert(false);
    return;
  }

  assert(getNumDofs() > 0);

  // This skeleton should contain _bodyNode
  assert(_bodyNode1->getSkeleton().get() == this);
  assert(_bodyNode2->getSkeleton().get() == this);

#ifndef NDEBUG
  // All the constraint impulse should be zero
  for (std::size_t i = 0; i < mSkelCache.mBodyNodes.size(); ++i)
    assert(mSkelCache.mBodyNodes[i]->mConstraintImpulse ==
           Eigen::Vector6d::Zero());
#endif

  // Set impulse to _bodyNode
  _bodyNode1->mConstraintImpulse = _imp1;
  _bodyNode2->mConstraintImpulse = _imp2;

  // Find which body is placed later in the list of body nodes in this skeleton
  std::size_t index1 = _bodyNode1->getIndexInSkeleton();
  std::size_t index2 = _bodyNode2->getIndexInSkeleton();

  std::size_t index = std::max(index1, index2);

  // Prepare cache data
  for (int i = index; 0 <= i; --i)
    mSkelCache.mBodyNodes[i]->updateBiasImpulse();

  _bodyNode1->mConstraintImpulse.setZero();
  _bodyNode2->mConstraintImpulse.setZero();
}

//==============================================================================
void Skeleton::updateBiasImpulse(SoftBodyNode* _softBodyNode,
                                 PointMass* _pointMass,
                                 const Eigen::Vector3d& _imp)
{
  // Assertions
  assert(_softBodyNode != nullptr);
  assert(getNumDofs() > 0);

  // This skeleton should contain _bodyNode
  assert(std::find(mSoftBodyNodes.begin(), mSoftBodyNodes.end(), _softBodyNode)
         != mSoftBodyNodes.end());

#ifndef NDEBUG
  // All the constraint impulse should be zero
  for (std::size_t i = 0; i < mSkelCache.mBodyNodes.size(); ++i)
    assert(mSkelCache.mBodyNodes[i]->mConstraintImpulse ==
           Eigen::Vector6d::Zero());
#endif

  // Set impulse to _bodyNode
  Eigen::Vector3d oldConstraintImpulse =_pointMass->getConstraintImpulses();
  _pointMass->setConstraintImpulse(_imp, true);

  // Prepare cache data
  BodyNode* it = _softBodyNode;
  while (it != nullptr)
  {
    it->updateBiasImpulse();
    it = it->getParentBodyNode();
  }

  // TODO(JS): Do we need to backup and restore the original value?
  _pointMass->setConstraintImpulse(oldConstraintImpulse);
}

//==============================================================================
void Skeleton::updateVelocityChange()
{
  for (auto& bodyNode : mSkelCache.mBodyNodes)
    bodyNode->updateVelocityChangeFD();
}

//==============================================================================
void Skeleton::setImpulseApplied(bool _val)
{
  mIsImpulseApplied = _val;
}

//==============================================================================
bool Skeleton::isImpulseApplied() const
{
  return mIsImpulseApplied;
}

//==============================================================================
void Skeleton::computeImpulseForwardDynamics()
{
  // Skip immobile or 0-dof skeleton
  if (!isMobile() || getNumDofs() == 0)
    return;

  // Note: we do not need to update articulated inertias here, because they will
  // be updated when BodyNode::updateBiasImpulse() calls
  // BodyNode::getArticulatedInertia()

  // Backward recursion
  for (auto it = mSkelCache.mBodyNodes.rbegin();
       it != mSkelCache.mBodyNodes.rend(); ++it)
    (*it)->updateBiasImpulse();

  // Forward recursion
  for (auto& bodyNode : mSkelCache.mBodyNodes)
  {
    bodyNode->updateVelocityChangeFD();
    bodyNode->updateTransmittedImpulse();
    bodyNode->updateJointImpulseFD();
    bodyNode->updateConstrainedTerms(mAspectProperties.mTimeStep);
  }
}

//==============================================================================
double Skeleton::computeKineticEnergy() const
{
  double KE = 0.0;

  for (auto* bodyNode : mSkelCache.mBodyNodes)
    KE += bodyNode->computeKineticEnergy();

  assert(KE >= 0.0 && "Kinetic energy should be positive value.");
  return KE;
}

//==============================================================================
double Skeleton::computePotentialEnergy() const
{
  double PE = 0.0;

  for (auto* bodyNode : mSkelCache.mBodyNodes)
  {
    PE += bodyNode->computePotentialEnergy(mAspectProperties.mGravity);
    PE += bodyNode->getParentJoint()->computePotentialEnergy();
  }

  return PE;
}

//==============================================================================
void Skeleton::clearCollidingBodies()
{
  for (auto i = 0u; i < getNumBodyNodes(); ++i)
  {
    auto bodyNode = getBodyNode(i);
DART_SUPPRESS_DEPRECATED_BEGIN
    bodyNode->setColliding(false);
DART_SUPPRESS_DEPRECATED_END

    auto softBodyNode = bodyNode->asSoftBodyNode();
    if (softBodyNode)
    {
      auto& pointMasses = softBodyNode->getPointMasses();

      for (auto pointMass : pointMasses)
        pointMass->setColliding(false);
    }
  }
}

//==============================================================================
Eigen::Vector3d Skeleton::getCOM(const Frame* _withRespectTo) const
{
  Eigen::Vector3d com = Eigen::Vector3d::Zero();

  const std::size_t numBodies = getNumBodyNodes();
  for (std::size_t i = 0; i < numBodies; ++i)
  {
    const BodyNode* bodyNode = getBodyNode(i);
    com += bodyNode->getMass() * bodyNode->getCOM(_withRespectTo);
  }

  assert(mTotalMass != 0.0);
  return com / mTotalMass;
}

//==============================================================================
// Templated function for computing different kinds of COM properties, like
// velocities and accelerations
template <
    typename PropertyType,
    PropertyType (BodyNode::*getPropertyFn)(const Frame*, const Frame*) const>
PropertyType getCOMPropertyTemplate(const Skeleton* _skel,
                                    const Frame* _relativeTo,
                                    const Frame* _inCoordinatesOf)
{
  PropertyType result(PropertyType::Zero());

  const std::size_t numBodies = _skel->getNumBodyNodes();
  for (std::size_t i = 0; i < numBodies; ++i)
  {
    const BodyNode* bodyNode = _skel->getBodyNode(i);
    result += bodyNode->getMass()
              * (bodyNode->*getPropertyFn)(_relativeTo, _inCoordinatesOf);
  }

  assert(_skel->getMass() != 0.0);
  return result / _skel->getMass();
}

//==============================================================================
Eigen::Vector6d Skeleton::getCOMSpatialVelocity(const Frame* _relativeTo,
    const Frame* _inCoordinatesOf) const
{
  return getCOMPropertyTemplate<Eigen::Vector6d,
      &BodyNode::getCOMSpatialVelocity>(this, _relativeTo, _inCoordinatesOf);
}

//==============================================================================
Eigen::Vector3d Skeleton::getCOMLinearVelocity(const Frame* _relativeTo,
    const Frame* _inCoordinatesOf) const
{
  return getCOMPropertyTemplate<Eigen::Vector3d,
      &BodyNode::getCOMLinearVelocity>(this, _relativeTo, _inCoordinatesOf);
}

//==============================================================================
Eigen::Vector6d Skeleton::getCOMSpatialAcceleration(const Frame* _relativeTo,
    const Frame* _inCoordinatesOf) const
{
  return getCOMPropertyTemplate<Eigen::Vector6d,
      &BodyNode::getCOMSpatialAcceleration>(
        this, _relativeTo, _inCoordinatesOf);
}

//==============================================================================
Eigen::Vector3d Skeleton::getCOMLinearAcceleration(const Frame* _relativeTo,
    const Frame* _inCoordinatesOf) const
{
  return getCOMPropertyTemplate<Eigen::Vector3d,
      &BodyNode::getCOMLinearAcceleration>(this, _relativeTo, _inCoordinatesOf);
}

//==============================================================================
// Templated function for computing different kinds of COM Jacobians and their
// derivatives
template <
    typename JacType, // JacType is the type of Jacobian we're computing
    JacType (TemplatedJacobianNode<BodyNode>::*getJacFn)(
        const Eigen::Vector3d&, const Frame*) const>
JacType getCOMJacobianTemplate(const Skeleton* _skel,
                               const Frame* _inCoordinatesOf)
{
  // Initialize the Jacobian to zero
  JacType J = JacType::Zero(JacType::RowsAtCompileTime, _skel->getNumDofs());

  // Iterate through each of the Skeleton's BodyNodes
  const std::size_t numBodies = _skel->getNumBodyNodes();
  for (std::size_t i = 0; i < numBodies; ++i)
  {
    const BodyNode* bn = _skel->getBodyNode(i);

    // (bn->*getJacFn) is a function pointer to the function that gives us the
    // kind of Jacobian we want from the BodyNodes. Calling it will give us the
    // relevant Jacobian for this BodyNode
    JacType bnJ = bn->getMass() * (bn->*getJacFn)(bn->getLocalCOM(),
                                                  _inCoordinatesOf);

    // For each column in the Jacobian of this BodyNode, we add it to the
    // appropriate column of the overall BodyNode
    for (std::size_t j=0, end=bn->getNumDependentGenCoords(); j < end; ++j)
    {
      std::size_t idx = bn->getDependentGenCoordIndex(j);
      J.col(idx) += bnJ.col(j);
    }
  }

  assert(_skel->getMass() != 0.0);
  return J / _skel->getMass();
}

//==============================================================================
math::Jacobian Skeleton::getCOMJacobian(const Frame* _inCoordinatesOf) const
{
  return getCOMJacobianTemplate<math::Jacobian,
          &TemplatedJacobianNode<BodyNode>::getJacobian>(
        this, _inCoordinatesOf);
}

//==============================================================================
math::LinearJacobian Skeleton::getCOMLinearJacobian(
    const Frame* _inCoordinatesOf) const
{
  return getCOMJacobianTemplate<math::LinearJacobian,
           &TemplatedJacobianNode<BodyNode>::getLinearJacobian>(
              this, _inCoordinatesOf);
}

//==============================================================================
math::Jacobian Skeleton::getCOMJacobianSpatialDeriv(
    const Frame* _inCoordinatesOf) const
{
  return getCOMJacobianTemplate<math::Jacobian,
      &TemplatedJacobianNode<BodyNode>::getJacobianSpatialDeriv>(
              this, _inCoordinatesOf);
}

//==============================================================================
math::LinearJacobian Skeleton::getCOMLinearJacobianDeriv(
    const Frame* _inCoordinatesOf) const
{
  return getCOMJacobianTemplate<math::LinearJacobian,
      &TemplatedJacobianNode<BodyNode>::getLinearJacobianDeriv>(
              this, _inCoordinatesOf);
}

//==============================================================================
Skeleton::DirtyFlags::DirtyFlags()
  : mArticulatedInertia(true),
    mMassMatrix(true),
    mAugMassMatrix(true),
    mInvMassMatrix(true),
    mInvAugMassMatrix(true),
    mGravityForces(true),
    mCoriolisForces(true),
    mCoriolisAndGravityForces(true),
    mExternalForces(true),
    mDampingForces(true),
    mSupport(true),
    mSupportVersion(0)
{
  // Do nothing
}

}  // namespace dynamics
}  // namespace dart
