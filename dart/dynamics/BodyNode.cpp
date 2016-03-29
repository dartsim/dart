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

#include "dart/dynamics/BodyNode.hpp"

#include <algorithm>
#include <vector>
#include <string>

#include "dart/common/Console.hpp"
#include "dart/common/StlHelpers.hpp"
#include "dart/math/Helpers.hpp"
#include "dart/renderer/RenderInterface.hpp"
#include "dart/dynamics/Joint.hpp"
#include "dart/dynamics/Shape.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/Chain.hpp"
#include "dart/dynamics/Marker.hpp"
#include "dart/dynamics/SoftBodyNode.hpp"
#include "dart/dynamics/EndEffector.hpp"

namespace dart {
namespace dynamics {

//==============================================================================
SkeletonRefCountingBase::SkeletonRefCountingBase()
  : mReferenceCount(0),
    mLockedSkeleton(std::make_shared<MutexedWeakSkeletonPtr>())
{
  // Do nothing
}

//==============================================================================
void SkeletonRefCountingBase::incrementReferenceCount() const
{
  int previous = std::atomic_fetch_add(&mReferenceCount, 1);
  if(0 == previous)
    mReferenceSkeleton = mSkeleton.lock();
}

//==============================================================================
void SkeletonRefCountingBase::decrementReferenceCount() const
{
  int previous = std::atomic_fetch_sub(&mReferenceCount, 1);
  if(1 == previous)
    mReferenceSkeleton = nullptr;
}

//==============================================================================
SkeletonPtr SkeletonRefCountingBase::getSkeleton()
{
  return mSkeleton.lock();
}

//==============================================================================
ConstSkeletonPtr SkeletonRefCountingBase::getSkeleton() const
{
  return mSkeleton.lock();
}

/// SKEL_SET_FLAGS : Lock a Skeleton pointer and activate dirty flags of X for
/// the tree that this BodyNode belongs to, as well as the flag for the Skeleton
/// overall
#define SKEL_SET_FLAGS( X ) { SkeletonPtr skel = getSkeleton(); if(skel) {      \
                            skel->mTreeCache[mTreeIndex].mDirty. X = true;      \
                            skel->mSkelCache.mDirty. X = true; } }

/// SET_FLAGS : A version of SKEL_SET_FLAGS that assumes a SkeletonPtr named
/// 'skel' has already been locked
#define SET_FLAGS( X ) skel->mTreeCache[mTreeIndex].mDirty. X = true;           \
                       skel->mSkelCache.mDirty. X = true;

/// CHECK_FLAG : Check if the dirty flag X for the tree of this BodyNode is
/// active
#define CHECK_FLAG( X ) skel->mTreeCache[mTreeIndex].mDirty. X

//==============================================================================
typedef std::set<Entity*> EntityPtrSet;

//==============================================================================
size_t BodyNode::msBodyNodeCount = 0;

namespace detail {

//==============================================================================
BodyNodeUniqueProperties::BodyNodeUniqueProperties(
    const Inertia& _inertia,
    bool _isCollidable, double _frictionCoeff,
    double _restitutionCoeff, bool _gravityMode)
  : mInertia(_inertia),
    mIsCollidable(_isCollidable),
    mFrictionCoeff(_frictionCoeff),
    mRestitutionCoeff(_restitutionCoeff),
    mGravityMode(_gravityMode)
{
  // Do nothing
}

//==============================================================================
BodyNodeProperties::BodyNodeProperties(
    const Entity::Properties& _entityProperties,
    const BodyNodeUniqueProperties& _bodyNodeProperties)
  : Entity::Properties(_entityProperties),
    BodyNodeUniqueProperties(_bodyNodeProperties)
{
  // Do nothing
}

//==============================================================================
BodyNodeExtendedProperties::BodyNodeExtendedProperties(
    const BodyNodeProperties& standardProperties,
    const NodeProperties& nodeProperties,
    const AddonProperties& addonProperties)
  : BodyNodeProperties(standardProperties),
    mNodeProperties(nodeProperties),
    mAddonProperties(addonProperties)
{
  // Do nothing
}

//==============================================================================
BodyNodeExtendedProperties::BodyNodeExtendedProperties(
    BodyNodeProperties&& standardProperties,
    NodeProperties&& nodeProperties,
    AddonProperties&& addonProperties)
  : BodyNodeProperties(std::move(standardProperties))
{
  mNodeProperties = std::move(nodeProperties);
  mAddonProperties = std::move(addonProperties);
}

} // namespace detail

//==============================================================================
BodyNode::~BodyNode()
{
  // Delete all Nodes
  mNodeMap.clear();

  // Release markers
  for (std::vector<Marker*>::const_iterator it = mMarkers.begin();
       it != mMarkers.end(); ++it)
  {
    delete (*it);
  }

  delete mParentJoint;
}

//==============================================================================
void BodyNode::setProperties(const ExtendedProperties& _properties)
{
  setProperties(static_cast<const Properties&>(_properties));

  setProperties(_properties.mNodeProperties);

  setProperties(_properties.mAddonProperties);
}

//==============================================================================
void BodyNode::setProperties(const NodeProperties& _properties)
{
  const NodePropertiesMap& propertiesMap = _properties.getMap();

  NodeMap::iterator node_it = mNodeMap.begin();
  NodePropertiesMap::const_iterator prop_it = propertiesMap.begin();

  while( mNodeMap.end() != node_it && propertiesMap.end() != prop_it )
  {
    if( node_it->first == prop_it->first )
    {
      if( prop_it->second )
      {
        const std::vector<Node*>& node_vec = node_it->second;
        const std::vector< std::unique_ptr<Node::Properties> >& prop_vec =
            prop_it->second->getVector();
        size_t stop = std::min(node_vec.size(), prop_vec.size());
        for(size_t i=0; i < stop; ++i)
        {
          if(prop_vec[i])
            node_vec[i]->setNodeProperties(*prop_vec[i]);
        }
      }

      ++node_it;
      ++prop_it;
    }
    else if( node_it->first < prop_it->first )
    {
      ++node_it;
    }
    else
    {
      ++prop_it;
    }
  }
}

//==============================================================================
void BodyNode::setProperties(const AddonProperties& _properties)
{
  setAddonProperties(_properties);
}

//==============================================================================
void BodyNode::setProperties(const Properties& _properties)
{
  Entity::setProperties(static_cast<const Entity::Properties&>(_properties));
  setProperties(static_cast<const UniqueProperties&>(_properties));
}

//==============================================================================
void BodyNode::setProperties(const UniqueProperties& _properties)
{
  setInertia(_properties.mInertia);
  setGravityMode(_properties.mGravityMode);
  setFrictionCoeff(_properties.mFrictionCoeff);
  setRestitutionCoeff(_properties.mRestitutionCoeff);

  mBodyP.mMarkerProperties = _properties.mMarkerProperties;
  // Remove current markers
  for(Marker* marker : mMarkers)
    delete marker;

  // Create new markers
  mMarkers.clear();
  for(const Marker::Properties& marker : mBodyP.mMarkerProperties)
    addMarker(new Marker(marker, this));
}

//==============================================================================
BodyNode::Properties BodyNode::getBodyNodeProperties() const
{
  return BodyNode::Properties(mEntityP, mBodyP);
}

//==============================================================================
BodyNode::NodeProperties BodyNode::getAttachedNodeProperties() const
{
  // TODO(MXG): Make a version of this function that will fill in a
  // NodeProperties instance instead of creating a new one
  NodePropertiesMap nodeProperties;

  for(const auto& entry : mNodeMap)
  {
    const std::vector<Node*>& nodes = entry.second;
    std::vector< std::unique_ptr<Node::Properties> > vec;
    for(size_t i=0; i < nodes.size(); ++i)
    {
      std::unique_ptr<Node::Properties> prop = nodes[i]->getNodeProperties();
      if(prop)
        vec.push_back(std::move(prop));
    }

    nodeProperties[entry.first] = common::make_unique<NodePropertiesVector>(
        std::move(vec));
  }

  return nodeProperties;
}

//==============================================================================
BodyNode::ExtendedProperties BodyNode::getExtendedProperties() const
{
  return ExtendedProperties(getBodyNodeProperties(),
                            getAttachedNodeProperties(),
                            getAddonProperties());
}

//==============================================================================
void BodyNode::copy(const BodyNode& _otherBodyNode)
{
  if(this == &_otherBodyNode)
    return;

  setProperties(_otherBodyNode.getExtendedProperties());
}

//==============================================================================
void BodyNode::copy(const BodyNode* _otherBodyNode)
{
  if(nullptr == _otherBodyNode)
    return;

  copy(*_otherBodyNode);
}

//==============================================================================
BodyNode& BodyNode::operator=(const BodyNode& _otherBodyNode)
{
  copy(_otherBodyNode);
  return *this;
}

//==============================================================================
void BodyNode::duplicateNodes(const BodyNode* otherBodyNode)
{
  if(nullptr == otherBodyNode)
  {
    dterr << "[BodyNode::duplicateNodes] You have asked to duplicate the Nodes "
          << "of a nullptr, which is not allowed!\n";
    assert(false);
    return;
  }

  const NodeMap& otherMap = otherBodyNode->mNodeMap;
  for(const auto& vec : otherMap)
  {
    for(const auto& node : vec.second)
      node->cloneNode(this)->attach();
  }
}

//==============================================================================
void BodyNode::matchNodes(const BodyNode* otherBodyNode)
{
  if(nullptr == otherBodyNode)
  {
    dterr << "[BodyNode::matchNodes] You have asked to match the Nodes of a "
          << "nullptr, which is not allowed!\n";
    assert(false);
    return;
  }

  for(auto& cleaner : mNodeDestructors)
    cleaner->getNode()->stageForRemoval();

  duplicateNodes(otherBodyNode);
}

//==============================================================================
const std::string& BodyNode::setName(const std::string& _name)
{
  // If it already has the requested name, do nothing
  if(mEntityP.mName == _name)
    return mEntityP.mName;

  // If the BodyNode belongs to a Skeleton, consult the Skeleton's NameManager
  const SkeletonPtr& skel = getSkeleton();
  if(skel)
  {
    skel->mNameMgrForBodyNodes.removeName(mEntityP.mName);
    SoftBodyNode* softnode = dynamic_cast<SoftBodyNode*>(this);
    if(softnode)
      skel->mNameMgrForSoftBodyNodes.removeName(mEntityP.mName);

    mEntityP.mName = _name;
    skel->addEntryToBodyNodeNameMgr(this);

    if(softnode)
      skel->addEntryToSoftBodyNodeNameMgr(softnode);
  }
  else
  {
    mEntityP.mName = _name;
  }

  // Return the final name (which might have been altered by the Skeleton's
  // NameManager)
  return mEntityP.mName;
}

//==============================================================================
void BodyNode::setGravityMode(bool _gravityMode)
{
  if (mBodyP.mGravityMode == _gravityMode)
    return;

  mBodyP.mGravityMode = _gravityMode;

  SKEL_SET_FLAGS(mGravityForces);
  SKEL_SET_FLAGS(mCoriolisAndGravityForces);
}

//==============================================================================
bool BodyNode::getGravityMode() const
{
  return mBodyP.mGravityMode;
}

//==============================================================================
bool BodyNode::isCollidable() const
{
  return mBodyP.mIsCollidable;
}

//==============================================================================
void BodyNode::setCollidable(bool _isCollidable)
{
  mBodyP.mIsCollidable = _isCollidable;
}

//==============================================================================
void BodyNode::setMass(double _mass)
{
  assert(_mass >= 0.0 && "Negative mass is not allowable.");

  mBodyP.mInertia.setMass(_mass);

  notifyArticulatedInertiaUpdate();
  const SkeletonPtr& skel = getSkeleton();
  if(skel)
    skel->updateTotalMass();
}

//==============================================================================
double BodyNode::getMass() const
{
  return mBodyP.mInertia.getMass();
}

//==============================================================================
void BodyNode::setMomentOfInertia(double _Ixx, double _Iyy, double _Izz,
                                  double _Ixy, double _Ixz, double _Iyz)
{
  mBodyP.mInertia.setMoment(_Ixx, _Iyy, _Izz,
                          _Ixy, _Ixz, _Iyz);

  notifyArticulatedInertiaUpdate();
}

//==============================================================================
void BodyNode::getMomentOfInertia(
    double& _Ixx, double& _Iyy, double& _Izz,
    double& _Ixy, double& _Ixz, double& _Iyz) const
{
  _Ixx = mBodyP.mInertia.getParameter(Inertia::I_XX);
  _Iyy = mBodyP.mInertia.getParameter(Inertia::I_YY);
  _Izz = mBodyP.mInertia.getParameter(Inertia::I_ZZ);

  _Ixy = mBodyP.mInertia.getParameter(Inertia::I_XY);
  _Ixz = mBodyP.mInertia.getParameter(Inertia::I_XZ);
  _Iyz = mBodyP.mInertia.getParameter(Inertia::I_YZ);
}

//==============================================================================
const Eigen::Matrix6d& BodyNode::getSpatialInertia() const
{
  return mBodyP.mInertia.getSpatialTensor();
}

//==============================================================================
void BodyNode::setInertia(const Inertia& _inertia)
{
  mBodyP.mInertia = _inertia;

  notifyArticulatedInertiaUpdate();
  const SkeletonPtr& skel = getSkeleton();
  if(skel)
    skel->updateTotalMass();
}

//==============================================================================
const Inertia& BodyNode::getInertia() const
{
  return mBodyP.mInertia;
}

//==============================================================================
const math::Inertia& BodyNode::getArticulatedInertia() const
{
  const ConstSkeletonPtr& skel = getSkeleton();
  if( skel && CHECK_FLAG(mArticulatedInertia) )
    skel->updateArticulatedInertia(mTreeIndex);

  return mArtInertia;
}

//==============================================================================
const math::Inertia& BodyNode::getArticulatedInertiaImplicit() const
{
  const ConstSkeletonPtr& skel = getSkeleton();
  if( skel && CHECK_FLAG(mArticulatedInertia) )
    skel->updateArticulatedInertia(mTreeIndex);

  return mArtInertiaImplicit;
}

//==============================================================================
void BodyNode::setLocalCOM(const Eigen::Vector3d& _com)
{
  mBodyP.mInertia.setLocalCOM(_com);

  notifyArticulatedInertiaUpdate();
}

//==============================================================================
const Eigen::Vector3d& BodyNode::getLocalCOM() const
{
  return mBodyP.mInertia.getLocalCOM();
}

//==============================================================================
Eigen::Vector3d BodyNode::getCOM(const Frame* _withRespectTo) const
{
  return getTransform(_withRespectTo) * getLocalCOM();
}

//==============================================================================
Eigen::Vector3d BodyNode::getCOMLinearVelocity(const Frame* _relativeTo,
                                            const Frame* _inCoordinatesOf) const
{
  return getLinearVelocity(getLocalCOM(), _relativeTo, _inCoordinatesOf);
}

//==============================================================================
Eigen::Vector6d BodyNode::getCOMSpatialVelocity() const
{
  return getSpatialVelocity(getLocalCOM());
}

//==============================================================================
Eigen::Vector6d BodyNode::getCOMSpatialVelocity(const Frame* _relativeTo,
                                            const Frame* _inCoordinatesOf) const
{
  return getSpatialVelocity(getLocalCOM(), _relativeTo, _inCoordinatesOf);
}

//==============================================================================
Eigen::Vector3d BodyNode::getCOMLinearAcceleration(const Frame* _relativeTo,
                                            const Frame* _inCoordinatesOf) const
{
  return getLinearAcceleration(getLocalCOM(), _relativeTo, _inCoordinatesOf);
}

//==============================================================================
Eigen::Vector6d BodyNode::getCOMSpatialAcceleration() const
{
  return getSpatialAcceleration(getLocalCOM());
}

//==============================================================================
Eigen::Vector6d BodyNode::getCOMSpatialAcceleration(const Frame* _relativeTo,
                                            const Frame* _inCoordinatesOf) const
{
  return getSpatialAcceleration(getLocalCOM(), _relativeTo, _inCoordinatesOf);
}

//==============================================================================
void BodyNode::setFrictionCoeff(double _coeff)
{
  assert(0.0 <= _coeff
         && "Coefficient of friction should be non-negative value.");
  mBodyP.mFrictionCoeff = _coeff;
}

//==============================================================================
double BodyNode::getFrictionCoeff() const
{
  return mBodyP.mFrictionCoeff;
}

//==============================================================================
void BodyNode::setRestitutionCoeff(double _coeff)
{
  assert(0.0 <= _coeff && _coeff <= 1.0
         && "Coefficient of restitution should be in range of [0, 1].");
  mBodyP.mRestitutionCoeff = _coeff;
}

//==============================================================================
double BodyNode::getRestitutionCoeff() const
{
  return mBodyP.mRestitutionCoeff;
}

//==============================================================================
size_t BodyNode::getIndexInSkeleton() const
{
  return mIndexInSkeleton;
}

//==============================================================================
size_t BodyNode::getIndexInTree() const
{
  return mIndexInTree;
}

//==============================================================================
size_t BodyNode::getTreeIndex() const
{
  return mTreeIndex;
}

//==============================================================================
static bool checkSkeletonNodeAgreement(
    const BodyNode* _bodyNode,
    const ConstSkeletonPtr& _newSkeleton, const BodyNode* _newParent,
    const std::string& _function,
    const std::string& _operation)
{
  if(nullptr == _newSkeleton)
  {
    dterr << "[BodyNode::" << _function << "] Attempting to " << _operation
          << " a BodyNode tree starting " << "from [" << _bodyNode->getName()
          << "] in the Skeleton named [" << _bodyNode->getSkeleton()->getName()
          << "] into a nullptr Skeleton.\n";
    return false;
  }

  if(_newParent && _newSkeleton != _newParent->getSkeleton())
  {
    dterr << "[BodyNode::" << _function << "] Mismatch between the specified "
          << "Skeleton [" << _newSkeleton->getName() << "] (" << _newSkeleton
          << ") and the specified new parent BodyNode ["
          << _newParent->getName() << "] whose actual Skeleton is named ["
          << _newParent->getSkeleton()->getName() <<  "] ("
          << _newParent->getSkeleton() << ") while attempting to " << _operation
          << " the BodyNode [" << _bodyNode->getName() << "] from the "
          << "Skeleton named [" << _bodyNode->getSkeleton()->getName() << "] ("
          << _bodyNode->getSkeleton() << ").\n";
    return false;
  }

  return true;
}

//==============================================================================
SkeletonPtr BodyNode::remove(const std::string& _name)
{
  return split(_name);
}

//==============================================================================
bool BodyNode::moveTo(BodyNode* _newParent)
{
  if(nullptr == _newParent)
    return getSkeleton()->moveBodyNodeTree(
          getParentJoint(), this, getSkeleton(), nullptr);
  else
    return getSkeleton()->moveBodyNodeTree(
          getParentJoint(), this, _newParent->getSkeleton(), _newParent);
}

//==============================================================================
bool BodyNode::moveTo(const SkeletonPtr& _newSkeleton, BodyNode* _newParent)
{
  if(checkSkeletonNodeAgreement(
       this, _newSkeleton, _newParent, "moveTo", "move"))
  {
    return getSkeleton()->moveBodyNodeTree(
          getParentJoint(), this, _newSkeleton, _newParent);
  }

  return false;
}

//==============================================================================
SkeletonPtr BodyNode::split(const std::string& _skeletonName)
{
  const SkeletonPtr& skel =
      Skeleton::create(getSkeleton()->getSkeletonProperties());
  skel->setName(_skeletonName);
  moveTo(skel, nullptr);
  return skel;
}

//==============================================================================
std::pair<Joint*, BodyNode*> BodyNode::copyTo(BodyNode* _newParent,
                                              bool _recursive)
{
  if(nullptr == _newParent)
    return getSkeleton()->cloneBodyNodeTree(
          nullptr, this, getSkeleton(), nullptr, _recursive);
  else
    return getSkeleton()->cloneBodyNodeTree(
          nullptr, this, _newParent->getSkeleton(), _newParent, _recursive);
}

//==============================================================================
std::pair<Joint*, BodyNode*> BodyNode::copyTo(const SkeletonPtr& _newSkeleton,
                                              BodyNode* _newParent,
                                              bool _recursive) const
{
  if(checkSkeletonNodeAgreement(
       this, _newSkeleton, _newParent, "copyTo", "copy"))
  {
    return getSkeleton()->cloneBodyNodeTree(
          nullptr, this, _newSkeleton, _newParent, _recursive);
  }

  return std::pair<Joint*, BodyNode*>(nullptr, nullptr);
}

//==============================================================================
SkeletonPtr BodyNode::copyAs(const std::string& _skeletonName,
                             bool _recursive) const
{
  const SkeletonPtr& skel =
      Skeleton::create(getSkeleton()->getSkeletonProperties());
  skel->setName(_skeletonName);
  copyTo(skel, nullptr, _recursive);
  return skel;
}

//==============================================================================
SkeletonPtr BodyNode::getSkeleton()
{
  return mSkeleton.lock();
}

//==============================================================================
ConstSkeletonPtr BodyNode::getSkeleton() const
{
  return mSkeleton.lock();
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
  assert(_body != nullptr);

  if(std::find(mChildBodyNodes.begin(), mChildBodyNodes.end(), _body) !=
     mChildBodyNodes.end())
  {
    dtwarn << "[BodyNode::addChildBodyNode] Attempting to add a BodyNode '"
           << _body->getName() << "' as a child BodyNode of '" << getName()
           << "', which is already its parent." << std::endl;
    return;
  }

  mChildBodyNodes.push_back(_body);
  _body->mParentBodyNode = this;
  _body->changeParentFrame(this);
}

//==============================================================================
size_t BodyNode::getNumChildBodyNodes() const
{
  return mChildBodyNodes.size();
}

//==============================================================================
BodyNode* BodyNode::getChildBodyNode(size_t _index)
{
  return common::getVectorObjectIfAvailable<BodyNode*>(_index, mChildBodyNodes);
}

//==============================================================================
const BodyNode* BodyNode::getChildBodyNode(size_t _index) const
{
  return common::getVectorObjectIfAvailable<BodyNode*>(_index, mChildBodyNodes);
}

//==============================================================================
size_t BodyNode::getNumChildJoints() const
{
  return mChildBodyNodes.size();
}

//==============================================================================
Joint* BodyNode::getChildJoint(size_t _index)
{
  BodyNode* childBodyNode = getChildBodyNode(_index);

  if(childBodyNode)
    return childBodyNode->getParentJoint();
  else
    return nullptr;
}

//==============================================================================
const Joint* BodyNode::getChildJoint(size_t _index) const
{
  return const_cast<BodyNode*>(this)->getChildJoint(_index);
}

//==============================================================================
ShapeNode* BodyNode::createShapeNode(const ShapePtr& shape)
{
  ShapeNode::Properties properties;
  properties.mShape = shape;

  return createShapeNode(properties, true);
}

//==============================================================================
ShapeNode* BodyNode::createShapeNode(const ShapePtr& shape,
                                     const std::string& name)
{
  ShapeNode::Properties properties;
  properties.mShape = shape;
  properties.mName = name;

  return createShapeNode(properties, false);
}

//==============================================================================
ShapeNode* BodyNode::createShapeNode(const ShapePtr& shape, const char* name)
{
  return createShapeNode(shape, std::string(name));
}

//==============================================================================
size_t BodyNode::getNumShapeNodes() const
{
  return getNumNodes<ShapeNode>();
}

//==============================================================================
ShapeNode* BodyNode::getShapeNode(size_t index)
{
  return getNode<ShapeNode>(index);
}

//==============================================================================
const ShapeNode* BodyNode::getShapeNode(size_t index) const
{
  return getNode<ShapeNode>(index);
}

//==============================================================================
const std::vector<ShapeNode*> BodyNode::getShapeNodes()
{
  const auto numShapeNodes = getNumShapeNodes();

  std::vector<ShapeNode*> shapeNodes(numShapeNodes);

  for (auto i = 0u; i < numShapeNodes; ++i)
    shapeNodes[i] = getShapeNode(i);

  return shapeNodes;
}

//==============================================================================
const std::vector<const ShapeNode*> BodyNode::getShapeNodes() const
{
  const auto numShapeNodes = getNumShapeNodes();

  std::vector<const ShapeNode*> shapeNodes(numShapeNodes);

  for (auto i = 0u; i < numShapeNodes; ++i)
    shapeNodes[i] = getShapeNode(i);

  return shapeNodes;
}

//==============================================================================
void BodyNode::removeAllShapeNodes()
{
  auto shapeNodes = getShapeNodes();
  for (auto shapeNode : shapeNodes)
    shapeNode->remove();
}

//==============================================================================
EndEffector* BodyNode::createEndEffector(const std::string& _name)
{
  EndEffector::Properties properties;
  properties.mName = _name;

  return createNode<EndEffector>(properties);
}

//==============================================================================
EndEffector* BodyNode::createEndEffector(const char* _name)
{
  return createEndEffector(std::string(_name));
}

//==============================================================================
void BodyNode::addMarker(Marker* _marker)
{
  mMarkers.push_back(_marker);
  const SkeletonPtr& skel = getSkeleton();
  if(skel)
    skel->addEntryToMarkerNameMgr(_marker);
}

//==============================================================================
size_t BodyNode::getNumMarkers() const
{
  return mMarkers.size();
}

//==============================================================================
Marker* BodyNode::getMarker(size_t _index)
{
  return common::getVectorObjectIfAvailable<Marker*>(_index, mMarkers);
}

//==============================================================================
const Marker* BodyNode::getMarker(size_t _index) const
{
  return common::getVectorObjectIfAvailable<Marker*>(_index, mMarkers);
}

//==============================================================================
bool BodyNode::dependsOn(size_t _genCoordIndex) const
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
  assert(_arrayIndex < mDependentGenCoordIndices.size());

  return mDependentGenCoordIndices[_arrayIndex];
}

//==============================================================================
const std::vector<size_t>& BodyNode::getDependentGenCoordIndices() const
{
  return mDependentGenCoordIndices;
}

//==============================================================================
size_t BodyNode::getNumDependentDofs() const
{
  return mDependentDofs.size();
}

//==============================================================================
DegreeOfFreedom* BodyNode::getDependentDof(size_t _index)
{
  return common::getVectorObjectIfAvailable<DegreeOfFreedom*>(
        _index, mDependentDofs);
}

//==============================================================================
const DegreeOfFreedom* BodyNode::getDependentDof(size_t _index) const
{
  return common::getVectorObjectIfAvailable<DegreeOfFreedom*>(
        _index, mDependentDofs);
}

//==============================================================================
const std::vector<DegreeOfFreedom*>& BodyNode::getDependentDofs()
{
  return mDependentDofs;
}

//==============================================================================
const std::vector<const DegreeOfFreedom*>& BodyNode::getDependentDofs() const
{
  return mConstDependentDofs;
}

//==============================================================================
const std::vector<const DegreeOfFreedom*> BodyNode::getChainDofs() const
{
  // TODO(MXG): Consider templating the Criteria for const BodyNodes so that we
  // don't need a const_cast here. That said, the const_cast isn't hurting
  // anything, because the Criteria function would work just as well operating
  // on const BodyNodes.
  Chain::Criteria criteria(const_cast<BodyNode*>(this), nullptr);
  std::vector<BodyNode*> bn_chain = criteria.satisfy();
  std::vector<const DegreeOfFreedom*> dofs;
  dofs.reserve(getNumDependentGenCoords());
  for(std::vector<BodyNode*>::reverse_iterator rit = bn_chain.rbegin();
      rit != bn_chain.rend(); ++rit)
  {
    size_t nDofs = (*rit)->getParentJoint()->getNumDofs();
    for(size_t i=0; i<nDofs; ++i)
      dofs.push_back( (*rit)->getParentJoint()->getDof(i) );
  }

  return dofs;
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
    updateBodyJacobian();

  return mBodyJacobian;
}

//==============================================================================
const math::Jacobian& BodyNode::getWorldJacobian() const
{
  if(mIsWorldJacobianDirty)
    updateWorldJacobian();

  return mWorldJacobian;
}

//==============================================================================
const math::Jacobian& BodyNode::getJacobianSpatialDeriv() const
{
  if(mIsBodyJacobianSpatialDerivDirty)
    updateBodyJacobianSpatialDeriv();

  return mBodyJacobianSpatialDeriv;
}

//==============================================================================
const math::Jacobian& BodyNode::getJacobianClassicDeriv() const
{
  if(mIsWorldJacobianClassicDerivDirty)
    updateWorldJacobianClassicDeriv();

  return mWorldJacobianClassicDeriv;
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

  SKEL_SET_FLAGS(mExternalForces);
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

  SKEL_SET_FLAGS(mExternalForces);
}

//==============================================================================
void BodyNode::addExtTorque(const Eigen::Vector3d& _torque, bool _isLocal)
{
  if (_isLocal)
    mFext.head<3>() += _torque;
  else
    mFext.head<3>() += getWorldTransform().linear().transpose() * _torque;

  SKEL_SET_FLAGS(mExternalForces);
}

//==============================================================================
void BodyNode::setExtTorque(const Eigen::Vector3d& _torque, bool _isLocal)
{
  if (_isLocal)
    mFext.head<3>() = _torque;
  else
    mFext.head<3>() = getWorldTransform().linear().transpose() * _torque;

  SKEL_SET_FLAGS(mExternalForces);
}

//==============================================================================
BodyNode::BodyNode(BodyNode* _parentBodyNode, Joint* _parentJoint,
                   const Properties& _properties)
  : Entity(ConstructFrame),
    Frame(Frame::World(), ""), // Name gets set later by setProperties
    TemplatedJacobianNode<BodyNode>(this),
    mID(BodyNode::msBodyNodeCount++),
    mIsColliding(false),
    mParentJoint(_parentJoint),
    mParentBodyNode(nullptr),
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
    mImpF(Eigen::Vector6d::Zero()),
    onColShapeAdded(mColShapeAddedSignal),
    onColShapeRemoved(mColShapeRemovedSignal),
    onStructuralChange(mStructuralChangeSignal)
{
  // Generate an inert destructor to make sure that it will not try to
  // double-delete this BodyNode when it gets destroyed.
  mSelfDestructor = std::shared_ptr<NodeDestructor>(new NodeDestructor(nullptr));
  mDestructor = mSelfDestructor;
  mAmAttached = true;

  mParentJoint->mChildBodyNode = this;
  setProperties(_properties);

  if(_parentBodyNode)
    _parentBodyNode->addChildBodyNode(this);
}

//==============================================================================
BodyNode* BodyNode::clone(BodyNode* _parentBodyNode, Joint* _parentJoint,
                          bool cloneNodes) const
{
  BodyNode* clonedBn =
      new BodyNode(_parentBodyNode, _parentJoint, getBodyNodeProperties());

  clonedBn->matchAddons(this);

  if(cloneNodes)
    clonedBn->matchNodes(this);

  return clonedBn;
}

//==============================================================================
Node* BodyNode::cloneNode(BodyNode* /*bn*/) const
{
  dterr << "[BodyNode::cloneNode] This function should never be called! Please "
        << "report this as an error!\n";
  assert(false);
  return nullptr;
}

//==============================================================================
void BodyNode::init(const SkeletonPtr& _skeleton)
{
  mSkeleton = _skeleton;
  assert(_skeleton);
  if(mReferenceCount > 0)
  {
    mReferenceSkeleton = mSkeleton.lock();
  }

  // Put the scope around this so that 'lock' releases the mutex immediately
  // after we're done with it
  {
    std::lock_guard<std::mutex> lock(mLockedSkeleton->mMutex);
    mLockedSkeleton->mSkeleton = mSkeleton;
  }

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

  mDependentDofs.clear();
  mDependentDofs.reserve(mDependentGenCoordIndices.size());
  mConstDependentDofs.clear();
  mConstDependentDofs.reserve(mDependentGenCoordIndices.size());
  for(const size_t& index : mDependentGenCoordIndices)
  {
    mDependentDofs.push_back(_skeleton->getDof(index));
    mConstDependentDofs.push_back(_skeleton->getDof(index));
  }

#ifndef NDEBUG
  // Check whether there is duplicated indices.
  size_t nDepGenCoordIndices = mDependentGenCoordIndices.size();
  for (size_t i = 0; i < nDepGenCoordIndices; ++i)
  {
    for (size_t j = i + 1; j < nDepGenCoordIndices; ++j)
    {
      assert(mDependentGenCoordIndices[i] !=
             mDependentGenCoordIndices[j] &&
             "Duplicated index is found in mDependentGenCoordIndices.");
    }
  }
#endif // NDEBUG

  //--------------------------------------------------------------------------
  // Set dimensions of dynamics matrices and vectors.
  //--------------------------------------------------------------------------
  size_t numDepGenCoords = getNumDependentGenCoords();
  mBodyJacobian.setZero(6, numDepGenCoords);
  mWorldJacobian.setZero(6, numDepGenCoords);
  mBodyJacobianSpatialDeriv.setZero(6, numDepGenCoords);
  mWorldJacobianClassicDeriv.setZero(6, numDepGenCoords);
  notifyTransformUpdate();
}

//==============================================================================
void BodyNode::processNewEntity(Entity* _newChildEntity)
{
  // If the Entity is a JacobianNode, add it to the list of JacobianNodes
  if(JacobianNode* node = dynamic_cast<JacobianNode*>(_newChildEntity))
    mChildJacobianNodes.insert(node);

  // Here we want to sort out whether the Entity that has been added is a child
  // BodyNode or not

  // Check if it's a child BodyNode (if not, then it's just some other arbitrary
  // type of Entity)
  if(find(mChildBodyNodes.begin(), mChildBodyNodes.end(), _newChildEntity) !=
     mChildBodyNodes.end())
    return;

  // Check if it's already accounted for in our Non-BodyNode Entities
  if(mNonBodyNodeEntities.find(_newChildEntity) != mNonBodyNodeEntities.end())
  {
    dtwarn << "[BodyNode::processNewEntity] Attempting to add an Entity ["
           << _newChildEntity->getName() << "] as a child Entity of ["
           << getName() << "], which is already its parent." << std::endl;
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

  if(JacobianNode* node = dynamic_cast<JacobianNode*>(_oldChildEntity))
  {
    std::unordered_set<JacobianNode*>::iterator node_it =
        mChildJacobianNodes.find(node);

    if(node_it != mChildJacobianNodes.end())
      mChildJacobianNodes.erase(node_it);
  }

  if(find(mNonBodyNodeEntities.begin(), mNonBodyNodeEntities.end(),
          _oldChildEntity) != mNonBodyNodeEntities.end())
    mNonBodyNodeEntities.erase(_oldChildEntity);
}

//==============================================================================
void BodyNode::draw(renderer::RenderInterface* ri,
                    const Eigen::Vector4d& color,
                    bool useDefaultColor, int /*depth*/) const
{
  if (nullptr == ri)
    return;

  ri->pushMatrix();

  // Use the relative transform of this Frame. We assume that we are being
  // called from the parent Frame's renderer.
  // TODO(MXG): This can cause trouble if the draw function is originally called
  // on an Entity or Frame which is not a child of the World Frame
  ri->transform(getRelativeTransform());

  // _ri->pushName(???); TODO(MXG): What should we do about this for Frames?
  auto shapeNodes = getShapeNodesWith<VisualAddon>();
  for (auto shapeNode : shapeNodes)
    shapeNode->draw(ri, color, useDefaultColor);
  // _ri.popName();

  // render the subtree
  for(Entity* entity : mChildEntities)
    entity->draw(ri, color, useDefaultColor);

  ri->popMatrix();
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
  for (size_t i = 0; i < mMarkers.size(); i++)
    mMarkers[i]->draw(_ri, true, _color, _useDefaultColor);

  for (size_t i = 0; i < mChildBodyNodes.size(); i++)
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

  const SkeletonPtr& skel = getSkeleton();
  if(skel)
  {
    // All of these depend on the world transform of this BodyNode, so they must
    // be dirtied whenever mNeedTransformUpdate is dirtied, and if
    // mTransformUpdate is already dirty, then these must already be dirty as
    // well
    SET_FLAGS(mCoriolisForces);
    SET_FLAGS(mGravityForces);
    SET_FLAGS(mCoriolisAndGravityForces);
    SET_FLAGS(mExternalForces);
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
  mIsPartialAccelerationDirty = true;

  const SkeletonPtr& skel = getSkeleton();
  if(skel)
  {
    SET_FLAGS(mCoriolisForces);
    SET_FLAGS(mCoriolisAndGravityForces);
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
void BodyNode::notifyArticulatedInertiaUpdate()
{
  const SkeletonPtr& skel = getSkeleton();
  if(skel)
    skel->notifyArticulatedInertiaUpdate(mTreeIndex);
}

//==============================================================================
void BodyNode::notifyExternalForcesUpdate()
{
  SKEL_SET_FLAGS(mExternalForces);
}

//==============================================================================
void BodyNode::notifyCoriolisUpdate()
{
  SKEL_SET_FLAGS(mCoriolisForces);
  SKEL_SET_FLAGS(mCoriolisAndGravityForces);
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
void BodyNode::updateAccelerationID()
{
  // Note: auto-updating has replaced this function
  getSpatialAcceleration();
  // Verification
  assert(!math::isNan(mAcceleration));
}

//==============================================================================
void BodyNode::updateTransmittedForceID(const Eigen::Vector3d& _gravity,
                                        bool _withExternalForces)
{
  // Gravity force
  const Eigen::Matrix6d& mI = mBodyP.mInertia.getSpatialTensor();
  if (mBodyP.mGravityMode == true)
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
    assert(childJoint != nullptr);

    mF += math::dAdInvT(childJoint->getLocalTransform(),
                        childBodyNode->getBodyForce());
  }

  // Verification
  assert(!math::isNan(mF));
}

//==============================================================================
void BodyNode::updateArtInertia(double _timeStep) const
{
  // Set spatial inertia to the articulated body inertia
  const Eigen::Matrix6d& mI = mBodyP.mInertia.getSpatialTensor();
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
  const Eigen::Matrix6d& mI = mBodyP.mInertia.getSpatialTensor();
  if (mBodyP.mGravityMode == true)
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
void BodyNode::updateTransmittedForceFD()
{
  mF = mBiasForce;
  mF.noalias() += getArticulatedInertiaImplicit() * getSpatialAcceleration();

  assert(!math::isNan(mF));
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
                                  bool _withDampingForces,
                                  bool _withSpringForces)
{
  assert(mParentJoint != nullptr);
  mParentJoint->updateForceID(mF, _timeStep,
                              _withDampingForces, _withSpringForces);
}

//==============================================================================
void BodyNode::updateJointForceFD(double _timeStep,
                                  bool _withDampingForces,
                                  bool _withSpringForces)
{
  assert(mParentJoint != nullptr);
  mParentJoint->updateForceFD(mF, _timeStep,
                              _withDampingForces, _withSpringForces);
}

//==============================================================================
void BodyNode::updateJointImpulseFD()
{
  assert(mParentJoint != nullptr);
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
void BodyNode::clearExternalForces()
{
  mFext.setZero();
  SKEL_SET_FLAGS(mExternalForces);
}

//==============================================================================
void BodyNode::clearInternalForces()
{
  mParentJoint->resetForces();
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
  const Eigen::Matrix6d& mI = mBodyP.mInertia.getSpatialTensor();
  return 0.5 * V.dot(mI * V);
}

//==============================================================================
double BodyNode::getPotentialEnergy(const Eigen::Vector3d& _gravity) const
{
  return -getMass() * getWorldTransform().translation().dot(_gravity);
}

//==============================================================================
Eigen::Vector3d BodyNode::getLinearMomentum() const
{
  const Eigen::Matrix6d& mI = mBodyP.mInertia.getSpatialTensor();
  return (mI * getSpatialVelocity()).tail<3>();
}

//==============================================================================
Eigen::Vector3d BodyNode::getAngularMomentum(const Eigen::Vector3d& _pivot)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  const Eigen::Matrix6d& mI = mBodyP.mInertia.getSpatialTensor();
  T.translation() = _pivot;
  return math::dAdT(T, mI * getSpatialVelocity()).head<3>();
}

//==============================================================================
bool BodyNode::isReactive() const
{
  const ConstSkeletonPtr& skel = getSkeleton();
  if (skel && skel->isMobile() && getNumDependentGenCoords() > 0)
  {
    // Check if all the ancestor joints are motion prescribed.
    const BodyNode* body = this;
    while (body != nullptr)
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
void BodyNode::aggregateCoriolisForceVector(Eigen::VectorXd& _C)
{
  aggregateCombinedVector(_C, Eigen::Vector3d::Zero());
}

//==============================================================================
void BodyNode::aggregateGravityForceVector(Eigen::VectorXd& _g,
                                           const Eigen::Vector3d& _gravity)
{
  const Eigen::Matrix6d& mI = mBodyP.mInertia.getSpatialTensor();
  if (mBodyP.mGravityMode == true)
    mG_F = mI * math::AdInvRLinear(getWorldTransform(), _gravity);
  else
    mG_F.setZero();

  for (std::vector<BodyNode*>::const_iterator it = mChildBodyNodes.begin();
       it != mChildBodyNodes.end(); ++it)
  {
    mG_F += math::dAdInvT((*it)->mParentJoint->getLocalTransform(),
                          (*it)->mG_F);
  }

  size_t nGenCoords = mParentJoint->getNumDofs();
  if (nGenCoords > 0)
  {
    Eigen::VectorXd g = -(mParentJoint->getLocalJacobian().transpose() * mG_F);
    size_t iStart = mParentJoint->getIndexInTree(0);
    _g.segment(iStart, nGenCoords) = g;
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
void BodyNode::aggregateCombinedVector(Eigen::VectorXd& _Cg,
                                       const Eigen::Vector3d& _gravity)
{
  // H(i) = I(i) * W(i) -
  //        dad{V}(I(i) * V(i)) + sum(k \in children) dAd_{T(i,j)^{-1}}(H(k))
  const Eigen::Matrix6d& mI = mBodyP.mInertia.getSpatialTensor();
  if (mBodyP.mGravityMode == true)
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

  size_t nGenCoords = mParentJoint->getNumDofs();
  if (nGenCoords > 0)
  {
    Eigen::VectorXd Cg
        = mParentJoint->getLocalJacobian().transpose() * mCg_F;
    size_t iStart = mParentJoint->getIndexInTree(0);
    _Cg.segment(iStart, nGenCoords) = Cg;
  }
}

//==============================================================================
void BodyNode::aggregateExternalForces(Eigen::VectorXd& _Fext)
{
  mFext_F = mFext;

  for (std::vector<BodyNode*>::const_iterator it = mChildBodyNodes.begin();
       it != mChildBodyNodes.end(); ++it)
  {
    mFext_F += math::dAdInvT((*it)->mParentJoint->getLocalTransform(),
                             (*it)->mFext_F);
  }

  size_t nGenCoords = mParentJoint->getNumDofs();
  if (nGenCoords > 0)
  {
    Eigen::VectorXd Fext = mParentJoint->getLocalJacobian().transpose()*mFext_F;
    size_t iStart = mParentJoint->getIndexInTree(0);
    _Fext.segment(iStart, nGenCoords) = Fext;
  }
}

//==============================================================================
void BodyNode::aggregateSpatialToGeneralized(Eigen::VectorXd& _generalized,
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
  size_t iStart = mParentJoint->getIndexInTree(0);
  _generalized.segment(iStart, mParentJoint->getNumDofs())
      = mParentJoint->getSpatialToGeneralized(mArbitrarySpatial);
}

//==============================================================================
void BodyNode::updateMassMatrix()
{
  mM_dV.setZero();
  size_t dof = mParentJoint->getNumDofs();
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
void BodyNode::aggregateMassMatrix(Eigen::MatrixXd& _MCol, size_t _col)
{
  const Eigen::Matrix6d& mI = mBodyP.mInertia.getSpatialTensor();
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
  size_t dof = mParentJoint->getNumDofs();
  if (dof > 0)
  {
    size_t iStart = mParentJoint->getIndexInTree(0);
    _MCol.block(iStart, _col, dof, 1).noalias() =
        mParentJoint->getLocalJacobian().transpose() * mM_F;
  }
}

//==============================================================================
void BodyNode::aggregateAugMassMatrix(Eigen::MatrixXd& _MCol, size_t _col,
                                      double _timeStep)
{
  // TODO(JS): Need to be reimplemented
  const Eigen::Matrix6d& mI = mBodyP.mInertia.getSpatialTensor();

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
  size_t dof = mParentJoint->getNumDofs();
  if (dof > 0)
  {
    Eigen::MatrixXd K = Eigen::MatrixXd::Zero(dof, dof);
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(dof, dof);
    for (size_t i = 0; i < dof; ++i)
    {
      K(i, i) = mParentJoint->getSpringStiffness(i);
      D(i, i) = mParentJoint->getDampingCoefficient(i);
    }

    size_t iStart = mParentJoint->getIndexInTree(0);

    _MCol.block(iStart, _col, dof, 1).noalias()
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
void BodyNode::aggregateInvMassMatrix(Eigen::MatrixXd& _InvMCol, size_t _col)
{
  if (mParentBodyNode)
  {
    //
    mParentJoint->getInvMassMatrixSegment(
          _InvMCol, _col, getArticulatedInertia(), mParentBodyNode->mInvM_U);

    //
    mInvM_U = math::AdInvT(mParentJoint->getLocalTransform(),
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
}

//==============================================================================
void BodyNode::aggregateInvAugMassMatrix(Eigen::MatrixXd& _InvMCol, size_t _col,
                                         double /*_timeStep*/)
{
  if (mParentBodyNode)
  {
    //
    mParentJoint->getInvAugMassMatrixSegment(
          _InvMCol, _col, getArticulatedInertiaImplicit(),
          mParentBodyNode->mInvM_U);

    //
    mInvM_U = math::AdInvT(mParentJoint->getLocalTransform(),
                           mParentBodyNode->mInvM_U);
  }
  else
  {
    //
    mParentJoint->getInvAugMassMatrixSegment(
          _InvMCol, _col, getArticulatedInertiaImplicit(),
          Eigen::Vector6d::Zero());

    //
    mInvM_U.setZero();
  }

  //
  mParentJoint->addInvMassMatrixSegmentTo(mInvM_U);
}

//==============================================================================
void BodyNode::updateBodyJacobian() const
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

  if(nullptr == mParentJoint)
    return;

  const size_t localDof     = mParentJoint->getNumDofs();
  assert(getNumDependentGenCoords() >= localDof);
  const size_t ascendantDof = getNumDependentGenCoords() - localDof;

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
void BodyNode::updateWorldJacobian() const
{
  mWorldJacobian = math::AdRJac(getWorldTransform(), getJacobian());

  mIsWorldJacobianDirty = false;
}

//==============================================================================
void BodyNode::updateBodyJacobianSpatialDeriv() const
{
  //--------------------------------------------------------------------------
  // Body Jacobian first spatial derivative update
  //
  // dJ = | Ad(T(i, parent(i)), dJ_parent(i))    ad(V(i), S(i)) + dS(i) |
  //
  // T(i, parent(i)): Transformation from this BodyNode to the parent BodyNode
  // dJ             : Spatial Jacobian derivative (6 x dependentDOF)
  // dJ_parent      : Parent Jacobian derivative (6 x (dependentDOF - localDOF))
  // V(i)           : Spatial velocity (6 x 1)
  // S(i)           : Local spatial Jacobian (6 x localDOF)
  // dS(i)          : Local spatial Jacobian deriavative (6 x localDOF)
  // Ad(T(1,2), V)  : Transformation a spatial motion from frame 2 to frame 1
  // ad(V, W)       : Spatial cross product for spatial motions
  //--------------------------------------------------------------------------

  if (nullptr == mParentJoint)
    return;

  const auto numLocalDOFs = mParentJoint->getNumDofs();
  assert(getNumDependentGenCoords() >= numLocalDOFs);
  const auto numParentDOFs = getNumDependentGenCoords() - numLocalDOFs;

  // Parent Jacobian: Ad(T(i, parent(i)), dJ_parent(i))
  if (mParentBodyNode)
  {
    const auto& dJ_parent = mParentBodyNode->getJacobianSpatialDeriv();

    assert(static_cast<size_t>(dJ_parent.cols()) + mParentJoint->getNumDofs()
           == static_cast<size_t>(mBodyJacobianSpatialDeriv.cols()));

    mBodyJacobianSpatialDeriv.leftCols(numParentDOFs)
        = math::AdInvTJac(mParentJoint->getLocalTransform(), dJ_parent);
  }

  // Local Jacobian: ad(V(i), S(i)) + dS(i)
  mBodyJacobianSpatialDeriv.rightCols(numLocalDOFs)
      = math::adJac(getSpatialVelocity(), mParentJoint->getLocalJacobian())
        + mParentJoint->getLocalJacobianTimeDeriv();

  mIsBodyJacobianSpatialDerivDirty = false;
}

//==============================================================================
void BodyNode::updateWorldJacobianClassicDeriv() const
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

  if(nullptr == mParentJoint)
    return;

  const size_t numLocalDOFs = mParentJoint->getNumDofs();
  assert(getNumDependentGenCoords() >= numLocalDOFs);
  const size_t numParentDOFs = getNumDependentGenCoords() - numLocalDOFs;


  if(mParentBodyNode)
  {
    const math::Jacobian& dJ_parent =
        mParentBodyNode->getJacobianClassicDeriv();
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

}  // namespace dynamics
}  // namespace dart
