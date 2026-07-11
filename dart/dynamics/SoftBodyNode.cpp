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

#include "dart/dynamics/SoftBodyNode.hpp"

#include "dart/common/Console.hpp"
#include "dart/common/Macros.hpp"
#include "dart/common/Profile.hpp"
#include "dart/dynamics/Joint.hpp"
#include "dart/dynamics/PointMass.hpp"
#include "dart/dynamics/Shape.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/SoftMeshShape.hpp"
#include "dart/math/Helpers.hpp"

#include <algorithm>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include <cmath>
#include <cstdint>

namespace dart {
namespace dynamics {

namespace detail {

//==============================================================================
SoftBodyNodeUniqueProperties::SoftBodyNodeUniqueProperties(
    double _Kv,
    double _Ke,
    double _DampCoeff,
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
bool SoftBodyNodeUniqueProperties::connectPointMasses(
    std::size_t i1, std::size_t i2)
{
  if (i1 >= mPointProps.size() || i2 >= mPointProps.size()) {
    if (mPointProps.size() == 0)
      dtwarn << "[SoftBodyNode::Properties::addConnection] Attempting to "
             << "add a connection between indices " << i1 << " and " << i2
             << ", but there are currently no entries in mPointProps!\n";
    else
      dtwarn << "[SoftBodyNode::Properties::addConnection] Attempting to "
             << "add a connection between indices " << i1 << " and " << i2
             << ", but the entries in mPointProps only go up to "
             << mPointProps.size() - 1 << "!\n";
    return false;
  }

  mPointProps[i1].mConnectedPointMassIndices.push_back(i2);
  mPointProps[i2].mConnectedPointMassIndices.push_back(i1);

  return true;
}

//==============================================================================
void SoftBodyNodeUniqueProperties::addFace(const Eigen::Vector3i& _newFace)
{
  DART_ASSERT(_newFace[0] != _newFace[1]);
  DART_ASSERT(_newFace[1] != _newFace[2]);
  DART_ASSERT(_newFace[2] != _newFace[0]);
  DART_ASSERT(
      0 <= _newFace[0]
      && static_cast<std::size_t>(_newFace[0]) < mPointProps.size());
  DART_ASSERT(
      0 <= _newFace[1]
      && static_cast<std::size_t>(_newFace[1]) < mPointProps.size());
  DART_ASSERT(
      0 <= _newFace[2]
      && static_cast<std::size_t>(_newFace[2]) < mPointProps.size());
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

namespace {

template <typename StateVector>
struct PointMassPhaseView
{
  using StatePointer = decltype(std::declval<StateVector&>().data());

  PointMassPhaseView(
      const std::vector<PointMass*>& _pointMasses,
      StateVector& _states,
      const std::vector<PointMass::Properties>& _properties)
    : pointMasses(_pointMasses.data()),
      states(_states.data()),
      properties(_properties.data()),
      count(_pointMasses.size())
  {
    DART_ASSERT(_states.size() == count);
    DART_ASSERT(_properties.size() == count);
  }

  std::size_t size() const
  {
    return count;
  }

  PointMass* const* pointMasses;
  StatePointer states;
  const PointMass::Properties* properties;
  std::size_t count;
};

template <typename StateVector>
PointMassPhaseView<StateVector> makePointMassPhaseView(
    const std::vector<PointMass*>& _pointMasses,
    StateVector& _states,
    const std::vector<PointMass::Properties>& _properties)
{
  return PointMassPhaseView<StateVector>(_pointMasses, _states, _properties);
}

inline void addPointForceContribution(
    Eigen::Vector6d& _spatialForce,
    const Eigen::Vector3d& _localPosition,
    const Eigen::Vector3d& _force)
{
  _spatialForce[0]
      += _localPosition[1] * _force[2] - _localPosition[2] * _force[1];
  _spatialForce[1]
      += _localPosition[2] * _force[0] - _localPosition[0] * _force[2];
  _spatialForce[2]
      += _localPosition[0] * _force[1] - _localPosition[1] * _force[0];
  _spatialForce[3] += _force[0];
  _spatialForce[4] += _force[1];
  _spatialForce[5] += _force[2];
}

inline void addPointInertiaContribution(
    Eigen::Matrix6d& _spatialInertia,
    const Eigen::Vector3d& _localPosition,
    double _mass)
{
  const Eigen::Matrix3d skew = math::makeSkewSymmetric(_localPosition);
  _spatialInertia.topLeftCorner<3, 3>() -= _mass * skew * skew;
  _spatialInertia.topRightCorner<3, 3>() += _mass * skew;
  _spatialInertia.bottomLeftCorner<3, 3>() -= _mass * skew;

  _spatialInertia(3, 3) += _mass;
  _spatialInertia(4, 4) += _mass;
  _spatialInertia(5, 5) += _mass;
}

class AdaptiveContactActivationNotifier final : public PointMassNotifier
{
public:
  AdaptiveContactActivationNotifier(
      SoftBodyNode* _parentSoftBody, const std::string& _name)
    : PointMassNotifier(_parentSoftBody, _name)
  {
    // Do nothing
  }

  void resize(const std::vector<PointMass::Properties>& properties)
  {
    const std::size_t count = properties.size();
    mActive.assign(count, 1u);
    mSeeded.assign(count, 0u);
    mLingerCounters.assign(count, 0u);
    mFrozenLocalPositions.resize(count);
    mVisitedStamps.assign(count, 0u);
    mRingQueue.resize(count);
    mActiveCount = count;
    mCurrentTargetAll = true;
    mCurrentTargetStamp = 0u;
    mForceAllActiveNextStep = true;
    mAllFrozenRestArtInertiaContribution.setZero();

    for (std::size_t i = 0; i < count; ++i) {
      mFrozenLocalPositions[i] = properties[i].mX0;
      addPointInertiaContribution(
          mAllFrozenRestArtInertiaContribution,
          properties[i].mX0,
          properties[i].mMass);
    }
  }

  void refreshRestGeometry(const std::vector<PointMass::Properties>& properties)
  {
    mAllFrozenRestArtInertiaContribution.setZero();
    const std::size_t count
        = std::min(properties.size(), mFrozenLocalPositions.size());
    for (std::size_t i = 0; i < count; ++i) {
      addPointInertiaContribution(
          mAllFrozenRestArtInertiaContribution,
          properties[i].mX0,
          properties[i].mMass);
    }
  }

  std::uint32_t nextVisitStamp()
  {
    if (mVisitStamp == std::numeric_limits<std::uint32_t>::max()) {
      std::fill(mVisitedStamps.begin(), mVisitedStamps.end(), 0u);
      mVisitStamp = 1u;
    } else {
      ++mVisitStamp;
    }

    return mVisitStamp;
  }

  bool isActive(std::size_t index) const
  {
    return index < mActive.size() && mActive[index] != 0u;
  }

  bool isCurrentTarget(std::size_t index) const
  {
    if (mCurrentTargetAll)
      return index < mActive.size();

    return index < mVisitedStamps.size()
           && mVisitedStamps[index] == mCurrentTargetStamp;
  }

  bool mEnabled = false;
  std::size_t mRingCount = 2u;
  std::size_t mLingerSteps = 8u;
  double mVelocityTolerance = 1.0e-4;
  double mPositionTolerance = 1.0e-4;

  std::vector<unsigned char> mActive;
  std::vector<unsigned char> mSeeded;
  std::vector<std::size_t> mLingerCounters;
  std::vector<Eigen::Vector3d> mFrozenLocalPositions;
  std::vector<std::uint32_t> mVisitedStamps;
  std::vector<std::size_t> mRingQueue;
  std::uint32_t mVisitStamp = 1u;
  std::uint32_t mCurrentTargetStamp = 0u;
  bool mCurrentTargetAll = true;
  bool mForceAllActiveNextStep = true;
  std::size_t mActiveCount = 0u;
  Eigen::Matrix6d mAllFrozenRestArtInertiaContribution
      = Eigen::Matrix6d::Zero();
};

AdaptiveContactActivationNotifier& adaptiveContactActivation(
    SoftBodyNode& softBody)
{
  return *static_cast<AdaptiveContactActivationNotifier*>(
      softBody.getNotifier());
}

const AdaptiveContactActivationNotifier& adaptiveContactActivation(
    const SoftBodyNode& softBody)
{
  return *static_cast<const AdaptiveContactActivationNotifier*>(
      softBody.getNotifier());
}

} // namespace

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
  if (mAspectState.mPointStates == state.mPointStates)
    return;

  mAspectState = state;
  auto& activation = adaptiveContactActivation(*this);
  activation.mForceAllActiveNextStep = true;
  activation.mCurrentTargetAll = true;
  activation.mCurrentTargetStamp = 0u;
  std::fill(activation.mSeeded.begin(), activation.mSeeded.end(), 0u);
  // Restored state is immediately all-active so instrumentation such as
  // getNumActivePointMasses() does not report stale pre-restore counts.
  std::fill(activation.mActive.begin(), activation.mActive.end(), 1u);
  std::fill(
      activation.mLingerCounters.begin(), activation.mLingerCounters.end(), 0u);
  activation.mActiveCount = activation.mActive.size();
  mNotifier->dirtyTransform();
}

//==============================================================================
void SoftBodyNode::setAspectProperties(const AspectProperties& properties)
{
  setVertexSpringStiffness(properties.mKv);
  setEdgeSpringStiffness(properties.mKe);
  setDampingCoefficient(properties.mDampCoeff);

  if (properties.mPointProps != mAspectProperties.mPointProps
      || properties.mFaces != mAspectProperties.mFaces) {
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
  if (this == &_otherSoftBodyNode)
    return;

  setProperties(_otherSoftBodyNode.getSoftBodyNodeProperties());

  const auto& otherActivation = adaptiveContactActivation(_otherSoftBodyNode);
  auto& activation = adaptiveContactActivation(*this);
  activation.mEnabled = otherActivation.mEnabled;
  activation.mRingCount = otherActivation.mRingCount;
  activation.mLingerSteps = otherActivation.mLingerSteps;
  activation.mVelocityTolerance = otherActivation.mVelocityTolerance;
  activation.mPositionTolerance = otherActivation.mPositionTolerance;
  activation.mForceAllActiveNextStep = true;
  activation.mCurrentTargetAll = true;
  activation.mCurrentTargetStamp = 0u;
  std::fill(activation.mSeeded.begin(), activation.mSeeded.end(), 0u);
}

//==============================================================================
void SoftBodyNode::copy(const SoftBodyNode* _otherSoftBodyNode)
{
  if (nullptr == _otherSoftBodyNode)
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
  DART_ASSERT(_idx < mPointMasses.size());
  if (_idx < mPointMasses.size())
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
SoftBodyNode::SoftBodyNode(
    BodyNode* _parentBodyNode,
    Joint* _parentJoint,
    const Properties& _properties)
  : Entity(Frame::World(), false),
    Frame(Frame::World()),
    Base(std::make_tuple(_parentBodyNode, _parentJoint, _properties)),
    mSoftShapeNode(nullptr)
{
  createSoftBodyAspect();
  mNotifier = new AdaptiveContactActivationNotifier(
      this, getName() + "_PointMassNotifier");
  ShapeNode* softNode
      = createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(
          std::make_shared<SoftMeshShape>(this), getName() + "_SoftMeshShape");
  mSoftShapeNode = softNode;

  // Dev's Note: We do this workaround (instead of just using setProperties(~))
  // because mSoftShapeNode cannot be used until init(SkeletonPtr) has been
  // called on this BodyNode, but that happens after construction is finished.
  mAspectProperties = _properties;
  configurePointMasses(softNode);
  mNotifier->dirtyTransform();
}

//==============================================================================
BodyNode* SoftBodyNode::clone(
    BodyNode* _parentBodyNode, Joint* _parentJoint, bool cloneNodes) const
{
  SoftBodyNode* clonedBn = new SoftBodyNode(
      _parentBodyNode, _parentJoint, getSoftBodyNodeProperties());

  clonedBn->matchAspects(this);

  const auto& activation = adaptiveContactActivation(*this);
  auto& clonedActivation = adaptiveContactActivation(*clonedBn);
  clonedActivation.mEnabled = activation.mEnabled;
  clonedActivation.mRingCount = activation.mRingCount;
  clonedActivation.mLingerSteps = activation.mLingerSteps;
  clonedActivation.mVelocityTolerance = activation.mVelocityTolerance;
  clonedActivation.mPositionTolerance = activation.mPositionTolerance;
  clonedActivation.mForceAllActiveNextStep = true;
  clonedActivation.mCurrentTargetAll = true;
  clonedActivation.mCurrentTargetStamp = 0u;
  std::fill(
      clonedActivation.mSeeded.begin(), clonedActivation.mSeeded.end(), 0u);

  if (cloneNodes)
    clonedBn->matchNodes(this);

  return clonedBn;
}

//==============================================================================
void SoftBodyNode::configurePointMasses(ShapeNode* softNode)
{
  const UniqueProperties& softProperties = mAspectProperties;

  std::size_t newCount = softProperties.mPointProps.size();
  std::size_t oldCount = mPointMasses.size();

  auto& activation = adaptiveContactActivation(*this);
  if (newCount == oldCount) {
    activation.resize(softProperties.mPointProps);
    return;
  }

  // Adjust the number of PointMass objects since that has changed
  if (newCount < oldCount) {
    for (std::size_t i = newCount; i < oldCount; ++i)
      delete mPointMasses[i];
    mPointMasses.resize(newCount);
  } else if (oldCount < newCount) {
    mPointMasses.resize(newCount);
    for (std::size_t i = oldCount; i < newCount; ++i) {
      mPointMasses[i] = new PointMass(this);
      mPointMasses[i]->mIndex = i;
      mPointMasses[i]->init();
    }
  }

  // Resize the number of States in the Aspect
  mAspectState.mPointStates.resize(
      softProperties.mPointProps.size(), PointMass::State());

  activation.resize(softProperties.mPointProps);

  // Access the SoftMeshShape and reallocate its meshes
  if (softNode) {
    std::shared_ptr<SoftMeshShape> softShape
        = std::dynamic_pointer_cast<SoftMeshShape>(softNode->getShape());

    if (softShape)
      softShape->_buildMesh();
  } else {
    dtwarn << "[SoftBodyNode::configurePointMasses] The ShapeNode containing "
           << "the SoftMeshShape for the SoftBodyNode named [" << getName()
           << "] (" << this << ") has been removed. The soft body features for "
           << "this SoftBodyNode cannot be used unless you recreate the "
           << "SoftMeshShape.\n";

    std::cout << "ShapeNodes: " << std::endl;
    for (std::size_t i = 0; i < getNumShapeNodes(); ++i) {
      std::cout << "- " << i << ") " << getShapeNode(i)->getName() << std::endl;
    }
  }

  incrementVersion();
  mNotifier->dirtyTransform();
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
  //  DART_ASSERT(mSoftVisualShape == nullptr);
  //  mSoftVisualShape = new SoftMeshShape(this);
  //  BodyNode::addVisualizationShape(mSoftVisualShape);

  //  //----------------------------------------------------------------------------
  //  // Collision shape
  //  //----------------------------------------------------------------------------
  //  DART_ASSERT(mSoftCollShape == nullptr);
  //  mSoftCollShape = new SoftMeshShape(this);
  //  BodyNode::addCollisionShape(mSoftCollShape);
}

//==============================================================================
// void SoftBodyNode::aggregateGenCoords(std::vector<GenCoord*>* _genCoords)
//{
//  BodyNode::aggregateGenCoords(_genCoords);
//  aggregatePointMassGenCoords(_genCoords);
//}

//==============================================================================
// void SoftBodyNode::aggregatePointMassGenCoords(
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

  for (const auto& pointProperty : mAspectProperties.mPointProps)
    totalMass += pointProperty.mMass;

  return totalMass;
}

//==============================================================================
void SoftBodyNode::handlePointMassMassChange()
{
  adaptiveContactActivation(*this).refreshRestGeometry(
      mAspectProperties.mPointProps);
  dirtyArticulatedInertia();
  const SkeletonPtr& skel = getSkeleton();
  if (skel) {
    skel->updateTotalMass();
    skel->incrementDeactivationStateVersion();
  }
  incrementVersion();
}

//==============================================================================
void SoftBodyNode::handlePointMassRestingPositionChange()
{
  auto& activation = adaptiveContactActivation(*this);
  activation.refreshRestGeometry(mAspectProperties.mPointProps);
  // Frozen hold positions and linger decay targets are relative to the rest
  // shape, so a rest-position change re-derives the activation state from
  // scratch.
  activation.mForceAllActiveNextStep = true;
  dirtyArticulatedInertia();
  incrementVersion();
}

//==============================================================================
void SoftBodyNode::setVertexSpringStiffness(double _kv)
{
  DART_ASSERT(0.0 <= _kv);

  if (_kv == mAspectProperties.mKv)
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
  DART_ASSERT(0.0 <= _ke);

  if (_ke == mAspectProperties.mKe)
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
  if (std::isnan(_damp) || _damp < 0.0) {
    DART_WARN(
        "[SoftBodyNode] Invalid damping coefficient ({}) set for soft body "
        "[{}]. Damping must be non-negative; NaN and negative values are "
        "clamped to 0.",
        _damp,
        this->getName());
    _damp = 0.0;
  }

  if (_damp == mAspectProperties.mDampCoeff)
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
void SoftBodyNode::setAdaptiveContactActivationEnabled(bool enabled)
{
  auto& activation = adaptiveContactActivation(*this);
  if (activation.mEnabled == enabled)
    return;

  activation.mEnabled = enabled;
  activation.mForceAllActiveNextStep = true;
  activation.mCurrentTargetAll = true;
  activation.mCurrentTargetStamp = 0u;
  activation.mActiveCount = mPointMasses.size();
  std::fill(activation.mActive.begin(), activation.mActive.end(), 1u);
  std::fill(activation.mSeeded.begin(), activation.mSeeded.end(), 0u);
  std::fill(
      activation.mLingerCounters.begin(), activation.mLingerCounters.end(), 0u);
  for (std::size_t i = 0; i < activation.mFrozenLocalPositions.size(); ++i)
    activation.mFrozenLocalPositions[i] = mAspectProperties.mPointProps[i].mX0;

  if (enabled)
    mNotifier->dirtyTransform();
}

//==============================================================================
bool SoftBodyNode::isAdaptiveContactActivationEnabled() const
{
  return adaptiveContactActivation(*this).mEnabled;
}

//==============================================================================
void SoftBodyNode::setAdaptiveContactActivationRingCount(std::size_t ringCount)
{
  auto& activation = adaptiveContactActivation(*this);
  if (activation.mRingCount == ringCount)
    return;

  activation.mRingCount = ringCount;
  activation.mForceAllActiveNextStep = true;
}

//==============================================================================
std::size_t SoftBodyNode::getAdaptiveContactActivationRingCount() const
{
  return adaptiveContactActivation(*this).mRingCount;
}

//==============================================================================
void SoftBodyNode::setAdaptiveContactActivationLingerSteps(
    std::size_t lingerSteps)
{
  auto& activation = adaptiveContactActivation(*this);
  if (activation.mLingerSteps == lingerSteps)
    return;

  activation.mLingerSteps = lingerSteps;
}

//==============================================================================
std::size_t SoftBodyNode::getAdaptiveContactActivationLingerSteps() const
{
  return adaptiveContactActivation(*this).mLingerSteps;
}

//==============================================================================
void SoftBodyNode::setAdaptiveContactActivationVelocityTolerance(
    double tolerance)
{
  if (std::isnan(tolerance) || tolerance < 0.0) {
    DART_WARN(
        "[SoftBodyNode] Invalid adaptive contact activation velocity "
        "tolerance ({}) set for soft body [{}]. The tolerance must be "
        "non-negative; NaN and negative values are clamped to 0.",
        tolerance,
        getName());
    tolerance = 0.0;
  }

  adaptiveContactActivation(*this).mVelocityTolerance = tolerance;
}

//==============================================================================
double SoftBodyNode::getAdaptiveContactActivationVelocityTolerance() const
{
  return adaptiveContactActivation(*this).mVelocityTolerance;
}

//==============================================================================
void SoftBodyNode::setAdaptiveContactActivationPositionTolerance(
    double tolerance)
{
  if (std::isnan(tolerance) || tolerance < 0.0) {
    DART_WARN(
        "[SoftBodyNode] Invalid adaptive contact activation position "
        "tolerance ({}) set for soft body [{}]. The tolerance must be "
        "non-negative; NaN and negative values are clamped to 0.",
        tolerance,
        getName());
    tolerance = 0.0;
  }

  adaptiveContactActivation(*this).mPositionTolerance = tolerance;
}

//==============================================================================
double SoftBodyNode::getAdaptiveContactActivationPositionTolerance() const
{
  return adaptiveContactActivation(*this).mPositionTolerance;
}

//==============================================================================
std::size_t SoftBodyNode::getNumActivePointMasses() const
{
  const auto& activation = adaptiveContactActivation(*this);
  if (!activation.mEnabled)
    return getNumPointMasses();

  return activation.mActiveCount;
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
  mPointMasses.back()->mIndex = mPointMasses.size() - 1;
  mAspectProperties.mPointProps.push_back(_properties);
  configurePointMasses(mSoftShapeNode.lock());

  return mPointMasses.back();
}

//==============================================================================
void SoftBodyNode::connectPointMasses(std::size_t _idx1, std::size_t _idx2)
{
  DART_ASSERT(_idx1 != _idx2);
  DART_ASSERT(_idx1 < mPointMasses.size());
  DART_ASSERT(_idx2 < mPointMasses.size());
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
  DART_ASSERT(_idx < mAspectProperties.mFaces.size());
  return mAspectProperties.mFaces[_idx];
}

//==============================================================================
std::size_t SoftBodyNode::getNumFaces() const
{
  return mAspectProperties.mFaces.size();
}

//==============================================================================
void SoftBodyNode::seedAdaptiveContactActivationFace(int faceId)
{
  auto& activation = adaptiveContactActivation(*this);
  if (!activation.mEnabled)
    return;

  if (faceId < 0 || static_cast<std::size_t>(faceId) >= getNumFaces())
    return;

  const Eigen::Vector3i& face = getFace(static_cast<std::size_t>(faceId));
  bool seededFrozenPoint = false;
  for (int i = 0; i < 3; ++i) {
    const int pointIndex = face[i];
    if (pointIndex < 0)
      continue;

    const std::size_t index = static_cast<std::size_t>(pointIndex);
    if (index < activation.mSeeded.size()) {
      if (activation.mSeeded[index] == 0u && !activation.isActive(index))
        seededFrozenPoint = true;
      activation.mSeeded[index] = 1u;
    }
  }

  // A quiescent body (for example a weld-jointed soft link) may never
  // re-dirty its articulated inertia, and the activation tick only runs from
  // updateArtInertia. Force the tick only for a genuine activation event —
  // seeding an already-active point neither changes the set nor lets linger
  // advance, and unconditional dirtying would churn caches every step.
  if (seededFrozenPoint)
    dirtyArticulatedInertia();
}

//==============================================================================
void SoftBodyNode::seedAdaptiveContactActivationPoint(std::size_t index)
{
  auto& activation = adaptiveContactActivation(*this);
  if (!activation.mEnabled)
    return;

  if (index >= activation.mSeeded.size())
    return;

  const bool activationEvent
      = activation.mSeeded[index] == 0u && !activation.isActive(index);
  activation.mSeeded[index] = 1u;
  if (activationEvent)
    dirtyArticulatedInertia();
}

//==============================================================================
bool SoftBodyNode::isAdaptiveContactPointMassActive(std::size_t index) const
{
  const auto& activation = adaptiveContactActivation(*this);
  if (!activation.mEnabled)
    return true;

  return activation.isActive(index);
}

//==============================================================================
void SoftBodyNode::integratePointMassPositions(double dt)
{
  auto& activation = adaptiveContactActivation(*this);
  if (!activation.mEnabled) {
    for (std::size_t i = 0; i < getNumPointMasses(); ++i)
      getPointMass(i)->integratePositions(dt);
    return;
  }

  for (std::size_t i = 0; i < getNumPointMasses(); ++i) {
    if (activation.isActive(i)) {
      getPointMass(i)->integratePositions(dt);
    } else {
      PointMass::State& state = mAspectState.mPointStates[i];
      state.mPositions = activation.mFrozenLocalPositions[i]
                         - mAspectProperties.mPointProps[i].mX0;
      state.mVelocities.setZero();
      state.mAccelerations.setZero();
    }
  }
}

//==============================================================================
void SoftBodyNode::integratePointMassVelocities(double dt)
{
  auto& activation = adaptiveContactActivation(*this);
  if (!activation.mEnabled) {
    for (std::size_t i = 0; i < getNumPointMasses(); ++i)
      getPointMass(i)->integrateVelocities(dt);
    return;
  }

  for (std::size_t i = 0; i < getNumPointMasses(); ++i) {
    if (activation.isActive(i)) {
      getPointMass(i)->integrateVelocities(dt);
    } else {
      PointMass::State& state = mAspectState.mPointStates[i];
      state.mPositions = activation.mFrozenLocalPositions[i]
                         - mAspectProperties.mPointProps[i].mX0;
      state.mVelocities.setZero();
      state.mAccelerations.setZero();
    }
  }
}

//==============================================================================
void SoftBodyNode::prepareAdaptiveContactActivationForDynamics(
    double /*timeStep*/) const
{
  const auto& constActivation = adaptiveContactActivation(*this);
  if (!constActivation.mEnabled)
    return;

  auto* self = const_cast<SoftBodyNode*>(this);
  auto& activation = adaptiveContactActivation(*self);
  const std::size_t count = self->getNumPointMasses();
  if (count == 0u) {
    activation.mActiveCount = 0u;
    return;
  }

  DART_ASSERT(activation.mActive.size() == count);
  DART_ASSERT(activation.mSeeded.size() == count);
  DART_ASSERT(activation.mFrozenLocalPositions.size() == count);
  DART_ASSERT(activation.mVisitedStamps.size() == count);
  DART_ASSERT(activation.mRingQueue.size() == count);

  bool stateChanged = false;
  if (activation.mForceAllActiveNextStep) {
    std::fill(activation.mActive.begin(), activation.mActive.end(), 1u);
    std::fill(
        activation.mLingerCounters.begin(),
        activation.mLingerCounters.end(),
        0u);
    std::fill(activation.mSeeded.begin(), activation.mSeeded.end(), 0u);
    activation.mActiveCount = count;
    activation.mCurrentTargetAll = true;
    activation.mCurrentTargetStamp = 0u;
    activation.mForceAllActiveNextStep = false;
    return;
  }

  const std::uint32_t stamp = activation.nextVisitStamp();
  std::size_t queueHead = 0u;
  std::size_t queueTail = 0u;
  for (std::size_t i = 0; i < count; ++i) {
    if (activation.mSeeded[i] == 0u)
      continue;

    activation.mSeeded[i] = 0u;
    if (activation.mVisitedStamps[i] == stamp)
      continue;

    activation.mVisitedStamps[i] = stamp;
    activation.mRingQueue[queueTail++] = i;
  }

  for (std::size_t ring = 0u; ring < activation.mRingCount; ++ring) {
    const std::size_t ringEnd = queueTail;
    for (; queueHead < ringEnd; ++queueHead) {
      const std::size_t index = activation.mRingQueue[queueHead];
      const auto& connections
          = mAspectProperties.mPointProps[index].mConnectedPointMassIndices;
      for (const std::size_t connectedIndex : connections) {
        if (connectedIndex >= count)
          continue;
        if (activation.mVisitedStamps[connectedIndex] == stamp)
          continue;

        activation.mVisitedStamps[connectedIndex] = stamp;
        DART_ASSERT(queueTail < activation.mRingQueue.size());
        activation.mRingQueue[queueTail++] = connectedIndex;
      }
    }
  }

  activation.mCurrentTargetAll = false;
  activation.mCurrentTargetStamp = stamp;

  const double velocityToleranceSquared
      = activation.mVelocityTolerance * activation.mVelocityTolerance;
  const double positionToleranceSquared
      = activation.mPositionTolerance * activation.mPositionTolerance;

  std::size_t activeCount = 0u;
  for (std::size_t i = 0; i < count; ++i) {
    PointMass::State& state = self->mAspectState.mPointStates[i];
    const PointMass::Properties& properties = mAspectProperties.mPointProps[i];
    const bool inTarget = activation.isCurrentTarget(i);

    if (inTarget) {
      if (activation.mActive[i] == 0u) {
        state.mPositions = activation.mFrozenLocalPositions[i] - properties.mX0;
        state.mVelocities.setZero();
        state.mAccelerations.setZero();
        stateChanged = true;
      }
      activation.mActive[i] = 1u;
      activation.mLingerCounters[i] = 0u;
      ++activeCount;
      continue;
    }

    if (activation.mActive[i] == 0u) {
      state.mPositions = activation.mFrozenLocalPositions[i] - properties.mX0;
      state.mVelocities.setZero();
      state.mAccelerations.setZero();
      continue;
    }

    const bool nearRest
        = state.mVelocities.squaredNorm() <= velocityToleranceSquared
          && state.mPositions.squaredNorm() <= positionToleranceSquared;
    if (nearRest && activation.mLingerCounters[i] >= activation.mLingerSteps) {
      activation.mActive[i] = 0u;
      activation.mFrozenLocalPositions[i] = properties.mX0;
      state.mPositions.setZero();
      state.mVelocities.setZero();
      state.mAccelerations.setZero();
      stateChanged = true;
    } else {
      ++activation.mLingerCounters[i];
      ++activeCount;
    }
  }

  activation.mActiveCount = activeCount;
  if (stateChanged)
    self->mNotifier->dirtyTransform();
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
  if (skel && skel->mTreeCache[mTreeIndex].mDirty.mArticulatedInertia)
    skel->updateArticulatedInertia(mTreeIndex);
}

//==============================================================================
void SoftBodyNode::updateTransform()
{
  DART_PROFILE_SCOPED_IF_N(
      dart::common::profile::isProfileRecordingEnabled(),
      "SoftBodyNode::updateTransform");
  BodyNode::updateTransform();

  auto phase = makePointMassPhaseView(
      mPointMasses, mAspectState.mPointStates, mAspectProperties.mPointProps);

  const Eigen::Isometry3d& parentWorldTransform = getWorldTransform();
  for (std::size_t i = 0; i < phase.size(); ++i) {
    PointMass& pointMass = *phase.pointMasses[i];
    pointMass.mX = phase.states[i].mPositions + phase.properties[i].mX0;
    DART_ASSERT(!math::isNan(pointMass.mX));

    pointMass.mW = parentWorldTransform.translation()
                   + parentWorldTransform.linear() * pointMass.mX;
    DART_ASSERT(!math::isNan(pointMass.mW));
  }

  mNotifier->clearTransformNotice();
}

//==============================================================================
void SoftBodyNode::updateVelocity()
{
  DART_PROFILE_SCOPED_IF_N(
      dart::common::profile::isProfileRecordingEnabled(),
      "SoftBodyNode::updateVelocity");
  BodyNode::updateVelocity();

  if (mNotifier->needsTransformUpdate())
    updateTransform();

  auto phase = makePointMassPhaseView(
      mPointMasses, mAspectState.mPointStates, mAspectProperties.mPointProps);

  const Eigen::Vector6d& parentSpatialVelocity = getSpatialVelocity();
  const Eigen::Vector3d parentAngularVelocity = parentSpatialVelocity.head<3>();
  const Eigen::Vector3d parentLinearVelocity = parentSpatialVelocity.tail<3>();
  for (std::size_t i = 0; i < phase.size(); ++i) {
    PointMass& pointMass = *phase.pointMasses[i];
    pointMass.mV = parentAngularVelocity.cross(pointMass.mX)
                   + parentLinearVelocity + phase.states[i].mVelocities;
    DART_ASSERT(!math::isNan(pointMass.mV));
  }

  mNotifier->clearVelocityNotice();
}

//==============================================================================
void SoftBodyNode::updatePartialAcceleration() const
{
  BodyNode::updatePartialAcceleration();

  auto phase = makePointMassPhaseView(
      mPointMasses, mAspectState.mPointStates, mAspectProperties.mPointProps);

  const Eigen::Vector3d parentAngularVelocity = getSpatialVelocity().head<3>();
  for (std::size_t i = 0; i < phase.size(); ++i) {
    PointMass& pointMass = *phase.pointMasses[i];
    pointMass.mEta = parentAngularVelocity.cross(phase.states[i].mVelocities);
    DART_ASSERT(!math::isNan(pointMass.mEta));
  }

  mNotifier->clearPartialAccelerationNotice();
}

//==============================================================================
void SoftBodyNode::updateAccelerationID()
{
  BodyNode::updateAccelerationID();

  if (mNotifier->needsTransformUpdate())
    updateTransform();
  if (mNotifier->needsPartialAccelerationUpdate())
    updatePartialAcceleration();

  const Eigen::Vector6d& parentAcceleration = getSpatialAcceleration();
  const Eigen::Vector3d parentAngularAcceleration
      = parentAcceleration.head<3>();
  const Eigen::Vector3d parentLinearAcceleration = parentAcceleration.tail<3>();
  auto phase = makePointMassPhaseView(
      mPointMasses, mAspectState.mPointStates, mAspectProperties.mPointProps);
  for (std::size_t i = 0; i < phase.size(); ++i) {
    PointMass& pointMass = *phase.pointMasses[i];
    pointMass.mA = parentAngularAcceleration.cross(pointMass.mX)
                   + parentLinearAcceleration + pointMass.mEta
                   + phase.states[i].mAccelerations;
    DART_ASSERT(!math::isNan(pointMass.mA));
  }

  mNotifier->clearAccelerationNotice();
}

//==============================================================================
void SoftBodyNode::updateTransmittedForceID(
    const Eigen::Vector3d& _gravity, bool _withExternalForces)
{
  const Eigen::Matrix6d& mI
      = BodyNode::mAspectProperties.mInertia.getSpatialTensor();

  if (mNotifier->needsVelocityUpdate())
    updateVelocity();
  if (mNotifier->needsAccelerationUpdate())
    updateAccelerationID();
  if (mNotifier->needsTransformUpdate())
    updateTransform();

  const bool gravityMode = BodyNode::mAspectProperties.mGravityMode;
  Eigen::Vector3d localGravity = Eigen::Vector3d::Zero();
  if (gravityMode)
    localGravity = getWorldTransform().linear().transpose() * _gravity;
  const Eigen::Vector3d parentAngularVelocity = getSpatialVelocity().head<3>();

  Eigen::Vector6d pointForceContribution = Eigen::Vector6d::Zero();
  auto phase = makePointMassPhaseView(
      mPointMasses, mAspectState.mPointStates, mAspectProperties.mPointProps);
  for (std::size_t i = 0; i < phase.size(); ++i) {
    PointMass& pointMass = *phase.pointMasses[i];
    const double mass = phase.properties[i].mMass;
    pointMass.mF.noalias() = mass * pointMass.mA;
    pointMass.mF
        += parentAngularVelocity.cross(mass * pointMass.mV) - pointMass.mFext;
    if (gravityMode)
      pointMass.mF -= mass * localGravity;
    DART_ASSERT(!math::isNan(pointMass.mF));
    addPointForceContribution(
        pointForceContribution, pointMass.mX, pointMass.mF);
  }

  // Gravity force
  if (gravityMode)
    mFgravity.noalias()
        = mI * math::AdInvRLinear(getWorldTransform(), _gravity);
  else
    mFgravity.setZero();

  // Inertial force
  mF.noalias() = mI * getSpatialAcceleration();

  // External force
  if (_withExternalForces)
    mF -= BodyNode::mAspectState.mFext;

  // Verification
  DART_ASSERT(!math::isNan(mF));

  // Gravity force
  mF -= mFgravity;

  // Coriolis force
  const Eigen::Vector6d& V = getSpatialVelocity();
  mF -= math::dad(V, mI * V);

  //
  for (const auto& childBodyNode : mChildBodyNodes) {
    Joint* childJoint = childBodyNode->getParentJoint();
    DART_ASSERT(childJoint != nullptr);

    mF += math::dAdInvT(
        childJoint->getRelativeTransform(), childBodyNode->getBodyForce());
  }
  mF += pointForceContribution;

  // Verification
  DART_ASSERT(!math::isNan(mF));
}

//==============================================================================
void SoftBodyNode::updateJointForceID(
    double _timeStep, bool _withDampingForces, bool _withSpringForces)
{
  auto phase = makePointMassPhaseView(
      mPointMasses, mAspectState.mPointStates, mAspectProperties.mPointProps);
  for (std::size_t i = 0; i < phase.size(); ++i)
    phase.states[i].mForces = phase.pointMasses[i]->mF;

  BodyNode::updateJointForceID(
      _timeStep, _withDampingForces, _withSpringForces);
}

//==============================================================================
void SoftBodyNode::updateJointForceFD(
    double _timeStep, bool _withDampingForces, bool _withSpringForces)
{
  BodyNode::updateJointForceFD(
      _timeStep, _withDampingForces, _withSpringForces);
}

//==============================================================================
void SoftBodyNode::updateJointImpulseFD()
{
  BodyNode::updateJointImpulseFD();
}

//==============================================================================
void SoftBodyNode::updateArtInertia(double _timeStep) const
{
  DART_PROFILE_SCOPED_IF_N(
      dart::common::profile::isProfileRecordingEnabled(),
      "SoftBodyNode::updateArtInertia");
  const bool adaptiveEnabled = isAdaptiveContactActivationEnabled();
  if (adaptiveEnabled)
    prepareAdaptiveContactActivationForDynamics(_timeStep);

  const Eigen::Matrix6d& mI
      = BodyNode::mAspectProperties.mInertia.getSpatialTensor();
  const double dampingCoefficient = getDampingCoefficient();
  const double vertexSpringStiffness = getVertexSpringStiffness();
  const double implicitOffset = _timeStep * dampingCoefficient
                                + _timeStep * _timeStep * vertexSpringStiffness;
  auto phase = makePointMassPhaseView(
      mPointMasses, mAspectState.mPointStates, mAspectProperties.mPointProps);
  const auto* activation
      = adaptiveEnabled ? &adaptiveContactActivation(*this) : nullptr;
  const bool allFrozen = activation && activation->mActiveCount == 0u;
  if (!adaptiveEnabled) {
    for (std::size_t i = 0; i < phase.size(); ++i) {
      PointMass& pointMass = *phase.pointMasses[i];
      const double mass = phase.properties[i].mMass;
      const double massSquared = mass * mass;
      pointMass.mPsi = 1.0 / mass;
      pointMass.mImplicitPsi = 1.0 / (mass + implicitOffset);
      DART_ASSERT(!math::isNan(pointMass.mImplicitPsi));

      pointMass.mPi = mass - massSquared * pointMass.mPsi;
      pointMass.mImplicitPi = mass - massSquared * pointMass.mImplicitPsi;
      DART_ASSERT(!math::isNan(pointMass.mPi));
      DART_ASSERT(!math::isNan(pointMass.mImplicitPi));
    }
  } else if (!allFrozen) {
    for (std::size_t i = 0; i < phase.size(); ++i) {
      PointMass& pointMass = *phase.pointMasses[i];
      const double mass = phase.properties[i].mMass;
      if (activation->isActive(i)) {
        const double massSquared = mass * mass;
        pointMass.mPsi = 1.0 / mass;
        pointMass.mImplicitPsi = 1.0 / (mass + implicitOffset);
        DART_ASSERT(!math::isNan(pointMass.mImplicitPsi));

        pointMass.mPi = mass - massSquared * pointMass.mPsi;
        pointMass.mImplicitPi = mass - massSquared * pointMass.mImplicitPsi;
      } else {
        pointMass.mPsi = 0.0;
        pointMass.mImplicitPsi = 0.0;
        pointMass.mPi = mass;
        pointMass.mImplicitPi = mass;
      }
      DART_ASSERT(!math::isNan(pointMass.mPi));
      DART_ASSERT(!math::isNan(pointMass.mImplicitPi));
    }
  }

  DART_ASSERT(mParentJoint != nullptr);

  // Set spatial inertia to the articulated body inertia
  mArtInertia = mI;
  mArtInertiaImplicit = mI;

  // and add child articulated body inertia
  for (const auto& child : mChildBodyNodes) {
    Joint* childJoint = child->getParentJoint();

    childJoint->addChildArtInertiaTo(mArtInertia, child->mArtInertia);
    childJoint->addChildArtInertiaImplicitTo(
        mArtInertiaImplicit, child->mArtInertiaImplicit);
  }

  if (!adaptiveEnabled) {
    for (std::size_t i = 0; i < phase.size(); ++i) {
      const PointMass& pointMass = *phase.pointMasses[i];
      const Eigen::Vector3d& localPosition = pointMass.getLocalPosition();
      _addPiToArtInertia(localPosition, pointMass.mPi);
      _addPiToArtInertiaImplicit(localPosition, pointMass.mImplicitPi);
    }
  } else if (allFrozen) {
    mArtInertia += activation->mAllFrozenRestArtInertiaContribution;
    mArtInertiaImplicit += activation->mAllFrozenRestArtInertiaContribution;
  } else {
    for (std::size_t i = 0; i < phase.size(); ++i) {
      const PointMass& pointMass = *phase.pointMasses[i];
      const Eigen::Vector3d& localPosition
          = activation->isActive(i) ? pointMass.getLocalPosition()
                                    : activation->mFrozenLocalPositions[i];
      _addPiToArtInertia(localPosition, pointMass.mPi);
      _addPiToArtInertiaImplicit(localPosition, pointMass.mImplicitPi);
    }
  }

  // Verification
  DART_ASSERT(!math::isNan(mArtInertia));
  DART_ASSERT(!math::isNan(mArtInertiaImplicit));

  // Update parent joint's inverse of projected articulated body inertia
  mParentJoint->updateInvProjArtInertia(mArtInertia);
  mParentJoint->updateInvProjArtInertiaImplicit(mArtInertiaImplicit, _timeStep);

  // Verification
  DART_ASSERT(!math::isNan(mArtInertia));
  DART_ASSERT(!math::isNan(mArtInertiaImplicit));
}

//==============================================================================
void SoftBodyNode::updateBiasForce(
    const Eigen::Vector3d& _gravity, double _timeStep)
{
  DART_PROFILE_SCOPED_IF_N(
      dart::common::profile::isProfileRecordingEnabled(),
      "SoftBodyNode::updateBiasForce");
  const Eigen::Matrix6d& mI
      = BodyNode::mAspectProperties.mInertia.getSpatialTensor();
  const bool gravityMode = BodyNode::mAspectProperties.mGravityMode;
  Eigen::Vector3d localGravity = Eigen::Vector3d::Zero();
  if (gravityMode)
    localGravity = getWorldTransform().linear().transpose() * _gravity;
  const Eigen::Vector3d parentAngularVelocity = getSpatialVelocity().head<3>();
  const double vertexSpringStiffness = getVertexSpringStiffness();
  const double edgeSpringStiffness = getEdgeSpringStiffness();
  const double dampingCoefficient = getDampingCoefficient();
  auto phase = makePointMassPhaseView(
      mPointMasses, mAspectState.mPointStates, mAspectProperties.mPointProps);
  const bool adaptiveEnabled = isAdaptiveContactActivationEnabled();

  if (mNotifier->needsVelocityUpdate())
    updateVelocity();
  if (mNotifier->needsPartialAccelerationUpdate())
    updatePartialAcceleration();
  if (mNotifier->needsTransformUpdate())
    updateTransform();
  checkArticulatedInertiaUpdate();

  Eigen::Vector6d frozenBoundaryContribution = Eigen::Vector6d::Zero();
  if (!adaptiveEnabled) {
    for (std::size_t i = 0; i < phase.size(); ++i) {
      PointMass& pointMass = *phase.pointMasses[i];
      const PointMass::State& state = phase.states[i];
      const PointMass::Properties& properties = phase.properties[i];
      const double mass = properties.mMass;

      // Reset internal forces of point masses before used.
      //
      // Once control force for point mass is introduced, assign it to the
      // internal force instead of always resetting the internal forces to zero.
      phase.states[i].mForces.setZero();

      pointMass.mB
          = parentAngularVelocity.cross(mass * pointMass.mV) - pointMass.mFext;
      if (gravityMode)
        pointMass.mB -= mass * localGravity;
      DART_ASSERT(!math::isNan(pointMass.mB));

      const std::vector<std::size_t>& connections
          = properties.mConnectedPointMassIndices;
      const std::size_t numConnections = connections.size();
      const std::size_t* connectionIndices = connections.data();
      const double springStiffness
          = vertexSpringStiffness + numConnections * edgeSpringStiffness;
      const double velocityScale
          = _timeStep * springStiffness + dampingCoefficient;
      pointMass.mAlpha = state.mForces - springStiffness * state.mPositions
                         - velocityScale * state.mVelocities
                         - mass * pointMass.mEta - pointMass.mB;
      for (std::size_t j = 0; j < numConnections; ++j) {
        const std::size_t connectedIndex = connectionIndices[j];
        DART_ASSERT(connectedIndex < phase.size());
        const PointMass::State& connectedState = phase.states[connectedIndex];
        pointMass.mAlpha += edgeSpringStiffness
                            * (connectedState.mPositions
                               + _timeStep * connectedState.mVelocities);
      }
      DART_ASSERT(!math::isNan(pointMass.mAlpha));

      pointMass.mBeta = pointMass.mB;
      pointMass.mBeta.noalias()
          += mass
             * (pointMass.mEta + pointMass.mImplicitPsi * pointMass.mAlpha);
      DART_ASSERT(!math::isNan(pointMass.mBeta));
    }
  } else {
    const auto& activation = adaptiveContactActivation(*this);
    for (std::size_t i = 0; i < phase.size(); ++i) {
      PointMass& pointMass = *phase.pointMasses[i];
      PointMass::State& state = phase.states[i];
      const PointMass::Properties& properties = phase.properties[i];
      const double mass = properties.mMass;

      state.mForces.setZero();

      pointMass.mB
          = parentAngularVelocity.cross(mass * pointMass.mV) - pointMass.mFext;
      if (gravityMode)
        pointMass.mB -= mass * localGravity;
      DART_ASSERT(!math::isNan(pointMass.mB));

      if (!activation.isActive(i)) {
        pointMass.mAlpha.setZero();
        pointMass.mBeta = pointMass.mB;
        DART_ASSERT(!math::isNan(pointMass.mBeta));
        continue;
      }

      const std::vector<std::size_t>& connections
          = properties.mConnectedPointMassIndices;
      const std::size_t numConnections = connections.size();
      const std::size_t* connectionIndices = connections.data();
      const double springStiffness
          = vertexSpringStiffness + numConnections * edgeSpringStiffness;
      const double velocityScale
          = _timeStep * springStiffness + dampingCoefficient;
      pointMass.mAlpha = state.mForces - springStiffness * state.mPositions
                         - velocityScale * state.mVelocities
                         - mass * pointMass.mEta - pointMass.mB;
      for (std::size_t j = 0; j < numConnections; ++j) {
        const std::size_t connectedIndex = connectionIndices[j];
        DART_ASSERT(connectedIndex < phase.size());
        const PointMass::State& connectedState = phase.states[connectedIndex];
        pointMass.mAlpha += edgeSpringStiffness
                            * (connectedState.mPositions
                               + _timeStep * connectedState.mVelocities);

        if (!activation.isActive(connectedIndex)) {
          const Eigen::Vector3d reaction
              = edgeSpringStiffness
                * (state.mPositions + _timeStep * state.mVelocities
                   - connectedState.mPositions
                   - _timeStep * connectedState.mVelocities);
          addPointForceContribution(
              frozenBoundaryContribution,
              activation.mFrozenLocalPositions[connectedIndex],
              reaction);
        }
      }

      if (!activation.isCurrentTarget(i)) {
        const double decayStiffness = springStiffness;
        const double decayDamping = 2.0 * std::sqrt(decayStiffness * mass);
        pointMass.mAlpha.noalias() -= decayStiffness * state.mPositions
                                      + decayDamping * state.mVelocities;
      }

      DART_ASSERT(!math::isNan(pointMass.mAlpha));

      pointMass.mBeta = pointMass.mB;
      pointMass.mBeta.noalias()
          += mass
             * (pointMass.mEta + pointMass.mImplicitPsi * pointMass.mAlpha);
      DART_ASSERT(!math::isNan(pointMass.mBeta));
    }
  }

  // Gravity force
  if (BodyNode::mAspectProperties.mGravityMode == true)
    mFgravity.noalias()
        = mI * math::AdInvRLinear(getWorldTransform(), _gravity);
  else
    mFgravity.setZero();

  // Set bias force
  const Eigen::Vector6d& V = getSpatialVelocity();
  mBiasForce = -math::dad(V, mI * V) - BodyNode::mAspectState.mFext - mFgravity;

  // Verifycation
  DART_ASSERT(!math::isNan(mBiasForce));

  // And add child bias force
  for (const auto& childBodyNode : mChildBodyNodes) {
    Joint* childJoint = childBodyNode->getParentJoint();

    childJoint->addChildBiasForceTo(
        mBiasForce,
        childBodyNode->getArticulatedInertiaImplicit(),
        childBodyNode->mBiasForce,
        childBodyNode->getPartialAcceleration());
  }

  if (!adaptiveEnabled) {
    for (std::size_t i = 0; i < phase.size(); ++i) {
      const PointMass& pointMass = *phase.pointMasses[i];
      addPointForceContribution(mBiasForce, pointMass.mX, pointMass.mBeta);
    }
  } else {
    const auto& activation = adaptiveContactActivation(*this);
    for (std::size_t i = 0; i < phase.size(); ++i) {
      const PointMass& pointMass = *phase.pointMasses[i];
      const Eigen::Vector3d& localPosition
          = activation.isActive(i) ? pointMass.mX
                                   : activation.mFrozenLocalPositions[i];
      addPointForceContribution(mBiasForce, localPosition, pointMass.mBeta);
    }
    mBiasForce += frozenBoundaryContribution;
  }

  // Verifycation
  DART_ASSERT(!math::isNan(mBiasForce));

  // Update parent joint's total force with implicit joint damping and spring
  // forces
  mParentJoint->updateTotalForce(
      getArticulatedInertiaImplicit() * getPartialAcceleration() + mBiasForce,
      _timeStep);
}

//==============================================================================
void SoftBodyNode::updateAccelerationFD()
{
  BodyNode::updateAccelerationFD();

  if (mNotifier->needsTransformUpdate())
    updateTransform();
  if (mNotifier->needsPartialAccelerationUpdate())
    updatePartialAcceleration();
  checkArticulatedInertiaUpdate();

  const Eigen::Vector6d& parentAcceleration = getSpatialAcceleration();
  const Eigen::Vector3d parentAngularAcceleration
      = parentAcceleration.head<3>();
  const Eigen::Vector3d parentLinearAcceleration = parentAcceleration.tail<3>();
  auto phase = makePointMassPhaseView(
      mPointMasses, mAspectState.mPointStates, mAspectProperties.mPointProps);

  const bool adaptiveEnabled = isAdaptiveContactActivationEnabled();
  if (!adaptiveEnabled) {
    for (std::size_t i = 0; i < phase.size(); ++i) {
      PointMass& pointMass = *phase.pointMasses[i];
      const Eigen::Vector3d parentPointAcceleration
          = parentAngularAcceleration.cross(pointMass.mX)
            + parentLinearAcceleration;
      const Eigen::Vector3d ddq
          = pointMass.mImplicitPsi
            * (pointMass.mAlpha
               - phase.properties[i].mMass * parentPointAcceleration);
      phase.states[i].mAccelerations = ddq;
      DART_ASSERT(!math::isNan(ddq));

      pointMass.mA = parentPointAcceleration + pointMass.mEta + ddq;
      DART_ASSERT(!math::isNan(pointMass.mA));
    }
  } else {
    const auto& activation = adaptiveContactActivation(*this);
    for (std::size_t i = 0; i < phase.size(); ++i) {
      PointMass& pointMass = *phase.pointMasses[i];
      const Eigen::Vector3d& localPosition
          = activation.isActive(i) ? pointMass.mX
                                   : activation.mFrozenLocalPositions[i];
      const Eigen::Vector3d parentPointAcceleration
          = parentAngularAcceleration.cross(localPosition)
            + parentLinearAcceleration;

      if (activation.isActive(i)) {
        const Eigen::Vector3d ddq
            = pointMass.mImplicitPsi
              * (pointMass.mAlpha
                 - phase.properties[i].mMass * parentPointAcceleration);
        phase.states[i].mAccelerations = ddq;
        DART_ASSERT(!math::isNan(ddq));

        pointMass.mA = parentPointAcceleration + pointMass.mEta + ddq;
      } else {
        phase.states[i].mAccelerations.setZero();
        pointMass.mA = parentPointAcceleration;
      }
      DART_ASSERT(!math::isNan(pointMass.mA));
    }
  }

  mNotifier->clearAccelerationNotice();
}

//==============================================================================
void SoftBodyNode::updateTransmittedForceFD()
{
  BodyNode::updateTransmittedForceFD();

  if (mNotifier->needsAccelerationUpdate())
    updateAccelerationFD();

  auto phase = makePointMassPhaseView(
      mPointMasses, mAspectState.mPointStates, mAspectProperties.mPointProps);
  for (std::size_t i = 0; i < phase.size(); ++i) {
    PointMass& pointMass = *phase.pointMasses[i];
    pointMass.mF = pointMass.mB;
    pointMass.mF.noalias() += phase.properties[i].mMass * pointMass.mA;
    DART_ASSERT(!math::isNan(pointMass.mF));
  }
}

//==============================================================================
void SoftBodyNode::updateBiasImpulse()
{
  for (auto& pointMassPtr : mPointMasses) {
    PointMass& pointMass = *pointMassPtr;
    pointMass.mImpB = -pointMass.mConstraintImpulses;
    DART_ASSERT(!math::isNan(pointMass.mImpB));

    pointMass.mImpAlpha = -pointMass.mImpB;
    DART_ASSERT(!math::isNan(pointMass.mImpAlpha));

    pointMass.mImpBeta.setZero();
    DART_ASSERT(!math::isNan(pointMass.mImpBeta));
  }

  // Update impulsive bias force
  mBiasImpulse = -mConstraintImpulse;

  // And add child bias impulse
  for (auto& childBodyNode : mChildBodyNodes) {
    Joint* childJoint = childBodyNode->getParentJoint();

    childJoint->addChildBiasImpulseTo(
        mBiasImpulse,
        childBodyNode->getArticulatedInertia(),
        childBodyNode->mBiasImpulse);
  }

  // Verification
  DART_ASSERT(!math::isNan(mBiasImpulse));

  // Update parent joint's total force
  mParentJoint->updateTotalImpulse(mBiasImpulse);
}

//==============================================================================
void SoftBodyNode::updateVelocityChangeFD()
{
  BodyNode::updateVelocityChangeFD();

  if (mNotifier->needsTransformUpdate())
    updateTransform();
  checkArticulatedInertiaUpdate();

  const Eigen::Vector6d& parentVelocityChange = getBodyVelocityChange();
  const Eigen::Vector3d parentAngularVelocityChange
      = parentVelocityChange.head<3>();
  const Eigen::Vector3d parentLinearVelocityChange
      = parentVelocityChange.tail<3>();

  auto phase = makePointMassPhaseView(
      mPointMasses, mAspectState.mPointStates, mAspectProperties.mPointProps);
  const bool adaptiveEnabled = isAdaptiveContactActivationEnabled();
  if (!adaptiveEnabled) {
    for (std::size_t i = 0; i < phase.size(); ++i) {
      PointMass& pointMass = *phase.pointMasses[i];
      const Eigen::Vector3d parentPointVelocityChange
          = parentAngularVelocityChange.cross(pointMass.mX)
            + parentLinearVelocityChange;
      pointMass.mVelocityChanges
          = pointMass.mPsi * pointMass.mImpAlpha - parentPointVelocityChange;
      DART_ASSERT(!math::isNan(pointMass.mVelocityChanges));

      pointMass.mDelV = parentPointVelocityChange + pointMass.mVelocityChanges;
      DART_ASSERT(!math::isNan(pointMass.mDelV));
    }
  } else {
    const auto& activation = adaptiveContactActivation(*this);
    for (std::size_t i = 0; i < phase.size(); ++i) {
      PointMass& pointMass = *phase.pointMasses[i];
      const Eigen::Vector3d& localPosition
          = activation.isActive(i) ? pointMass.mX
                                   : activation.mFrozenLocalPositions[i];
      const Eigen::Vector3d parentPointVelocityChange
          = parentAngularVelocityChange.cross(localPosition)
            + parentLinearVelocityChange;
      if (activation.isActive(i)) {
        pointMass.mVelocityChanges
            = pointMass.mPsi * pointMass.mImpAlpha - parentPointVelocityChange;
        pointMass.mDelV
            = parentPointVelocityChange + pointMass.mVelocityChanges;
      } else {
        pointMass.mVelocityChanges.setZero();
        pointMass.mDelV = parentPointVelocityChange;
      }
      DART_ASSERT(!math::isNan(pointMass.mVelocityChanges));
      DART_ASSERT(!math::isNan(pointMass.mDelV));
    }
  }
}

//==============================================================================
void SoftBodyNode::updateTransmittedImpulse()
{
  BodyNode::updateTransmittedImpulse();

  auto phase = makePointMassPhaseView(
      mPointMasses, mAspectState.mPointStates, mAspectProperties.mPointProps);

  for (std::size_t i = 0; i < phase.size(); ++i) {
    PointMass& pointMass = *phase.pointMasses[i];
    pointMass.mImpF = pointMass.mImpB;
    pointMass.mImpF.noalias() += phase.properties[i].mMass * pointMass.mDelV;
    DART_ASSERT(!math::isNan(pointMass.mImpF));
  }
}

//==============================================================================
void SoftBodyNode::updateConstrainedTerms(double _timeStep)
{
  BodyNode::updateConstrainedTerms(_timeStep);

  DART_ASSERT(_timeStep > 0.0);
  auto phase = makePointMassPhaseView(
      mPointMasses, mAspectState.mPointStates, mAspectProperties.mPointProps);

  const double invTimeStep = 1.0 / _timeStep;
  const bool adaptiveEnabled = isAdaptiveContactActivationEnabled();
  if (!adaptiveEnabled) {
    for (std::size_t i = 0; i < phase.size(); ++i) {
      PointMass& pointMass = *phase.pointMasses[i];
      PointMass::State& state = phase.states[i];

      state.mVelocities += pointMass.mVelocityChanges;
      state.mAccelerations.noalias()
          += invTimeStep * (pointMass.mVelocityChanges + pointMass.mDelV);
      state.mForces.noalias() += invTimeStep * pointMass.mConstraintImpulses;
      pointMass.mF.noalias() += _timeStep * pointMass.mImpF;

      pointMass.mNotifier->dirtyVelocity();
      pointMass.mNotifier->dirtyAcceleration();
    }
  } else {
    const auto& activation = adaptiveContactActivation(*this);
    for (std::size_t i = 0; i < phase.size(); ++i) {
      PointMass& pointMass = *phase.pointMasses[i];
      PointMass::State& state = phase.states[i];

      if (activation.isActive(i)) {
        state.mVelocities += pointMass.mVelocityChanges;
        state.mAccelerations.noalias()
            += invTimeStep * (pointMass.mVelocityChanges + pointMass.mDelV);
        state.mForces.noalias() += invTimeStep * pointMass.mConstraintImpulses;
        pointMass.mF.noalias() += _timeStep * pointMass.mImpF;
      } else {
        state.mPositions = activation.mFrozenLocalPositions[i]
                           - mAspectProperties.mPointProps[i].mX0;
        state.mVelocities.setZero();
        state.mAccelerations.setZero();
        state.mForces.setZero();
      }

      pointMass.mNotifier->dirtyVelocity();
      pointMass.mNotifier->dirtyAcceleration();
    }
  }
}

//==============================================================================
void SoftBodyNode::updateMassMatrix()
{
  BodyNode::updateMassMatrix();

  if (mNotifier->needsTransformUpdate())
    updateTransform();

  auto phase = makePointMassPhaseView(
      mPointMasses, mAspectState.mPointStates, mAspectProperties.mPointProps);
  for (std::size_t i = 0; i < phase.size(); ++i) {
    PointMass& pointMass = *phase.pointMasses[i];
    pointMass.mM_dV = phase.states[i].mAccelerations
                      + mM_dV.head<3>().cross(pointMass.mX) + mM_dV.tail<3>();
    DART_ASSERT(!math::isNan(pointMass.mM_dV));
  }
}

//==============================================================================
void SoftBodyNode::aggregateMassMatrix(Eigen::MatrixXd& _MCol, std::size_t _col)
{
  if (mNotifier->needsTransformUpdate())
    updateTransform();

  //----------------------- SoftBodyNode Part ----------------------------------
  const Eigen::Matrix6d& mI
      = BodyNode::mAspectProperties.mInertia.getSpatialTensor();
  mM_F.noalias() = mI * mM_dV;
  DART_ASSERT(!math::isNan(mM_F));

  for (std::vector<BodyNode*>::const_iterator it = mChildBodyNodes.begin();
       it != mChildBodyNodes.end();
       ++it) {
    mM_F += math::dAdInvT(
        (*it)->getParentJoint()->getRelativeTransform(), (*it)->mM_F);
  }

  //------------------------ PointMass Part ------------------------------------
  auto phase = makePointMassPhaseView(
      mPointMasses, mAspectState.mPointStates, mAspectProperties.mPointProps);
  for (std::size_t i = 0; i < phase.size(); ++i) {
    PointMass& pointMass = *phase.pointMasses[i];
    pointMass.mM_F.noalias() = phase.properties[i].mMass * pointMass.mM_dV;
    DART_ASSERT(!math::isNan(pointMass.mM_F));
    addPointForceContribution(mM_F, pointMass.mX, pointMass.mM_F);
  }

  DART_ASSERT(!math::isNan(mM_F));

  const std::size_t dof = mParentJoint->getNumDofs();
  if (dof > 0) {
    const std::size_t iStart = mParentJoint->getIndexInTree(0);
    _MCol.block(iStart, _col, dof, 1).noalias()
        = mParentJoint->getRelativeJacobian().transpose() * mM_F;
  }
}

//==============================================================================
void SoftBodyNode::aggregateAugMassMatrix(
    Eigen::MatrixXd& _MCol, std::size_t _col, double _timeStep)
{
  // TODO(JS): Need to be reimplemented
  if (mNotifier->needsTransformUpdate())
    updateTransform();

  const Eigen::Matrix6d& mI
      = BodyNode::mAspectProperties.mInertia.getSpatialTensor();

  //----------------------- SoftBodyNode Part ----------------------------------
  mM_F.noalias() = mI * mM_dV;
  DART_ASSERT(!math::isNan(mM_F));

  for (std::vector<BodyNode*>::const_iterator it = mChildBodyNodes.begin();
       it != mChildBodyNodes.end();
       ++it) {
    mM_F += math::dAdInvT(
        (*it)->getParentJoint()->getRelativeTransform(), (*it)->mM_F);
  }

  //------------------------ PointMass Part ------------------------------------
  auto phase = makePointMassPhaseView(
      mPointMasses, mAspectState.mPointStates, mAspectProperties.mPointProps);
  for (std::size_t i = 0; i < phase.size(); ++i) {
    PointMass& pointMass = *phase.pointMasses[i];
    pointMass.mM_F.noalias() = phase.properties[i].mMass * pointMass.mM_dV;
    DART_ASSERT(!math::isNan(pointMass.mM_F));
    addPointForceContribution(mM_F, pointMass.mX, pointMass.mM_F);
  }
  DART_ASSERT(!math::isNan(mM_F));

  std::size_t dof = mParentJoint->getNumDofs();
  if (dof > 0) {
    const std::size_t iStart = mParentJoint->getIndexInTree(0);

    // TODO(JS): Revisit the augmented-mass spring/damping formulation.
    auto segment = _MCol.block(iStart, _col, dof, 1);
    segment.noalias() = mParentJoint->getRelativeJacobian().transpose() * mM_F;

    const double timeStepSquared = _timeStep * _timeStep;
    for (std::size_t i = 0; i < dof; ++i) {
      segment(i, 0) += (_timeStep * mParentJoint->getDampingCoefficient(i)
                        + timeStepSquared * mParentJoint->getSpringStiffness(i))
                       * mParentJoint->getAcceleration(i);
    }
  }
}

//==============================================================================
void SoftBodyNode::updateInvMassMatrix()
{
  //----------------------- SoftBodyNode Part ----------------------------------
  //
  mInvM_c.setZero();

  //
  for (std::vector<BodyNode*>::const_iterator it = mChildBodyNodes.begin();
       it != mChildBodyNodes.end();
       ++it) {
    (*it)->getParentJoint()->addChildBiasForceForInvMassMatrix(
        mInvM_c, (*it)->getArticulatedInertia(), (*it)->mInvM_c);
  }

  //------------------------ PointMass Part ------------------------------------
  if (mNotifier->needsTransformUpdate())
    updateTransform();

  auto phase = makePointMassPhaseView(
      mPointMasses, mAspectState.mPointStates, mAspectProperties.mPointProps);
  for (std::size_t i = 0; i < phase.size(); ++i) {
    PointMass& pointMass = *phase.pointMasses[i];
    pointMass.mBiasForceForInvMeta = phase.states[i].mForces;
    addPointForceContribution(
        mInvM_c, pointMass.mX, pointMass.mBiasForceForInvMeta);
  }

  // Verification
  DART_ASSERT(!math::isNan(mInvM_c));

  // Update parent joint's total force for inverse mass matrix
  mParentJoint->updateTotalForceForInvMassMatrix(mInvM_c);
}

//==============================================================================
void SoftBodyNode::updateInvAugMassMatrix()
{
  BodyNode::updateInvAugMassMatrix();

  //  //------------------------ PointMass Part
  //  ------------------------------------
  ////  for (std::size_t i = 0; i < mPointMasses.size(); ++i)
  ////    mPointMasses.at(i)->updateInvAugMassMatrix();

  //  //----------------------- SoftBodyNode Part
  //  ----------------------------------
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
  ////    mInvM_c.head<3>() +=
  ///(*it)->getLocalPosition().cross((*it)->mBiasForceForInvMeta); /
  /// mInvM_c.tail<3>() += (*it)->mBiasForceForInvMeta; /  }

  //  // Verification
  //  DART_ASSERT(!math::isNan(mInvM_c));

  //  // Update parent joint's total force for inverse mass matrix
  //  mParentJoint->updateTotalForceForInvMassMatrix(mInvM_c);
}

//==============================================================================
void SoftBodyNode::aggregateInvMassMatrix(
    Eigen::MatrixXd& _InvMCol, std::size_t _col)
{
  if (mParentBodyNode) {
    //
    mParentJoint->getInvMassMatrixSegment(
        _InvMCol, _col, getArticulatedInertia(), mParentBodyNode->mInvM_U);

    //
    mInvM_U = math::AdInvT(
        mParentJoint->getRelativeTransform(), mParentBodyNode->mInvM_U);
  } else {
    //
    mParentJoint->getInvMassMatrixSegment(
        _InvMCol, _col, getArticulatedInertia(), Eigen::Vector6d::Zero());

    //
    mInvM_U.setZero();
  }

  //
  mParentJoint->addInvMassMatrixSegmentTo(mInvM_U);

  // PointMass::aggregateInvMassMatrix() is intentionally a no-op in DART 6.
  // Preserve that public behavior without dispatching once per point mass.
}

//==============================================================================
void SoftBodyNode::aggregateInvAugMassMatrix(
    Eigen::MatrixXd& _InvMCol, std::size_t _col, double _timeStep)
{
  BodyNode::aggregateInvAugMassMatrix(_InvMCol, _col, _timeStep);

  //  if (mParentBodyNode)
  //  {
  //    //
  //    mParentJoint->getInvAugMassMatrixSegment(
  //          *_InvMCol, _col, getArticulatedInertiaImplicit(),
  //          mParentBodyNode->mInvM_U);

  //    //
  //    mInvM_U = math::AdInvT(mParentJoint->mT, mParentBodyNode->mInvM_U);
  //  }
  //  else
  //  {
  //    //
  //    mParentJoint->getInvAugMassMatrixSegment(
  //          *_InvMCol, _col, getArticulatedInertiaImplicit(),
  //          Eigen::Vector6d::Zero());

  //    //
  //    mInvM_U.setZero();
  //  }

  //  //
  //  mParentJoint->addInvMassMatrixSegmentTo(mInvM_U);

  //  //
  ////  for (std::size_t i = 0; i < mPointMasses.size(); ++i)
  ////    mPointMasses.at(i)->aggregateInvAugMassMatrix(_InvMCol, _col,
  ///_timeStep);
}

//==============================================================================
void SoftBodyNode::aggregateCoriolisForceVector(Eigen::VectorXd& _C)
{
  BodyNode::aggregateCoriolisForceVector(_C);
}

//==============================================================================
void SoftBodyNode::aggregateGravityForceVector(
    Eigen::VectorXd& _g, const Eigen::Vector3d& _gravity)
{
  const Eigen::Matrix6d& mI
      = BodyNode::mAspectProperties.mInertia.getSpatialTensor();
  const bool gravityMode = BodyNode::mAspectProperties.mGravityMode;
  Eigen::Vector3d localGravity = Eigen::Vector3d::Zero();
  if (gravityMode)
    localGravity = getWorldTransform().linear().transpose() * _gravity;

  //----------------------- SoftBodyNode Part ----------------------------------
  if (gravityMode)
    mG_F = mI * math::AdInvRLinear(getWorldTransform(), _gravity);
  else
    mG_F.setZero();

  for (std::vector<BodyNode*>::const_iterator it = mChildBodyNodes.begin();
       it != mChildBodyNodes.end();
       ++it) {
    mG_F += math::dAdInvT(
        (*it)->mParentJoint->getRelativeTransform(), (*it)->mG_F);
  }

  //------------------------ PointMass Part ------------------------------------
  if (mNotifier->needsTransformUpdate())
    updateTransform();

  auto phase = makePointMassPhaseView(
      mPointMasses, mAspectState.mPointStates, mAspectProperties.mPointProps);
  for (std::size_t i = 0; i < phase.size(); ++i) {
    PointMass& pointMass = *phase.pointMasses[i];
    if (gravityMode)
      pointMass.mG_F.noalias() = phase.properties[i].mMass * localGravity;
    else
      pointMass.mG_F.setZero();
    DART_ASSERT(!math::isNan(pointMass.mG_F));
    addPointForceContribution(mG_F, pointMass.mX, pointMass.mG_F);
  }

  const std::size_t nGenCoords = mParentJoint->getNumDofs();
  if (nGenCoords > 0) {
    const std::size_t iStart = mParentJoint->getIndexInTree(0);
    auto segment = _g.segment(iStart, nGenCoords);
    segment.noalias() = mParentJoint->getRelativeJacobian().transpose() * mG_F;
    segment = -segment;
  }
}

//==============================================================================
void SoftBodyNode::updateCombinedVector()
{
  BodyNode::updateCombinedVector();

  if (mNotifier->needsPartialAccelerationUpdate())
    updatePartialAcceleration();
  if (mNotifier->needsTransformUpdate())
    updateTransform();

  auto phase = makePointMassPhaseView(
      mPointMasses, mAspectState.mPointStates, mAspectProperties.mPointProps);
  for (std::size_t i = 0; i < phase.size(); ++i) {
    PointMass& pointMass = *phase.pointMasses[i];
    pointMass.mCg_dV = pointMass.mEta + mCg_dV.head<3>().cross(pointMass.mX)
                       + mCg_dV.tail<3>();
  }
}

//==============================================================================
void SoftBodyNode::aggregateCombinedVector(
    Eigen::VectorXd& _Cg, const Eigen::Vector3d& _gravity)
{
  //------------------------ PointMass Part ------------------------------------
  const bool gravityMode = BodyNode::mAspectProperties.mGravityMode;
  Eigen::Vector3d localGravity = Eigen::Vector3d::Zero();
  if (gravityMode)
    localGravity = getWorldTransform().linear().transpose() * _gravity;
  const Eigen::Vector3d parentAngularVelocity = getSpatialVelocity().head<3>();

  //----------------------- SoftBodyNode Part ----------------------------------
  const Eigen::Matrix6d& mI
      = BodyNode::mAspectProperties.mInertia.getSpatialTensor();

  if (gravityMode)
    mFgravity = mI * math::AdInvRLinear(getWorldTransform(), _gravity);
  else
    mFgravity.setZero();

  const Eigen::Vector6d& V = getSpatialVelocity();
  mCg_F = mI * mCg_dV;
  mCg_F -= mFgravity;
  mCg_F -= math::dad(V, mI * V);

  for (std::vector<BodyNode*>::iterator it = mChildBodyNodes.begin();
       it != mChildBodyNodes.end();
       ++it) {
    mCg_F += math::dAdInvT(
        (*it)->getParentJoint()->getRelativeTransform(), (*it)->mCg_F);
  }

  //------------------------ PointMass Part ------------------------------------
  if (mNotifier->needsTransformUpdate())
    updateTransform();

  auto phase = makePointMassPhaseView(
      mPointMasses, mAspectState.mPointStates, mAspectProperties.mPointProps);
  for (std::size_t i = 0; i < phase.size(); ++i) {
    PointMass& pointMass = *phase.pointMasses[i];
    const double mass = phase.properties[i].mMass;
    pointMass.mCg_F.noalias() = mass * pointMass.mCg_dV;
    if (gravityMode)
      pointMass.mCg_F.noalias() -= mass * localGravity;
    pointMass.mCg_F.noalias()
        += parentAngularVelocity.cross(mass * pointMass.mV);
    DART_ASSERT(!math::isNan(pointMass.mCg_F));
    addPointForceContribution(mCg_F, pointMass.mX, pointMass.mCg_F);
  }

  const std::size_t nGenCoords = mParentJoint->getNumDofs();
  if (nGenCoords > 0) {
    const std::size_t iStart = mParentJoint->getIndexInTree(0);
    _Cg.segment(iStart, nGenCoords).noalias()
        = mParentJoint->getRelativeJacobian().transpose() * mCg_F;
  }
}

//==============================================================================
void SoftBodyNode::aggregateExternalForces(Eigen::VectorXd& _Fext)
{
  //----------------------- SoftBodyNode Part ----------------------------------
  mFext_F = BodyNode::mAspectState.mFext;

  for (std::vector<BodyNode*>::const_iterator it = mChildBodyNodes.begin();
       it != mChildBodyNodes.end();
       ++it) {
    mFext_F += math::dAdInvT(
        (*it)->mParentJoint->getRelativeTransform(), (*it)->mFext_F);
  }

  if (mNotifier->needsTransformUpdate())
    updateTransform();

  auto phase = makePointMassPhaseView(
      mPointMasses, mAspectState.mPointStates, mAspectProperties.mPointProps);
  for (std::size_t i = 0; i < phase.size(); ++i) {
    const PointMass& pointMass = *phase.pointMasses[i];
    mFext_F.head<3>() += pointMass.mX.cross(pointMass.mFext);
    mFext_F.tail<3>() += pointMass.mFext;
  }

  const std::size_t nGenCoords = mParentJoint->getNumDofs();
  if (nGenCoords > 0) {
    const std::size_t iStart = mParentJoint->getIndexInTree(0);
    _Fext.segment(iStart, nGenCoords).noalias()
        = mParentJoint->getRelativeJacobian().transpose() * mFext_F;
  }
}

//==============================================================================
void SoftBodyNode::clearExternalForces()
{
  BodyNode::clearExternalForces();

  bool hadExternalPointForce = false;
  for (auto* pointMass : mPointMasses) {
    hadExternalPointForce
        = hadExternalPointForce || !pointMass->mFext.isZero(0.0);
    pointMass->mFext.setZero();
  }
  if (hadExternalPointForce)
    dirtyExternalForces();
}

//==============================================================================
void SoftBodyNode::clearInternalForces()
{
  BodyNode::clearInternalForces();

  for (auto& pointState : mAspectState.mPointStates)
    pointState.mForces.setZero();
}

//==============================================================================
void SoftBodyNode::scanPointMassExternalForces(
    double _tolerance, bool& _hasResidual, bool& _disturbed) const
{
  for (const auto* pointMass : mPointMasses) {
    const auto& externalForce = pointMass->mFext;
    if (!externalForce.isZero(0.0))
      _hasResidual = true;
    if (externalForce.cwiseAbs().maxCoeff() > _tolerance)
      _disturbed = true;
  }
}

//==============================================================================
void SoftBodyNode::_addPiToArtInertia(
    const Eigen::Vector3d& _p, double _Pi) const
{
  Eigen::Matrix3d tmp = math::makeSkewSymmetric(_p);

  mArtInertia.topLeftCorner<3, 3>() -= _Pi * tmp * tmp;
  mArtInertia.topRightCorner<3, 3>() += _Pi * tmp;
  mArtInertia.bottomLeftCorner<3, 3>() -= _Pi * tmp;

  mArtInertia(3, 3) += _Pi;
  mArtInertia(4, 4) += _Pi;
  mArtInertia(5, 5) += _Pi;
}

//==============================================================================
void SoftBodyNode::_addPiToArtInertiaImplicit(
    const Eigen::Vector3d& _p, double _ImplicitPi) const
{
  Eigen::Matrix3d tmp = math::makeSkewSymmetric(_p);

  mArtInertiaImplicit.topLeftCorner<3, 3>() -= _ImplicitPi * tmp * tmp;
  mArtInertiaImplicit.topRightCorner<3, 3>() += _ImplicitPi * tmp;
  mArtInertiaImplicit.bottomLeftCorner<3, 3>() -= _ImplicitPi * tmp;

  mArtInertiaImplicit(3, 3) += _ImplicitPi;
  mArtInertiaImplicit(4, 4) += _ImplicitPi;
  mArtInertiaImplicit(5, 5) += _ImplicitPi;
}

//==============================================================================
void SoftBodyNode::updateInertiaWithPointMass()
{
  // TODO(JS): Not implemented

  const Eigen::Matrix6d& mI
      = BodyNode::mAspectProperties.mInertia.getSpatialTensor();
  mI2 = mI;

  for (std::size_t i = 0; i < mPointMasses.size(); ++i) {
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
  std::vector<Eigen::Vector3d> restingPos(
      nPointMasses, Eigen::Vector3d::Zero());
  restingPos[0] = _size.cwiseProduct(Eigen::Vector3d(-1.0, -1.0, -1.0)) * 0.5;
  restingPos[1] = _size.cwiseProduct(Eigen::Vector3d(+1.0, -1.0, -1.0)) * 0.5;
  restingPos[2] = _size.cwiseProduct(Eigen::Vector3d(-1.0, +1.0, -1.0)) * 0.5;
  restingPos[3] = _size.cwiseProduct(Eigen::Vector3d(+1.0, +1.0, -1.0)) * 0.5;
  restingPos[4] = _size.cwiseProduct(Eigen::Vector3d(-1.0, -1.0, +1.0)) * 0.5;
  restingPos[5] = _size.cwiseProduct(Eigen::Vector3d(+1.0, -1.0, +1.0)) * 0.5;
  restingPos[6] = _size.cwiseProduct(Eigen::Vector3d(-1.0, +1.0, +1.0)) * 0.5;
  restingPos[7] = _size.cwiseProduct(Eigen::Vector3d(+1.0, +1.0, +1.0)) * 0.5;

  // Point masses
  for (std::size_t i = 0; i < nPointMasses; ++i) {
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
  properties.addFace(Eigen::Vector3i(1, 0, 2)); // 0
  properties.addFace(Eigen::Vector3i(1, 2, 3)); // 1

  // -- +Z
  properties.addFace(Eigen::Vector3i(5, 6, 4)); // 2
  properties.addFace(Eigen::Vector3i(5, 7, 6)); // 3

  // -- -Y
  properties.addFace(Eigen::Vector3i(0, 5, 4)); // 4
  properties.addFace(Eigen::Vector3i(0, 1, 5)); // 5

  // -- +Y
  properties.addFace(Eigen::Vector3i(1, 3, 7)); // 6
  properties.addFace(Eigen::Vector3i(1, 7, 5)); // 7

  // -- -X
  properties.addFace(Eigen::Vector3i(3, 2, 6)); // 8
  properties.addFace(Eigen::Vector3i(3, 6, 7)); // 9

  // -- +X
  properties.addFace(Eigen::Vector3i(2, 0, 4)); // 10
  properties.addFace(Eigen::Vector3i(2, 4, 6)); // 11

  return properties;
}

//==============================================================================
void SoftBodyNodeHelper::setBox(
    SoftBodyNode* _softBodyNode,
    const Eigen::Vector3d& _size,
    const Eigen::Isometry3d& _localTransform,
    double _totalMass,
    double _vertexStiffness,
    double _edgeStiffness,
    double _dampingCoeff)
{
  DART_ASSERT(_softBodyNode != nullptr);
  _softBodyNode->setProperties(makeBoxProperties(
      _size,
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

  for (int i = 0; i < 3; ++i) {
    if (frags[i] <= 2) {
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
  DART_ASSERT(frags[0] > 1 && frags[1] > 1 && frags[2] > 1);

  std::size_t nCorners = 8;
  std::size_t nVerticesAtEdgeX = frags[0] - 2;
  std::size_t nVerticesAtEdgeY = frags[1] - 2;
  std::size_t nVerticesAtEdgeZ = frags[2] - 2;
  std::size_t nVerticesAtSideX = nVerticesAtEdgeY * nVerticesAtEdgeZ;
  std::size_t nVerticesAtSideY = nVerticesAtEdgeZ * nVerticesAtEdgeX;
  std::size_t nVerticesAtSideZ = nVerticesAtEdgeX * nVerticesAtEdgeY;
  std::size_t nVertices
      = nCorners + 4 * (nVerticesAtEdgeX + nVerticesAtEdgeY + nVerticesAtEdgeZ)
        + 2 * (nVerticesAtSideX + nVerticesAtSideY + nVerticesAtSideZ);

  // Mass per vertices
  double mass = _totalMass / nVertices;

  Eigen::Vector3d segLength(
      _size[0] / (frags[0] - 1),
      _size[1] / (frags[1] - 1),
      _size[2] / (frags[2] - 1));

  typedef std::pair<PointMass::Properties, std::size_t> PointPair;

  std::vector<PointPair> corners(nCorners);

  std::vector<std::vector<PointPair>> edgeX(
      4, std::vector<PointPair>(nVerticesAtEdgeX));
  std::vector<std::vector<PointPair>> edgeY(
      4, std::vector<PointPair>(nVerticesAtEdgeY));
  std::vector<std::vector<PointPair>> edgeZ(
      4, std::vector<PointPair>(nVerticesAtEdgeZ));

  std::vector<std::vector<PointPair>> sideXNeg(
      nVerticesAtEdgeY, std::vector<PointPair>(nVerticesAtEdgeZ));
  std::vector<std::vector<PointPair>> sideXPos(
      nVerticesAtEdgeY, std::vector<PointPair>(nVerticesAtEdgeZ));

  std::vector<std::vector<PointPair>> sideYNeg(
      nVerticesAtEdgeZ, std::vector<PointPair>(nVerticesAtEdgeX));
  std::vector<std::vector<PointPair>> sideYPos(
      nVerticesAtEdgeZ, std::vector<PointPair>(nVerticesAtEdgeX));

  std::vector<std::vector<PointPair>> sideZNeg(
      nVerticesAtEdgeX, std::vector<PointPair>(nVerticesAtEdgeY));
  std::vector<std::vector<PointPair>> sideZPos(
      nVerticesAtEdgeX, std::vector<PointPair>(nVerticesAtEdgeY));

  Eigen::Vector3d x0y0z0
      = _size.cwiseProduct(Eigen::Vector3d(-1.0, -1.0, -1.0)) * 0.5;
  Eigen::Vector3d x1y0z0
      = _size.cwiseProduct(Eigen::Vector3d(+1.0, -1.0, -1.0)) * 0.5;
  Eigen::Vector3d x1y1z0
      = _size.cwiseProduct(Eigen::Vector3d(+1.0, +1.0, -1.0)) * 0.5;
  Eigen::Vector3d x0y1z0
      = _size.cwiseProduct(Eigen::Vector3d(-1.0, +1.0, -1.0)) * 0.5;
  Eigen::Vector3d x0y0z1
      = _size.cwiseProduct(Eigen::Vector3d(-1.0, -1.0, +1.0)) * 0.5;
  Eigen::Vector3d x1y0z1
      = _size.cwiseProduct(Eigen::Vector3d(+1.0, -1.0, +1.0)) * 0.5;
  Eigen::Vector3d x1y1z1
      = _size.cwiseProduct(Eigen::Vector3d(+1.0, +1.0, +1.0)) * 0.5;
  Eigen::Vector3d x0y1z1
      = _size.cwiseProduct(Eigen::Vector3d(-1.0, +1.0, +1.0)) * 0.5;

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

  for (std::size_t i = 0; i < nCorners; ++i) {
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

  for (std::size_t i = 0; i < 4; ++i) {
    restPos = beginPts[i];

    for (std::size_t j = 0; j < nVerticesAtEdgeX; ++j) {
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

  for (std::size_t i = 0; i < 4; ++i) {
    restPos = beginPts[i];

    for (std::size_t j = 0; j < nVerticesAtEdgeY; ++j) {
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

  for (std::size_t i = 0; i < 4; ++i) {
    restPos = beginPts[i];

    for (std::size_t j = 0; j < nVerticesAtEdgeZ; ++j) {
      restPos[2] += segLength[2];

      edgeZ[i][j].first.setRestingPosition(_localTransform * restPos);
      edgeZ[i][j].first.setMass(mass);
      properties.addPointMass(edgeZ[i][j].first);

      edgeZ[i][j].second = id++;
    }
  }

  // Negative X side
  restPos = x0y0z0;

  for (std::size_t i = 0; i < nVerticesAtEdgeY; ++i) {
    restPos[2] = x0y0z0[2];
    restPos[1] += segLength[1];

    for (std::size_t j = 0; j < nVerticesAtEdgeZ; ++j) {
      restPos[2] += segLength[2];

      sideXNeg[i][j].first.setRestingPosition(_localTransform * restPos);
      sideXNeg[i][j].first.setMass(mass);
      properties.addPointMass(sideXNeg[i][j].first);

      sideXNeg[i][j].second = id++;
    }
  }

  // Positive X side
  restPos = x1y0z0;

  for (std::size_t i = 0; i < nVerticesAtEdgeY; ++i) {
    restPos[2] = x1y0z0[2];
    restPos[1] += segLength[1];

    for (std::size_t j = 0; j < nVerticesAtEdgeZ; ++j) {
      restPos[2] += segLength[2];

      sideXPos[i][j].first.setRestingPosition(_localTransform * restPos);
      sideXPos[i][j].first.setMass(mass);
      properties.addPointMass(sideXPos[i][j].first);

      sideXPos[i][j].second = id++;
    }
  }

  // Negative Y side
  restPos = x0y0z0;

  for (std::size_t i = 0; i < nVerticesAtEdgeZ; ++i) {
    restPos[0] = x0y0z0[0];
    restPos[2] += segLength[2];

    for (std::size_t j = 0; j < nVerticesAtEdgeX; ++j) {
      restPos[0] += segLength[0];

      sideYNeg[i][j].first.setRestingPosition(_localTransform * restPos);
      sideYNeg[i][j].first.setMass(mass);
      properties.addPointMass(sideYNeg[i][j].first);

      sideYNeg[i][j].second = id++;
    }
  }

  // Positive Y side
  restPos = x0y1z0;

  for (std::size_t i = 0; i < nVerticesAtEdgeZ; ++i) {
    restPos[0] = x0y1z0[0];
    restPos[2] += segLength[2];

    for (std::size_t j = 0; j < nVerticesAtEdgeX; ++j) {
      restPos[0] += segLength[0];

      sideYPos[i][j].first.setRestingPosition(_localTransform * restPos);
      sideYPos[i][j].first.setMass(mass);
      properties.addPointMass(sideYPos[i][j].first);

      sideYPos[i][j].second = id++;
    }
  }

  // Negative Z side
  restPos = x0y0z0;

  for (std::size_t i = 0; i < nVerticesAtEdgeX; ++i) {
    restPos[1] = x0y0z0[1];
    restPos[0] += segLength[0];

    for (std::size_t j = 0; j < nVerticesAtEdgeY; ++j) {
      restPos[1] += segLength[1];

      sideZNeg[i][j].first.setRestingPosition(_localTransform * restPos);
      sideZNeg[i][j].first.setMass(mass);
      properties.addPointMass(sideZNeg[i][j].first);

      sideZNeg[i][j].second = id++;
    }
  }

  // Positive Z side
  restPos = x0y0z1;

  for (std::size_t i = 0; i < nVerticesAtEdgeX; ++i) {
    restPos[1] = x0y0z1[1];
    restPos[0] += segLength[0];

    for (std::size_t j = 0; j < nVerticesAtEdgeY; ++j) {
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
  std::size_t nFacesX = 2 * (frags[1] - 1) * (frags[2] - 1);
  std::size_t nFacesY = 2 * (frags[2] - 1) * (frags[0] - 1);
  std::size_t nFacesZ = 2 * (frags[0] - 1) * (frags[1] - 1);
  std::size_t nFaces = 2 * nFacesX + 2 * nFacesY + 2 * nFacesZ;

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
  for (std::size_t i = 0; i < edgeX[0].size() - 1; ++i) {
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
  for (std::size_t i = 0; i < edgeX[1].size() - 1; ++i) {
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
  for (std::size_t i = 0; i < edgeX[2].size() - 1; ++i) {
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
  for (std::size_t i = 0; i < edgeX[3].size() - 1; ++i) {
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
  for (std::size_t i = 0; i < edgeY[0].size() - 1; ++i) {
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
  for (std::size_t i = 0; i < edgeY[1].size() - 1; ++i) {
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
  for (std::size_t i = 0; i < edgeY[2].size() - 1; ++i) {
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
  for (std::size_t i = 0; i < edgeY[3].size() - 1; ++i) {
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
  for (std::size_t i = 0; i < edgeZ[0].size() - 1; ++i) {
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
  for (std::size_t i = 0; i < edgeZ[1].size() - 1; ++i) {
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
  for (std::size_t i = 0; i < edgeZ[2].size() - 1; ++i) {
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
  for (std::size_t i = 0; i < edgeZ[3].size() - 1; ++i) {
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
  for (std::size_t i = 0; i < sideXNeg.size() - 1; ++i) {
    for (std::size_t j = 0; j < sideXNeg[i].size() - 1; ++j) {
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
  for (std::size_t i = 0; i < sideXPos.size() - 1; ++i) {
    for (std::size_t j = 0; j < sideXPos[i].size() - 1; ++j) {
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
  for (std::size_t i = 0; i < sideYNeg.size() - 1; ++i) {
    for (std::size_t j = 0; j < sideYNeg[i].size() - 1; ++j) {
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
  for (std::size_t i = 0; i < sideYPos.size() - 1; ++i) {
    for (std::size_t j = 0; j < sideYPos[i].size() - 1; ++j) {
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
  for (std::size_t i = 0; i < sideZNeg.size() - 1; ++i) {
    for (std::size_t j = 0; j < sideZNeg[i].size() - 1; ++j) {
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
  for (std::size_t i = 0; i < sideZPos.size() - 1; ++i) {
    for (std::size_t j = 0; j < sideZPos[i].size() - 1; ++j) {
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
void SoftBodyNodeHelper::setBox(
    SoftBodyNode* _softBodyNode,
    const Eigen::Vector3d& _size,
    const Eigen::Isometry3d& _localTransform,
    const Eigen::Vector3i& _frags,
    double _totalMass,
    double _vertexStiffness,
    double _edgeStiffness,
    double _dampingCoeff)
{
  DART_ASSERT(_softBodyNode != nullptr);
  _softBodyNode->setProperties(makeBoxProperties(
      _size,
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
  std::vector<Eigen::Vector3d> restingPos(
      nPointMasses, Eigen::Vector3d::Zero());
  restingPos[0] = Eigen::Vector3d(+0.1, +0.1, +0.1);

  // Point masses
  for (std::size_t i = 0; i < nPointMasses; ++i) {
    PointMass::Properties point(restingPos[i], mass);
    properties.addPointMass(point);
  }

  return properties;
}

//==============================================================================
void SoftBodyNodeHelper::setSinglePointMass(
    SoftBodyNode* _softBodyNode,
    double _totalMass,
    double _vertexStiffness,
    double _edgeStiffness,
    double _dampingCoeff)
{
  DART_ASSERT(_softBodyNode != nullptr);
  _softBodyNode->setProperties(makeSinglePointMassProperties(
      _totalMass, _vertexStiffness, _edgeStiffness, _dampingCoeff));
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
      Eigen::Vector3d::Constant(_radius * 2.0),
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

  SoftBodyNode::UniqueProperties properties(
      _vertexStiffness, _edgeStiffness, _dampingCoeff);

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
  for (std::size_t i = 1; i < _nStacks; i++) {
    float rho = i * drho;
    float srho = (sin(rho));
    float crho = (cos(rho));

    for (std::size_t j = 0; j < _nSlices; j++) {
      float theta = (j == _nSlices) ? 0.0f : j * dtheta;
      float stheta = (-sin(theta));
      float ctheta = (cos(theta));

      float x = 0.5 * srho * stheta;
      float y = 0.5 * srho * ctheta;
      float z = 0.5 * crho;

      properties.addPointMass(PointMass::Properties(
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
      properties.connectPointMasses(
          i * _nSlices + j + 1, (i + 1) * _nSlices + j + 1);
  // -- bottom
  for (std::size_t i = 0; i < _nSlices; i++)
    properties.connectPointMasses(
        (_nStacks - 1) * _nSlices + 1, (_nStacks - 2) * _nSlices + i + 1);

  // b) latitudinal
  for (std::size_t i = 0; i < _nStacks - 1; i++) {
    for (std::size_t j = 0; j < _nSlices - 1; j++) {
      properties.connectPointMasses(i * _nSlices + j + 1, i * _nSlices + j + 2);
    }
    properties.connectPointMasses((i + 1) * _nSlices, i * _nSlices + 1);
  }

  // c) cross (shear)
  for (std::size_t i = 0; i < _nStacks - 2; i++) {
    for (std::size_t j = 0; j < _nSlices - 1; j++) {
      properties.connectPointMasses(
          i * _nSlices + j + 1, (i + 1) * _nSlices + j + 2);
      properties.connectPointMasses(
          i * _nSlices + j + 2, (i + 1) * _nSlices + j + 1);
    }
    properties.connectPointMasses((i + 1) * _nSlices, (i + 1) * _nSlices + 1);
    properties.connectPointMasses(i * _nSlices + 1, (i + 2) * _nSlices);
  }

  //----------------------------------------------------------------------------
  // Faces
  //----------------------------------------------------------------------------
  int meshIdx1 = 0;
  int meshIdx2 = 0;
  int meshIdx3 = 0;

  // top
  meshIdx1 = 0;
  for (std::size_t i = 0; i < _nSlices - 1; i++) {
    meshIdx2 = i + 1;
    meshIdx3 = i + 2;
    properties.addFace(Eigen::Vector3i(meshIdx1, meshIdx2, meshIdx3));
  }
  meshIdx2 = _nSlices;
  meshIdx3 = 1;
  properties.addFace(Eigen::Vector3i(meshIdx1, meshIdx2, meshIdx3));

  // middle
  for (std::size_t i = 0; i < _nStacks - 2; i++) {
    for (std::size_t j = 0; j < _nSlices - 1; j++) {
      meshIdx1 = i * _nSlices + j + 1;
      meshIdx2 = (i + 1) * _nSlices + j + 1;
      meshIdx3 = i * _nSlices + j + 2;
      properties.addFace(Eigen::Vector3i(meshIdx1, meshIdx2, meshIdx3));

      meshIdx1 = i * _nSlices + j + 2;
      meshIdx2 = (i + 1) * _nSlices + j + 1;
      meshIdx3 = (i + 1) * _nSlices + j + 2;
      properties.addFace(Eigen::Vector3i(meshIdx1, meshIdx2, meshIdx3));
    }

    meshIdx1 = (i + 1) * _nSlices;
    meshIdx2 = (i + 2) * _nSlices;
    meshIdx3 = i * _nSlices + 1;
    properties.addFace(Eigen::Vector3i(meshIdx1, meshIdx2, meshIdx3));

    meshIdx1 = i * _nSlices + 1;
    meshIdx2 = (i + 2) * _nSlices;
    meshIdx3 = (i + 2) * _nSlices + 1;
    properties.addFace(Eigen::Vector3i(meshIdx1, meshIdx2, meshIdx3));
  }

  // bottom
  meshIdx1 = (_nStacks - 1) * _nSlices + 1;
  for (std::size_t i = 0; i < _nSlices - 1; i++) {
    meshIdx2 = (_nStacks - 2) * _nSlices + i + 2;
    meshIdx3 = (_nStacks - 2) * _nSlices + i + 1;
    properties.addFace(Eigen::Vector3i(meshIdx1, meshIdx2, meshIdx3));
  }
  meshIdx2 = (_nStacks - 2) * _nSlices + 2;
  meshIdx3 = (_nStacks - 1) * _nSlices;
  properties.addFace(Eigen::Vector3i(meshIdx1, meshIdx2, meshIdx3));

  return properties;
}

//==============================================================================
void SoftBodyNodeHelper::setEllipsoid(
    SoftBodyNode* _softBodyNode,
    const Eigen::Vector3d& _size,
    std::size_t _nSlices,
    std::size_t _nStacks,
    double _totalMass,
    double _vertexStiffness,
    double _edgeStiffness,
    double _dampingCoeff)
{
  DART_ASSERT(_softBodyNode != nullptr);
  _softBodyNode->setProperties(makeEllipsoidProperties(
      _size,
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

  SoftBodyNode::UniqueProperties properties(
      _vertexStiffness, _edgeStiffness, _dampingCoeff);

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
  properties.addPointMass(
      PointMass::Properties(Eigen::Vector3d(0.0, 0.0, 0.5 * _height), mass));

  for (std::size_t i = 1; i < _nRings; ++i) {
    float z = 0.5;
    float radius = i * dradius;

    for (std::size_t j = 0; j < _nSlices; j++) {
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
  float dz = -1.0 / static_cast<float>(_nStacks);
  for (std::size_t i = 0; i < _nStacks + 1; i++) {
    float z = 0.5 + i * dz;

    for (std::size_t j = 0; j < _nSlices; j++) {
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
  for (std::size_t i = 1; i < _nRings; ++i) {
    float z = -0.5;
    float radius = _radius - i * dradius;

    for (std::size_t j = 0; j < _nSlices; j++) {
      float theta = (j == _nSlices) ? 0.0f : j * dtheta;
      float stheta = (-sin(theta));
      float ctheta = (cos(theta));

      float x = stheta;
      float y = ctheta;

      properties.addPointMass(PointMass::Properties(
          Eigen::Vector3d(x * radius, y * radius, z * _height), mass));
    }
  }

  properties.addPointMass(
      PointMass::Properties(Eigen::Vector3d(0.0, 0.0, -0.5 * _height), mass));

  //----------------------------------------------------------------------------
  // Edges
  //----------------------------------------------------------------------------
  // A. Drum part

  // a) longitudinal
  // -- top
  for (std::size_t i = 0; i < _nSlices; i++)
    properties.connectPointMasses(0, i + 1);
  for (std::size_t i = 0; i < _nRings - 1; i++) {
    for (std::size_t j = 0; j < _nSlices; j++) {
      properties.connectPointMasses(
          _nSlices + 1 + (i + 0) * _nSlices + j,
          _nSlices + 1 + (i + 1) * _nSlices + j);
    }
  }
  // -- middle
  for (std::size_t i = 0; i < _nStacks - 1; i++) {
    for (std::size_t j = 0; j < _nSlices; j++) {
      properties.connectPointMasses(
          nTopPointMasses + (i + 0) * _nSlices + j,
          nTopPointMasses + (i + 1) * _nSlices + j);
    }
  }
  // -- bottom
  for (std::size_t i = 0; i < _nRings - 1; i++) {
    for (std::size_t j = 0; j < _nSlices; j++) {
      properties.connectPointMasses(
          nTopPointMasses + (nDrumPointMasses - _nSlices) + (i + 0) * _nSlices
              + j,
          nTopPointMasses + (nDrumPointMasses - _nSlices) + (i + 1) * _nSlices
              + j);
    }
  }
  for (std::size_t i = 1; i < _nSlices; i++)
    properties.connectPointMasses(nTotalMasses - 1 - i, nTotalMasses - 1);

  // b) latitudinal
  for (std::size_t i = 0; i < _nStacks; i++) {
    for (std::size_t j = 0; j < _nSlices - 1; j++) {
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
  for (std::size_t i = 0; i < _nStacks - 2; i++) {
    for (std::size_t j = 0; j < _nSlices - 1; j++) {
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
  for (std::size_t i = 0; i < _nSlices - 1; i++) {
    meshIdx2 = i + 1;
    meshIdx3 = i + 2;
    properties.addFace(Eigen::Vector3i(meshIdx1, meshIdx2, meshIdx3));
  }
  meshIdx2 = _nSlices;
  meshIdx3 = 1;
  properties.addFace(Eigen::Vector3i(meshIdx1, meshIdx2, meshIdx3));
  for (std::size_t i = 0; i < _nRings - 1; ++i) {
    for (std::size_t j = 0; j < _nSlices - 1; ++j) {
      meshIdx1 = (i + 0) * _nSlices + j;
      meshIdx2 = (i + 1) * _nSlices + j;
      meshIdx3 = (i + 0) * _nSlices + j + 1;
      properties.addFace(Eigen::Vector3i(
          nConePointMass + meshIdx1,
          nConePointMass + meshIdx2,
          nConePointMass + meshIdx3));

      meshIdx1 = (i + 0) * _nSlices + j + 1;
      meshIdx2 = (i + 1) * _nSlices + j;
      meshIdx3 = (i + 1) * _nSlices + j + 1;
      properties.addFace(Eigen::Vector3i(
          nConePointMass + meshIdx1,
          nConePointMass + meshIdx2,
          nConePointMass + meshIdx3));
    }

    meshIdx1 = (i + 0) * _nSlices + _nSlices - 1;
    meshIdx2 = (i + 1) * _nSlices + _nSlices - 1;
    meshIdx3 = (i + 0) * _nSlices + 0;
    properties.addFace(Eigen::Vector3i(
        nConePointMass + meshIdx1,
        nConePointMass + meshIdx2,
        nConePointMass + meshIdx3));

    meshIdx1 = (i + 0) * _nSlices + 0;
    meshIdx2 = (i + 1) * _nSlices + _nSlices - 1;
    meshIdx3 = (i + 1) * _nSlices + 0;
    properties.addFace(Eigen::Vector3i(
        nConePointMass + meshIdx1,
        nConePointMass + meshIdx2,
        nConePointMass + meshIdx3));
  }

  // middle
  for (std::size_t i = 0; i < _nStacks; i++) {
    for (std::size_t j = 0; j < _nSlices - 1; j++) {
      meshIdx1 = (i + 0) * _nSlices + j;
      meshIdx2 = (i + 1) * _nSlices + j;
      meshIdx3 = (i + 0) * _nSlices + j + 1;
      properties.addFace(Eigen::Vector3i(
          nTopPointMasses + meshIdx1,
          nTopPointMasses + meshIdx2,
          nTopPointMasses + meshIdx3));

      meshIdx1 = (i + 0) * _nSlices + j + 1;
      meshIdx2 = (i + 1) * _nSlices + j;
      meshIdx3 = (i + 1) * _nSlices + j + 1;
      properties.addFace(Eigen::Vector3i(
          nTopPointMasses + meshIdx1,
          nTopPointMasses + meshIdx2,
          nTopPointMasses + meshIdx3));
    }

    meshIdx1 = (i + 0) * _nSlices;
    meshIdx2 = (i + 0) * _nSlices + _nSlices - 1;
    meshIdx3 = (i + 1) * _nSlices;
    properties.addFace(Eigen::Vector3i(
        nTopPointMasses + meshIdx1,
        nTopPointMasses + meshIdx2,
        nTopPointMasses + meshIdx3));

    meshIdx1 = (i + 0) * _nSlices + _nSlices - 1;
    meshIdx2 = (i + 1) * _nSlices + _nSlices - 1;
    meshIdx3 = (i + 1) * _nSlices + 0;
    properties.addFace(Eigen::Vector3i(
        nTopPointMasses + meshIdx1,
        nTopPointMasses + meshIdx2,
        nTopPointMasses + meshIdx3));
  }

  // bottom
  for (std::size_t i = 0; i < _nRings - 1; ++i) {
    for (std::size_t j = 0; j < _nSlices - 1; ++j) {
      meshIdx1 = (i + 0) * _nSlices + j;
      meshIdx2 = (i + 1) * _nSlices + j;
      meshIdx3 = (i + 0) * _nSlices + j + 1;
      properties.addFace(Eigen::Vector3i(
          nTopPointMasses + (nDrumPointMasses - _nSlices) + meshIdx1,
          nTopPointMasses + (nDrumPointMasses - _nSlices) + meshIdx2,
          nTopPointMasses + (nDrumPointMasses - _nSlices) + meshIdx3));

      meshIdx1 = (i + 0) * _nSlices + j + 1;
      meshIdx2 = (i + 1) * _nSlices + j;
      meshIdx3 = (i + 1) * _nSlices + j + 1;
      properties.addFace(Eigen::Vector3i(
          nTopPointMasses + (nDrumPointMasses - _nSlices) + meshIdx1,
          nTopPointMasses + (nDrumPointMasses - _nSlices) + meshIdx2,
          nTopPointMasses + (nDrumPointMasses - _nSlices) + meshIdx3));
    }

    meshIdx1 = (i + 0) * _nSlices + _nSlices - 1;
    meshIdx2 = (i + 1) * _nSlices + _nSlices - 1;
    meshIdx3 = (i + 0) * _nSlices + 0;
    properties.addFace(Eigen::Vector3i(
        nTopPointMasses + (nDrumPointMasses - _nSlices) + meshIdx1,
        nTopPointMasses + (nDrumPointMasses - _nSlices) + meshIdx2,
        nTopPointMasses + (nDrumPointMasses - _nSlices) + meshIdx3));

    meshIdx1 = (i + 0) * _nSlices + 0;
    meshIdx2 = (i + 1) * _nSlices + _nSlices - 1;
    meshIdx3 = (i + 1) * _nSlices + 0;
    properties.addFace(Eigen::Vector3i(
        nTopPointMasses + (nDrumPointMasses - _nSlices) + meshIdx1,
        nTopPointMasses + (nDrumPointMasses - _nSlices) + meshIdx2,
        nTopPointMasses + (nDrumPointMasses - _nSlices) + meshIdx3));
  }
  meshIdx1 = 1;
  for (std::size_t i = 0; i < _nSlices - 1; i++) {
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
void SoftBodyNodeHelper::setCylinder(
    SoftBodyNode* _softBodyNode,
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
  DART_ASSERT(_softBodyNode != nullptr);
  _softBodyNode->setProperties(makeCylinderProperties(
      _radius,
      _height,
      _nSlices,
      _nStacks,
      _nRings,
      _totalMass,
      _vertexStiffness,
      _edgeStiffness,
      _dampingCoeff));
}

} // namespace dynamics
} // namespace dart
