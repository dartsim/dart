/*
 * Copyright (c) 2016, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
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

#include "dart/dynamics/SoftBodyAspect.hpp"

#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/SoftMeshShape.hpp"

namespace dart {
namespace dynamics {

//==============================================================================
SoftBodyAspect::SoftBodyAspect(
    common::Composite* composite,
    const StateData& state,
    const PropertiesData& properties)
  : AspectImpl(composite, state, properties)
{
  // Dev's Note: We do this workaround (instead of just using setProperties(~))
  // because mSoftShapeNode cannot be used until init(SkeletonPtr) has been
  // called on this BodyNode, but that happens after construction is finished.
  mState = state;
  mProperties = properties;
}

//==============================================================================
SoftBodyAspect::SoftBodyAspect(
    common::Composite* composite,
    const PropertiesData& properties,
    const StateData state)
  : AspectImpl(composite, properties, state)
{
  // Dev's Note: We do this workaround (instead of just using setProperties(~))
  // because mSoftShapeNode cannot be used until init(SkeletonPtr) has been
  // called on this BodyNode, but that happens after construction is finished.
  mState = state;
  mProperties = properties;
}

//==============================================================================
PointMassNotifier* SoftBodyAspect::getNotifier()
{
  return mNotifier.get();
}

//==============================================================================
const PointMassNotifier* SoftBodyAspect::getNotifier() const
{
  return mNotifier.get();
}

//==============================================================================
double SoftBodyAspect::getMass() const
{
  auto mass = 0.0;

  for (const auto& pointMass : mPointMasses)
    mass += pointMass->getMass();

  return mass;
}

//==============================================================================
void SoftBodyAspect::setVertexSpringStiffness(double kv)
{
  assert(0.0 <= kv);

  if (kv == mProperties.mKv)
    return;

  mProperties.mKv = kv;
  incrementVersion();
}

//==============================================================================
double SoftBodyAspect::getVertexSpringStiffness() const
{
  return mProperties.mKv;
}

//==============================================================================
void SoftBodyAspect::setEdgeSpringStiffness(double _ke)
{
  assert(0.0 <= _ke);

  if(_ke == mProperties.mKe)
    return;

  mProperties.mKe = _ke;
  incrementVersion();
}

//==============================================================================
double SoftBodyAspect::getEdgeSpringStiffness() const
{
  return mProperties.mKe;
}

//==============================================================================
void SoftBodyAspect::setDampingCoefficient(double _damp)
{
  assert(_damp >= 0.0);

  if(_damp == mProperties.mDampCoeff)
    return;

  mProperties.mDampCoeff = _damp;
  incrementVersion();
}

//==============================================================================
double SoftBodyAspect::getDampingCoefficient() const
{
  return mProperties.mDampCoeff;
}

//==============================================================================
void SoftBodyAspect::removeAllPointMasses()
{
  mPointMasses.clear();
  mProperties.mPointProps.clear();
  mProperties.mFaces.clear();
  configurePointMasses(mSoftShapeNode.lock());
}

//==============================================================================
PointMass* SoftBodyAspect::addPointMass(const PointMass::Properties& properties)
{
  mPointMasses.push_back(new PointMass(mComposite));
  mPointMasses.back()->mIndex = mPointMasses.size() - 1u;
  mProperties.mPointProps.push_back(properties);
  configurePointMasses(mSoftShapeNode.lock());

  return mPointMasses.back();
}

//==============================================================================
std::size_t SoftBodyAspect::getNumPointMasses() const
{
  return mPointMasses.size();
}

//==============================================================================
PointMass* SoftBodyAspect::getPointMass(std::size_t index)
{
  assert(index < mPointMasses.size());
  if(index < mPointMasses.size())
    return mPointMasses[index];

  return nullptr;
}

//==============================================================================
const PointMass* SoftBodyAspect::getPointMass(std::size_t index) const
{
  return const_cast<SoftBodyAspect*>(this)->getPointMass(index);
}

//==============================================================================
const std::vector<PointMass*>& SoftBodyAspect::getPointMasses() const
{
  return mPointMasses;
}

//==============================================================================
void SoftBodyAspect::connectPointMasses(std::size_t _idx1, std::size_t _idx2)
{
  assert(_idx1 != _idx2);
  assert(_idx1 < mPointMasses.size());
  assert(_idx2 < mPointMasses.size());
  mPointMasses[_idx1]->addConnectedPointMass(mPointMasses[_idx2]);
  mPointMasses[_idx2]->addConnectedPointMass(mPointMasses[_idx1]);
  // Version incremented in addConnectedPointMass
}

//==============================================================================
void SoftBodyAspect::addFace(const Eigen::Vector3i& _face)
{
  mProperties.addFace(_face);
  configurePointMasses(mSoftShapeNode.lock());
}

//==============================================================================
const Eigen::Vector3i& SoftBodyAspect::getFace(std::size_t _idx) const
{
  assert(_idx < mProperties.mFaces.size());
  return mProperties.mFaces[_idx];
}

//==============================================================================
std::size_t SoftBodyAspect::getNumFaces() const
{
  return mProperties.mFaces.size();
}

//==============================================================================
void SoftBodyAspect::clearConstraintImpulse()
{
  for (auto& pointMass : mPointMasses)
    pointMass->clearConstraintImpulse();
}

//==============================================================================
void SoftBodyAspect::setComposite(common::Composite* newComposite)
{
  common::CompositeTrackingAspect<CompositeType>::setComposite(newComposite);

  init();
}

//==============================================================================
void SoftBodyAspect::init()
{
  mNotifier.reset(new PointMassNotifier(
        mComposite, mComposite->getName() + "_PointMassNotifier"));

  ShapeNode* softNode = mComposite->createShapeNodeWith<
      VisualAspect, CollisionAspect, DynamicsAspect>(
        std::make_shared<SoftMeshShape>(this),
        mComposite->getName() + "_SoftMeshShape");
  mSoftShapeNode = softNode;

  configurePointMasses(softNode);
  mNotifier->notifyTransformUpdate();

  for (auto& pointMass : mPointMasses)
    pointMass->init();
}

//==============================================================================
void SoftBodyAspect::configurePointMasses(ShapeNode* softNode)
{
  std::size_t newCount = mProperties.mPointProps.size();
  std::size_t oldCount = mPointMasses.size();

  if (newCount == oldCount)
    return;

  // Adjust the number of PointMass objects since that has changed
  if (newCount < oldCount)
  {
    for (std::size_t i = newCount; i < oldCount; ++i)
      delete mPointMasses[i];

    mPointMasses.resize(newCount);
  }
  else if (oldCount < newCount)
  {
    mPointMasses.resize(newCount);

    for(std::size_t i = oldCount; i < newCount; ++i)
    {
      mPointMasses[i] = new PointMass(mComposite);
      mPointMasses[i]->mIndex = i;
      mPointMasses[i]->init();
    }
  }

  // Resize the number of States in the Aspect
  mState.mPointStates.resize(mProperties.mPointProps.size(),
                             PointMass::State());

  // Access the SoftMeshShape and reallocate its meshes
  if (softNode)
  {
    auto softShape
        = std::dynamic_pointer_cast<SoftMeshShape>(softNode->getShape());

    if (softShape)
      softShape->_buildMesh();
  }
  else
  {
    dtwarn << "[SoftBodyNode::configurePointMasses] The ShapeNode containing "
           << "the SoftMeshShape for the SoftBodyNode named ["
           << mComposite->getName()
           << "] (" << this << ") has been removed. The soft body features for "
           << "this SoftBodyNode cannot be used unless you recreate the "
           << "SoftMeshShape.\n";

    std::cout << "ShapeNodes: " << std::endl;
    for (auto i = 0u; i < mComposite->getNumShapeNodes(); ++i)
    {
      std::cout << "- " << i << ") " << mComposite->getShapeNode(i)->getName()
                << "\n";
    }
  }

  incrementVersion();
  mNotifier->notifyTransformUpdate();
}

//==============================================================================
void SoftBodyAspect::checkArticulatedInertiaUpdate() const
{
  const auto skel = mComposite->getSkeleton();

  if (skel && skel->mTreeCache[mComposite->mTreeIndex].mDirty.mArticulatedInertia)
    skel->updateArticulatedInertia(mComposite->mTreeIndex);
}

//==============================================================================
void SoftBodyAspect::addPiToArtInertia(
    const Eigen::Vector3d& p, double Pi) const
{
  Eigen::Matrix3d tmp = math::makeSkewSymmetric(p);

  mComposite->mArtInertia.topLeftCorner<3, 3>()    -= Pi * tmp * tmp;
  mComposite->mArtInertia.topRightCorner<3, 3>()   += Pi * tmp;
  mComposite->mArtInertia.bottomLeftCorner<3, 3>() -= Pi * tmp;

  mComposite->mArtInertia(3, 3) += Pi;
  mComposite->mArtInertia(4, 4) += Pi;
  mComposite->mArtInertia(5, 5) += Pi;
}

//==============================================================================
void SoftBodyAspect::addPiToArtInertiaImplicit(
    const Eigen::Vector3d& p, double ImplicitPi) const
{
  Eigen::Matrix3d tmp = math::makeSkewSymmetric(p);

  mComposite->mArtInertiaImplicit.topLeftCorner<3, 3>()    -= ImplicitPi * tmp * tmp;
  mComposite->mArtInertiaImplicit.topRightCorner<3, 3>()   += ImplicitPi * tmp;
  mComposite->mArtInertiaImplicit.bottomLeftCorner<3, 3>() -= ImplicitPi * tmp;

  mComposite->mArtInertiaImplicit(3, 3) += ImplicitPi;
  mComposite->mArtInertiaImplicit(4, 4) += ImplicitPi;
  mComposite->mArtInertiaImplicit(5, 5) += ImplicitPi;
}

//==============================================================================
SoftBodyAspect::PropertiesData SoftBodyNodeHelper::makeBoxProperties(
    const Eigen::Vector3d& _size,
    const Eigen::Isometry3d& _localTransform,
    double _totalMass,
    double _vertexStiffness,
    double _edgeStiffness,
    double _dampingCoeff)
{
  SoftBodyAspect::PropertiesData properties(
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
void SoftBodyNodeHelper::setBox(BodyNode*            _softBodyNode,
                                const Eigen::Vector3d&   _size,
                                const Eigen::Isometry3d& _localTransform,
                                double                   _totalMass,
                                double                   _vertexStiffness,
                                double                   _edgeStiffness,
                                double                   _dampingCoeff)
{
  assert(_softBodyNode != nullptr);

  _softBodyNode->getSoftBodyAspect()->setProperties(
        makeBoxProperties(_size,
                          _localTransform,
                          _totalMass,
                          _vertexStiffness,
                          _edgeStiffness,
                          _dampingCoeff));
}

//==============================================================================
SoftBodyAspect::PropertiesData SoftBodyNodeHelper::makeBoxProperties(
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

  SoftBodyAspect::PropertiesData properties(
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
void SoftBodyNodeHelper::setBox(BodyNode*            _softBodyNode,
                                const Eigen::Vector3d&   _size,
                                const Eigen::Isometry3d& _localTransform,
                                const Eigen::Vector3i&   _frags,
                                double                   _totalMass,
                                double                   _vertexStiffness,
                                double                   _edgeStiffness,
                                double                   _dampingCoeff)
{
  assert(_softBodyNode != nullptr);

  _softBodyNode->getSoftBodyAspect()->setProperties(
        makeBoxProperties(_size,
                          _localTransform,
                          _frags,
                          _totalMass,
                          _vertexStiffness,
                          _edgeStiffness,
                          _dampingCoeff));
}

//==============================================================================
SoftBodyAspect::PropertiesData
SoftBodyNodeHelper::makeSinglePointMassProperties(
    double _totalMass,
    double _vertexStiffness,
    double _edgeStiffness,
    double _dampingCoeff)
{
  SoftBodyAspect::PropertiesData properties(
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
void SoftBodyNodeHelper::setSinglePointMass(BodyNode* _softBodyNode,
                                        double _totalMass,
                                        double _vertexStiffness,
                                        double _edgeStiffness,
                                        double _dampingCoeff)
{
  assert(_softBodyNode != nullptr);

  _softBodyNode->getSoftBodyAspect()->setProperties(
        makeSinglePointMassProperties(_totalMass,
                                      _vertexStiffness,
                                      _edgeStiffness,
                                      _dampingCoeff));
}

//==============================================================================
SoftBodyAspect::PropertiesData SoftBodyNodeHelper::makeEllipsoidProperties(
    const Eigen::Vector3d& _size,
    std::size_t _nSlices,
    std::size_t _nStacks,
    double _totalMass,
    double _vertexStiffness,
    double _edgeStiffness,
    double _dampingCoeff)
{
  using namespace dart::math::suffixes;

  SoftBodyAspect::PropertiesData properties(_vertexStiffness,
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
void SoftBodyNodeHelper::setEllipsoid(BodyNode*          _softBodyNode,
                                      const Eigen::Vector3d& _size,
                                      std::size_t                 _nSlices,
                                      std::size_t                 _nStacks,
                                      double                 _totalMass,
                                      double                 _vertexStiffness,
                                      double                 _edgeStiffness,
                                      double                 _dampingCoeff)
{
  assert(_softBodyNode != nullptr);
  _softBodyNode->getSoftBodyAspect()->setProperties(
        makeEllipsoidProperties(_size,
                                _nSlices,
                                _nStacks,
                                _totalMass,
                                _vertexStiffness,
                                _edgeStiffness,
                                _dampingCoeff));
}

//==============================================================================
SoftBodyAspect::PropertiesData SoftBodyNodeHelper::makeCylinderProperties(
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

  SoftBodyAspect::PropertiesData properties(_vertexStiffness,
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
void SoftBodyNodeHelper::setCylinder(BodyNode* _softBodyNode,
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
  _softBodyNode->getSoftBodyAspect()->setProperties(
        makeCylinderProperties(_radius,
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
