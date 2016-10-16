/*
 * Copyright (c) 2015-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2015-2016, Humanoid Lab, Georgia Tech Research Corporation
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
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/SimpleFrame.hpp"
#include "dart/dynamics/SphereShape.hpp"
#include "dart/math/Helpers.hpp"

#include "dart/gui/osg/SupportPolygonVisual.hpp"

namespace dart {
namespace gui {
namespace osg {

//==============================================================================
SupportPolygonVisual::SupportPolygonVisual(const dart::dynamics::SkeletonPtr& skeleton,
                             double elevation)
  : mSkeleton(skeleton),
    mTreeIndex(dart::dynamics::INVALID_INDEX),
    mElevation(elevation)
{
  initialize();
}

//==============================================================================
SupportPolygonVisual::SupportPolygonVisual(const dart::dynamics::SkeletonPtr& skeleton,
                             std::size_t treeIndex, double elevation)
  : mSkeleton(skeleton),
    mTreeIndex(treeIndex),
    mElevation(elevation)
{
  initialize();
}

//==============================================================================
void SupportPolygonVisual::setSkeleton(const dart::dynamics::SkeletonPtr& skeleton)
{
  mSkeleton = skeleton;
}

//==============================================================================
dart::dynamics::SkeletonPtr SupportPolygonVisual::getSkeleton() const
{
  return mSkeleton.lock();
}

//==============================================================================
void SupportPolygonVisual::visualizeWholeSkeleton()
{
  mTreeIndex = dart::dynamics::INVALID_INDEX;
}

//==============================================================================
void SupportPolygonVisual::visualizeTree(std::size_t treeIndex)
{
  mTreeIndex = treeIndex;
}

//==============================================================================
void SupportPolygonVisual::setDisplayElevation(double elevation)
{
  mElevation = elevation;
}

//==============================================================================
double SupportPolygonVisual::getDisplayElevation() const
{
  return mElevation;
}

//==============================================================================
void SupportPolygonVisual::displayPolygon(bool display)
{
  if(mDisplayPolygon == display)
    return;

  mDisplayPolygon = display;

  if(mDisplayPolygon)
    addChild(mPolygonGeode);
  else
    removeChild(mPolygonGeode);
}

//==============================================================================
void SupportPolygonVisual::setPolygonColor(const Eigen::Vector4d& color)
{
  (*mPolygonColor)[0] = ::osg::Vec4(color[0], color[1], color[2], color[3]);
  mPolygonGeom->setColorArray(mPolygonColor, ::osg::Array::BIND_OVERALL);
}

//==============================================================================
Eigen::Vector4d SupportPolygonVisual::getPolygonColor() const
{
  const ::osg::Vec4& c = (*mPolygonColor)[0];
  return Eigen::Vector4d(c[0], c[1], c[2], c[3]);
}

//==============================================================================
void SupportPolygonVisual::displayCentroid(bool display)
{
  if(mDisplayCentroid == display)
    return;

  mDisplayCentroid = display;

  if(mDisplayCentroid)
    addChild(mCentroidNode);
  else
    removeChild(mCentroidNode);
}

//==============================================================================
bool SupportPolygonVisual::isCentroidDisplayed() const
{
  return mDisplayCentroid;
}

//==============================================================================
void SupportPolygonVisual::setCentroidRadius(double radius)
{
  if(mCentroidRadius == radius)
    return;

  mCentroidRadius = radius;
  const dart::dynamics::ShapePtr& shape = mCentroid->getShape();
  std::static_pointer_cast<dart::dynamics::SphereShape>(shape)->setRadius(
        mCentroidRadius/4.0);
  shape->addDataVariance(dart::dynamics::Shape::DYNAMIC_PRIMITIVE);
}

//==============================================================================
double SupportPolygonVisual::getCentroidRadius() const
{
  return mCentroidRadius;
}

//==============================================================================
void SupportPolygonVisual::displayCenterOfMass(bool display)
{
  if(mDisplayCOM == display)
    return;

  mDisplayCOM = display;

  if(mDisplayCOM)
    addChild(mComNode);
  else
    removeChild(mComNode);
}

//==============================================================================
bool SupportPolygonVisual::isCenterOfMassDisplayed() const
{
  return mDisplayCOM;
}

//==============================================================================
void SupportPolygonVisual::setCenterOfMassRadius(double radius)
{
  if(mComRadius == radius)
    return;

  mComRadius = radius;
  const dart::dynamics::ShapePtr& shape = mCom->getShape();
  std::static_pointer_cast<dart::dynamics::SphereShape>(shape)->setRadius(
        mComRadius/4.0);
  shape->addDataVariance(dart::dynamics::Shape::DYNAMIC_PRIMITIVE);
}

//==============================================================================
double SupportPolygonVisual::getCenterOfMassRadius() const
{
  return mComRadius;
}

//==============================================================================
void SupportPolygonVisual::setValidCOMColor(const Eigen::Vector4d& color)
{
  mValidColor = color;
}

//==============================================================================
const Eigen::Vector4d& SupportPolygonVisual::getValidCOMColor() const
{
  return mValidColor;
}

//==============================================================================
void SupportPolygonVisual::setInvalidCOMColor(const Eigen::Vector4d& color)
{
  mInvalidColor = color;
}

//==============================================================================
const Eigen::Vector4d& SupportPolygonVisual::getInvalidCOMColor() const
{
  return mInvalidColor;
}

//==============================================================================
void SupportPolygonVisual::refresh()
{
  const dart::dynamics::SkeletonPtr& skel = mSkeleton.lock();
  if(nullptr == skel)
    return;

  const dart::math::SupportPolygon& poly =
      (dart::dynamics::INVALID_INDEX == mTreeIndex)?
        skel->getSupportPolygon() : skel->getSupportPolygon(mTreeIndex);

  const std::pair<Eigen::Vector3d, Eigen::Vector3d>& axes =
      (dart::dynamics::INVALID_INDEX == mTreeIndex)?
        skel->getSupportAxes() : skel->getSupportAxes(mTreeIndex);
  const Eigen::Vector3d& up = axes.first.cross(axes.second);

  if(mDisplayPolygon)
  {
    mVertices->resize(poly.size());
    mFaces->resize(poly.size());
    for(std::size_t i=0; i < poly.size(); ++i)
    {
      const Eigen::Vector3d& v = axes.first*poly[i][0] + axes.second*poly[i][1]
                                  + up*mElevation;
      (*mVertices)[i] = ::osg::Vec3(v[0], v[1], v[2]);
      (*mFaces)[i] = i;
    }

    mPolygonGeom->setVertexArray(mVertices);
    mPolygonGeom->setPrimitiveSet(0, mFaces);
  }

  if(mDisplayCentroid)
  {
    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());

    if(poly.size() > 0)
    {
      const Eigen::Vector2d& Cp = (dart::dynamics::INVALID_INDEX == mTreeIndex)?
            skel->getSupportCentroid() : skel->getSupportCentroid(mTreeIndex);

      const Eigen::Vector3d& C = Cp[0]*axes.first + Cp[1]*axes.second
                                 + up*mElevation;

      tf.translation() = C;
    }
    else
    {
      // If there is no support polygon, then just lay the centroid over the
      // center of mass
      tf.translation() = skel->getCOM();
    }

    mCentroid->setTransform(tf);

    mCentroidNode->refresh();

    // Turn off primitive variance each cycle to avoid unnecessary re-updating
    mCentroid->getShape()->removeDataVariance(
          dart::dynamics::Shape::DYNAMIC_PRIMITIVE);
  }

  if(mDisplayCOM)
  {
    Eigen::Vector3d com(Eigen::Vector3d::Zero());
    if(dart::dynamics::INVALID_INDEX == mTreeIndex)
    {
      com = skel->getCOM();
    }
    else
    {
      // We need to calculate tree COM ourselves, because that is not provided
      // by the API (yet)
      const std::vector<dart::dynamics::BodyNode*>& bns =
          skel->getTreeBodyNodes(mTreeIndex);

      double mass = 0.0;
      for(std::size_t i=0; i < bns.size(); ++i)
      {
        dart::dynamics::BodyNode* bn = bns[i];
        com += bn->getMass() * bn->getCOM();
        mass += bn->getMass();
      }

      assert(mass != 0.0);
      com = com/mass;
    }

    const Eigen::Vector2d& Cproj = Eigen::Vector2d(com.dot(axes.first),
                                                   com.dot(axes.second));
    const Eigen::Vector3d& C = Cproj[0]*axes.first
                             + Cproj[1]*axes.second
                             + up*mElevation;

    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    tf.translation() = C;
    mCom->setTransform(tf);

    if(dart::math::isInsideSupportPolygon(Cproj, poly))
      mCom->getVisualAspect(true)->setColor(mValidColor);
    else
      mCom->getVisualAspect(true)->setColor(mInvalidColor);

    mComNode->refresh();

    // Turn off primitive variance each cycle to avoid unnecessary re-updating
    mCom->getShape()->removeDataVariance(
          dart::dynamics::Shape::DYNAMIC_PRIMITIVE);
  }
}

//==============================================================================
void SupportPolygonVisual::initialize()
{
  mDisplayPolygon = true;
  mPolygonGeode = new ::osg::Geode;
  mPolygonGeode->getOrCreateStateSet()->setMode(GL_LIGHTING, ::osg::StateAttribute::OFF);
  addChild(mPolygonGeode);

  mPolygonGeom = new ::osg::Geometry;
  mPolygonGeode->addDrawable(mPolygonGeom);

  mVertices = new ::osg::Vec3Array;
  mPolygonGeom->setVertexArray(mVertices);
  mPolygonGeom->setDataVariance(::osg::Object::DYNAMIC);

  ::osg::Vec4 color(0.1, 0.9, 0.1, 1.0);
  mPolygonColor = new ::osg::Vec4Array;
  mPolygonColor->resize(1);
  (*mPolygonColor)[0] = color;
  mPolygonGeom->setColorArray(mPolygonColor);
  mPolygonGeom->setColorBinding(::osg::Geometry::BIND_OVERALL);

  mFaces = new ::osg::DrawElementsUShort(::osg::PrimitiveSet::TRIANGLE_FAN, 0);
  mPolygonGeom->addPrimitiveSet(mFaces);
  mPolygonGeom->setColorArray(mPolygonColor, ::osg::Array::BIND_OVERALL);

  mDisplayCentroid = true;
  mCentroid = std::make_shared<dart::dynamics::SimpleFrame>(
        dart::dynamics::Frame::World(), "centroid");

  mCentroidRadius = 0.12;
  mCentroid->setShape(
        std::make_shared<dart::dynamics::SphereShape>(mCentroidRadius/4.0));

  mCentroid->getVisualAspect(true)->setColor(
        Eigen::Vector4d(color[0], color[1], color[2], color[3]));

  mCentroidNode = new ShapeFrameNode(mCentroid.get(), nullptr);
  addChild(mCentroidNode);

  mValidColor = dart::Color::Blue(1.0);
  mInvalidColor = dart::Color::Red(1.0);

  mDisplayCOM = true;
  mCom = std::make_shared<dart::dynamics::SimpleFrame>(
        dart::dynamics::Frame::World(), "com");

  mComRadius = mCentroidRadius;
  mCom->setShape(std::make_shared<dart::dynamics::SphereShape>(mComRadius/4.0));
  mCom->getShape()->addDataVariance(dart::dynamics::Shape::DYNAMIC_COLOR);

  mComNode = new ShapeFrameNode(mCom.get(), nullptr);
  addChild(mComNode);
}

} // namespace osg
} // namespace gui
} // namespace dart
