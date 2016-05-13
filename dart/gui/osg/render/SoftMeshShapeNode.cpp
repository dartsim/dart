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

#include <osg/Geode>
#include <osg/Geometry>

#include "dart/gui/osg/render/SoftMeshShapeNode.hpp"
#include "dart/gui/osg/Utils.hpp"

#include "dart/dynamics/SoftMeshShape.hpp"
#include "dart/dynamics/SoftBodyNode.hpp"
#include "dart/dynamics/PointMass.hpp"
#include "dart/dynamics/SimpleFrame.hpp"

namespace dart {
namespace gui {
namespace osg {
namespace render {

class SoftMeshShapeGeode : public ShapeNode, public ::osg::Geode
{
public:

  SoftMeshShapeGeode(dart::dynamics::SoftMeshShape* shape,
                     ShapeFrameNode* parentShapeFrame,
                     SoftMeshShapeNode* parentNode);

  void refresh();
  void extractData();

protected:

  virtual ~SoftMeshShapeGeode();

  dart::dynamics::SoftMeshShape* mSoftMeshShape;
  dart::dynamics::VisualAspect* mVisualAspect;
  SoftMeshShapeDrawable* mDrawable;

};

//==============================================================================
class SoftMeshShapeDrawable : public ::osg::Geometry
{
public:

  SoftMeshShapeDrawable(dart::dynamics::SoftMeshShape* shape,
                        dart::dynamics::VisualAspect* visualAspect);

  void refresh(bool firstTime);

protected:

  virtual ~SoftMeshShapeDrawable();

  ::osg::ref_ptr<::osg::Vec3Array> mVertices;
  ::osg::ref_ptr<::osg::Vec3Array> mNormals;
  ::osg::ref_ptr<::osg::Vec4Array> mColors;

  std::vector<Eigen::Vector3d> mEigNormals;

  dart::dynamics::SoftMeshShape* mSoftMeshShape;
  dart::dynamics::VisualAspect* mVisualAspect;

};

//==============================================================================
SoftMeshShapeNode::SoftMeshShapeNode(
    std::shared_ptr<dart::dynamics::SoftMeshShape> shape,
    ShapeFrameNode* parent)
  : ShapeNode(shape, parent, this),
    mSoftMeshShape(shape),
    mGeode(nullptr)
{
  extractData(true);
  setNodeMask(mVisualAspect->isHidden()? 0x0 : ~0x0);
}

//==============================================================================
void SoftMeshShapeNode::refresh()
{
  mUtilized = true;

  setNodeMask(mVisualAspect->isHidden()? 0x0 : ~0x0);

  if(mShape->getDataVariance() == dart::dynamics::Shape::STATIC)
    return;

  extractData(false);
}

//==============================================================================
void SoftMeshShapeNode::extractData(bool /*firstTime*/)
{
  if(nullptr == mGeode)
  {
    mGeode = new SoftMeshShapeGeode(mSoftMeshShape.get(), mParentShapeFrameNode, this);
    addChild(mGeode);
    return;
  }

  mGeode->refresh();
}

//==============================================================================
SoftMeshShapeNode::~SoftMeshShapeNode()
{
  // Do nothing
}

//==============================================================================
SoftMeshShapeGeode::SoftMeshShapeGeode(
    dart::dynamics::SoftMeshShape* shape,
    ShapeFrameNode* parentShapeFrame,
    SoftMeshShapeNode* parentNode)
  : ShapeNode(parentNode->getShape(), parentShapeFrame, this),
    mSoftMeshShape(shape),
    mVisualAspect(parentNode->getVisualAspect()),
    mDrawable(nullptr)
{
  getOrCreateStateSet()->setMode(GL_BLEND, ::osg::StateAttribute::ON);
  extractData();
}

//==============================================================================
void SoftMeshShapeGeode::refresh()
{
  mUtilized = true;

  extractData();
}

//==============================================================================
void SoftMeshShapeGeode::extractData()
{
  if(nullptr == mDrawable)
  {
    mDrawable = new SoftMeshShapeDrawable(mSoftMeshShape, mVisualAspect);
    addDrawable(mDrawable);
    return;
  }

  mDrawable->refresh(false);
}

//==============================================================================
SoftMeshShapeGeode::~SoftMeshShapeGeode()
{
  // Do nothing
}

//==============================================================================
SoftMeshShapeDrawable::SoftMeshShapeDrawable(
    dart::dynamics::SoftMeshShape* shape,
    dart::dynamics::VisualAspect* visualAspect)
  : mVertices(new ::osg::Vec3Array),
    mNormals(new ::osg::Vec3Array),
    mColors(new ::osg::Vec4Array),
    mSoftMeshShape(shape),
    mVisualAspect(visualAspect)
{
  refresh(true);
}

static Eigen::Vector3d normalFromVertex(const dart::dynamics::SoftBodyNode* bn,
                                        const Eigen::Vector3i& face,
                                        std::size_t v)
{
  const Eigen::Vector3d& v0 = bn->getPointMass(face[v])->getLocalPosition();
  const Eigen::Vector3d& v1 = bn->getPointMass(face[(v+1)%3])->getLocalPosition();
  const Eigen::Vector3d& v2 = bn->getPointMass(face[(v+2)%3])->getLocalPosition();

  const Eigen::Vector3d dv1 = v1-v0;
  const Eigen::Vector3d dv2 = v2-v0;
  const Eigen::Vector3d n = dv1.cross(dv2);

  double weight = n.norm()/(dv1.norm()*dv2.norm());
  weight = std::max( -1.0, std::min( 1.0, weight) );

  return n.normalized() * asin(weight);
}

static void computeNormals(std::vector<Eigen::Vector3d>& normals,
                           const dart::dynamics::SoftBodyNode* bn)
{
  for(std::size_t i=0; i<normals.size(); ++i)
    normals[i] = Eigen::Vector3d::Zero();

  for(std::size_t i=0; i<bn->getNumFaces(); ++i)
  {
    const Eigen::Vector3i& face = bn->getFace(i);
    for(std::size_t j=0; j<3; ++j)
      normals[face[j]] += normalFromVertex(bn, face, j);
  }

  for(std::size_t i=0; i<normals.size(); ++i)
    normals[i].normalize();
}

//==============================================================================
void SoftMeshShapeDrawable::refresh(bool firstTime)
{
  if(mSoftMeshShape->getDataVariance() == dart::dynamics::Shape::STATIC)
    setDataVariance(::osg::Object::STATIC);
  else
    setDataVariance(::osg::Object::DYNAMIC);

  const dart::dynamics::SoftBodyNode* bn = mSoftMeshShape->getSoftBodyNode();

  if(mSoftMeshShape->checkDataVariance(dart::dynamics::Shape::DYNAMIC_ELEMENTS)
     || firstTime)
  {
    ::osg::ref_ptr<::osg::DrawElementsUInt> elements =
        new ::osg::DrawElementsUInt(GL_TRIANGLES);
    elements->reserve(3*bn->getNumFaces());

    for(std::size_t i=0; i < bn->getNumFaces(); ++i)
    {
      const Eigen::Vector3i& F = bn->getFace(i);
      for(std::size_t j=0; j<3; ++j)
        elements->push_back(F[j]);
    }

    addPrimitiveSet(elements);
  }

  if(   mSoftMeshShape->checkDataVariance(dart::dynamics::Shape::DYNAMIC_VERTICES)
     || mSoftMeshShape->checkDataVariance(dart::dynamics::Shape::DYNAMIC_ELEMENTS)
     || firstTime)
  {
    if(mVertices->size() != bn->getNumPointMasses())
      mVertices->resize(bn->getNumPointMasses());

    if(mNormals->size() != bn->getNumPointMasses())
      mNormals->resize(bn->getNumPointMasses());

    if(mEigNormals.size() != bn->getNumPointMasses())
      mEigNormals.resize(bn->getNumPointMasses());

    computeNormals(mEigNormals, bn);
    for(std::size_t i=0; i<bn->getNumPointMasses(); ++i)
    {
      (*mVertices)[i] = eigToOsgVec3(bn->getPointMass(i)->getLocalPosition());
      (*mNormals)[i] = eigToOsgVec3(mEigNormals[i]);
    }

    setVertexArray(mVertices);
    setNormalArray(mNormals, ::osg::Array::BIND_PER_VERTEX);
  }

  if(   mSoftMeshShape->checkDataVariance(dart::dynamics::Shape::DYNAMIC_COLOR)
     || firstTime)
  {
    if(mColors->size() != 1)
      mColors->resize(1);

    (*mColors)[0] = eigToOsgVec4(mVisualAspect->getRGBA());

    setColorArray(mColors, ::osg::Array::BIND_OVERALL);
  }
}

//==============================================================================
SoftMeshShapeDrawable::~SoftMeshShapeDrawable()
{
  // Do nothing
}

} // namespace render
} // namespace osg
} // namespace gui
} // namespace dart
