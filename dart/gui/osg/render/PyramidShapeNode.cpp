/*
 * Copyright (c) 2011-2022, The DART development contributors
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

#include "dart/gui/osg/render/PyramidShapeNode.hpp"

#include "dart/dynamics/PyramidShape.hpp"
#include "dart/dynamics/SimpleFrame.hpp"
#include "dart/gui/osg/ShapeFrameNode.hpp"
#include "dart/gui/osg/Utils.hpp"

#include <osg/CullFace>
#include <osg/Depth>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/ShapeDrawable>

#include <array>

namespace dart {
namespace gui {
namespace osg {
namespace render {

class PyramidShapeGeode : public ShapeNode, public ::osg::Geode
{
public:
  PyramidShapeGeode(
      std::shared_ptr<dart::dynamics::PyramidShape> shape,
      ShapeFrameNode* parent);

  void refresh();
  void extractData(bool firstTime);

protected:
  virtual ~PyramidShapeGeode();

  std::shared_ptr<dart::dynamics::PyramidShape> mPyramidShape;
  PyramidShapeDrawable* mDrawable;

  ::osg::ref_ptr<::osg::LineWidth> mLineWidth;
};

//==============================================================================
class PyramidShapeDrawable : public ::osg::Geometry
{
public:
  PyramidShapeDrawable(
      dart::dynamics::PyramidShape* shape,
      dart::dynamics::VisualAspect* visualAspect);

  void refresh(bool firstTime);

protected:
  virtual ~PyramidShapeDrawable();

  dart::dynamics::PyramidShape* mPyramidShape;
  dart::dynamics::VisualAspect* mVisualAspect;

  ::osg::ref_ptr<::osg::Vec3Array> mVertices;
  ::osg::ref_ptr<::osg::Vec3Array> mNormals;
  ::osg::ref_ptr<::osg::Vec4Array> mColors;
  std::array<::osg::ref_ptr<::osg::DrawElementsUInt>, 6> mElements;
};

//==============================================================================
PyramidShapeNode::PyramidShapeNode(
    std::shared_ptr<dart::dynamics::PyramidShape> shape, ShapeFrameNode* parent)
  : ShapeNode(shape, parent, this), mPyramidShape(shape), mGeode(nullptr)
{
  mNode = this;
  extractData(true);
  setNodeMask(mVisualAspect->isHidden() ? 0x0 : ~0x0);
}

//==============================================================================
void PyramidShapeNode::refresh()
{
  mUtilized = true;

  setNodeMask(mVisualAspect->isHidden() ? 0x0 : ~0x0);

  if (mShape->getDataVariance() == dart::dynamics::Shape::STATIC)
    return;

  extractData(false);
}

//==============================================================================
void PyramidShapeNode::extractData(bool /*firstTime*/)
{
  if (nullptr == mGeode)
  {
    mGeode = new PyramidShapeGeode(mPyramidShape, mParentShapeFrameNode);
    addChild(mGeode);
    return;
  }

  mGeode->refresh();
}

//==============================================================================
PyramidShapeNode::~PyramidShapeNode()
{
  // Do nothing
}

//==============================================================================
PyramidShapeGeode::PyramidShapeGeode(
    std::shared_ptr<dart::dynamics::PyramidShape> shape, ShapeFrameNode* parent)
  : ShapeNode(shape, parent, this),
    mPyramidShape(shape),
    mDrawable(nullptr),
    mLineWidth(new ::osg::LineWidth)
{
  getOrCreateStateSet()->setAttributeAndModes(
      new ::osg::CullFace(::osg::CullFace::BACK));
  getOrCreateStateSet()->setMode(GL_LIGHTING, ::osg::StateAttribute::ON);
  extractData(true);
}

//==============================================================================
void PyramidShapeGeode::refresh()
{
  mUtilized = true;

  extractData(false);
}

//==============================================================================
void PyramidShapeGeode::extractData(bool /*firstTime*/)
{
  if (nullptr == mDrawable)
  {
    mDrawable = new PyramidShapeDrawable(mPyramidShape.get(), mVisualAspect);
    addDrawable(mDrawable);
    return;
  }

  mDrawable->refresh(false);
}

//==============================================================================
PyramidShapeGeode::~PyramidShapeGeode()
{
  // Do nothing
}

//==============================================================================
PyramidShapeDrawable::PyramidShapeDrawable(
    dart::dynamics::PyramidShape* shape,
    dart::dynamics::VisualAspect* visualAspect)
  : mPyramidShape(shape),
    mVisualAspect(visualAspect),
    mVertices(new ::osg::Vec3Array),
    mNormals(new ::osg::Vec3Array),
    mColors(new ::osg::Vec4Array)
{
  for (auto& e : mElements)
  {
    e = new ::osg::DrawElementsUInt(::osg::PrimitiveSet::TRIANGLES);
    addPrimitiveSet(e);
  }
  refresh(true);
}

//==============================================================================
void PyramidShapeDrawable::refresh(bool firstTime)
{
  if (mPyramidShape->getDataVariance() == dart::dynamics::Shape::STATIC)
    setDataVariance(::osg::Object::STATIC);
  else
    setDataVariance(::osg::Object::DYNAMIC);

  if (firstTime)
  {
    for (auto& e : mElements)
      e->resize(3);

    // Side triangle 1
    mElements[0]->at(0) = 0;
    mElements[0]->at(1) = 1;
    mElements[0]->at(2) = 2;

    // Side triangle 2
    mElements[1]->at(0) = 0;
    mElements[1]->at(1) = 2;
    mElements[1]->at(2) = 3;

    // Side triangle 3
    mElements[2]->at(0) = 0;
    mElements[2]->at(1) = 3;
    mElements[2]->at(2) = 4;

    // Side triangle 4
    mElements[3]->at(0) = 0;
    mElements[3]->at(1) = 4;
    mElements[3]->at(2) = 1;

    // Base triangle 1
    mElements[4]->at(0) = 1;
    mElements[4]->at(1) = 3;
    mElements[4]->at(2) = 2;

    // Base triangle 2
    mElements[5]->at(0) = 1;
    mElements[5]->at(1) = 4;
    mElements[5]->at(2) = 3;

    setPrimitiveSet(0, mElements[0]);
    setPrimitiveSet(1, mElements[1]);
    setPrimitiveSet(2, mElements[2]);
    setPrimitiveSet(3, mElements[3]);
    setPrimitiveSet(4, mElements[4]);
    setPrimitiveSet(5, mElements[5]);
  }

  if (mPyramidShape->checkDataVariance(dart::dynamics::Shape::DYNAMIC_VERTICES)
      || firstTime)
  {
    const float w = static_cast<float>(mPyramidShape->getBaseWidth());
    const float d = static_cast<float>(mPyramidShape->getBaseDepth());
    const float h = static_cast<float>(mPyramidShape->getBaseWidth());

    const float hTop = h / 2;
    const float hBottom = -hTop;
    const float left = -w / 2;
    const float right = w / 2;
    const float front = -d / 2;
    const float back = d / 2;

    mVertices->resize(5);
    mVertices->at(0).set(0, 0, hTop);
    mVertices->at(1).set(right, back, hBottom);
    mVertices->at(2).set(left, back, hBottom);
    mVertices->at(3).set(left, front, hBottom);
    mVertices->at(4).set(right, front, hBottom);
    setVertexArray(mVertices);

    mNormals->clear();
    mNormals->reserve(mElements.size()); // 6
    for (const auto& e : mElements)
    {
      const auto& v1 = mVertices->at(e->at(0));
      const auto& v2 = mVertices->at(e->at(1));
      const auto& v3 = mVertices->at(e->at(2));
      ::osg::Vec3 n = (v1 - v2) ^ (v2 - v3);
      n.normalize();

      mNormals->push_back(n);
    }
    setNormalArray(mNormals, ::osg::Array::BIND_PER_PRIMITIVE_SET);
  }

  if (mPyramidShape->checkDataVariance(dart::dynamics::Shape::DYNAMIC_COLOR)
      || firstTime)
  {
    // Set color
    const ::osg::Vec4d color = eigToOsgVec4d(mVisualAspect->getRGBA());
    mColors->resize(1);
    (*mColors)[0] = color;
    setColorArray(mColors, ::osg::Array::BIND_OVERALL);

    // Set alpha specific properties
    ::osg::StateSet* ss = getOrCreateStateSet();
    if (std::abs(color.a()) > 1 - getAlphaThreshold())
    {
      ss->setMode(GL_BLEND, ::osg::StateAttribute::OFF);
      ss->setRenderingHint(::osg::StateSet::OPAQUE_BIN);
      ::osg::ref_ptr<::osg::Depth> depth = new ::osg::Depth;
      depth->setWriteMask(true);
      ss->setAttributeAndModes(depth, ::osg::StateAttribute::ON);
    }
    else
    {
      ss->setMode(GL_BLEND, ::osg::StateAttribute::ON);
      ss->setRenderingHint(::osg::StateSet::TRANSPARENT_BIN);
      ::osg::ref_ptr<::osg::Depth> depth = new ::osg::Depth;
      depth->setWriteMask(false);
      ss->setAttributeAndModes(depth, ::osg::StateAttribute::ON);
    }
  }
}

//==============================================================================
PyramidShapeDrawable::~PyramidShapeDrawable()
{
  // Do nothing
}

} // namespace render
} // namespace osg
} // namespace gui
} // namespace dart
