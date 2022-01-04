/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#include "dart/gui/osg/render/MultiSphereShapeNode.hpp"

#include <osg/CullFace>
#include <osg/Depth>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Light>
#include <osg/Material>

#include "dart/dynamics/MultiSphereConvexHullShape.hpp"
#include "dart/dynamics/SimpleFrame.hpp"
#include "dart/gui/osg/Utils.hpp"
#include "dart/math/Icosphere.hpp"

namespace dart {
namespace gui {
namespace osg {
namespace render {

//==============================================================================
class MultiSphereShapeGeode : public ShapeNode, public ::osg::Geode
{
public:
  MultiSphereShapeGeode(
      dart::dynamics::MultiSphereConvexHullShape* shape,
      ShapeFrameNode* parentShapeFrame,
      MultiSphereShapeNode* parentNode);

  void refresh();
  void extractData();

protected:
  virtual ~MultiSphereShapeGeode();

  MultiSphereShapeNode* mParentNode;
  dart::dynamics::MultiSphereConvexHullShape* mMultiSphereShape;
  MultiSphereShapeDrawable* mDrawable;
};

//==============================================================================
class MultiSphereShapeDrawable : public ::osg::Geometry
{
public:
  MultiSphereShapeDrawable(
      dart::dynamics::MultiSphereConvexHullShape* shape,
      dart::dynamics::VisualAspect* visualAspect,
      MultiSphereShapeGeode* parent);

  void refresh(bool firstTime);

protected:
  virtual ~MultiSphereShapeDrawable();

  dart::dynamics::MultiSphereConvexHullShape* mMultiSphereShape;
  dart::dynamics::VisualAspect* mVisualAspect;
  MultiSphereShapeGeode* mParent;
  ::osg::ref_ptr<::osg::Vec3Array> mVertices;
  ::osg::ref_ptr<::osg::Vec3Array> mNormals;
  ::osg::ref_ptr<::osg::Vec4Array> mColors;
};

//==============================================================================
MultiSphereShapeNode::MultiSphereShapeNode(
    std::shared_ptr<dart::dynamics::MultiSphereConvexHullShape> shape,
    ShapeFrameNode* parent)
  : ShapeNode(shape, parent, this), mMultiSphereShape(shape), mGeode(nullptr)
{
  extractData(true);
  setNodeMask(mVisualAspect->isHidden() ? 0x0 : ~0x0);
}

//==============================================================================
void MultiSphereShapeNode::refresh()
{
  mUtilized = true;

  setNodeMask(mVisualAspect->isHidden() ? 0x0 : ~0x0);

  if (mShape->getDataVariance() == dart::dynamics::Shape::STATIC)
    return;

  extractData(false);
}

//==============================================================================
void MultiSphereShapeNode::extractData(bool /*firstTime*/)
{
  if (nullptr == mGeode)
  {
    mGeode = new MultiSphereShapeGeode(
        mMultiSphereShape.get(), mParentShapeFrameNode, this);
    addChild(mGeode);
    return;
  }

  mGeode->refresh();
}

//==============================================================================
MultiSphereShapeNode::~MultiSphereShapeNode()
{
  // Do nothing
}

//==============================================================================
MultiSphereShapeGeode::MultiSphereShapeGeode(
    dart::dynamics::MultiSphereConvexHullShape* shape,
    ShapeFrameNode* parentShapeFrame,
    MultiSphereShapeNode* parentNode)
  : ShapeNode(parentNode->getShape(), parentShapeFrame, this),
    mParentNode(parentNode),
    mMultiSphereShape(shape),
    mDrawable(nullptr)
{
  getOrCreateStateSet()->setAttributeAndModes(
      new ::osg::CullFace(::osg::CullFace::BACK));
  extractData();
}

//==============================================================================
void MultiSphereShapeGeode::refresh()
{
  mUtilized = true;

  extractData();
}

//==============================================================================
void MultiSphereShapeGeode::extractData()
{
  if (nullptr == mDrawable)
  {
    mDrawable
        = new MultiSphereShapeDrawable(mMultiSphereShape, mVisualAspect, this);
    addDrawable(mDrawable);
    return;
  }

  mDrawable->refresh(false);
}

//==============================================================================
MultiSphereShapeGeode::~MultiSphereShapeGeode()
{
  // Do nothing
}

//==============================================================================
MultiSphereShapeDrawable::MultiSphereShapeDrawable(
    dart::dynamics::MultiSphereConvexHullShape* shape,
    dart::dynamics::VisualAspect* visualAspect,
    MultiSphereShapeGeode* parent)
  : mMultiSphereShape(shape),
    mVisualAspect(visualAspect),
    mParent(parent),
    mVertices(new ::osg::Vec3Array),
    mNormals(new ::osg::Vec3Array),
    mColors(new ::osg::Vec4Array)
{
  refresh(true);
}

//==============================================================================
void MultiSphereShapeDrawable::refresh(bool firstTime)
{
  if (mMultiSphereShape->getDataVariance() == dart::dynamics::Shape::STATIC)
    setDataVariance(::osg::Object::STATIC);
  else
    setDataVariance(::osg::Object::DYNAMIC);

  if (mMultiSphereShape->checkDataVariance(
          dart::dynamics::Shape::DYNAMIC_PRIMITIVE)
      || firstTime)
  {
    const auto subdivisions = 3; // TODO(JS): Make this configurable
    const auto& spheres = mMultiSphereShape->getSpheres();
    auto numVertices
        = spheres.size() * math::Icosphered::getNumVertices(subdivisions);
    mVertices->resize(numVertices);

    // Create meshes of sphere and combine them into a single mesh
    auto mesh = math::TriMeshd();
    for (const auto& sphere : spheres)
    {
      const double& radius = sphere.first;
      const Eigen::Vector3d& center = sphere.second;

      auto icosphere = math::Icosphered(radius, subdivisions);
      icosphere.translate(center);

      mesh += icosphere;
    }

    // Create a convex hull from the combined mesh
    auto convexHull = mesh.generateConvexHull();
    convexHull->computeVertexNormals();
    const auto& meshVertices = convexHull->getVertices();
    const auto& meshNormals = convexHull->getVertexNormals();
    const auto& meshTriangles = convexHull->getTriangles();
    assert(meshVertices.size() == meshNormals.size());

    // Convert the convex hull to OSG data types
    mVertices->resize(meshVertices.size());
    mNormals->resize(meshVertices.size());
    for (auto i = 0u; i < meshVertices.size(); ++i)
    {
      const auto& v = meshVertices[i];
      const auto& n = meshNormals[i];
      (*mVertices)[i] = ::osg::Vec3(v[0], v[1], v[2]);
      (*mNormals)[i] = ::osg::Vec3(n[0], n[1], n[2]);
    }
    setVertexArray(mVertices);
    setNormalArray(mNormals, ::osg::Array::BIND_PER_VERTEX);

    ::osg::ref_ptr<::osg::DrawElementsUInt> drawElements
        = new ::osg::DrawElementsUInt(GL_TRIANGLES);
    drawElements->resize(3 * meshTriangles.size());
    for (auto i = 0u; i < meshTriangles.size(); ++i)
    {
      const auto& triangle = meshTriangles[i];
      (*drawElements)[3 * i] = triangle[0];
      (*drawElements)[3 * i + 1] = triangle[1];
      (*drawElements)[3 * i + 2] = triangle[2];
    }
    addPrimitiveSet(drawElements);

    dirtyDisplayList();
  }

  if (mMultiSphereShape->checkDataVariance(dart::dynamics::Shape::DYNAMIC_COLOR)
      || firstTime)
  {
    // Set color
    const ::osg::Vec4d color = eigToOsgVec4d(mVisualAspect->getRGBA());
    (*mColors).resize(1);
    (*mColors)[0] = color;
    setColorArray(mColors);
    setColorBinding(::osg::Geometry::BIND_OVERALL);

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
MultiSphereShapeDrawable::~MultiSphereShapeDrawable()
{
  // Do nothing
}

} // namespace render
} // namespace osg
} // namespace gui
} // namespace dart
