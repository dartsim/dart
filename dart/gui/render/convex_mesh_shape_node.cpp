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

#include "dart/gui/render/convex_mesh_shape_node.hpp"

#include "dart/dynamics/convex_mesh_shape.hpp"
#include "dart/dynamics/simple_frame.hpp"
#include "dart/gui/shape_frame_node.hpp"
#include "dart/gui/utils.hpp"

#include <osg/CullFace>
#include <osg/Depth>
#include <osg/Geode>
#include <osg/Geometry>

namespace dart {
namespace gui {
namespace render {

namespace {

using TriMesh = dart::dynamics::ConvexMeshShape::TriMeshType;
using Vertices = TriMesh::Vertices;
using Normals = TriMesh::Normals;
using Triangles = TriMesh::Triangles;

void computeVertexNormals(
    const Vertices& vertices, const Triangles& triangles, Normals& normals)
{
  normals.assign(vertices.size(), Eigen::Vector3d::Zero());
  for (const auto& triangle : triangles) {
    if (triangle[0] >= vertices.size() || triangle[1] >= vertices.size()
        || triangle[2] >= vertices.size()) {
      continue;
    }

    const Eigen::Vector3d& v0 = vertices[triangle[0]];
    const Eigen::Vector3d& v1 = vertices[triangle[1]];
    const Eigen::Vector3d& v2 = vertices[triangle[2]];
    const Eigen::Vector3d n = (v1 - v0).cross(v2 - v0);
    if (n.squaredNorm() < 1e-12) {
      continue;
    }

    normals[triangle[0]] += n;
    normals[triangle[1]] += n;
    normals[triangle[2]] += n;
  }

  for (auto& normal : normals) {
    const double norm = normal.norm();
    if (norm > 1e-12) {
      normal /= norm;
    }
  }
}

} // namespace

class ConvexMeshShapeDrawable;

class ConvexMeshShapeGeode : public ShapeNode, public ::osg::Geode
{
public:
  ConvexMeshShapeGeode(
      dart::dynamics::ConvexMeshShape* shape,
      ShapeFrameNode* parentShapeFrame,
      ConvexMeshShapeNode* parentNode);

  void refresh();
  void extractData();

protected:
  virtual ~ConvexMeshShapeGeode();

  dart::dynamics::ConvexMeshShape* mConvexMeshShape;
  dart::dynamics::VisualAspect* mVisualAspect;
  ConvexMeshShapeDrawable* mDrawable;
};

//==============================================================================
class ConvexMeshShapeDrawable : public ::osg::Geometry
{
public:
  ConvexMeshShapeDrawable(
      dart::dynamics::ConvexMeshShape* shape,
      dart::dynamics::VisualAspect* visualAspect);

  void refresh(bool firstTime);

protected:
  virtual ~ConvexMeshShapeDrawable();

  ::osg::ref_ptr<::osg::Vec3Array> mVertices;
  ::osg::ref_ptr<::osg::Vec3Array> mNormals;
  ::osg::ref_ptr<::osg::Vec4Array> mColors;
  ::osg::ref_ptr<::osg::DrawElementsUInt> mElements;

  Normals mComputedNormals;

  dart::dynamics::ConvexMeshShape* mConvexMeshShape;
  dart::dynamics::VisualAspect* mVisualAspect;
};

//==============================================================================
ConvexMeshShapeNode::ConvexMeshShapeNode(
    std::shared_ptr<dart::dynamics::ConvexMeshShape> shape,
    ShapeFrameNode* parent)
  : ShapeNode(shape, parent, this),
    mConvexMeshShape(std::move(shape)),
    mGeode(nullptr)
{
  extractData(true);
  setNodeMask(mVisualAspect->isHidden() ? 0x0 : ~0x0);
}

//==============================================================================
void ConvexMeshShapeNode::refresh()
{
  mUtilized = true;

  setNodeMask(mVisualAspect->isHidden() ? 0x0 : ~0x0);

  if (mShape->getDataVariance() == dart::dynamics::Shape::STATIC) {
    return;
  }

  extractData(false);
}

//==============================================================================
void ConvexMeshShapeNode::extractData(bool /*firstTime*/)
{
  if (nullptr == mGeode) {
    mGeode = new ConvexMeshShapeGeode(
        mConvexMeshShape.get(), mParentShapeFrameNode, this);
    addChild(mGeode);
    return;
  }

  mGeode->refresh();
}

//==============================================================================
ConvexMeshShapeNode::~ConvexMeshShapeNode()
{
  // Do nothing
}

//==============================================================================
ConvexMeshShapeGeode::ConvexMeshShapeGeode(
    dart::dynamics::ConvexMeshShape* shape,
    ShapeFrameNode* parentShapeFrame,
    ConvexMeshShapeNode* parentNode)
  : ShapeNode(parentNode->getShape(), parentShapeFrame, this),
    mConvexMeshShape(shape),
    mVisualAspect(parentNode->getVisualAspect()),
    mDrawable(nullptr)
{
  getOrCreateStateSet()->setAttributeAndModes(
      new ::osg::CullFace(::osg::CullFace::BACK));
  extractData();
}

//==============================================================================
void ConvexMeshShapeGeode::refresh()
{
  mUtilized = true;

  extractData();
}

//==============================================================================
void ConvexMeshShapeGeode::extractData()
{
  if (nullptr == mDrawable) {
    mDrawable = new ConvexMeshShapeDrawable(mConvexMeshShape, mVisualAspect);
    addDrawable(mDrawable);
    return;
  }

  mDrawable->refresh(false);
}

//==============================================================================
ConvexMeshShapeGeode::~ConvexMeshShapeGeode()
{
  // Do nothing
}

//==============================================================================
ConvexMeshShapeDrawable::ConvexMeshShapeDrawable(
    dart::dynamics::ConvexMeshShape* shape,
    dart::dynamics::VisualAspect* visualAspect)
  : mVertices(new ::osg::Vec3Array),
    mNormals(new ::osg::Vec3Array),
    mColors(new ::osg::Vec4Array),
    mElements(new ::osg::DrawElementsUInt(::osg::PrimitiveSet::TRIANGLES)),
    mConvexMeshShape(shape),
    mVisualAspect(visualAspect)
{
  addPrimitiveSet(mElements);
  refresh(true);
}

//==============================================================================
void ConvexMeshShapeDrawable::refresh(bool firstTime)
{
  if (mConvexMeshShape->getDataVariance() == dart::dynamics::Shape::STATIC) {
    setDataVariance(::osg::Object::STATIC);
  } else {
    setDataVariance(::osg::Object::DYNAMIC);
  }

  const auto mesh = mConvexMeshShape->getMesh();
  const Vertices* vertices = mesh ? &mesh->getVertices() : nullptr;
  const Triangles* triangles = mesh ? &mesh->getTriangles() : nullptr;

  if (mConvexMeshShape->checkDataVariance(
          dart::dynamics::Shape::DYNAMIC_ELEMENTS)
      || firstTime) {
    mElements->clear();
    if (triangles && vertices) {
      mElements->reserve(3 * triangles->size());
      for (const auto& triangle : *triangles) {
        if (triangle[0] >= vertices->size() || triangle[1] >= vertices->size()
            || triangle[2] >= vertices->size()) {
          continue;
        }
        mElements->push_back(static_cast<unsigned int>(triangle[0]));
        mElements->push_back(static_cast<unsigned int>(triangle[1]));
        mElements->push_back(static_cast<unsigned int>(triangle[2]));
      }
    }

    setPrimitiveSet(0, mElements);
  }

  if (mConvexMeshShape->checkDataVariance(
          dart::dynamics::Shape::DYNAMIC_VERTICES)
      || mConvexMeshShape->checkDataVariance(
          dart::dynamics::Shape::DYNAMIC_ELEMENTS)
      || firstTime) {
    const std::size_t vertexCount = vertices ? vertices->size() : 0u;
    if (mVertices->size() != vertexCount) {
      mVertices->resize(vertexCount);
    }
    if (mNormals->size() != vertexCount) {
      mNormals->resize(vertexCount);
    }

    for (std::size_t i = 0; i < vertexCount; ++i) {
      (*mVertices)[i] = eigToOsgVec3((*vertices)[i]);
    }
    setVertexArray(mVertices);

    const Normals* normals = nullptr;
    if (mesh && mesh->hasVertexNormals()
        && mesh->getVertexNormals().size() == vertexCount) {
      normals = &mesh->getVertexNormals();
    } else if (triangles && vertices) {
      computeVertexNormals(*vertices, *triangles, mComputedNormals);
      normals = &mComputedNormals;
    }

    if (normals) {
      for (std::size_t i = 0; i < vertexCount; ++i) {
        (*mNormals)[i] = eigToOsgVec3((*normals)[i]);
      }
      setNormalArray(mNormals, ::osg::Array::BIND_PER_VERTEX);
    }
  }

  if (mConvexMeshShape->checkDataVariance(dart::dynamics::Shape::DYNAMIC_COLOR)
      || firstTime) {
    const ::osg::Vec4d color = eigToOsgVec4d(mVisualAspect->getRGBA());
    mColors->resize(1);
    (*mColors)[0] = color;
    setColorArray(mColors, ::osg::Array::BIND_OVERALL);

    ::osg::StateSet* ss = getOrCreateStateSet();
    if (std::abs(color.a()) > 1 - getAlphaThreshold()) {
      ss->setMode(GL_BLEND, ::osg::StateAttribute::OFF);
      ss->setRenderingHint(::osg::StateSet::OPAQUE_BIN);
      ::osg::ref_ptr<::osg::Depth> depth = new ::osg::Depth;
      depth->setWriteMask(true);
      ss->setAttributeAndModes(depth, ::osg::StateAttribute::ON);
    } else {
      ss->setMode(GL_BLEND, ::osg::StateAttribute::ON);
      ss->setRenderingHint(::osg::StateSet::TRANSPARENT_BIN);
      ::osg::ref_ptr<::osg::Depth> depth = new ::osg::Depth;
      depth->setWriteMask(false);
      ss->setAttributeAndModes(depth, ::osg::StateAttribute::ON);
    }
  }
}

//==============================================================================
ConvexMeshShapeDrawable::~ConvexMeshShapeDrawable()
{
  // Do nothing
}

} // namespace render
} // namespace gui
} // namespace dart
