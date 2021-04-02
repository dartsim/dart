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

#ifndef DART_GUI_OSG_RENDER_HEIGHTMAPSHAPENODE_HPP_
#define DART_GUI_OSG_RENDER_HEIGHTMAPSHAPENODE_HPP_

#include "dart/config.hpp"

#include <osg/CullFace>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Light>
#include <osg/Material>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>

#include "dart/dynamics/HeightmapShape.hpp"
#include "dart/dynamics/SimpleFrame.hpp"
#include "dart/gui/osg/Utils.hpp"
#include "dart/gui/osg/render/ShapeNode.hpp"

namespace dart {
namespace gui {
namespace osg {
namespace render {

template <typename S>
class HeightmapShapeGeode;

template <typename S_>
class HeightmapShapeNode : public ShapeNode, public ::osg::MatrixTransform
{
public:
  using S = S_;

  HeightmapShapeNode(
      std::shared_ptr<dynamics::HeightmapShape<S>> shape,
      ShapeFrameNode* parent);

  void refresh() override;
  void extractData(bool firstTime);

protected:
  virtual ~HeightmapShapeNode() override;

  std::shared_ptr<dynamics::HeightmapShape<S>> mHeightmapShape;
  HeightmapShapeGeode<S>* mGeode;
  std::size_t mHeightmapVersion;
};

//==============================================================================
template <typename S>
class HeightmapShapeDrawable : public ::osg::Geometry
{
public:
  using Vector3 = Eigen::Matrix<S, 3, 1>;

  using osgVec3 = typename std::conditional<
      std::is_same<S, float>::value,
      ::osg::Vec3f,
      ::osg::Vec3d>::type;
  using Vec3Array = typename std::conditional<
      std::is_same<S, float>::value,
      ::osg::Vec3Array,
      ::osg::Vec3dArray>::type;
  using Vec4Array = typename std::conditional<
      std::is_same<S, float>::value,
      ::osg::Vec4Array,
      ::osg::Vec4dArray>::type;

  HeightmapShapeDrawable(
      dynamics::HeightmapShape<S>* shape,
      dynamics::VisualAspect* visualAspect,
      HeightmapShapeGeode<S>* parent);

  void refresh(bool firstTime);

protected:
  ~HeightmapShapeDrawable() override = default;

  dynamics::HeightmapShape<S>* mHeightmapShape;
  dynamics::VisualAspect* mVisualAspect;
  HeightmapShapeGeode<S>* mParent;

private:
  ::osg::ref_ptr<Vec3Array> mVertices;
  ::osg::ref_ptr<::osg::DrawElementsUInt> mElements;
  ::osg::ref_ptr<Vec3Array> mNormals;
  ::osg::ref_ptr<::osg::Vec4Array> mColors;
};

//==============================================================================
template <typename S>
class HeightmapShapeGeode : public ShapeNode, public ::osg::Geode
{
public:
  HeightmapShapeGeode(
      dynamics::HeightmapShape<S>* shape,
      ShapeFrameNode* parentShapeFrame,
      HeightmapShapeNode<S>* parentNode);

  void refresh();
  void extractData();

protected:
  virtual ~HeightmapShapeGeode();

  HeightmapShapeNode<S>* mParentNode;
  dynamics::HeightmapShape<S>* mHeightmapShape;
  HeightmapShapeDrawable<S>* mDrawable;
};

//==============================================================================
template <typename S>
HeightmapShapeNode<S>::HeightmapShapeNode(
    std::shared_ptr<dynamics::HeightmapShape<S>> shape, ShapeFrameNode* parent)
  : ShapeNode(shape, parent, this),
    mHeightmapShape(shape),
    mGeode(nullptr),
    mHeightmapVersion(dynamics::INVALID_INDEX)
{
  extractData(true);
  setNodeMask(mVisualAspect->isHidden() ? 0x0u : ~0x0u);
}

//==============================================================================
template <typename S>
void HeightmapShapeNode<S>::refresh()
{
  mUtilized = true;

  setNodeMask(mVisualAspect->isHidden() ? 0x0u : ~0x0u);

  if (mShape->getDataVariance() == dynamics::Shape::STATIC
      && mHeightmapVersion == mHeightmapShape->getVersion())
    return;

  extractData(false);

  mHeightmapVersion = mHeightmapShape->getVersion();
}

//==============================================================================
template <typename S>
void HeightmapShapeNode<S>::extractData(bool /*firstTime*/)
{
  if (nullptr == mGeode)
  {
    mGeode = new HeightmapShapeGeode<S>(
        mHeightmapShape.get(), mParentShapeFrameNode, this);
    addChild(mGeode);
    return;
  }

  mGeode->refresh();
}

//==============================================================================
template <typename S>
HeightmapShapeNode<S>::~HeightmapShapeNode()
{
  // Do nothing
}

//==============================================================================
template <typename S>
HeightmapShapeGeode<S>::HeightmapShapeGeode(
    dynamics::HeightmapShape<S>* shape,
    ShapeFrameNode* parentShapeFrame,
    HeightmapShapeNode<S>* parentNode)
  : ShapeNode(parentNode->getShape(), parentShapeFrame, this),
    mParentNode(parentNode),
    mHeightmapShape(shape),
    mDrawable(nullptr)
{
  getOrCreateStateSet()->setMode(GL_BLEND, ::osg::StateAttribute::ON);
  getOrCreateStateSet()->setRenderingHint(::osg::StateSet::TRANSPARENT_BIN);
  getOrCreateStateSet()->setAttributeAndModes(
      new ::osg::CullFace(::osg::CullFace::BACK));
  getOrCreateStateSet()->setMode(GL_LIGHTING, ::osg::StateAttribute::ON);
  extractData();
}

//==============================================================================
template <typename S>
void HeightmapShapeGeode<S>::refresh()
{
  mUtilized = true;

  extractData();
}

//==============================================================================
template <typename S>
void HeightmapShapeGeode<S>::extractData()
{
  if (nullptr == mDrawable)
  {
    mDrawable
        = new HeightmapShapeDrawable<S>(mHeightmapShape, mVisualAspect, this);
    addDrawable(mDrawable);
    return;
  }

  mDrawable->refresh(false);
}

//==============================================================================
template <typename S>
HeightmapShapeGeode<S>::~HeightmapShapeGeode()
{
  // Do nothing
}

//==============================================================================
template <typename S>
HeightmapShapeDrawable<S>::HeightmapShapeDrawable(
    dynamics::HeightmapShape<S>* shape,
    dynamics::VisualAspect* visualAspect,
    HeightmapShapeGeode<S>* parent)
  : mHeightmapShape(shape), mVisualAspect(visualAspect), mParent(parent)
{
  static_assert(
      std::is_same<S, float>::value || std::is_same<S, double>::value,
      "Scalar type should be float or double");

  // See:
  // https://osg-users.openscenegraph.narkive.com/VY16YIMs/crash-due-to-triangle-functor-does-not-support-vec3d-vertex-arrays
  // https://github.com/openscenegraph/OpenSceneGraph/blob/5b688eb99dd5db94f7068ee18fb94f120720e3d1/include/osg/TriangleFunctor#L73
  static_assert(
      !std::is_same<S, double>::value,
      "OpenSceneGraph currently doesn't support double precision for "
      "Heightmap");

  mVertices = new Vec3Array;
  mNormals = new Vec3Array;
  mColors = new Vec4Array;
  // TODO(JS): Switch to TRIANGLE_STRIP to save storage for indicies
  mElements = new ::osg::DrawElementsUInt(::osg::PrimitiveSet::TRIANGLES);
  addPrimitiveSet(mElements);
  refresh(true);
}

//==============================================================================
template <typename S>
Eigen::Matrix<S, 3, 1> getNormal(
    const Eigen::Matrix<S, 3, 1>& p1,
    const Eigen::Matrix<S, 3, 1>& p2,
    const Eigen::Matrix<S, 3, 1>& p3)
{
  return (p2 - p1).cross(p3 - p1).normalized();
}

//==============================================================================
inline ::osg::Vec3f getNormal(
    const ::osg::Vec3f& p1, const ::osg::Vec3f& p2, const ::osg::Vec3f& p3)
{
  auto normal = (p2 - p1) ^ (p3 - p1);
  normal.normalize();
  return normal;
}

//==============================================================================
inline ::osg::Vec3d getNormal(
    const ::osg::Vec3d& p1, const ::osg::Vec3d& p2, const ::osg::Vec3d& p3)
{
  auto normal = (p2 - p1) ^ (p3 - p1);
  normal.normalize();
  return normal;
}

//==============================================================================
template <typename S>
void setVertices(
    const typename dynamics::HeightmapShape<S>::HeightField& heightmap,
    typename HeightmapShapeDrawable<S>::Vec3Array& vertices,
    ::osg::DrawElementsUInt& faces,
    typename HeightmapShapeDrawable<S>::Vec3Array& normals,
    typename HeightmapShapeDrawable<S>::Vector3 scale)
{
  // Returns an index array for a GL_TRIANGLES heightmap

  const auto rows = heightmap.rows();
  const auto cols = heightmap.cols();

  faces.clear();
  normals.clear();
  if (rows < 2 || cols < 2)
  {
    vertices.clear();
    return;
  }

  vertices.resize(static_cast<std::size_t>(heightmap.size()));

  // Note that heightmap(i, j) represents the height value at (j, -i) in XY
  // coordinates.
  for (auto i = 0; i < rows; ++i)
  {
    for (auto j = 0; j < cols; ++j)
    {
      const auto index = cols * i + j;
      vertices[index].set(
          j * scale.x(), -(i * scale.y()), heightmap(i, j) * scale.z());
    }
  }

  //
  //                                                  X
  //    +----------------------------------------------->
  //    |
  //    |             |           |            |
  //    |             |           |            |
  //    |        -----o-----------o------------o-----
  //    | p1(i-1,j-1) | upper   / | p2(i-1,j)  |
  //    |             |       /   |            |
  //    |             |     /     |            |
  //    |             |   /       |            |
  //    |             | /   lower |            |
  //    |        -----o-----------o------------o-----
  //    |   p3(i, j-1)|           | curr(i, j) | p4(i,j+1)
  //    |             |           |            |
  //    |             |           |            |
  //    |             |           |            |
  //    |             |           |            |
  //    |        -----o-----------o------------o-----
  //    |             |           | p5(i+1,j)  |
  // -Y |             |           |            |
  //    V
  //
  //

  // For row-major matrix
  faces.reserve(6 * (rows - 1) * (cols - 1));
  for (auto i = 1; i < rows; ++i)
  {
    for (auto j = 1; j < cols; ++j)
    {
      // Indices for matrix
      const auto p1i = i - 1;
      const auto p1j = j - 1;

      const auto p2i = i - 1;
      const auto p2j = j;

      const auto p3i = i;
      const auto p3j = j - 1;

      // Indices for vector
      const auto p1 = p1i * cols + p1j;
      const auto p2 = p2i * cols + p2j;
      const auto p3 = p3i * cols + p3j;
      const auto curr = i * cols + j;

      // Upper triangle
      faces.push_back(p1);
      faces.push_back(p3);
      faces.push_back(p2);

      // Lower triangle
      faces.push_back(p2);
      faces.push_back(p3);
      faces.push_back(curr);
    }
  }

  normals.reserve(heightmap.size());
  for (auto i = 0; i < rows; ++i)
  {
    for (auto j = 0; j < cols; ++j)
    {
      // Indices for matrix
      const auto p2i = i - 1;
      const auto p2j = j;

      const auto p3i = i;
      const auto p3j = j - 1;

      const auto p4i = i;
      const auto p4j = j + 1;

      const auto p5i = i + 1;
      const auto p5j = j;

      // Indices for vector
      const auto p2 = p2i * cols + p2j;
      const auto p3 = p3i * cols + p3j;
      const auto p4 = p4i * cols + p4j;
      const auto p5 = p5i * cols + p5j;
      const auto curr = i * cols + j;

      const auto& ptCurr = vertices[curr];

      auto sum = typename HeightmapShapeDrawable<S>::osgVec3();

      if (i > 0 && j > 0)
        sum += getNormal(ptCurr, vertices[p2], vertices[p3]);

      if (i + 1 < rows && j > 0)
        sum += getNormal(ptCurr, vertices[p3], vertices[p5]);

      if (i + 1 < rows && j + 1 < cols)
        sum += getNormal(ptCurr, vertices[p5], vertices[p4]);

      if (i > 0 && j + 1 < cols)
        sum += getNormal(ptCurr, vertices[p4], vertices[p2]);

      sum.normalize();

      normals.push_back(sum);
    }
  }
}

//==============================================================================
template <typename S>
void HeightmapShapeDrawable<S>::refresh(bool /*firstTime*/)
{
  if (mHeightmapShape->getDataVariance() == dynamics::Shape::STATIC)
    setDataVariance(::osg::Object::STATIC);
  else
    setDataVariance(::osg::Object::DYNAMIC);

  // Row major matrix where top left corner is the height at (0, 0), and bottom
  // right corner is the height at (rows, -cols) in (x, y) coordinates.
  const auto& heightmap = mHeightmapShape->getHeightField();

  // This function is called whenever the heightmap version is increased, and
  // the heightmap could be updated in the version up. So we always update the
  // heightmap.
  {
    assert(mElements);
    assert(mNormals);
    setVertices<S>(
        heightmap,
        *mVertices,
        *mElements,
        *mNormals,
        mHeightmapShape->getScale());
    addPrimitiveSet(mElements);

    setVertexArray(mVertices);
    setNormalArray(mNormals, ::osg::Array::BIND_PER_VERTEX);
  }

  // This function is called whenever the heightmap version is increased, and
  // the color could be updated in the version up. So we always update the
  // color.
  {
    if (mColors->size() != 1)
      mColors->resize(1);

    (*mColors)[0] = eigToOsgVec4d(mVisualAspect->getRGBA());

    setColorArray(mColors, ::osg::Array::BIND_OVERALL);
  }
}

} // namespace render
} // namespace osg
} // namespace gui
} // namespace dart

#endif // DART_GUI_OSG_RENDER_HEIGHTMAPSHAPENODE_HPP_
