/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include "dart/gui/osg/InteractiveFrame.hpp"

#include "dart/common/Console.hpp"
#include "dart/dynamics/ArrowShape.hpp"
#include "dart/dynamics/LineSegmentShape.hpp"
#include "dart/dynamics/MeshShape.hpp"

namespace dart {
namespace gui {
namespace osg {

//==============================================================================
InteractiveTool::InteractiveTool(
    InteractiveFrame* frame, double defaultAlpha, const std::string& name)
  : Entity(ConstructFrame),
    Frame(frame),
    SimpleFrame(frame, name),
    mDefaultAlpha(defaultAlpha),
    mEnabled(true),
    mInteractiveFrame(frame)
{
  // Do nothing
}

//==============================================================================
void InteractiveTool::setEnabled(bool enabled)
{
  mEnabled = enabled;
  for (auto& frame : mSimpleFrames)
    frame->getVisualAspect(true)->setHidden(!enabled);
}

//==============================================================================
bool InteractiveTool::getEnabled() const
{
  return mEnabled;
}

//==============================================================================
void InteractiveTool::setAlpha(double alpha)
{
  for (auto& frame : mSimpleFrames)
    frame->getVisualAspect(true)->setAlpha(alpha);
}

//==============================================================================
void InteractiveTool::resetAlpha()
{
  setAlpha(mDefaultAlpha);
}

//==============================================================================
void InteractiveTool::setDefaultAlpha(double alpha, bool reset)
{
  mDefaultAlpha = alpha;
  if (reset)
    resetAlpha();
}

//==============================================================================
double InteractiveTool::getDefaultAlpha() const
{
  return mDefaultAlpha;
}

//==============================================================================
InteractiveFrame* InteractiveTool::getInteractiveFrame()
{
  return mInteractiveFrame;
}

//==============================================================================
const InteractiveFrame* InteractiveTool::getInteractiveFrame() const
{
  return mInteractiveFrame;
}

//==============================================================================
dart::dynamics::SimpleFrame* InteractiveTool::addShapeFrame(
    const dart::dynamics::ShapePtr& shape)
{
  mSimpleFrames.push_back(std::make_unique<dart::dynamics::SimpleFrame>(this));

  auto shapeFrame = mSimpleFrames.back().get();
  shapeFrame->setShape(shape);
  shapeFrame->createVisualAspect();
  // Disable shadowing for InteractiveTool
  shapeFrame->getVisualAspect(true)->setShadowed(false);

  return shapeFrame;
}

//==============================================================================
const std::vector<dart::dynamics::SimpleFrame*>
InteractiveTool::getShapeFrames()
{
  std::vector<dart::dynamics::SimpleFrame*> frames(mSimpleFrames.size());

  for (auto i = 0u; i < frames.size(); ++i)
    frames[i] = mSimpleFrames[i].get();

  return frames;
}

//==============================================================================
const std::vector<const dart::dynamics::SimpleFrame*>
InteractiveTool::getShapeFrames() const
{
  std::vector<const dart::dynamics::SimpleFrame*> frames(mSimpleFrames.size());

  for (auto i = 0u; i < frames.size(); ++i)
    frames[i] = mSimpleFrames[i].get();

  return frames;
}

//==============================================================================
void InteractiveTool::removeAllShapeFrames()
{
  mSimpleFrames.clear();
}

//==============================================================================
InteractiveFrame::InteractiveFrame(
    dart::dynamics::Frame* referenceFrame,
    const std::string& name,
    const Eigen::Isometry3d& relativeTransform,
    double size_scale,
    double thickness_scale)
  : Entity(referenceFrame, false),
    Frame(referenceFrame),
    SimpleFrame(referenceFrame, name, relativeTransform)
{
  for (std::size_t i = 0; i < 3; ++i) {
    std::string affix = (i == 0) ? "x" : (i == 1) ? "y" : "z";

    mTools[InteractiveTool::LINEAR][i]
        = new InteractiveTool(this, 0.8, "LINEAR_" + affix);
    mTools[InteractiveTool::ANGULAR][i]
        = new InteractiveTool(this, 0.8, "ANGULAR_" + affix);
    mTools[InteractiveTool::PLANAR][i]
        = new InteractiveTool(this, 0.7, "PLANAR_" + affix);
  }

  resizeStandardVisuals(size_scale, thickness_scale);
}

//==============================================================================
InteractiveFrame::~InteractiveFrame()
{
  deleteAllVisualizationShapes();
  deleteAllTools();
}

//==============================================================================
void InteractiveFrame::resizeStandardVisuals(
    double size_scale, double thickness_scale)
{
  deleteAllVisualizationShapes();
  createStandardVisualizationShapes(size_scale, thickness_scale);
}

//==============================================================================
InteractiveTool* InteractiveFrame::getTool(
    InteractiveTool::Type tool, std::size_t coordinate)
{
  if (InteractiveTool::NUM_TYPES <= tool) {
    dtwarn << "[InteractiveFrame::getTool] Attempting to access tool #" << tool
           << ", but tools only go up to " << InteractiveTool::NUM_TYPES
           << "\n";
    return nullptr;
  }

  if (3 <= coordinate) {
    dtwarn << "[InteractiveFrame::getTool] Attempting to access a tool with "
           << "coordinate #" << coordinate << ", but tool coordinates only go "
           << "up to 3\n";
    return nullptr;
  }

  return mTools[(std::size_t)tool][coordinate];
}

//==============================================================================
const InteractiveTool* InteractiveFrame::getTool(
    InteractiveTool::Type tool, std::size_t coordinate) const
{
  return const_cast<InteractiveFrame*>(this)->getTool(tool, coordinate);
}

//==============================================================================
dart::dynamics::SimpleFrame* InteractiveFrame::addShapeFrame(
    const dart::dynamics::ShapePtr& shape)
{
  mSimpleFrames.push_back(std::make_unique<dart::dynamics::SimpleFrame>(this));

  auto shapeFrame = mSimpleFrames.back().get();
  shapeFrame->setShape(shape);
  shapeFrame->createVisualAspect();
  // Disable shadowing for InteractiveFrame
  shapeFrame->getVisualAspect(true)->setShadowed(false);

  return shapeFrame;
}

//==============================================================================
const std::vector<dart::dynamics::SimpleFrame*>
InteractiveFrame::getShapeFrames()
{
  std::vector<dart::dynamics::SimpleFrame*> frames(mSimpleFrames.size());

  for (auto i = 0u; i < frames.size(); ++i)
    frames[i] = mSimpleFrames[i].get();

  return frames;
}

//==============================================================================
const std::vector<const dart::dynamics::SimpleFrame*>
InteractiveFrame::getShapeFrames() const
{
  std::vector<const dart::dynamics::SimpleFrame*> frames(mSimpleFrames.size());

  for (auto i = 0u; i < frames.size(); ++i)
    frames[i] = mSimpleFrames[i].get();

  return frames;
}

//==============================================================================
void InteractiveFrame::removeAllShapeFrames()
{
  mSimpleFrames.clear();
}

//==============================================================================
void InteractiveFrame::createStandardVisualizationShapes(
    double size, double thickness)
{
  const auto pi = math::constantsd::pi();

  thickness = std::min(10.0, std::max(0.0, thickness));
  std::size_t resolution = 72;
  double ring_outer_scale = 0.7 * size;
  double ring_inner_scale = ring_outer_scale * (1 - 0.1 * thickness);
  double plane_corner = 0.9 * ring_inner_scale;
  double plane_length = plane_corner / sqrt(2);

  // Create translation arrows
  for (std::size_t a = 0; a < 3; ++a) {
    Eigen::Vector3d tail(Eigen::Vector3d::Zero());
    tail[a] = ring_inner_scale;
    Eigen::Vector3d head(Eigen::Vector3d::Zero());
    head[a] = size;
    Eigen::Vector4d color(Eigen::Vector4d::Ones());
    color *= 0.2;
    color[a] = 0.9;
    color[3] = getTool(InteractiveTool::LINEAR, a)->getDefaultAlpha();

    dart::dynamics::ArrowShape::Properties p;
    p.mRadius = thickness * size * 0.03;
    p.mHeadRadiusScale = 2;
    p.mHeadLengthScale = 0.4;
    p.mDoubleArrow = false;

    mTools[InteractiveTool::LINEAR][a]->addShapeFrame(dart::dynamics::ShapePtr(
        new dart::dynamics::ArrowShape(tail, head, p, color, 100)));

    tail[a] = -ring_inner_scale;
    head[a] = -size;

    mTools[InteractiveTool::LINEAR][a]->addShapeFrame(dart::dynamics::ShapePtr(
        new dart::dynamics::ArrowShape(tail, head, p, color, 100)));
  }

  // Create rotation rings - Generate TriMesh directly
  for (std::size_t r = 0; r < 3; ++r) {
    auto triMesh = std::make_shared<dart::math::TriMesh<double>>();

    std::size_t numVertices = 8 * resolution;
    std::size_t R = 4 * resolution;
    std::size_t numFaces = 4 * resolution;
    std::size_t H = resolution / 2;

    triMesh->reserveVertices(numVertices);
    triMesh->reserveTriangles(numFaces);

    // Generate vertices
    for (std::size_t j = 0; j < 2; ++j) {
      for (std::size_t i = 0; i < resolution; ++i) {
        double theta = (double)(i) / (double)(resolution)*2 * pi;

        // Inner ring vertices
        double x = 0;
        double y = ring_inner_scale * cos(theta);
        double z = ring_inner_scale * sin(theta);
        triMesh->addVertex(Eigen::Vector3d(x, y, z)); // Front: index 4*i + j

        // Outer ring vertices
        y = ring_outer_scale * cos(theta);
        z = ring_outer_scale * sin(theta);
        triMesh->addVertex(
            Eigen::Vector3d(x, y, z)); // Front: index 4*i + 2 + j
      }
    }

    // Add back face vertices (duplicate of front, for proper rendering)
    for (std::size_t j = 0; j < 2; ++j) {
      for (std::size_t i = 0; i < resolution; ++i) {
        double theta = (double)(i) / (double)(resolution)*2 * pi;

        // Inner ring vertices (back)
        double x = 0;
        double y = ring_inner_scale * cos(theta);
        double z = ring_inner_scale * sin(theta);
        triMesh->addVertex(Eigen::Vector3d(x, y, z)); // Back: index 4*i + j + R

        // Outer ring vertices (back)
        y = ring_outer_scale * cos(theta);
        z = ring_outer_scale * sin(theta);
        triMesh->addVertex(
            Eigen::Vector3d(x, y, z)); // Back: index 4*i + 2 + j + R
      }
    }

    // Generate triangles
    for (std::size_t i = 0; i < H; ++i) {
      // Front faces
      triMesh->addTriangle(dart::math::TriMesh<double>::Triangle(
          8 * i, 8 * i + 2, (i + 1 < H) ? 8 * i + 6 : 2));

      triMesh->addTriangle(dart::math::TriMesh<double>::Triangle(
          8 * i, (i + 1 < H) ? 8 * i + 6 : 2, (i + 1 < H) ? 8 * i + 4 : 0));

      triMesh->addTriangle(dart::math::TriMesh<double>::Triangle(
          8 * i + 5, 8 * i + 7, (i + 1 < H) ? 8 * i + 11 : 3));

      triMesh->addTriangle(dart::math::TriMesh<double>::Triangle(
          8 * i + 5,
          (i + 1 < H) ? 8 * i + 11 : 3,
          (i + 1 < H) ? 8 * i + 9 : 1));

      // Back faces
      triMesh->addTriangle(dart::math::TriMesh<double>::Triangle(
          8 * i + R, (i + 1 < H) ? 8 * i + 6 + R : 2 + R, 8 * i + 2 + R));

      triMesh->addTriangle(dart::math::TriMesh<double>::Triangle(
          8 * i + R,
          (i + 1 < H) ? 8 * i + 4 + R : 0,
          (i + 1 < H) ? 8 * i + 6 + R : 2));

      triMesh->addTriangle(dart::math::TriMesh<double>::Triangle(
          8 * i + 5 + R, (i + 1 < H) ? 8 * i + 11 + R : 3 + R, 8 * i + 7 + R));

      triMesh->addTriangle(dart::math::TriMesh<double>::Triangle(
          8 * i + 5 + R,
          (i + 1 < H) ? 8 * i + 9 + R : 1 + R,
          (i + 1 < H) ? 8 * i + 11 + R : 3 + R));
    }

    std::shared_ptr<dart::dynamics::MeshShape> shape(
        new dart::dynamics::MeshShape(Eigen::Vector3d::Ones(), triMesh));
    shape->setColorMode(dart::dynamics::MeshShape::COLOR_INDEX);

    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    if (r == 1)
      tf.rotate(Eigen::AngleAxisd(pi / 2, Eigen::Vector3d(0, 0, 1)));
    else if (r == 2)
      tf.rotate(Eigen::AngleAxisd(pi / 2, Eigen::Vector3d(0, 1, 0)));

    auto shapeFrame = mTools[InteractiveTool::ANGULAR][r]->addShapeFrame(shape);
    shapeFrame->setRelativeTransform(tf);
  }

  // Create translation planes - Generate TriMesh directly
  for (std::size_t p = 0; p < 3; ++p) {
    auto triMesh = std::make_shared<dart::math::TriMesh<double>>();

    double L = plane_length;

    // Add 8 vertices (2 quads for front and back faces)
    triMesh->reserveVertices(8);
    triMesh->reserveTriangles(4);

    // Front face vertices (indices 0-3)
    triMesh->addVertex(Eigen::Vector3d(0, -L, -L)); // 0
    triMesh->addVertex(Eigen::Vector3d(0, L, -L));  // 1
    triMesh->addVertex(Eigen::Vector3d(0, -L, L));  // 2
    triMesh->addVertex(Eigen::Vector3d(0, L, L));   // 3

    // Back face vertices (indices 4-7)
    triMesh->addVertex(Eigen::Vector3d(0, -L, -L)); // 4
    triMesh->addVertex(Eigen::Vector3d(0, L, -L));  // 5
    triMesh->addVertex(Eigen::Vector3d(0, -L, L));  // 6
    triMesh->addVertex(Eigen::Vector3d(0, L, L));   // 7

    // Add triangles (2 per face, 4 total)
    // Front face
    triMesh->addTriangle(dart::math::TriMesh<double>::Triangle(0, 1, 3));
    triMesh->addTriangle(dart::math::TriMesh<double>::Triangle(0, 3, 2));

    // Back face
    triMesh->addTriangle(dart::math::TriMesh<double>::Triangle(4, 7, 5));
    triMesh->addTriangle(dart::math::TriMesh<double>::Triangle(4, 6, 7));

    std::shared_ptr<dart::dynamics::MeshShape> shape(
        new dart::dynamics::MeshShape(Eigen::Vector3d::Ones(), triMesh));
    shape->setColorMode(dart::dynamics::MeshShape::COLOR_INDEX);

    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    if (p == 1)
      tf.rotate(Eigen::AngleAxisd(pi / 2, Eigen::Vector3d(0, 0, 1)));
    else if (p == 2)
      tf.rotate(Eigen::AngleAxisd(pi / 2, Eigen::Vector3d(0, 1, 0)));

    auto shapeFrame = mTools[InteractiveTool::PLANAR][p]->addShapeFrame(shape);
    shapeFrame->setRelativeTransform(tf);
  }

  for (std::size_t i = 0; i < InteractiveTool::NUM_TYPES; ++i) {
    for (std::size_t j = 0; j < 3; ++j) {
      const auto& shapesFrames = mTools[i][j]->getShapeFrames();
      for (std::size_t s = 0; s < shapesFrames.size(); ++s) {
        shapesFrames[s]->getShape()->setDataVariance(
            dart::dynamics::Shape::DYNAMIC_COLOR);
      }
    }
  }

  // Create axes
  for (std::size_t i = 0; i < 3; ++i) {
    std::shared_ptr<dart::dynamics::LineSegmentShape> line(
        new dart::dynamics::LineSegmentShape(3.0));
    line->addVertex(Eigen::Vector3d::Zero());
    Eigen::Vector3d v(Eigen::Vector3d::Zero());
    v[i] = 0.9 * size;
    line->addVertex(v);
    Eigen::Vector3d c(Eigen::Vector3d::Zero());
    c[i] = 1.0;
    auto shapeFrame = addShapeFrame(line);
    shapeFrame->getVisualAspect(true)->setColor(c);
  }
}

//==============================================================================
void InteractiveFrame::deleteAllVisualizationShapes()
{
  removeAllShapeFrames();

  for (std::size_t i = 0; i < InteractiveTool::NUM_TYPES; ++i) {
    for (std::size_t j = 0; j < 3; ++j) {
      InteractiveTool* tool = mTools[i][j];
      tool->removeAllShapeFrames();
    }
  }
}

//==============================================================================
void InteractiveFrame::deleteAllTools()
{
  for (std::size_t i = 0; i < InteractiveTool::NUM_TYPES; ++i)
    for (std::size_t j = 0; j < 3; ++j)
      delete mTools[i][j];
}

} // namespace osg
} // namespace gui
} // namespace dart
