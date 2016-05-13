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

#include "dart/gui/osg/InteractiveFrame.hpp"
#include "dart/dynamics/MeshShape.hpp"
#include "dart/dynamics/ArrowShape.hpp"
#include "dart/dynamics/LineSegmentShape.hpp"
#include "dart/common/Console.hpp"

namespace dart {
namespace gui {
namespace osg {

//==============================================================================
InteractiveTool::InteractiveTool(InteractiveFrame* frame, double defaultAlpha,
                                 const std::string& name)
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
  for(auto& frame : mSimpleFrames)
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
  for(auto& frame : mSimpleFrames)
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
  if(reset)
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
  mSimpleFrames.push_back(
      dart::common::make_unique<dart::dynamics::SimpleFrame>(this));

  auto shapeFrame = mSimpleFrames.back().get();
  shapeFrame->setShape(shape);
  shapeFrame->createVisualAspect();

  return shapeFrame;
}

//==============================================================================
const std::vector<dart::dynamics::SimpleFrame*>
InteractiveTool::getShapeFrames()
{
  std::vector<dart::dynamics::SimpleFrame*> frames(mSimpleFrames.size());

  for(auto i = 0u; i < frames.size(); ++i)
    frames[i] = mSimpleFrames[i].get();

  return frames;
}

//==============================================================================
const std::vector<const dart::dynamics::SimpleFrame*>
InteractiveTool::getShapeFrames() const
{
  std::vector<const dart::dynamics::SimpleFrame*> frames(mSimpleFrames.size());

  for(auto i = 0u; i < frames.size(); ++i)
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
    double size_scale, double thickness_scale)
  : Entity(referenceFrame, false),
    Frame(referenceFrame),
    SimpleFrame(referenceFrame, name, relativeTransform)
{
  for(std::size_t i=0; i<3; ++i)
  {
    std::string affix = (i==0)? "x" : (i==1)? "y" : "z";

    mTools[InteractiveTool::LINEAR][i] =
        new InteractiveTool(this, 0.8, "LINEAR_"+affix);
    mTools[InteractiveTool::ANGULAR][i] =
        new InteractiveTool(this, 0.8, "ANGULAR_"+affix);
    mTools[InteractiveTool::PLANAR][i] =
        new InteractiveTool(this, 0.7, "PLANAR_"+affix);
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
void InteractiveFrame::resizeStandardVisuals(double size_scale,
                                             double thickness_scale)
{
  deleteAllVisualizationShapes();
  createStandardVisualizationShapes(size_scale, thickness_scale);
}

//==============================================================================
InteractiveTool* InteractiveFrame::getTool(InteractiveTool::Type tool,
                                           std::size_t coordinate)
{
  if(InteractiveTool::NUM_TYPES <= tool)
  {
    dtwarn << "[InteractiveFrame::getTool] Attempting to access tool #"
           << tool << ", but tools only go up to "
           << InteractiveTool::NUM_TYPES << "\n";
    return nullptr;
  }

  if(3 <= coordinate)
  {
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
  mSimpleFrames.push_back(
      dart::common::make_unique<dart::dynamics::SimpleFrame>(this));

  auto shapeFrame = mSimpleFrames.back().get();
  shapeFrame->setShape(shape);
  shapeFrame->createVisualAspect();

  return shapeFrame;
}

//==============================================================================
const std::vector<dart::dynamics::SimpleFrame*>
InteractiveFrame::getShapeFrames()
{
  std::vector<dart::dynamics::SimpleFrame*> frames(mSimpleFrames.size());

  for(auto i = 0u; i < frames.size(); ++i)
    frames[i] = mSimpleFrames[i].get();

  return frames;
}

//==============================================================================
const std::vector<const dart::dynamics::SimpleFrame*>
InteractiveFrame::getShapeFrames() const
{
  std::vector<const dart::dynamics::SimpleFrame*> frames(mSimpleFrames.size());

  for(auto i = 0u; i < frames.size(); ++i)
    frames[i] = mSimpleFrames[i].get();

  return frames;
}

//==============================================================================
void InteractiveFrame::removeAllShapeFrames()
{
  mSimpleFrames.clear();
}

//==============================================================================
void InteractiveFrame::createStandardVisualizationShapes(double size,
                                                         double thickness)
{
  thickness = std::min(10.0, std::max(0.0, thickness));
  std::size_t resolution = 72;
  double ring_outer_scale = 0.7*size;
  double ring_inner_scale = ring_outer_scale*(1-0.1*thickness);
  double plane_corner = 0.9*ring_inner_scale;
  double plane_length = plane_corner/sqrt(2);

  // Create translation arrows
  for(std::size_t a=0; a<3; ++a)
  {
    Eigen::Vector3d tail(Eigen::Vector3d::Zero());
//    tail[a] = 1.2*plane_length;
    tail[a] = ring_inner_scale;
    Eigen::Vector3d head(Eigen::Vector3d::Zero());
    head[a] = size;
    Eigen::Vector4d color(Eigen::Vector4d::Ones());
    color *= 0.2;
    color[a] = 0.9;
    color[3] = getTool(InteractiveTool::LINEAR,a)->getDefaultAlpha();

    dart::dynamics::ArrowShape::Properties p;
    p.mRadius = thickness*size*0.03;
    p.mHeadRadiusScale = 2;
    p.mHeadLengthScale = 0.4;
    p.mDoubleArrow = false;

    mTools[InteractiveTool::LINEAR][a]->addShapeFrame(
          dart::dynamics::ShapePtr(
            new dart::dynamics::ArrowShape(tail, head, p, color, 100)));

//    tail[a] = -1.2*plane_length;
    tail[a] = -ring_inner_scale;
    head[a] = -size;

    mTools[InteractiveTool::LINEAR][a]->addShapeFrame(
          dart::dynamics::ShapePtr(
            new dart::dynamics::ArrowShape(tail, head, p, color, 100)));
  }

  // Create rotation rings
  for(std::size_t r=0; r<3; ++r)
  {
    aiMesh* mesh = new aiMesh;
    mesh->mMaterialIndex = (unsigned int)(-1);

    std::size_t numVertices = 8*resolution;
    std::size_t R = 4*resolution;
    mesh->mNumVertices = numVertices;
    mesh->mVertices = new aiVector3D[numVertices];
    mesh->mNormals = new aiVector3D[numVertices];
    mesh->mColors[0] = new aiColor4D[numVertices];
    aiVector3D vertex;
    aiVector3D normal;
    aiColor4D color1;
    aiColor4D color2;
    for(std::size_t j=0; j<2; ++j)
    {
      for(std::size_t i=0; i<resolution; ++i)
      {
        double theta = (double)(i)/(double)(resolution)*2*M_PI;

        double x = 0;
        double y = ring_inner_scale*cos(theta);
        double z = ring_inner_scale*sin(theta);
        vertex.Set(x, y, z);
        mesh->mVertices[4*i+j] = vertex; // Front

        mesh->mVertices[4*i+j+R] = vertex; // Back

        y = ring_outer_scale*cos(theta);
        z = ring_outer_scale*sin(theta);
        vertex.Set(x, y, z);
        mesh->mVertices[4*i+2+j] = vertex; // Front

        mesh->mVertices[4*i+2+j+R] = vertex; // Back

        normal.Set(1.0f, 0.0f, 0.0f);
        mesh->mNormals[4*i+j] = normal;
        mesh->mNormals[4*i+2+j] = normal;

        normal.Set(-1.0f, 0.0f, 0.0f);
        mesh->mNormals[4*i+j+R] = normal;
        mesh->mNormals[4*i+2+j+R] = normal;

        for(std::size_t c=0; c<3; ++c)
        {
          color1[c] = 0.0;
          color2[c] = 0.0;
        }
        color1[r] = 1.0;
        color2[r] = 0.6;
        color1[3] = getTool(InteractiveTool::ANGULAR,r)->getDefaultAlpha();
        color2[3] = getTool(InteractiveTool::ANGULAR,r)->getDefaultAlpha();
        mesh->mColors[0][4*i+j] = ((4*i+j)%2 == 0)? color1 : color2;
        mesh->mColors[0][4*i+j+R] = ((4*i+j+R)%2 == 0)? color1 : color2;
        mesh->mColors[0][4*i+2+j] = ((4*i+2+j)%2 == 0)? color1 : color2;
        mesh->mColors[0][4*i+2+j+R] = ((4*i+2+j+R)%2 == 0)? color1 : color2;
      }
    }

    std::size_t numFaces = 4*resolution;
    std::size_t F = 2*resolution;
    std::size_t H = resolution/2;
    mesh->mNumFaces = numFaces;
    mesh->mFaces = new aiFace[numFaces];
    for(std::size_t i=0; i<H; ++i)
    {
      // Front
      aiFace* face = &mesh->mFaces[2*i];
      face->mNumIndices = 3;
      face->mIndices = new unsigned int[3];
      face->mIndices[0] = 8*i;
      face->mIndices[1] = 8*i+2;
      face->mIndices[2] = (i+1 < H)? 8*i+6 : 2;

      face = &mesh->mFaces[2*i+2*H];
      face->mNumIndices = 3;
      face->mIndices = new unsigned int[3];
      face->mIndices[0] = 8*i;
      face->mIndices[1] = (i+1 < H)? 8*i+6 : 2;
      face->mIndices[2] = (i+1 < H)? 8*i+4 : 0;


      // Back
      face = &mesh->mFaces[2*i+F];
      face->mNumIndices = 3;
      face->mIndices = new unsigned int[3];
      face->mIndices[0] = 8*i+R;
      face->mIndices[1] = (i+1 < H)? 8*i+6+R : 2+R;
      face->mIndices[2] = 8*i+2+R;

      face = &mesh->mFaces[2*i+2*H+F];
      face->mNumIndices = 3;
      face->mIndices = new unsigned int[3];
      face->mIndices[0] = 8*i+R;
      face->mIndices[1] = (i+1 < H)? 8*i+4+R : 0;
      face->mIndices[2] = (i+1 < H)? 8*i+6+R : 2;


      // Front
      face = &mesh->mFaces[2*i+1];
      face->mNumIndices = 3;
      face->mIndices = new unsigned int[3];
      face->mIndices[0] = 8*i+5;
      face->mIndices[1] = 8*i+7;
      face->mIndices[2] = (i+1 < H)? 8*i+11 : 3;

      face = &mesh->mFaces[2*i+1+2*H];
      face->mNumIndices = 3;
      face->mIndices = new unsigned int[3];
      face->mIndices[0] = 8*i+5;
      face->mIndices[1] = (i+1 < H)? 8*i+11 : 3;
      face->mIndices[2] = (i+1 < H)? 8*i+9  : 1;


      // Back
      face = &mesh->mFaces[2*i+1+F];
      face->mNumIndices = 3;
      face->mIndices = new unsigned int[3];
      face->mIndices[0] = 8*i+5+R;
      face->mIndices[1] = (i+1 < H)? 8*i+11+R : 3+R;
      face->mIndices[2] = 8*i+7+R;

      face = &mesh->mFaces[2*i+1+2*H+F];
      face->mNumIndices = 3;
      face->mIndices = new unsigned int[3];
      face->mIndices[0] = 8*i+5+R;
      face->mIndices[1] = (i+1 < H)? 8*i+9+R  : 1+R;
      face->mIndices[2] = (i+1 < H)? 8*i+11+R : 3+R;
    }

    aiNode* node = new aiNode;
    node->mNumMeshes = 1;
    node->mMeshes = new unsigned int[1];
    node->mMeshes[0] = 0;

    aiScene* scene = new aiScene;
    scene->mNumMeshes = 1;
    scene->mMeshes = new aiMesh*[1];
    scene->mMeshes[0] = mesh;
    scene->mRootNode = node;

    std::shared_ptr<dart::dynamics::MeshShape> shape(
        new dart::dynamics::MeshShape(Eigen::Vector3d::Ones(), scene));
    shape->setColorMode(dart::dynamics::MeshShape::COLOR_INDEX);

    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    if( r == 1 )
      tf.rotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(0,0,1)));
    else if( r == 2 )
      tf.rotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(0,1,0)));

    auto shapeFrame = mTools[InteractiveTool::ANGULAR][r]->addShapeFrame(shape);
    shapeFrame->setRelativeTransform(tf);
  }

  // Create translation planes
  for(std::size_t p=0; p<3; ++p)
  {
    aiMesh* mesh = new aiMesh;
    mesh->mMaterialIndex = (unsigned int)(-1);

    std::size_t numVertices = 8;
    mesh->mNumVertices = numVertices;
    mesh->mVertices = new aiVector3D[numVertices];
    mesh->mNormals = new aiVector3D[numVertices];
    mesh->mColors[0] = new aiColor4D[numVertices];

    double L = plane_length;
    for(std::size_t i=0; i<2; ++i)
    {
      mesh->mVertices[4*i+0] = aiVector3D(0, -L, -L);
      mesh->mVertices[4*i+1] = aiVector3D(0,  L, -L);
      mesh->mVertices[4*i+2] = aiVector3D(0, -L,  L);
      mesh->mVertices[4*i+3] = aiVector3D(0,  L,  L);
    }

    for(std::size_t i=0; i<4; ++i)
    {
      mesh->mNormals[i] = aiVector3D(1, 0, 0);
      mesh->mNormals[i+4] = aiVector3D(-1, 0, 0);
    }

    aiColor4D color(0.1, 0.1, 0.1,
                    getTool(InteractiveTool::PLANAR,p)->getDefaultAlpha());
    color[p] = 0.9;
    for(std::size_t i=0; i<numVertices; ++i)
      mesh->mColors[0][i] = color;

    std::size_t numFaces = 4;
    mesh->mNumFaces = numFaces;
    mesh->mFaces = new aiFace[numFaces];
    for(std::size_t i=0; i<numFaces; ++i)
    {
      aiFace* face = &mesh->mFaces[i];
      face->mNumIndices = 3;
      face->mIndices = new unsigned int[3];
    }

    aiFace* face = &mesh->mFaces[0];
    face->mIndices[0] = 0;
    face->mIndices[1] = 1;
    face->mIndices[2] = 3;

    face = &mesh->mFaces[1];
    face->mIndices[0] = 0;
    face->mIndices[1] = 3;
    face->mIndices[2] = 2;

    face = &mesh->mFaces[2];
    face->mIndices[0] = 4;
    face->mIndices[1] = 7;
    face->mIndices[2] = 5;

    face = &mesh->mFaces[3];
    face->mIndices[0] = 4;
    face->mIndices[1] = 6;
    face->mIndices[2] = 7;

    aiNode* node = new aiNode;
    node->mNumMeshes = 1;
    node->mMeshes = new unsigned int[1];
    node->mMeshes[0] = 0;

    aiScene* scene = new aiScene;
    scene->mNumMeshes = 1;
    scene->mMeshes = new aiMesh*[1];
    scene->mMeshes[0] = mesh;
    scene->mRootNode = node;

    std::shared_ptr<dart::dynamics::MeshShape> shape(
        new dart::dynamics::MeshShape(Eigen::Vector3d::Ones(), scene));
    shape->setColorMode(dart::dynamics::MeshShape::COLOR_INDEX);

    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    if( p == 1 )
      tf.rotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(0,0,1)));
    else if( p == 2 )
      tf.rotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(0,1,0)));

    auto shapeFrame
        = mTools[InteractiveTool::PLANAR][p]->addShapeFrame(shape);
    shapeFrame->setRelativeTransform(tf);
  }

  for(std::size_t i=0; i<InteractiveTool::NUM_TYPES; ++i)
  {
    for(std::size_t j=0; j<3; ++j)
    {
      const auto& shapesFrames = mTools[i][j]->getShapeFrames();
      for(std::size_t s=0; s<shapesFrames.size(); ++s)
      {
        shapesFrames[s]->getShape()->setDataVariance(
              dart::dynamics::Shape::DYNAMIC_COLOR);
      }
    }
  }

  // Create axes
  for(std::size_t i=0; i<3; ++i)
  {
    std::shared_ptr<dart::dynamics::LineSegmentShape> line(
        new dart::dynamics::LineSegmentShape(3.0));
    line->addVertex(Eigen::Vector3d::Zero());
    Eigen::Vector3d v(Eigen::Vector3d::Zero());
    v[i] = 0.9*size;
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

  for(std::size_t i=0; i<InteractiveTool::NUM_TYPES; ++i)
  {
    for(std::size_t j=0; j<3; ++j)
    {
      InteractiveTool* tool = mTools[i][j];
      tool->removeAllShapeFrames();
    }
  }
}

//==============================================================================
void InteractiveFrame::deleteAllTools()
{
  for(std::size_t i=0; i<InteractiveTool::NUM_TYPES; ++i)
    for(std::size_t j=0; j<3; ++j)
      delete mTools[i][j];
}

} // namespace osg
} // namespace gui
} // namespace dart
