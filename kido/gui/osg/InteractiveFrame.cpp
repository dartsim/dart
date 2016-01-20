/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <mxgrey@gatech.edu>
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

#include "osgKido/InteractiveFrame.hpp"
#include "kido/dynamics/MeshShape.hpp"
#include "kido/dynamics/ArrowShape.hpp"
#include "kido/dynamics/LineSegmentShape.hpp"
#include "kido/common/Console.hpp"

namespace osgKido {

//==============================================================================
InteractiveTool::InteractiveTool(InteractiveFrame* frame, double defaultAlpha,
                                 const std::string& name)
  : Entity(frame, name, false),
    mDefaultAlpha(defaultAlpha),
    mEnabled(true),
    mInteractiveFrame(frame)
{

}

//==============================================================================
void InteractiveTool::setEnabled(bool enabled)
{
  mEnabled = enabled;
  for(size_t i=0; i<mEntityP.mVizShapes.size(); ++i)
    mEntityP.mVizShapes[i]->setHidden(!enabled);
}

//==============================================================================
bool InteractiveTool::getEnabled() const
{
  return mEnabled;
}

//==============================================================================
void InteractiveTool::setAlpha(double alpha)
{
  for(size_t i=0; i<mEntityP.mVizShapes.size(); ++i)
    mEntityP.mVizShapes[i]->setAlpha(alpha);
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
InteractiveFrame::InteractiveFrame(
    kido::dynamics::Frame* referenceFrame,
    const std::string& name,
    const Eigen::Isometry3d& relativeTransform,
    double size_scale, double thickness_scale)
  : Entity(referenceFrame, name, false),
    SimpleFrame(referenceFrame, name, relativeTransform)
{
  for(size_t i=0; i<3; ++i)
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
                                           size_t coordinate)
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

  return mTools[(size_t)tool][coordinate];
}

//==============================================================================
const InteractiveTool* InteractiveFrame::getTool(
    InteractiveTool::Type tool, size_t coordinate) const
{
  return const_cast<InteractiveFrame*>(this)->getTool(tool, coordinate);
}

//==============================================================================
void InteractiveFrame::createStandardVisualizationShapes(double size,
                                                         double thickness)
{
  thickness = std::min(10.0, std::max(0.0, thickness));
  size_t resolution = 72;
  double ring_outer_scale = 0.7*size;
  double ring_inner_scale = ring_outer_scale*(1-0.1*thickness);
  double plane_corner = 0.9*ring_inner_scale;
  double plane_length = plane_corner/sqrt(2);

  // Create translation arrows
  for(size_t a=0; a<3; ++a)
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

    kido::dynamics::ArrowShape::Properties p;
    p.mRadius = thickness*size*0.03;
    p.mHeadRadiusScale = 2;
    p.mHeadLengthScale = 0.4;
    p.mDoubleArrow = false;

    mTools[InteractiveTool::LINEAR][a]->addVisualizationShape(
          kido::dynamics::ShapePtr(
            new kido::dynamics::ArrowShape(tail, head, p, color, 100)));

//    tail[a] = -1.2*plane_length;
    tail[a] = -ring_inner_scale;
    head[a] = -size;

    mTools[InteractiveTool::LINEAR][a]->addVisualizationShape(
          kido::dynamics::ShapePtr(
            new kido::dynamics::ArrowShape(tail, head, p, color, 100)));
  }

  // Create rotation rings
  for(size_t r=0; r<3; ++r)
  {
    aiMesh* mesh = new aiMesh;
    mesh->mMaterialIndex = (unsigned int)(-1);

    size_t numVertices = 8*resolution;
    size_t R = 4*resolution;
    mesh->mNumVertices = numVertices;
    mesh->mVertices = new aiVector3D[numVertices];
    mesh->mNormals = new aiVector3D[numVertices];
    mesh->mColors[0] = new aiColor4D[numVertices];
    aiVector3D vertex;
    aiVector3D normal;
    aiColor4D color1;
    aiColor4D color2;
    for(size_t j=0; j<2; ++j)
    {
      for(size_t i=0; i<resolution; ++i)
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

        for(size_t c=0; c<3; ++c)
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

    size_t numFaces = 4*resolution;
    size_t F = 2*resolution;
    size_t H = resolution/2;
    mesh->mNumFaces = numFaces;
    mesh->mFaces = new aiFace[numFaces];
    for(size_t i=0; i<H; ++i)
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

    std::shared_ptr<kido::dynamics::MeshShape> shape(
        new kido::dynamics::MeshShape(Eigen::Vector3d::Ones(), scene));
    shape->setColorMode(kido::dynamics::MeshShape::COLOR_INDEX);

    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    if( r == 1 )
      tf.rotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(0,0,1)));
    else if( r == 2 )
      tf.rotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(0,1,0)));

    shape->setLocalTransform(tf);

    mTools[InteractiveTool::ANGULAR][r]->addVisualizationShape(shape);
  }

  // Create translation planes
  for(size_t p=0; p<3; ++p)
  {
    aiMesh* mesh = new aiMesh;
    mesh->mMaterialIndex = (unsigned int)(-1);

    size_t numVertices = 8;
    mesh->mNumVertices = numVertices;
    mesh->mVertices = new aiVector3D[numVertices];
    mesh->mNormals = new aiVector3D[numVertices];
    mesh->mColors[0] = new aiColor4D[numVertices];

    double L = plane_length;
    for(size_t i=0; i<2; ++i)
    {
      mesh->mVertices[4*i+0] = aiVector3D(0, -L, -L);
      mesh->mVertices[4*i+1] = aiVector3D(0,  L, -L);
      mesh->mVertices[4*i+2] = aiVector3D(0, -L,  L);
      mesh->mVertices[4*i+3] = aiVector3D(0,  L,  L);
    }

    for(size_t i=0; i<4; ++i)
    {
      mesh->mNormals[i] = aiVector3D(1, 0, 0);
      mesh->mNormals[i+4] = aiVector3D(-1, 0, 0);
    }

    aiColor4D color(0.1, 0.1, 0.1,
                    getTool(InteractiveTool::PLANAR,p)->getDefaultAlpha());
    color[p] = 0.9;
    for(size_t i=0; i<numVertices; ++i)
      mesh->mColors[0][i] = color;

    size_t numFaces = 4;
    mesh->mNumFaces = numFaces;
    mesh->mFaces = new aiFace[numFaces];
    for(size_t i=0; i<numFaces; ++i)
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

    std::shared_ptr<kido::dynamics::MeshShape> shape(
        new kido::dynamics::MeshShape(Eigen::Vector3d::Ones(), scene));
    shape->setColorMode(kido::dynamics::MeshShape::COLOR_INDEX);

    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    if( p == 1 )
      tf.rotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(0,0,1)));
    else if( p == 2 )
      tf.rotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(0,1,0)));

    shape->setLocalTransform(tf);

    mTools[InteractiveTool::PLANAR][p]->addVisualizationShape(shape);
  }

  for(size_t i=0; i<InteractiveTool::NUM_TYPES; ++i)
  {
    for(size_t j=0; j<3; ++j)
    {
      const std::vector<kido::dynamics::ShapePtr>& shapes =
          mTools[i][j]->getVisualizationShapes();
      for(size_t s=0; s<shapes.size(); ++s)
        shapes[s]->setDataVariance(kido::dynamics::Shape::DYNAMIC_COLOR);
    }
  }

  // Create axes
  for(size_t i=0; i<3; ++i)
  {
    std::shared_ptr<kido::dynamics::LineSegmentShape> line(
        new kido::dynamics::LineSegmentShape(3.0));
    line->addVertex(Eigen::Vector3d::Zero());
    Eigen::Vector3d v(Eigen::Vector3d::Zero());
    v[i] = 0.9*size;
    line->addVertex(v);
    Eigen::Vector3d c(Eigen::Vector3d::Zero());
    c[i] = 1.0;
    line->setColor(c);
    addVisualizationShape(line);
  }
}

//==============================================================================
void InteractiveFrame::deleteAllVisualizationShapes()
{
  mEntityP.mVizShapes.clear();

  for(size_t i=0; i<InteractiveTool::NUM_TYPES; ++i)
  {
    for(size_t j=0; j<3; ++j)
    {
      InteractiveTool* tool = mTools[i][j];
      tool->removeAllVisualizationShapes();
    }
  }
}

//==============================================================================
void InteractiveFrame::deleteAllTools()
{
  for(size_t i=0; i<InteractiveTool::NUM_TYPES; ++i)
    for(size_t j=0; j<3; ++j)
      delete mTools[i][j];
}

} // namespace osgKido

