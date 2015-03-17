/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <mxgrey@gatech.edu>
 *            Pete Vieira <pete.vieira@gatech.edu>
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

#include <map>

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/CullFace>

#include "osgDart/render/MeshShapeNode.h"
#include "osgDart/utils.h"

#include "dart/dynamics/MeshShape.h"

namespace osgDart {
namespace render {

class osgAiNode : public ShapeNode, public osg::MatrixTransform
{
public:

  osgAiNode(dart::dynamics::MeshShape* shape, EntityNode* parentEntity,
            const aiNode* node);

  void refresh();
  void extractData(bool firstTime);

  const aiNode* getAiNodePtr() const { return mAiNode; }

protected:

  virtual ~osgAiNode();

  void clearChildUtilizationFlags();
  void clearUnusedNodes();

  dart::dynamics::MeshShape* mMeshShape;
  const aiNode* mAiNode;
  MeshShapeGeode* mGeode;
  std::map<const aiNode*,osgAiNode*> mChildNodes;

};

//==============================================================================
class MeshShapeGeode : public ShapeNode, public osg::Geode
{
public:

  MeshShapeGeode(dart::dynamics::MeshShape* shape, EntityNode* parentEntity,
                 const aiNode* node);

  void refresh();
  void extractData(bool firstTime);

protected:

  virtual ~MeshShapeGeode();

  void clearChildUtilizationFlags();
  void clearUnusedMeshes();

  dart::dynamics::MeshShape* mMeshShape;
  const aiNode* mAiNode;
  std::map<const aiMesh*,MeshShapeGeometry*> mMeshes;

};

//==============================================================================
class MeshShapeGeometry : public ShapeNode, public osg::Geometry
{
public:

  MeshShapeGeometry(dart::dynamics::MeshShape* shape, EntityNode* parentEntity,
                    MeshShapeGeode* parentGeode, aiMesh* mesh);

  void refresh();

  void extractData(bool firstTime);

protected:

  virtual ~MeshShapeGeometry();

  osg::ref_ptr<osg::Vec3Array> mVertices;
  osg::ref_ptr<osg::Vec3Array> mNormals;
  osg::ref_ptr<osg::Vec4Array> mColors;

  dart::dynamics::MeshShape* mMeshShape;
  const aiMesh* mAiMesh;

};

//==============================================================================
MeshShapeNode::MeshShapeNode(dart::dynamics::MeshShape* shape,
                             EntityNode* parentEntity)
  : ShapeNode(shape, parentEntity, this),
    mMeshShape(shape),
    mRootAiNode(nullptr)
{
  extractData(true);
}

//==============================================================================
void MeshShapeNode::refresh()
{
  mUtilized = true;

  if(mShape->getDataVariance() == dart::dynamics::Shape::STATIC)
    return;

  extractData(false);
}

//==============================================================================
void MeshShapeNode::extractData(bool firstTime)
{
  if(   mShape->checkDataVariance(dart::dynamics::Shape::DYNAMIC_TRANSFORM)
     || mShape->checkDataVariance(dart::dynamics::Shape::DYNAMIC_PRIMITIVE)
     || firstTime)
  {
    Eigen::Matrix4d S(Eigen::Matrix4d::Zero());
    const Eigen::Vector3d& s = mMeshShape->getScale();
    S(0,0) = s[0]; S(1,1) = s[1]; S(2,2) = s[2]; S(3,3) = 1.0;
    setMatrix(eigToOsgMatrix(mShape->getLocalTransform()*S));
  }

  const aiScene* scene = mMeshShape->getMesh();
  const aiNode* root = scene->mRootNode;

  if(mRootAiNode)
  {
    // If the root node pointer has changed, delete our current mRootNode and
    // then recreate it
    if(mRootAiNode->getAiNodePtr() != root)
    {
      removeChild(mRootAiNode);
      mRootAiNode = nullptr;
    }
  }

  if( (nullptr == mRootAiNode) && root)
  {
    mRootAiNode = new osgAiNode(mMeshShape, mParentEntity, root);
    addChild(mRootAiNode);
    return;
  }

  mRootAiNode->refresh();
}

//==============================================================================
MeshShapeNode::~MeshShapeNode()
{
  // Do nothing
}

//==============================================================================
osgAiNode::osgAiNode(dart::dynamics::MeshShape* shape,
                     EntityNode* parentEntity, const aiNode* node)
  : ShapeNode(shape, parentEntity, this),
    mMeshShape(shape),
    mAiNode(node),
    mGeode(nullptr)
{
  extractData(true);
}

//==============================================================================
void osgAiNode::refresh()
{
  mUtilized = true;

  if(mShape->getDataVariance() == dart::dynamics::Shape::STATIC)
    return;

  extractData(false);
}

//==============================================================================
void osgAiNode::extractData(bool firstTime)
{
  clearChildUtilizationFlags();

  if(   mShape->checkDataVariance(dart::dynamics::Shape::DYNAMIC_TRANSFORM)
     || firstTime)
  {
    aiMatrix4x4 M = mAiNode->mTransformation;
    M.Transpose();
    setMatrix(osg::Matrixf((float*)(&M)));
  }

  for(size_t i=0; i<mAiNode->mNumChildren; ++i)
  {
    aiNode* child = mAiNode->mChildren[i];
    std::map<const aiNode*,osgAiNode*>::iterator it = mChildNodes.find(child);

    if(it == mChildNodes.end())
    {
      osgAiNode* newChild = new osgAiNode(mMeshShape, mParentEntity, child);
      addChild(newChild);
    }
    else
      it->second->refresh();
  }

  if(nullptr == mGeode)
  {
    mGeode = new MeshShapeGeode(mMeshShape, mParentEntity, mAiNode);
    addChild(mGeode);
  }
  else
    mGeode->refresh();

  clearUnusedNodes();
}

//==============================================================================
osgAiNode::~osgAiNode()
{
  // Do nothing
}

//==============================================================================
void osgAiNode::clearChildUtilizationFlags()
{
  for(auto& node_pair : mChildNodes)
    node_pair.second->clearUtilization();
}

//==============================================================================
void osgAiNode::clearUnusedNodes()
{
  for(auto& node_pair : mChildNodes)
  {
    osgAiNode* node = node_pair.second;
    if(!node->wasUtilized())
    {
      mChildNodes.erase(node_pair.first);
      removeChild(node);
    }
  }
}

//==============================================================================
MeshShapeGeode::MeshShapeGeode(dart::dynamics::MeshShape* shape,
                               EntityNode* parentEntity, const aiNode* node)
  : ShapeNode(shape, parentEntity, this),
    mMeshShape(shape),
    mAiNode(node)
{
  getOrCreateStateSet()->setAttributeAndModes(new osg::CullFace(osg::CullFace::BACK));
  extractData(true);
}

//==============================================================================
void MeshShapeGeode::refresh()
{
  mUtilized = true;

  if(mShape->getDataVariance() == dart::dynamics::Shape::STATIC)
    return;

  extractData(false);
}

//==============================================================================
void MeshShapeGeode::extractData(bool)
{
  clearChildUtilizationFlags();

  const aiScene* scene = mMeshShape->getMesh();
  for(size_t i=0; i<mAiNode->mNumMeshes; ++i)
  {
    aiMesh* mesh = scene->mMeshes[mAiNode->mMeshes[i]];

    std::map<const aiMesh*,MeshShapeGeometry*>::iterator it =
        mMeshes.find(mesh);

    if(it == mMeshes.end())
    {
      MeshShapeGeometry* newMesh =
          new MeshShapeGeometry(mMeshShape, mParentEntity, this, mesh);
      addDrawable(newMesh);
      mMeshes[mesh] = newMesh;
    }
    else
      it->second->refresh();
  }

  clearUnusedMeshes();
}

//==============================================================================
MeshShapeGeode::~MeshShapeGeode()
{
  // Do nothing
}

//==============================================================================
void MeshShapeGeode::clearChildUtilizationFlags()
{
  for(auto& node_pair : mMeshes)
  {
    MeshShapeGeometry* geom = node_pair.second;
    geom->clearUtilization();
  }
}

//==============================================================================
void MeshShapeGeode::clearUnusedMeshes()
{
  for(auto& node_pair : mMeshes)
  {
    MeshShapeGeometry* geom = node_pair.second;
    if(!geom->wasUtilized())
    {
      mMeshes.erase(node_pair.first);
      removeDrawable(geom);
    }
  }
}

//==============================================================================
MeshShapeGeometry::MeshShapeGeometry(dart::dynamics::MeshShape* shape,
                                     EntityNode* parentEntity,
                                     MeshShapeGeode* parentGeode,
                                     aiMesh* mesh)
  : ShapeNode(shape, parentEntity, parentGeode),
    mVertices(new osg::Vec3Array),
    mNormals(new osg::Vec3Array),
    mColors(new osg::Vec4Array),
    mMeshShape(shape),
    mAiMesh(mesh)
{
  extractData(true);
}

//==============================================================================
void MeshShapeGeometry::refresh()
{
  mUtilized = true;

  if(mShape->getDataVariance() == dart::dynamics::Shape::STATIC)
    return;

  extractData(false);
}

//==============================================================================
void MeshShapeGeometry::extractData(bool firstTime)
{
  if(mShape->getDataVariance() == dart::dynamics::Shape::STATIC)
    setDataVariance(osg::Object::STATIC);
  else
    setDataVariance(osg::Object::DYNAMIC);

  if(   mShape->checkDataVariance(dart::dynamics::Shape::DYNAMIC_ELEMENTS)
     || firstTime)
  {
    osg::ref_ptr<osg::DrawElementsUInt> elements[4];
    elements[0] = new osg::DrawElementsUInt(GL_POINTS);
    elements[1] = new osg::DrawElementsUInt(GL_LINES);
    elements[2] = new osg::DrawElementsUInt(GL_TRIANGLES);
    elements[3] = new osg::DrawElementsUInt(GL_QUADS);

    for(size_t i=0; i<mAiMesh->mNumFaces; ++i)
    {
      const aiFace& face = mAiMesh->mFaces[i];

      if(face.mNumIndices > 4) // We have some arbitrary polygon
      {
        osg::ref_ptr<osg::DrawElementsUInt> polygon =
            new osg::DrawElementsUInt(GL_POLYGON);
        for(size_t j=0; j<face.mNumIndices; ++j)
          polygon->push_back(face.mIndices[j]);
        addPrimitiveSet(polygon);
      }
      else if(face.mNumIndices > 0)
      {
        osg::DrawElementsUInt* elem = elements[face.mNumIndices-1];
        for(size_t j=0; j<face.mNumIndices; ++j)
          elem->push_back(face.mIndices[j]);
      }
    }

    for(size_t i=0; i<4; ++i)
      if(elements[i]->size() > 0)
        addPrimitiveSet(elements[i]);
  }

  if(   mShape->checkDataVariance(dart::dynamics::Shape::DYNAMIC_VERTICES)
     || mShape->checkDataVariance(dart::dynamics::Shape::DYNAMIC_ELEMENTS)
     || firstTime)
  {
    if(mVertices->size() != mAiMesh->mNumVertices)
      mVertices->resize(mAiMesh->mNumVertices);

    if(mNormals->size() != mAiMesh->mNumVertices)
      mNormals->resize(mAiMesh->mNumVertices);

    for(size_t i=0; i<mAiMesh->mNumVertices; ++i)
    {
      const aiVector3D& v = mAiMesh->mVertices[i];
      (*mVertices)[i] = osg::Vec3(v.x, v.y, v.z);

      if(mAiMesh->mNormals)
      {
        const aiVector3D& n = mAiMesh->mNormals[i];
        (*mNormals)[i] = osg::Vec3(n.x, n.y, n.z);
      }
      // TODO(MXG): Consider computing normals for meshes that don't come with
      // normal data per vertex
    }

    setVertexArray(mVertices);
    if(mAiMesh->mNormals)
      setNormalArray(mNormals, osg::Array::BIND_PER_VERTEX);
  }

  if(   mShape->checkDataVariance(dart::dynamics::Shape::DYNAMIC_COLOR)
     || firstTime)
  {
    // TODO(MXG): Should we support using the alternative 8 color sets that
    // assimp allows?
    if(mAiMesh->mColors[0])
    {
      if(mColors->size() != mVertices->size())
        mColors->resize(mVertices->size());

      for(size_t i=0; i<mAiMesh->mNumVertices; ++i)
      {
        const aiColor4D& c = mAiMesh->mColors[0][i];
        (*mColors)[i] = osg::Vec4(c.r, c.g, c.b, c.a);
      }

      setColorArray(mColors);
      setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    }
    // TODO(MXG): Consider setting a default coloring if none is provided
    // by the mesh

    uint unit = 0;
    const aiVector3D* aiTexCoords = mAiMesh->mTextureCoords[unit];
    while(nullptr != aiTexCoords)
    {
      switch(mAiMesh->mNumUVComponents[unit])
      {
        case 1:
        {
          osg::ref_ptr<osg::FloatArray> texture =
              new osg::FloatArray(mAiMesh->mNumVertices);
          for(size_t i=0; i<mAiMesh->mNumVertices; ++i)
            (*texture)[i] = aiTexCoords[i].x;
          setTexCoordArray(unit, texture, osg::Array::BIND_PER_VERTEX);
          break;
        }
        case 2:
        {
          osg::ref_ptr<osg::Vec2Array> texture =
              new osg::Vec2Array(mAiMesh->mNumVertices);
          for(size_t i=0; i<mAiMesh->mNumVertices; ++i)
          {
            const aiVector3D& t = aiTexCoords[i];
            (*texture)[i] = osg::Vec2(t.x, t.y);
          }
          setTexCoordArray(unit, texture, osg::Array::BIND_PER_VERTEX);
          break;
        }
        case 3:
        {
          osg::ref_ptr<osg::Vec3Array> texture =
              new osg::Vec3Array(mAiMesh->mNumVertices);
          for(size_t i=0; i<mAiMesh->mNumVertices; ++i)
          {
            const aiVector3D& t = aiTexCoords[i];
            (*texture)[i] = osg::Vec3(t.x, t.y, t.z);
          }
          setTexCoordArray(unit, texture, osg::Array::BIND_PER_VERTEX);
          break;
        }
      } // switch(mAiMesh->mNumUVComponents[unit])
    } // while(nullptr != aiTexCoords)
  }
}

//==============================================================================
MeshShapeGeometry::~MeshShapeGeometry()
{
  // Do nothing
}

} // render
} // osgDart
