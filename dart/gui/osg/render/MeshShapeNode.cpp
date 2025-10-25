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

#include "dart/gui/osg/render/MeshShapeNode.hpp"

#include "dart/dynamics/MeshMaterial.hpp"
#include "dart/dynamics/MeshShape.hpp"
#include "dart/dynamics/SimpleFrame.hpp"
#include "dart/gui/osg/Utils.hpp"

#include <assimp/scene.h>
#include <osg/CullFace>
#include <osg/Depth>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Texture2D>
#include <osgDB/ReadFile>

#include <map>

namespace dart {
namespace gui {
namespace osg {
namespace render {

namespace {

//==============================================================================
bool isTransparent(const ::osg::Material* material)
{
  if (std::abs(material->getAmbient(::osg::Material::FRONT).a())
      < 1 - getAlphaThreshold<float>())
    return true;

  if (std::abs(material->getDiffuse(::osg::Material::FRONT).a())
      < 1 - getAlphaThreshold<float>())
    return true;

  if (std::abs(material->getSpecular(::osg::Material::FRONT).a())
      < 1 - getAlphaThreshold<float>())
    return true;

  if (std::abs(material->getEmission(::osg::Material::FRONT).a())
      < 1 - getAlphaThreshold<float>())
    return true;

  return false;
}

} // namespace

class osgAiNode : public ShapeNode, public ::osg::MatrixTransform
{
public:
  osgAiNode(
      dart::dynamics::MeshShape* shape,
      ShapeFrameNode* parentEntity,
      MeshShapeNode* parentNode,
      const aiNode* node);

  void refresh();
  void extractData(bool firstTime);

  const aiNode* getAiNodePtr() const
  {
    return mAiNode;
  }

protected:
  virtual ~osgAiNode();

  void clearChildUtilizationFlags();
  void clearUnusedNodes();

  MeshShapeNode* mMainNode;
  dart::dynamics::MeshShape* mMeshShape;
  const aiNode* mAiNode;
  MeshShapeGeode* mGeode;
  std::map<const aiNode*, osgAiNode*> mChildNodes;
};

//==============================================================================
class MeshShapeGeode : public ShapeNode, public ::osg::Geode
{
public:
  MeshShapeGeode(
      dart::dynamics::MeshShape* shape,
      ShapeFrameNode* parentShapeFrame,
      MeshShapeNode* parentNode,
      const aiNode* node);

  void refresh();
  void extractData(bool firstTime);

protected:
  virtual ~MeshShapeGeode();

  void clearChildUtilizationFlags();
  void clearUnusedMeshes();

  dart::dynamics::MeshShape* mMeshShape;
  const aiNode* mAiNode;
  MeshShapeNode* mMainNode;
  std::map<const aiMesh*, MeshShapeGeometry*> mMeshes;
};

//==============================================================================
class MeshShapeGeometry : public ShapeNode, public ::osg::Geometry
{
public:
  MeshShapeGeometry(
      dart::dynamics::MeshShape* shape,
      ShapeFrameNode* parentShapeFrame,
      MeshShapeNode* parentNode,
      MeshShapeGeode* parentGeode,
      aiMesh* mesh);

  void refresh();

  void extractData(bool firstTime);

protected:
  virtual ~MeshShapeGeometry();

  ::osg::ref_ptr<::osg::Vec3Array> mVertices;
  ::osg::ref_ptr<::osg::Vec3Array> mNormals;
  ::osg::ref_ptr<::osg::Vec4Array> mColors;

  dart::dynamics::MeshShape* mMeshShape;
  const aiMesh* mAiMesh;
  MeshShapeNode* mMainNode;
};

//==============================================================================
MeshShapeNode::MeshShapeNode(
    std::shared_ptr<dart::dynamics::MeshShape> shape,
    ShapeFrameNode* parentNode)
  : ShapeNode(shape, parentNode, this), mMeshShape(shape), mRootAiNode(nullptr)
{
  extractData(true);
  setNodeMask(mVisualAspect->isHidden() ? 0x0 : ~0x0);
}

//==============================================================================
void MeshShapeNode::refresh()
{
  mUtilized = true;

  setNodeMask(mVisualAspect->isHidden() ? 0x0 : ~0x0);

  if (mShape->getDataVariance() == dart::dynamics::Shape::STATIC)
    return;

  extractData(false);
}

//==============================================================================
bool checkSpecularSanity(const aiColor4D& c)
{
  if (c.r >= 1.0 && c.g >= 1.0 && c.b >= 1.0 && c.a >= 1.0)
    return false;

  return true;
}

//==============================================================================
void MeshShapeNode::extractData(bool firstTime)
{
  // Use deprecated getMesh() for now to maintain backward compatibility
  // with the scene graph structure (aiScene contains node hierarchy)
  // TODO: Future work - refactor to use TriMesh directly with a scene graph
  const aiScene* scene = mMeshShape->getMesh();
  const aiNode* root = scene->mRootNode;

  if (firstTime) // extract material properties from MeshShape (Assimp-free)
  {
    const auto& meshMaterials = mMeshShape->getMaterials();
    mMaterials.reserve(meshMaterials.size());
    mTextureImageArrays.reserve(meshMaterials.size());

    for (const auto& meshMat : meshMaterials) {
      ::osg::ref_ptr<::osg::Material> material = new ::osg::Material;

      // Convert from MeshMaterial to OSG Material
      material->setAmbient(
          ::osg::Material::FRONT_AND_BACK,
          ::osg::Vec4(
              meshMat.ambient[0],
              meshMat.ambient[1],
              meshMat.ambient[2],
              meshMat.ambient[3]));

      material->setDiffuse(
          ::osg::Material::FRONT_AND_BACK,
          ::osg::Vec4(
              meshMat.diffuse[0],
              meshMat.diffuse[1],
              meshMat.diffuse[2],
              meshMat.diffuse[3]));

      // Check specular sanity
      const bool specularSane
          = !(meshMat.specular[0] >= 1.0f && meshMat.specular[1] >= 1.0f
              && meshMat.specular[2] >= 1.0f && meshMat.specular[3] >= 1.0f);
      if (specularSane) {
        material->setSpecular(
            ::osg::Material::FRONT_AND_BACK,
            ::osg::Vec4(
                meshMat.specular[0],
                meshMat.specular[1],
                meshMat.specular[2],
                meshMat.specular[3]));
      }

      material->setEmission(
          ::osg::Material::FRONT_AND_BACK,
          ::osg::Vec4(
              meshMat.emissive[0],
              meshMat.emissive[1],
              meshMat.emissive[2],
              meshMat.emissive[3]));

      material->setShininess(
          ::osg::Material::FRONT_AND_BACK, meshMat.shininess);

      mMaterials.push_back(material);

      // Use texture paths from MeshMaterial
      mTextureImageArrays.emplace_back(meshMat.textureImagePaths);
    }
  }

  if (mShape->checkDataVariance(dart::dynamics::Shape::DYNAMIC_TRANSFORM)
      || mShape->checkDataVariance(dart::dynamics::Shape::DYNAMIC_PRIMITIVE)
      || firstTime) {
    Eigen::Matrix4d S(Eigen::Matrix4d::Zero());
    const Eigen::Vector3d& s = mMeshShape->getScale();
    S(0, 0) = s[0];
    S(1, 1) = s[1];
    S(2, 2) = s[2];
    S(3, 3) = 1.0;
    setMatrix(eigToOsgMatrix(S));
  }

  if (mRootAiNode) {
    // If the root node pointer has changed, delete our current mRootNode and
    // then recreate it
    if (mRootAiNode->getAiNodePtr() != root) {
      removeChild(mRootAiNode);
      mRootAiNode = nullptr;
    }
  }

  if ((nullptr == mRootAiNode) && root) {
    mRootAiNode
        = new osgAiNode(mMeshShape.get(), mParentShapeFrameNode, this, root);
    addChild(mRootAiNode);
    return;
  }

  mRootAiNode->refresh();
}

//==============================================================================
::osg::Material* MeshShapeNode::getMaterial(std::size_t index) const
{
  if (index < mMaterials.size())
    return mMaterials[index];

  // Silently return nullptr when materials don't exist - this is expected for
  // many mesh formats (e.g., STL, plain OBJ) and robotics meshes without
  // embedded materials. The renderer will use default materials.
  return nullptr;
}

//==============================================================================
std::vector<std::string> MeshShapeNode::getTextureImagePaths(
    std::size_t index) const
{
  if (index < mTextureImageArrays.size())
    return mTextureImageArrays[index];

  // We sometimes use this value for meshes that do not have material
  // information, since assimp does not seem to have a built-in way to express
  // that case.
  if (index == std::numeric_limits<unsigned int>::max())
    return {};

  // Silently return empty vector when textures don't exist - this is expected
  // for meshes without embedded materials
  return std::vector<std::string>();
}

//==============================================================================
MeshShapeNode::~MeshShapeNode()
{
  // Do nothing
}

//==============================================================================
osgAiNode::osgAiNode(
    dart::dynamics::MeshShape* shape,
    ShapeFrameNode* parentEntity,
    MeshShapeNode* parentNode,
    const aiNode* node)
  : ShapeNode(parentNode->getShape(), parentEntity, this),
    mMainNode(parentNode),
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

  if (mShape->getDataVariance() == dart::dynamics::Shape::STATIC)
    return;

  extractData(false);
}

//==============================================================================
void osgAiNode::extractData(bool firstTime)
{
  clearChildUtilizationFlags();

  if (mShape->checkDataVariance(dart::dynamics::Shape::DYNAMIC_TRANSFORM)
      || firstTime) {
    aiMatrix4x4 M = mAiNode->mTransformation;
    M.Transpose();
    setMatrix(::osg::Matrixf((float*)(&M)));
  }

  for (std::size_t i = 0; i < mAiNode->mNumChildren; ++i) {
    aiNode* child = mAiNode->mChildren[i];
    std::map<const aiNode*, osgAiNode*>::iterator it = mChildNodes.find(child);

    if (it == mChildNodes.end()) {
      osgAiNode* newChild
          = new osgAiNode(mMeshShape, mParentShapeFrameNode, mMainNode, child);
      addChild(newChild);
    } else
      it->second->refresh();
  }

  if (nullptr == mGeode) {
    mGeode = new MeshShapeGeode(
        mMeshShape, mParentShapeFrameNode, mMainNode, mAiNode);
    addChild(mGeode);
  } else
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
  for (auto& node_pair : mChildNodes)
    node_pair.second->clearUtilization();
}

//==============================================================================
void osgAiNode::clearUnusedNodes()
{
  for (auto& node_pair : mChildNodes) {
    osgAiNode* node = node_pair.second;
    if (!node->wasUtilized()) {
      mChildNodes.erase(node_pair.first);
      removeChild(node);
    }
  }
}

//==============================================================================
MeshShapeGeode::MeshShapeGeode(
    dart::dynamics::MeshShape* shape,
    ShapeFrameNode* parentShapeFrame,
    MeshShapeNode* parentNode,
    const aiNode* node)
  : ShapeNode(parentNode->getShape(), parentShapeFrame, parentNode),
    mMeshShape(shape),
    mAiNode(node),
    mMainNode(parentNode)
{
  getOrCreateStateSet()->setAttributeAndModes(
      new ::osg::CullFace(::osg::CullFace::BACK));
  extractData(true);
}

//==============================================================================
void MeshShapeGeode::refresh()
{
  mUtilized = true;

  if (mShape->getDataVariance() == dart::dynamics::Shape::STATIC)
    return;

  extractData(false);
}

//==============================================================================
void MeshShapeGeode::extractData(bool)
{
  clearChildUtilizationFlags();

  const aiScene* scene = mMeshShape->getMesh();
  for (std::size_t i = 0; i < mAiNode->mNumMeshes; ++i) {
    aiMesh* mesh = scene->mMeshes[mAiNode->mMeshes[i]];

    std::map<const aiMesh*, MeshShapeGeometry*>::iterator it
        = mMeshes.find(mesh);

    if (it == mMeshes.end()) {
      MeshShapeGeometry* newMesh = new MeshShapeGeometry(
          mMeshShape, mParentShapeFrameNode, mMainNode, this, mesh);
      addDrawable(newMesh);
      mMeshes[mesh] = newMesh;
    } else
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
  for (auto& node_pair : mMeshes) {
    MeshShapeGeometry* geom = node_pair.second;
    geom->clearUtilization();
  }
}

//==============================================================================
void MeshShapeGeode::clearUnusedMeshes()
{
  for (auto& node_pair : mMeshes) {
    MeshShapeGeometry* geom = node_pair.second;
    if (!geom->wasUtilized()) {
      mMeshes.erase(node_pair.first);
      removeDrawable(geom);
    }
  }
}

//==============================================================================
MeshShapeGeometry::MeshShapeGeometry(
    dart::dynamics::MeshShape* shape,
    ShapeFrameNode* parentShapeFrame,
    MeshShapeNode* parentNode,
    MeshShapeGeode* parentGeode,
    aiMesh* mesh)
  : ShapeNode(parentNode->getShape(), parentShapeFrame, parentGeode),
    mVertices(new ::osg::Vec3Array),
    mNormals(new ::osg::Vec3Array),
    mColors(new ::osg::Vec4Array),
    mMeshShape(shape),
    mAiMesh(mesh),
    mMainNode(parentNode)
{
  extractData(true);
}

//==============================================================================
void MeshShapeGeometry::refresh()
{
  mUtilized = true;

  if (mShape->getDataVariance() == dart::dynamics::Shape::STATIC)
    return;

  extractData(false);
}

//==============================================================================
void blendMaterialAlpha(::osg::Material* material, const float alpha)
{
  ::osg::Vec4 ambient
      = material->getAmbient(::osg::Material::Face::FRONT_AND_BACK);
  ambient.a() *= alpha;
  material->setAmbient(::osg::Material::Face::FRONT_AND_BACK, ambient);

  ::osg::Vec4 diffuse
      = material->getDiffuse(::osg::Material::Face::FRONT_AND_BACK);
  diffuse.a() *= alpha;
  material->setDiffuse(::osg::Material::Face::FRONT_AND_BACK, diffuse);

  ::osg::Vec4 specular
      = material->getSpecular(::osg::Material::Face::FRONT_AND_BACK);
  specular.a() *= alpha;
  material->setSpecular(::osg::Material::Face::FRONT_AND_BACK, specular);

  ::osg::Vec4 emission
      = material->getEmission(::osg::Material::Face::FRONT_AND_BACK);
  emission.a() *= alpha;
  material->setEmission(::osg::Material::Face::FRONT_AND_BACK, emission);
}

//==============================================================================
void MeshShapeGeometry::extractData(bool firstTime)
{
  if (mShape->getDataVariance() == dart::dynamics::Shape::STATIC)
    setDataVariance(::osg::Object::STATIC);
  else
    setDataVariance(::osg::Object::DYNAMIC);

  if (mShape->checkDataVariance(dart::dynamics::Shape::DYNAMIC_ELEMENTS)
      || firstTime) {
    ::osg::ref_ptr<::osg::DrawElementsUInt> elements[4];
    elements[0] = new ::osg::DrawElementsUInt(GL_POINTS);
    elements[1] = new ::osg::DrawElementsUInt(GL_LINES);
    elements[2] = new ::osg::DrawElementsUInt(GL_TRIANGLES);
    elements[3] = new ::osg::DrawElementsUInt(GL_QUADS);

    for (std::size_t i = 0; i < mAiMesh->mNumFaces; ++i) {
      const aiFace& face = mAiMesh->mFaces[i];

      if (face.mNumIndices > 4) // We have some arbitrary polygon
      {
        ::osg::ref_ptr<::osg::DrawElementsUInt> polygon
            = new ::osg::DrawElementsUInt(GL_POLYGON);
        for (std::size_t j = 0; j < face.mNumIndices; ++j)
          polygon->push_back(face.mIndices[j]);
        addPrimitiveSet(polygon);
      } else if (face.mNumIndices > 0) {
        ::osg::DrawElementsUInt* elem = elements[face.mNumIndices - 1];
        for (std::size_t j = 0; j < face.mNumIndices; ++j)
          elem->push_back(face.mIndices[j]);
      }
    }

    for (std::size_t i = 0; i < 4; ++i)
      if (elements[i]->size() > 0)
        addPrimitiveSet(elements[i]);
  }

  if (mShape->checkDataVariance(dart::dynamics::Shape::DYNAMIC_VERTICES)
      || mShape->checkDataVariance(dart::dynamics::Shape::DYNAMIC_ELEMENTS)
      || firstTime) {
    if (mVertices->size() != mAiMesh->mNumVertices)
      mVertices->resize(mAiMesh->mNumVertices);

    if (mNormals->size() != mAiMesh->mNumVertices)
      mNormals->resize(mAiMesh->mNumVertices);

    const Eigen::Vector3f s = mMeshShape->getScale().cast<float>();

    for (std::size_t i = 0; i < mAiMesh->mNumVertices; ++i) {
      const aiVector3D& v = mAiMesh->mVertices[i];
      (*mVertices)[i] = ::osg::Vec3(v.x, v.y, v.z);

      if (mAiMesh->mNormals) {
        const aiVector3D& n = mAiMesh->mNormals[i];
        (*mNormals)[i] = ::osg::Vec3(n.x * s[0], n.y * s[1], n.z * s[2]);
      }
      // TODO(MXG): Consider computing normals for meshes that don't come with
      // normal data per vertex
    }

    setVertexArray(mVertices);
    if (mAiMesh->mNormals)
      setNormalArray(mNormals, ::osg::Array::BIND_PER_VERTEX);
  }

  if (mShape->checkDataVariance(dart::dynamics::Shape::DYNAMIC_COLOR)
      || firstTime) {
    bool isColored = false;
    ::osg::StateSet* ss = getOrCreateStateSet();

    if (mMeshShape->getColorMode() == dart::dynamics::MeshShape::COLOR_INDEX) {
      int index = mMeshShape->getColorIndex();
      if (index >= AI_MAX_NUMBER_OF_COLOR_SETS)
        index = AI_MAX_NUMBER_OF_COLOR_SETS - 1;

      aiColor4D* colors = nullptr;
      while (nullptr == colors && index >= 0) {
        colors = mAiMesh->mColors[index];
        --index;
      }

      if (colors) {
        isColored = true;

        if (mColors->size() != mVertices->size())
          mColors->resize(mVertices->size());

        for (std::size_t i = 0; i < mAiMesh->mNumVertices; ++i) {
          const aiColor4D& c = colors[i];
          if (mMeshShape->getAlphaMode() == dynamics::MeshShape::SHAPE_ALPHA) {
            (*mColors)[i] = ::osg::Vec4(
                c.r, c.g, c.b, static_cast<float>(mVisualAspect->getAlpha()));
          } else if (mMeshShape->getAlphaMode() == dynamics::MeshShape::BLEND) {
            (*mColors)[i] = ::osg::Vec4(
                c.r,
                c.g,
                c.b,
                c.a * static_cast<float>(mVisualAspect->getAlpha()));
          } else {
            (*mColors)[i] = ::osg::Vec4(c.r, c.g, c.b, c.a);
          }
        }

        setColorArray(mColors);
        setColorBinding(::osg::Geometry::BIND_PER_VERTEX);

        // Set as a transparent object by default
        // TODO(JS): Revisit if this doesn't make sense
        ss->setMode(GL_BLEND, ::osg::StateAttribute::ON);
        ss->setRenderingHint(::osg::StateSet::TRANSPARENT_BIN);
        ::osg::ref_ptr<::osg::Depth> depth = new ::osg::Depth;
        depth->setWriteMask(false);
        ss->setAttributeAndModes(depth, ::osg::StateAttribute::ON);
      }
    }

    if (mMeshShape->getColorMode()
        == dart::dynamics::MeshShape::MATERIAL_COLOR) {
      const unsigned int matIndex = mAiMesh->mMaterialIndex;
      if (matIndex
          != static_cast<unsigned int>(
              -1)) // -1 is being used by us to indicate no material
      {
        isColored = true;
        ::osg::Material* material = mMainNode->getMaterial(matIndex);

        // Check if material exists - if not, fall back to default rendering
        // This is expected for mesh files without embedded materials
        if (!material) {
          ss->removeAttribute(::osg::StateAttribute::MATERIAL);
          ss->setMode(GL_BLEND, ::osg::StateAttribute::OFF);
          ss->setRenderingHint(::osg::StateSet::OPAQUE_BIN);
          ::osg::ref_ptr<::osg::Depth> depth = new ::osg::Depth;
          depth->setWriteMask(true);
          ss->setAttributeAndModes(depth, ::osg::StateAttribute::ON);
        } else if (
            mMeshShape->getAlphaMode() == dynamics::MeshShape::SHAPE_ALPHA) {
          const float shapeAlpha
              = static_cast<float>(mVisualAspect->getAlpha());

          ::osg::ref_ptr<::osg::Material> newMaterial
              = new ::osg::Material(*material);
          newMaterial->setAlpha(
              ::osg::Material::Face::FRONT_AND_BACK, shapeAlpha);
          ss->setAttributeAndModes(newMaterial);

          // Set alpha specific properties
          if (std::abs(shapeAlpha) > 1 - getAlphaThreshold<float>()) {
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
        } else if (mMeshShape->getAlphaMode() == dynamics::MeshShape::BLEND) {
          float shapeAlpha = static_cast<float>(mVisualAspect->getAlpha());
          ::osg::ref_ptr<::osg::Material> newMaterial
              = new ::osg::Material(*material);
          blendMaterialAlpha(newMaterial, shapeAlpha);
          ss->setAttributeAndModes(newMaterial);

          // Set alpha specific properties
          if (std::abs(shapeAlpha) > 1 - getAlphaThreshold<float>()
              && !isTransparent(material)) {
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
        } else {
          ss->setAttributeAndModes(material);

          // Set alpha specific properties
          if (!isTransparent(material)) {
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
      } else {
        ss->removeAttribute(::osg::StateAttribute::MATERIAL);
        ss->setMode(GL_BLEND, ::osg::StateAttribute::OFF);
        ss->setRenderingHint(::osg::StateSet::OPAQUE_BIN);
        ::osg::ref_ptr<::osg::Depth> depth = new ::osg::Depth;
        depth->setWriteMask(true);
        ss->setAttributeAndModes(depth, ::osg::StateAttribute::ON);
      }
    } else {
      ss->removeAttribute(::osg::StateAttribute::MATERIAL);
      ss->setMode(GL_BLEND, ::osg::StateAttribute::OFF);
      ss->setRenderingHint(::osg::StateSet::OPAQUE_BIN);
      ::osg::ref_ptr<::osg::Depth> depth = new ::osg::Depth;
      depth->setWriteMask(true);
      ss->setAttributeAndModes(depth, ::osg::StateAttribute::ON);
    }

    const aiVector3D* aiTexCoords = mAiMesh->mTextureCoords[0];
    if (aiTexCoords)
      isColored = true;

    if (!isColored
        || mMeshShape->getColorMode()
               == dart::dynamics::MeshShape::SHAPE_COLOR) {
      // Set color
      const Eigen::Vector4f& c = mVisualAspect->getRGBA().cast<float>();
      mColors->resize(1);
      (*mColors)[0] = ::osg::Vec4(c[0], c[1], c[2], c[3]);
      setColorArray(mColors);
      setColorBinding(::osg::Geometry::BIND_OVERALL);

      // Set alpha specific properties
      ::osg::StateSet* ss = getOrCreateStateSet();
      if (std::abs(c[3]) > 1 - getAlphaThreshold<float>()) {
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

  // Load textures on the first pass through
  if (firstTime) {
    unsigned int unit = 0;
    const aiVector3D* aiTexCoords = mAiMesh->mTextureCoords[unit];

    while (nullptr != aiTexCoords) {
      switch (mAiMesh->mNumUVComponents[unit]) {
        case 1: {
          ::osg::ref_ptr<::osg::FloatArray> texture
              = new ::osg::FloatArray(mAiMesh->mNumVertices);
          for (std::size_t i = 0; i < mAiMesh->mNumVertices; ++i)
            (*texture)[i] = aiTexCoords[i].x;
          setTexCoordArray(unit, texture, ::osg::Array::BIND_PER_VERTEX);
          break;
        }
        case 2: {
          ::osg::ref_ptr<::osg::Vec2Array> texture
              = new ::osg::Vec2Array(mAiMesh->mNumVertices);
          for (std::size_t i = 0; i < mAiMesh->mNumVertices; ++i) {
            const aiVector3D& t = aiTexCoords[i];
            (*texture)[i] = ::osg::Vec2(t.x, t.y);
          }
          setTexCoordArray(unit, texture, ::osg::Array::BIND_PER_VERTEX);
          break;
        }
        case 3: {
          ::osg::ref_ptr<::osg::Vec3Array> texture
              = new ::osg::Vec3Array(mAiMesh->mNumVertices);
          for (std::size_t i = 0; i < mAiMesh->mNumVertices; ++i) {
            const aiVector3D& t = aiTexCoords[i];
            (*texture)[i] = ::osg::Vec3(t.x, t.y, t.z);
          }
          setTexCoordArray(unit, texture, ::osg::Array::BIND_PER_VERTEX);
          break;
        }
      } // switch(mAiMesh->mNumUVComponents[unit])
      aiTexCoords = mAiMesh->mTextureCoords[++unit];
    } // while(nullptr != aiTexCoords)

    const auto imagePaths
        = mMainNode->getTextureImagePaths(mAiMesh->mMaterialIndex);
    for (auto i = 0u; i < imagePaths.size(); ++i) {
      if (imagePaths[i].empty())
        continue;

      ::osg::ref_ptr<::osg::Image> image
          = osgDB::readRefImageFile(imagePaths[i]);
      if (!image) {
        dtwarn << "[MeshShapeNode] Failed to load texture image '"
               << imagePaths[i] << "'\n";
        continue;
      }
      ::osg::ref_ptr<::osg::Texture2D> texture
          = new ::osg::Texture2D(image.get());
      texture->setResizeNonPowerOfTwoHint(false);
      texture->setUnRefImageDataAfterApply(true);
      getOrCreateStateSet()->setTextureAttributeAndModes(
          i, texture, ::osg::StateAttribute::ON);
    }
  } // if(firstTime)
}

//==============================================================================
MeshShapeGeometry::~MeshShapeGeometry()
{
  // Do nothing
}

} // namespace render
} // namespace osg
} // namespace gui
} // namespace dart
