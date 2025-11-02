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

#include "dart/dynamics/MeshShape.hpp"

#include "dart/common/Console.hpp"
#include "dart/common/Deprecated.hpp"
#include "dart/common/Filesystem.hpp"
#include "dart/common/LocalResourceRetriever.hpp"
#include "dart/common/Uri.hpp"
#include "dart/config.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/MeshMaterial.hpp"
#include "dart/dynamics/detail/AssimpInputResourceAdaptor.hpp"

#include <assimp/Importer.hpp>
#include <assimp/cimport.h>
#include <assimp/postprocess.h>

#include <limits>
#include <string>

#if !(ASSIMP_AISCENE_CTOR_DTOR_DEFINED)
// We define our own constructor and destructor for aiScene, because it seems to
// be missing from the standard assimp library (see #451)
aiScene::aiScene()
  : mFlags(0),
    mRootNode(nullptr),
    mNumMeshes(0),
    mMeshes(nullptr),
    mNumMaterials(0),
    mMaterials(nullptr),
    mAnimations(nullptr),
    mNumTextures(0),
    mTextures(nullptr),
    mNumLights(0),
    mLights(nullptr),
    mNumCameras(0),
    mCameras(nullptr)
{
}

aiScene::~aiScene()
{
  delete mRootNode;

  if (mNumMeshes && mMeshes)
    for (std::size_t a = 0; a < mNumMeshes; ++a)
      delete mMeshes[a];
  delete[] mMeshes;

  if (mNumMaterials && mMaterials)
    for (std::size_t a = 0; a < mNumMaterials; ++a)
      delete mMaterials[a];
  delete[] mMaterials;

  if (mNumAnimations && mAnimations)
    for (std::size_t a = 0; a < mNumAnimations; ++a)
      delete mAnimations[a];
  delete[] mAnimations;

  if (mNumTextures && mTextures)
    for (std::size_t a = 0; a < mNumTextures; ++a)
      delete mTextures[a];
  delete[] mTextures;

  if (mNumLights && mLights)
    for (std::size_t a = 0; a < mNumLights; ++a)
      delete mLights[a];
  delete[] mLights;

  if (mNumCameras && mCameras)
    for (std::size_t a = 0; a < mNumCameras; ++a)
      delete mCameras[a];
  delete[] mCameras;
}
#endif // #if !(ASSIMP_AISCENE_CTOR_DTOR_DEFINED)

// We define our own constructor and destructor for aiMaterial, because it seems
// to be missing from the standard assimp library (see #451)
#if !(ASSIMP_AIMATERIAL_CTOR_DTOR_DEFINED)
aiMaterial::aiMaterial()
{
  mNumProperties = 0;
  mNumAllocated = 5;
  mProperties = new aiMaterialProperty*[5];
  for (std::size_t i = 0; i < 5; ++i)
    mProperties[i] = nullptr;
}

aiMaterial::~aiMaterial()
{
  for (std::size_t i = 0; i < mNumProperties; ++i)
    delete mProperties[i];

  delete[] mProperties;
}
#endif // #if !(ASSIMP_AIMATERIAL_CTOR_DTOR_DEFINED)

namespace dart {
namespace dynamics {

//==============================================================================
MeshShape::MeshShape(
    const Eigen::Vector3d& scale,
    const aiScene* mesh,
    const common::Uri& uri,
    common::ResourceRetrieverPtr resourceRetriever)
  : Shape(Shape::MESH),
    mTriMesh(convertAssimpMesh(mesh)),
    mCachedAiScene(nullptr),
    mMeshUri(uri),
    mMeshPath(uri.getPath()),
    mResourceRetriever(std::move(resourceRetriever)),
    mDisplayList(0),
    mScale(scale),
    mColorMode(MATERIAL_COLOR),
    mAlphaMode(BLEND),
    mColorIndex(0)
{
  // NOTE: This constructor converts aiScene to TriMesh and stores only TriMesh.
  // The aiScene parameter is not retained. If getMesh() is called later, it
  // will perform an expensive conversion back to aiScene.

  // Extract materials from aiScene for Assimp-free rendering
  extractMaterialsFromScene(mesh, uri.getPath());

  setScale(scale);
}

//==============================================================================
MeshShape::MeshShape(
    const Eigen::Vector3d& scale,
    std::shared_ptr<math::TriMesh<double>> mesh,
    const common::Uri& uri)
  : Shape(Shape::MESH),
    mTriMesh(std::move(mesh)),
    mCachedAiScene(nullptr),
    mMeshUri(uri),
    mMeshPath(uri.getPath()),
    mResourceRetriever(nullptr),
    mDisplayList(0),
    mScale(scale),
    mColorMode(MATERIAL_COLOR),
    mAlphaMode(BLEND),
    mColorIndex(0)
{
  setScale(scale);
}

//==============================================================================
MeshShape::~MeshShape()
{
  // Clean up cached aiScene if it was created
  if (mCachedAiScene) {
    aiReleaseImport(mCachedAiScene);
    mCachedAiScene = nullptr;
  }
}

//==============================================================================
const std::string& MeshShape::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& MeshShape::getStaticType()
{
  static const std::string type("MeshShape");
  return type;
}

//==============================================================================
std::shared_ptr<math::TriMesh<double>> MeshShape::getTriMesh() const
{
  return mTriMesh;
}

//==============================================================================
const aiScene* MeshShape::getMesh() const
{
  // Lazy conversion: only convert once and cache the result
  // NOTE: This is still expensive on first call! Please use getTriMesh()
  // instead.
  if (!mCachedAiScene) {
    mCachedAiScene = convertToAssimpMesh();
  }
  return mCachedAiScene;
}

//==============================================================================
const aiScene* MeshShape::convertToAssimpMesh() const
{
  if (!mTriMesh) {
    return nullptr;
  }

  // Create a new aiScene structure
  aiScene* scene = new aiScene();

  // Set up basic scene structure
  scene->mNumMeshes = 1;
  scene->mMeshes = new aiMesh*[1];
  scene->mMeshes[0] = new aiMesh();

  aiMesh* mesh = scene->mMeshes[0];

  const auto& vertices = mTriMesh->getVertices();
  const auto& triangles = mTriMesh->getTriangles();
  const auto& normals = mTriMesh->getVertexNormals();

  // Set vertex data
  mesh->mNumVertices = vertices.size();
  mesh->mVertices = new aiVector3D[mesh->mNumVertices];
  for (size_t i = 0; i < vertices.size(); ++i) {
    mesh->mVertices[i]
        = aiVector3D(vertices[i].x(), vertices[i].y(), vertices[i].z());
  }

  // Set normal data
  if (!normals.empty()) {
    mesh->mNormals = new aiVector3D[mesh->mNumVertices];
    for (size_t i = 0; i < normals.size(); ++i) {
      mesh->mNormals[i]
          = aiVector3D(normals[i].x(), normals[i].y(), normals[i].z());
    }
  }

  // Set face data
  mesh->mNumFaces = triangles.size();
  mesh->mFaces = new aiFace[mesh->mNumFaces];
  for (size_t i = 0; i < triangles.size(); ++i) {
    aiFace& face = mesh->mFaces[i];
    face.mNumIndices = 3;
    face.mIndices = new unsigned int[3];
    face.mIndices[0] = triangles[i].x();
    face.mIndices[1] = triangles[i].y();
    face.mIndices[2] = triangles[i].z();
  }

  mesh->mPrimitiveTypes = aiPrimitiveType_TRIANGLE;
  mesh->mMaterialIndex = 0;

  // Set up root node
  scene->mRootNode = new aiNode();
  scene->mRootNode->mNumMeshes = 1;
  scene->mRootNode->mMeshes = new unsigned int[1];
  scene->mRootNode->mMeshes[0] = 0;

  // Set up a default material
  scene->mNumMaterials = 1;
  scene->mMaterials = new aiMaterial*[1];
  scene->mMaterials[0] = new aiMaterial();

  return scene;
}

//==============================================================================
std::shared_ptr<math::TriMesh<double>> MeshShape::convertAssimpMesh(
    const aiScene* scene)
{
  if (!scene || scene->mNumMeshes == 0) {
    return nullptr;
  }

  auto triMesh = std::make_shared<math::TriMesh<double>>();

  // Process all meshes in the scene
  // TODO: Support merging multiple meshes from the scene
  const aiMesh* assimpMesh = scene->mMeshes[0];

  // Reserve space for vertices
  triMesh->reserveVertices(assimpMesh->mNumVertices);

  // Parse vertices
  for (auto i = 0u; i < assimpMesh->mNumVertices; ++i) {
    const aiVector3D& vertex = assimpMesh->mVertices[i];
    triMesh->addVertex(
        static_cast<double>(vertex.x),
        static_cast<double>(vertex.y),
        static_cast<double>(vertex.z));
  }

  // Parse faces (triangles)
  triMesh->reserveTriangles(assimpMesh->mNumFaces);
  for (auto i = 0u; i < assimpMesh->mNumFaces; ++i) {
    const aiFace& face = assimpMesh->mFaces[i];

    // Skip non-triangular faces
    if (face.mNumIndices != 3) {
      dtwarn << "[MeshShape::convertAssimpMesh] Non-triangular face detected. "
             << "Skipping this face.\n";
      continue;
    }

    triMesh->addTriangle(face.mIndices[0], face.mIndices[1], face.mIndices[2]);
  }

  // Parse vertex normals
  if (assimpMesh->mNormals) {
    triMesh->reserveVertexNormals(assimpMesh->mNumVertices);
    for (auto i = 0u; i < assimpMesh->mNumVertices; ++i) {
      const aiVector3D& normal = assimpMesh->mNormals[i];
      triMesh->addVertexNormal(
          static_cast<double>(normal.x),
          static_cast<double>(normal.y),
          static_cast<double>(normal.z));
    }
  } else {
    // Compute vertex normals if not provided
    triMesh->computeVertexNormals();
  }

  return triMesh;
}

//==============================================================================
std::string MeshShape::getMeshUri() const
{
  return mMeshUri.toString();
}

//==============================================================================
const common::Uri& MeshShape::getMeshUri2() const
{
  return mMeshUri;
}

//==============================================================================
void MeshShape::update()
{
  // Do nothing
}

//==============================================================================
const std::string& MeshShape::getMeshPath() const
{
  return mMeshPath;
}

//==============================================================================
common::ResourceRetrieverPtr MeshShape::getResourceRetriever()
{
  return mResourceRetriever;
}

//==============================================================================
void MeshShape::setMesh(
    const aiScene* mesh,
    const std::string& path,
    common::ResourceRetrieverPtr resourceRetriever)
{
  setMesh(mesh, common::Uri(path), std::move(resourceRetriever));
}

//==============================================================================
void MeshShape::setMesh(
    const aiScene* mesh,
    const common::Uri& uri,
    common::ResourceRetrieverPtr resourceRetriever)
{
  // Clear cached aiScene to prevent stale data
  if (mCachedAiScene) {
    aiReleaseImport(mCachedAiScene);
    mCachedAiScene = nullptr;
  }

  // Convert aiScene to TriMesh
  mTriMesh = convertAssimpMesh(mesh);

  if (!mTriMesh) {
    mMeshUri.clear();
    mMeshPath.clear();
    mResourceRetriever = nullptr;
    mMaterials.clear();
    return;
  }

  mMeshUri = uri;

  if (resourceRetriever)
    mMeshPath = resourceRetriever->getFilePath(uri);
  else
    mMeshPath.clear();

  mResourceRetriever = std::move(resourceRetriever);

  // Refresh materials from the new mesh
  extractMaterialsFromScene(mesh, mMeshPath);

  incrementVersion();
}

//==============================================================================
void MeshShape::setScale(const Eigen::Vector3d& scale)
{
  mScale = scale;
  mIsBoundingBoxDirty = true;
  mIsVolumeDirty = true;

  incrementVersion();
}

//==============================================================================
void MeshShape::setScale(const double scale)
{
  setScale(Eigen::Vector3d::Constant(scale));
}

//==============================================================================
const Eigen::Vector3d& MeshShape::getScale() const
{
  return mScale;
}

//==============================================================================
void MeshShape::setColorMode(ColorMode mode)
{
  mColorMode = mode;
}

//==============================================================================
MeshShape::ColorMode MeshShape::getColorMode() const
{
  return mColorMode;
}

//==============================================================================
void MeshShape::setAlphaMode(MeshShape::AlphaMode mode)
{
  mAlphaMode = mode;
}

//==============================================================================
MeshShape::AlphaMode MeshShape::getAlphaMode() const
{
  return mAlphaMode;
}

//==============================================================================
void MeshShape::setColorIndex(int index)
{
  mColorIndex = index;
}

//==============================================================================
int MeshShape::getColorIndex() const
{
  return mColorIndex;
}

//==============================================================================
int MeshShape::getDisplayList() const
{
  return mDisplayList;
}

//==============================================================================
void MeshShape::setDisplayList(int index)
{
  mDisplayList = index;
}

//==============================================================================
const std::vector<MeshMaterial>& MeshShape::getMaterials() const
{
  return mMaterials;
}

//==============================================================================
std::size_t MeshShape::getNumMaterials() const
{
  return mMaterials.size();
}

//==============================================================================
const MeshMaterial* MeshShape::getMaterial(std::size_t index) const
{
  if (index < mMaterials.size()) {
    return &mMaterials[index];
  }
  return nullptr;
}

//==============================================================================
void MeshShape::extractMaterialsFromScene(
    const aiScene* scene, const std::string& basePath)
{
  if (!scene || scene->mNumMaterials == 0) {
    return;
  }

  mMaterials.clear();
  mMaterials.reserve(scene->mNumMaterials);

  for (std::size_t i = 0; i < scene->mNumMaterials; ++i) {
    aiMaterial* aiMat = scene->mMaterials[i];
    assert(aiMat);

    MeshMaterial material;

    // Extract colors
    aiColor4D c;
    if (aiGetMaterialColor(aiMat, AI_MATKEY_COLOR_AMBIENT, &c) == AI_SUCCESS) {
      material.ambient = Eigen::Vector4f(c.r, c.g, c.b, c.a);
    }

    if (aiGetMaterialColor(aiMat, AI_MATKEY_COLOR_DIFFUSE, &c) == AI_SUCCESS) {
      material.diffuse = Eigen::Vector4f(c.r, c.g, c.b, c.a);
    }

    if (aiGetMaterialColor(aiMat, AI_MATKEY_COLOR_SPECULAR, &c) == AI_SUCCESS) {
      material.specular = Eigen::Vector4f(c.r, c.g, c.b, c.a);
    }

    if (aiGetMaterialColor(aiMat, AI_MATKEY_COLOR_EMISSIVE, &c) == AI_SUCCESS) {
      material.emissive = Eigen::Vector4f(c.r, c.g, c.b, c.a);
    }

    // Extract shininess
    unsigned int maxValue = 1;
    float shininess = 0.0f, strength = 1.0f;
    if (aiGetMaterialFloatArray(
            aiMat, AI_MATKEY_SHININESS, &shininess, &maxValue)
        == AI_SUCCESS) {
      maxValue = 1;
      if (aiGetMaterialFloatArray(
              aiMat, AI_MATKEY_SHININESS_STRENGTH, &strength, &maxValue)
          == AI_SUCCESS) {
        shininess *= strength;
      }
      material.shininess = shininess;
    }

    // Extract texture paths for all texture types
    // Check common texture types and store them
    const aiTextureType textureTypes[]
        = {aiTextureType_DIFFUSE,
           aiTextureType_SPECULAR,
           aiTextureType_NORMALS,
           aiTextureType_AMBIENT,
           aiTextureType_EMISSIVE,
           aiTextureType_HEIGHT,
           aiTextureType_SHININESS,
           aiTextureType_OPACITY,
           aiTextureType_DISPLACEMENT,
           aiTextureType_LIGHTMAP,
           aiTextureType_REFLECTION};

    for (const auto& type : textureTypes) {
      const auto count = aiMat->GetTextureCount(type);
      for (auto j = 0u; j < count; ++j) {
        aiString imagePath;
        if (aiMat->GetTexture(type, j, &imagePath) == AI_SUCCESS) {
          const common::filesystem::path relativeImagePath = imagePath.C_Str();
          const common::filesystem::path meshPath = basePath;
          std::error_code ec;
          const common::filesystem::path absoluteImagePath
              = common::filesystem::canonical(
                  meshPath.parent_path() / relativeImagePath, ec);

          if (ec) {
            dtwarn
                << "[MeshShape::extractMaterialsFromScene] Failed to resolve "
                << "texture image path from (base: `" << meshPath.parent_path()
                << "', relative: '" << relativeImagePath << "').\n";
          } else {
            material.textureImagePaths.emplace_back(absoluteImagePath.string());
          }
        }
      }
    }

    mMaterials.emplace_back(std::move(material));
  }
}

//==============================================================================
Eigen::Matrix3d MeshShape::computeInertia(double _mass) const
{
  // Use bounding box to represent the mesh
  return BoxShape::computeInertia(getBoundingBox().computeFullExtents(), _mass);
}

//==============================================================================
ShapePtr MeshShape::clone() const
{
  aiScene* new_scene = cloneMesh();

  auto new_shape = std::make_shared<MeshShape>(
      mScale, new_scene, mMeshUri, mResourceRetriever);
  new_shape->mMeshPath = mMeshPath;
  new_shape->mDisplayList = mDisplayList;
  new_shape->mColorMode = mColorMode;
  new_shape->mAlphaMode = mAlphaMode;
  new_shape->mColorIndex = mColorIndex;

  return new_shape;
}

//==============================================================================
void MeshShape::updateBoundingBox() const
{
  if (!mTriMesh) {
    mBoundingBox.setMin(Eigen::Vector3d::Zero());
    mBoundingBox.setMax(Eigen::Vector3d::Zero());
    mIsBoundingBoxDirty = false;
    return;
  }

  Eigen::Vector3d minPoint
      = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d maxPoint = -minPoint;

  const auto& vertices = mTriMesh->getVertices();
  for (const auto& vertex : vertices) {
    const Eigen::Vector3d scaledVertex = vertex.cwiseProduct(mScale);
    minPoint = minPoint.cwiseMin(scaledVertex);
    maxPoint = maxPoint.cwiseMax(scaledVertex);
  }

  mBoundingBox.setMin(minPoint);
  mBoundingBox.setMax(maxPoint);

  mIsBoundingBoxDirty = false;
}

//==============================================================================
void MeshShape::updateVolume() const
{
  const Eigen::Vector3d bounds = getBoundingBox().computeFullExtents();
  mVolume = bounds.x() * bounds.y() * bounds.z();
  mIsVolumeDirty = false;
}

//==============================================================================
aiScene* MeshShape::cloneMesh() const
{
  // Convert TriMesh to aiScene for cloning
  const aiScene* mMesh = convertToAssimpMesh();
  if (!mMesh) {
    return nullptr;
  }

  // Create new assimp mesh
  aiScene* new_scene = new aiScene();
  // Copy basic data
  new_scene->mNumAnimations = 0; // we do not care about animations
  new_scene->mNumCameras = 0;    // we do not care about cameras
  new_scene->mNumLights = 0;     // we do not care about lights
  new_scene->mNumMaterials = mMesh->mNumMaterials;
  new_scene->mNumMeshes = mMesh->mNumMeshes;
  new_scene->mNumTextures = mMesh->mNumTextures;
  new_scene->mFlags = mMesh->mFlags;
  // Initialize empty structs
  new_scene->mAnimations = nullptr;
  new_scene->mCameras = nullptr;
  new_scene->mLights = nullptr;

  // Copy materials
  new_scene->mMaterials = new aiMaterial*[new_scene->mNumMaterials];
  for (unsigned int i = 0; i < new_scene->mNumMaterials; i++) {
    new_scene->mMaterials[i] = new aiMaterial();
    new_scene->mMaterials[i]->mNumAllocated
        = mMesh->mMaterials[i]->mNumAllocated;
    new_scene->mMaterials[i]->mNumProperties
        = mMesh->mMaterials[i]->mNumProperties;

    new_scene->mMaterials[i]->mProperties
        = new aiMaterialProperty*[new_scene->mMaterials[i]->mNumProperties];

    for (unsigned int j = 0; j < new_scene->mMaterials[i]->mNumProperties;
         j++) {
      new_scene->mMaterials[i]->mProperties[j] = new aiMaterialProperty();
      auto& prop = new_scene->mMaterials[i]->mProperties[j];
      auto& other = mMesh->mMaterials[i]->mProperties[j];

      prop->mKey = other->mKey;
      prop->mSemantic = other->mSemantic;
      prop->mIndex = other->mIndex;
      prop->mDataLength = other->mDataLength;
      prop->mType = other->mType;
      prop->mData = new char[prop->mDataLength];
      memcpy(prop->mData, other->mData, prop->mDataLength);
    }
  }

  // Copy textures
  new_scene->mTextures = new aiTexture*[new_scene->mNumTextures];
  for (unsigned int i = 0; i < new_scene->mNumTextures; i++) {
    new_scene->mTextures[i] = new aiTexture();
    strcpy(
        new_scene->mTextures[i]->achFormatHint,
        mMesh->mTextures[i]->achFormatHint);
    new_scene->mTextures[i]->mHeight = mMesh->mTextures[i]->mHeight;
    new_scene->mTextures[i]->mWidth = mMesh->mTextures[i]->mWidth;
    // new_scene->mTextures[i]->mFilename = mMesh->mTextures[i]->mFilename;
    unsigned int size = new_scene->mTextures[i]->mWidth;
    if (new_scene->mTextures[i]->mHeight > 0)
      size *= new_scene->mTextures[i]->mHeight;
    new_scene->mTextures[i]->pcData = new aiTexel[size];
    memcpy(
        new_scene->mTextures[i]->pcData,
        mMesh->mTextures[i]->pcData,
        size * sizeof(aiTexel));
  }

  // Copy meshes
  new_scene->mMeshes = new aiMesh*[new_scene->mNumMeshes];
  for (unsigned int i = 0; i < new_scene->mNumMeshes; i++) {
    new_scene->mMeshes[i] = new aiMesh();
    auto& mesh = new_scene->mMeshes[i];
    auto& other = mMesh->mMeshes[i];
    // Empty - we do not care about animation meshes or bones
    mesh->mAnimMeshes = nullptr;
    mesh->mBones = nullptr;
    mesh->mNumAnimMeshes = 0;
    mesh->mNumBones = 0;
    // Basic info
    mesh->mMaterialIndex = other->mMaterialIndex;
    mesh->mName = other->mName;
    mesh->mNumFaces = other->mNumFaces;
    memcpy(
        &mesh->mNumUVComponents[0],
        &other->mNumUVComponents[0],
        AI_MAX_NUMBER_OF_TEXTURECOORDS * sizeof(unsigned int));
    mesh->mNumVertices = other->mNumVertices;
    mesh->mPrimitiveTypes = other->mPrimitiveTypes;

    if (mesh->mNumVertices > 0) {
      // Copy verticies
      mesh->mVertices = new aiVector3D[mesh->mNumVertices];
      memcpy(
          mesh->mVertices,
          other->mVertices,
          mesh->mNumVertices * sizeof(aiVector3D));

      // Copy normals
      mesh->mNormals = new aiVector3D[mesh->mNumVertices];
      memcpy(
          mesh->mNormals,
          other->mNormals,
          mesh->mNumVertices * sizeof(aiVector3D));

      // Copy faces
      mesh->mFaces = new aiFace[mesh->mNumFaces];
      for (unsigned int a = 0; a < mesh->mNumFaces; a++) {
        mesh->mFaces[a].mNumIndices = other->mFaces[a].mNumIndices;
        mesh->mFaces[a].mIndices
            = new unsigned int[mesh->mFaces[a].mNumIndices];
        memcpy(
            mesh->mFaces[a].mIndices,
            other->mFaces[a].mIndices,
            mesh->mFaces[a].mNumIndices * sizeof(unsigned int));
      }

      // Copy tangents
      if (other->mTangents) {
        mesh->mTangents = new aiVector3D[mesh->mNumVertices];
        memcpy(
            mesh->mTangents,
            other->mTangents,
            mesh->mNumVertices * sizeof(aiVector3D));
      }
      // Copy bi-tangents
      if (other->mBitangents) {
        mesh->mBitangents = new aiVector3D[mesh->mNumVertices];
        memcpy(
            mesh->mBitangents,
            other->mBitangents,
            mesh->mNumVertices * sizeof(aiVector3D));
      }

      // Copy color sets
      for (unsigned int a = 0; a < AI_MAX_NUMBER_OF_COLOR_SETS; a++) {
        if (other->mColors[a]) {
          mesh->mColors[a] = new aiColor4D[mesh->mNumVertices];
          memcpy(
              mesh->mColors[a],
              other->mColors[a],
              mesh->mNumVertices * sizeof(aiColor4D));
        }
      }

      // Copy texture coordinates
      for (unsigned int a = 0; a < AI_MAX_NUMBER_OF_TEXTURECOORDS; a++) {
        if (other->mTextureCoords[a]) {
          mesh->mTextureCoords[a] = new aiVector3D[mesh->mNumVertices];
          memcpy(
              mesh->mTextureCoords[a],
              other->mTextureCoords[a],
              mesh->mNumVertices * sizeof(aiVector3D));
        }
      }
    }
  }

  // Copy nodes
  std::function<void(aiNode*, aiNode*, aiNode*)> aiNodeCopyRecursive
      = [&aiNodeCopyRecursive](aiNode* dest, aiNode* src, aiNode* parent) {
          dest->mNumMeshes = src->mNumMeshes;
          dest->mNumChildren = src->mNumChildren;
          dest->mName = src->mName;
          dest->mTransformation = src->mTransformation;
          dest->mParent = parent;

          dest->mMeshes = new unsigned int[dest->mNumMeshes];
          memcpy(
              dest->mMeshes,
              src->mMeshes,
              dest->mNumMeshes * sizeof(unsigned int));

          dest->mChildren = new aiNode*[dest->mNumChildren];
          for (unsigned int i = 0; i < dest->mNumChildren; i++) {
            dest->mChildren[i] = new aiNode();
            aiNodeCopyRecursive(dest->mChildren[i], src->mChildren[i], dest);
          }
        };

  if (mMesh->mRootNode) {
    new_scene->mRootNode = new aiNode();
    aiNodeCopyRecursive(new_scene->mRootNode, mMesh->mRootNode, nullptr);
  }

  return new_scene;
}

//==============================================================================
const aiScene* MeshShape::loadMesh(
    const std::string& _uri, const common::ResourceRetrieverPtr& retriever)
{
  // Remove points and lines from the import.
  aiPropertyStore* propertyStore = aiCreatePropertyStore();
  aiSetImportPropertyInteger(
      propertyStore,
      AI_CONFIG_PP_SBP_REMOVE,
      aiPrimitiveType_POINT | aiPrimitiveType_LINE);

  // Wrap ResourceRetriever in an IOSystem from Assimp's C++ API.  Then wrap
  // the IOSystem in an aiFileIO from Assimp's C API. Yes, this API is
  // completely ridiculous...
  // Suppress deprecation warnings - we need to use this for backward
  // compatibility
  DART_SUPPRESS_DEPRECATED_BEGIN
  AssimpInputResourceRetrieverAdaptor systemIO(retriever);
  aiFileIO fileIO = createFileIO(&systemIO);
  DART_SUPPRESS_DEPRECATED_END

  // Import the file.
  const aiScene* scene = aiImportFileExWithProperties(
      _uri.c_str(),
      aiProcess_GenNormals | aiProcess_Triangulate
          | aiProcess_JoinIdenticalVertices | aiProcess_SortByPType
          | aiProcess_OptimizeMeshes,
      &fileIO,
      propertyStore);

  // If succeeded, store the importer in the scene to keep it alive. This is
  // necessary because the importer owns the memory that it allocates.
  if (!scene) {
    dtwarn << "[MeshShape::loadMesh] Failed loading mesh '" << _uri << "'.\n";
    aiReleasePropertyStore(propertyStore);
    return nullptr;
  }

  // Assimp rotates collada files such that the up-axis (specified in the
  // collada file) aligns with assimp's y-axis. Here we are reverting this
  // rotation. We are only catching files with the .dae file ending here. We
  // might miss files with an .xml file ending, which would need to be looked
  // into to figure out whether they are collada files.
  std::string extension;
  const std::size_t extensionIndex = _uri.find_last_of('.');
  if (extensionIndex != std::string::npos)
    extension = _uri.substr(extensionIndex);

  std::transform(
      std::begin(extension),
      std::end(extension),
      std::begin(extension),
      ::tolower);

  if (extension == ".dae" || extension == ".zae")
    scene->mRootNode->mTransformation = aiMatrix4x4();

  // Finally, pre-transform the vertices. We can't do this as part of the
  // import process, because we may have changed mTransformation above.
  scene = aiApplyPostProcessing(scene, aiProcess_PreTransformVertices);
  if (!scene)
    dtwarn << "[MeshShape::loadMesh] Failed pre-transforming vertices.\n";

  aiReleasePropertyStore(propertyStore);

  return scene;
}

//==============================================================================
const aiScene* MeshShape::loadMesh(
    const common::Uri& uri, const common::ResourceRetrieverPtr& retriever)
{
  return loadMesh(uri.toString(), retriever);
}

//==============================================================================
const aiScene* MeshShape::loadMesh(const std::string& filePath)
{
  const auto retriever = std::make_shared<common::LocalResourceRetriever>();
  return loadMesh("file://" + filePath, retriever);
}

} // namespace dynamics
} // namespace dart
