/*
 * Copyright (c) 2011-2023, The DART development contributors
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

#pragma once

#include <dart/io/Fwd.hpp>
#include <dart/io/MeshLoader.hpp>

namespace dart::io {

/// MeshLoader implementation for Assimp
/// @tparam S Scalar type
template <typename S>
class AssimpMeshLoader : public MeshLoader<S>
{
public:
  using Scalar = S;
  using Mesh = typename MeshLoader<S>::Mesh;

  AssimpMeshLoader() = default;
  ~AssimpMeshLoader() override = default;

  [[nodiscard]] std::unique_ptr<Mesh> load(
      const std::string& filepath,
      common::ResourceRetrieverPtr retriever = nullptr) override;
};

} // namespace dart::io

//==============================================================================
// Implementation
//==============================================================================

#include <dart/dynamics/AssimpInputResourceAdaptor.hpp> // TODO(JS): Move this to dart::io

#include <dart/common/LocalResourceRetriever.hpp>
#include <dart/common/Logging.hpp>
#include <dart/common/Macros.hpp>

#include <assimp/Importer.hpp>
#include <assimp/cimport.h>
#include <assimp/mesh.h>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

namespace dart::io {

//==============================================================================
DART_IO_API inline std::unique_ptr<const aiScene> loadAssimpScene(
    const std::string& uri, common::ResourceRetrieverPtr retriever)
{
  DART_ASSERT(retriever);

  // Remove points and lines from the import.
  aiPropertyStore* propertyStore = aiCreatePropertyStore();
  aiSetImportPropertyInteger(
      propertyStore,
      AI_CONFIG_PP_SBP_REMOVE,
      aiPrimitiveType_POINT | aiPrimitiveType_LINE);

  // Wrap ResourceRetriever in an IOSystem from Assimp's C++ API.  Then wrap
  // the IOSystem in an aiFileIO from Assimp's C API. Yes, this API is
  // completely ridiculous...
  dynamics::AssimpInputResourceRetrieverAdaptor systemIO(retriever);
  aiFileIO fileIO = createFileIO(&systemIO);

  // Import the file.
  const aiScene* scene = aiImportFileExWithProperties(
      uri.c_str(),
      aiProcess_GenNormals | aiProcess_Triangulate
          | aiProcess_JoinIdenticalVertices | aiProcess_SortByPType
          | aiProcess_OptimizeMeshes,
      &fileIO,
      propertyStore);

  // If succeeded, store the importer in the scene to keep it alive. This is
  // necessary because the importer owns the memory that it allocates.
  if (!scene) {
    DART_WARN("[MeshShape::loadMesh] Failed loading mesh '{}'.", uri);
    aiReleasePropertyStore(propertyStore);
    return nullptr;
  }

  // Assimp rotates Collada files such that the up-axis (specified in the
  // Collada file) aligns with Assimp's y-axis. Here we are reverting this
  // rotation. We are only catching files with the .dae file ending here. We
  // might miss files with an .xml file ending, which would need to be looked
  // into to figure out whether they are Collada files.
  std::string extension;
  const std::size_t extensionIndex = uri.find_last_of('.');
  if (extensionIndex != std::string::npos)
    extension = uri.substr(extensionIndex);

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
    DART_WARN("[MeshShape::loadMesh] Failed pre-transforming vertices.");

  aiReleasePropertyStore(propertyStore);

  return std::unique_ptr<const aiScene>(scene);
};

//==============================================================================
template <typename S>
std::unique_ptr<typename AssimpMeshLoader<S>::Mesh> AssimpMeshLoader<S>::load(
    const std::string& uri, common::ResourceRetrieverPtr retriever)
{
  if (!retriever)
    retriever = std::make_shared<common::LocalResourceRetriever>();

  // Load the scene and return nullptr if it fails.
  std::unique_ptr<const aiScene> assimpScene = loadAssimpScene(uri, retriever);
  if (!assimpScene)
    return nullptr;

  // Return nullptr if there are no meshes.
  if (assimpScene->mNumMeshes == 0)
    return nullptr;

  // TODO(JS): Support multiple meshes.
  auto assimpMesh = assimpScene->mMeshes[0];
  DART_ASSERT(assimpMesh);

  auto mesh = std::make_unique<math::TriMesh<S>>();

  // Parse faces
  mesh->reserveTriangles(assimpMesh->mNumFaces);
  for (auto i = 0u; i < assimpMesh->mNumFaces; ++i) {
    const aiFace& face = assimpMesh->mFaces[i];
    // TODO(JS): Support more than triangles.
    if (face.mNumIndices != 3)
      return nullptr;
    mesh->addTriangle(face.mIndices[0], face.mIndices[1], face.mIndices[2]);
  }

  // Parse vertices
  mesh->reserveVertices(assimpMesh->mNumVertices);
  for (auto i = 0u; i < assimpMesh->mNumVertices; ++i) {
    const aiVector3D& assimpVertex = assimpMesh->mVertices[i];
    mesh->addVertex(assimpVertex.x, assimpVertex.y, assimpVertex.z);
  }

  // Parse vertex normals
  if (assimpMesh->mNormals) {
    mesh->reserveVertexNormals(assimpMesh->mNumVertices);
    for (auto i = 0u; i < assimpMesh->mNumVertices; ++i) {
      const aiVector3D& assimpNormal = assimpMesh->mNormals[i];
      mesh->addVertexNormal(assimpNormal.x, assimpNormal.y, assimpNormal.z);
    }
  } else {
    // Note: Faces should be set in prior.
    mesh->computeVertexNormals();
  }

  return mesh;
}

} // namespace dart::io
