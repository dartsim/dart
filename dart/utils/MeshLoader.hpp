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

#ifndef DART_UTILS_MESHLOADER_HPP_
#define DART_UTILS_MESHLOADER_HPP_

#include <dart/dynamics/detail/AssimpInputResourceAdaptor.hpp>

#include <dart/math/TriMesh.hpp>

#include <dart/common/Deprecated.hpp>
#include <dart/common/ResourceRetriever.hpp>
#include <dart/common/Uri.hpp>

#include <assimp/Importer.hpp>
#include <assimp/cfileio.h>
#include <assimp/cimport.h>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <algorithm>

#include <cassert>

namespace dart {
namespace utils {

/// Mesh loader using the Assimp library.
///
/// This loader supports various 3D mesh formats through Assimp including:
/// - COLLADA (.dae)
/// - Wavefront OBJ (.obj)
/// - STL (.stl)
/// - PLY (.ply)
/// - And many more formats supported by Assimp
///
/// @tparam S Scalar type for mesh coordinates (typically float or double)
template <typename S>
class MeshLoader
{
public:
  using Scalar = S;
  using Mesh = math::TriMesh<S>;

  /// Constructor.
  MeshLoader() = default;

  /// Destructor.
  ~MeshLoader() = default;

  /// Loads a mesh from a file using Assimp.
  ///
  /// @param[in] filepath Path or URI to the mesh file
  /// @param[in] retriever Optional resource retriever for loading from URIs
  /// @return A unique pointer to the loaded mesh, or nullptr if loading fails
  [[nodiscard]] std::unique_ptr<Mesh> load(
      const std::string& filepath,
      const common::ResourceRetrieverPtr& retriever = nullptr);

private:
  /// Custom deleter for aiScene that calls aiReleaseImport.
  struct aiSceneDeleter
  {
    void operator()(const aiScene* scene) const
    {
      if (scene) {
        aiReleaseImport(scene);
      }
    }
  };

  using aiScenePtr = std::unique_ptr<const aiScene, aiSceneDeleter>;

  /// Loads an aiScene using Assimp with the given filepath and retriever.
  static aiScenePtr loadScene(
      const std::string& filepath,
      const common::ResourceRetrieverPtr& retriever);
};

using MeshLoaderf = MeshLoader<float>;
using MeshLoaderd = MeshLoader<double>;

} // namespace utils
} // namespace dart

//==============================================================================
//
// Implementation
//
//==============================================================================

#include <dart/common/Console.hpp>

#include <filesystem>

namespace dart {
namespace utils {

//==============================================================================
template <typename S>
std::unique_ptr<typename MeshLoader<S>::Mesh> MeshLoader<S>::load(
    const std::string& filepath, const common::ResourceRetrieverPtr& retriever)
{
  // Load the scene and return nullptr if it fails
  aiScenePtr scene = loadScene(filepath, retriever);
  if (!scene) {
    dtwarn << "[MeshLoader::load] Failed to load mesh from: " << filepath
           << "\n";
    return nullptr;
  }

  // Return nullptr if there are no meshes
  if (scene->mNumMeshes == 0) {
    dtwarn << "[MeshLoader::load] No meshes found in: " << filepath << "\n";
    return nullptr;
  }

  // Create the output mesh
  auto mesh = std::make_unique<Mesh>();

  // Process all meshes in the scene
  // TODO(JS): Support merging multiple meshes from the scene
  const aiMesh* assimpMesh = scene->mMeshes[0];
  assert(assimpMesh);

  // Reserve space for vertices
  mesh->reserveVertices(assimpMesh->mNumVertices);

  // Parse vertices
  for (auto i = 0u; i < assimpMesh->mNumVertices; ++i) {
    const aiVector3D& vertex = assimpMesh->mVertices[i];
    mesh->addVertex(
        static_cast<S>(vertex.x),
        static_cast<S>(vertex.y),
        static_cast<S>(vertex.z));
  }

  // Parse faces (triangles)
  mesh->reserveTriangles(assimpMesh->mNumFaces);
  for (auto i = 0u; i < assimpMesh->mNumFaces; ++i) {
    const aiFace& face = assimpMesh->mFaces[i];

    // TODO(JS): Support non-triangular faces
    if (face.mNumIndices != 3) {
      dtwarn << "[MeshLoader::load] Non-triangular face detected in: "
             << filepath << ". Skipping this face.\n";
      continue;
    }

    mesh->addTriangle(face.mIndices[0], face.mIndices[1], face.mIndices[2]);
  }

  // Parse vertex normals
  if (assimpMesh->mNormals) {
    mesh->reserveVertexNormals(assimpMesh->mNumVertices);
    for (auto i = 0u; i < assimpMesh->mNumVertices; ++i) {
      const aiVector3D& normal = assimpMesh->mNormals[i];
      mesh->addVertexNormal(
          static_cast<S>(normal.x),
          static_cast<S>(normal.y),
          static_cast<S>(normal.z));
    }
  } else {
    // Compute vertex normals if not provided
    mesh->computeVertexNormals();
  }

  return mesh;
}

//==============================================================================
template <typename S>
typename MeshLoader<S>::aiScenePtr MeshLoader<S>::loadScene(
    const std::string& filepath, const common::ResourceRetrieverPtr& retriever)
{
  // Remove points and lines from the import
  aiPropertyStore* propertyStore = aiCreatePropertyStore();
  aiSetImportPropertyInteger(
      propertyStore,
      AI_CONFIG_PP_SBP_REMOVE,
      aiPrimitiveType_POINT | aiPrimitiveType_LINE);

  // Set up the Assimp IOSystem for resource retrieval
  // Suppress deprecation warnings - internal implementation detail
  DART_SUPPRESS_DEPRECATED_BEGIN
  std::unique_ptr<dynamics::AssimpInputResourceRetrieverAdaptor> systemIO;
  aiFileIO fileIO;

  if (retriever) {
    systemIO = std::make_unique<dynamics::AssimpInputResourceRetrieverAdaptor>(
        retriever);
    fileIO = dynamics::createFileIO(systemIO.get());
  }
  DART_SUPPRESS_DEPRECATED_END

  // Import the file with post-processing flags
  const aiScene* scene = aiImportFileExWithProperties(
      filepath.c_str(),
      aiProcess_GenNormals | aiProcess_Triangulate
          | aiProcess_JoinIdenticalVertices | aiProcess_SortByPType
          | aiProcess_OptimizeMeshes,
      retriever ? &fileIO : nullptr,
      propertyStore);

  // If loading failed, clean up and return
  if (!scene) {
    dtwarn << "[MeshLoader::loadScene] Failed to import mesh from: " << filepath
           << "\n";
    dtwarn << "  Assimp error: " << aiGetErrorString() << "\n";
    aiReleasePropertyStore(propertyStore);
    return nullptr;
  }

  // Handle Collada file transformation quirks
  // Assimp rotates Collada files to align the up-axis with y-axis.
  // We revert this transformation here.
  std::string extension;
  const std::size_t extensionIndex = filepath.find_last_of('.');
  if (extensionIndex != std::string::npos) {
    extension = filepath.substr(extensionIndex);
  }

  std::transform(
      std::begin(extension),
      std::end(extension),
      std::begin(extension),
      ::tolower);

  if (extension == ".dae" || extension == ".zae") {
    const_cast<aiScene*>(scene)->mRootNode->mTransformation = aiMatrix4x4();
  }

  // Apply vertex pre-transformation
  // We can't do this as part of the import process because we may have
  // changed mTransformation above
  scene = aiApplyPostProcessing(scene, aiProcess_PreTransformVertices);
  if (!scene) {
    dtwarn << "[MeshLoader::loadScene] Failed to pre-transform vertices.\n";
  }

  aiReleasePropertyStore(propertyStore);

  return aiScenePtr(scene);
}

} // namespace utils
} // namespace dart

#endif // DART_UTILS_MESHLOADER_HPP_
