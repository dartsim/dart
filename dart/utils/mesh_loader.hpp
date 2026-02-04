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

#ifndef DART_UTILS_MESHLOADER_HPP_
#define DART_UTILS_MESHLOADER_HPP_

#include <dart/dynamics/detail/assimp_input_resource_adaptor.hpp>

#include <dart/math/polygon_mesh.hpp>
#include <dart/math/tri_mesh.hpp>

#include <dart/common/diagnostics.hpp>
#include <dart/common/resource.hpp>
#include <dart/common/resource_retriever.hpp>
#include <dart/common/uri.hpp>

#include <assimp/Importer.hpp>
#include <assimp/cfileio.h>
#include <assimp/cimport.h>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <algorithm>
#include <memory>
#include <string>
#include <string_view>

#include <cassert>
#include <cctype>
#include <cstddef>

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
  using PolygonMesh = math::PolygonMesh<S>;

  /// Constructor.
  MeshLoader() = default;

  /// Destructor.
  ~MeshLoader() = default;

  /// Loads a mesh from a file using Assimp.
  /// The mesh is triangulated using PolygonMesh::triangulate().
  ///
  /// @param[in] filepath Path or URI to the mesh file
  /// @param[in] retriever Optional resource retriever for loading from URIs
  /// @return A unique pointer to the loaded mesh, or nullptr if loading fails
  [[nodiscard]] std::unique_ptr<Mesh> load(
      std::string_view filepath,
      const common::ResourceRetrieverPtr& retriever = nullptr);

  /// Loads a polygon mesh from a file using Assimp.
  /// Polygon faces are preserved (no pre-triangulation).
  ///
  /// @param[in] filepath Path or URI to the mesh file
  /// @param[in] retriever Optional resource retriever for loading from URIs
  /// @return A unique pointer to the loaded polygon mesh, or nullptr if loading
  /// fails
  [[nodiscard]] std::unique_ptr<PolygonMesh> loadPolygonMesh(
      std::string_view filepath,
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
      std::string_view filepath, const common::ResourceRetrieverPtr& retriever);
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

#include <dart/common/logging.hpp>

#include <filesystem>

namespace dart {
namespace utils {

//==============================================================================
template <typename S>
std::unique_ptr<typename MeshLoader<S>::Mesh> MeshLoader<S>::load(
    std::string_view filepath, const common::ResourceRetrieverPtr& retriever)
{
  auto polygonMesh = loadPolygonMesh(filepath, retriever);
  if (!polygonMesh) {
    return nullptr;
  }

  auto triMesh = std::make_unique<Mesh>(polygonMesh->triangulate());
  return triMesh;
}

//==============================================================================
template <typename S>
std::unique_ptr<typename MeshLoader<S>::PolygonMesh>
MeshLoader<S>::loadPolygonMesh(
    std::string_view filepath, const common::ResourceRetrieverPtr& retriever)
{
  // Load the scene and return nullptr if it fails
  aiScenePtr scene = loadScene(filepath, retriever);
  if (!scene) {
    DART_WARN(
        "[MeshLoader::loadPolygonMesh] Failed to load mesh from: {}", filepath);
    return nullptr;
  }

  // Return nullptr if there are no meshes
  if (scene->mNumMeshes == 0) {
    DART_WARN("[MeshLoader::loadPolygonMesh] No meshes found in: {}", filepath);
    return nullptr;
  }

  // Create the output mesh
  auto mesh = std::make_unique<PolygonMesh>();

  // Merge all aiMeshes into a single PolygonMesh.
  std::size_t totalVertices = 0;
  std::size_t totalFaces = 0;
  for (std::size_t i = 0; i < scene->mNumMeshes; ++i) {
    const aiMesh* assimpMesh = scene->mMeshes[i];
    if (!assimpMesh) {
      continue;
    }
    totalVertices += assimpMesh->mNumVertices;
    totalFaces += assimpMesh->mNumFaces;
  }

  mesh->reserveVertices(totalVertices);
  mesh->reserveFaces(totalFaces);
  mesh->reserveVertexNormals(totalVertices);

  for (std::size_t meshIndex = 0; meshIndex < scene->mNumMeshes; ++meshIndex) {
    const aiMesh* assimpMesh = scene->mMeshes[meshIndex];
    if (!assimpMesh) {
      continue;
    }

    const std::size_t vertexOffset = mesh->getVertices().size();

    // Parse vertices
    for (auto i = 0u; i < assimpMesh->mNumVertices; ++i) {
      const aiVector3D& vertex = assimpMesh->mVertices[i];
      mesh->addVertex(
          static_cast<S>(vertex.x),
          static_cast<S>(vertex.y),
          static_cast<S>(vertex.z));
    }

    // Parse faces
    for (auto i = 0u; i < assimpMesh->mNumFaces; ++i) {
      const aiFace& face = assimpMesh->mFaces[i];
      if (face.mNumIndices < 3) {
        DART_WARN(
            "[MeshLoader::loadPolygonMesh] Face with fewer than 3 indices "
            "detected in: {} (mesh {}). Skipping this face.",
            filepath,
            meshIndex);
        continue;
      }

      typename PolygonMesh::Face polygonFace;
      polygonFace.reserve(face.mNumIndices);
      for (auto index = 0u; index < face.mNumIndices; ++index) {
        polygonFace.push_back(face.mIndices[index] + vertexOffset);
      }
      mesh->addFace(std::move(polygonFace));
    }

    // Parse vertex normals
    if (assimpMesh->mNormals) {
      for (auto i = 0u; i < assimpMesh->mNumVertices; ++i) {
        const aiVector3D& normal = assimpMesh->mNormals[i];
        mesh->addVertexNormal(
            static_cast<S>(normal.x),
            static_cast<S>(normal.y),
            static_cast<S>(normal.z));
      }
    }
  }

  return mesh;
}

//==============================================================================
template <typename S>
typename MeshLoader<S>::aiScenePtr MeshLoader<S>::loadScene(
    std::string_view filepath, const common::ResourceRetrieverPtr& retriever)
{
  auto hasColladaExtension = [](std::string_view path) -> bool {
    const std::size_t extensionIndex = path.find_last_of('.');
    if (extensionIndex == std::string_view::npos) {
      return false;
    }

    std::string extension(path.substr(extensionIndex));
    std::transform(
        extension.begin(), extension.end(), extension.begin(), ::tolower);
    return extension == ".dae" || extension == ".zae";
  };

  auto isColladaResource = [&](std::string_view uri) -> bool {
    if (hasColladaExtension(uri)) {
      return true;
    }

    const std::string uriString(uri);
    const auto parsedUri = common::Uri::createFromStringOrPath(uriString);
    if (parsedUri.mScheme.get_value_or("file") == "file" && parsedUri.mPath) {
      if (hasColladaExtension(parsedUri.mPath.get())) {
        return true;
      }
    }

    if (!retriever) {
      return false;
    }

    const auto resource = retriever->retrieve(parsedUri);
    if (!resource) {
      return false;
    }

    constexpr std::size_t kMaxProbeSize = 4096;
    const auto sampleSize = std::min(kMaxProbeSize, resource->getSize());
    std::string buffer(sampleSize, '\0');
    const auto read = resource->read(buffer.data(), 1, sampleSize);
    buffer.resize(read);
    resource->seek(0, common::Resource::SEEKTYPE_SET);

    const auto upper = buffer.find("COLLADA");
    const auto lower = buffer.find("collada");
    const auto mixed = buffer.find("Collada");
    return upper != std::string::npos || lower != std::string::npos
           || mixed != std::string::npos;
  };

  const std::string filepathString(filepath);
  const bool isCollada = isColladaResource(filepathString);

  // Remove points and lines from the import
  aiPropertyStore* propertyStore = aiCreatePropertyStore();
  aiSetImportPropertyInteger(
      propertyStore,
      AI_CONFIG_PP_SBP_REMOVE,
      aiPrimitiveType_POINT | aiPrimitiveType_LINE);

#ifdef AI_CONFIG_IMPORT_COLLADA_IGNORE_UP_DIRECTION
  if (isCollada) {
    // Keep authoring up-axis and allow us to preserve the Collada unit scale.
    aiSetImportPropertyInteger(
        propertyStore, AI_CONFIG_IMPORT_COLLADA_IGNORE_UP_DIRECTION, 1);
  }
#endif

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

  // Import the file with post-processing flags (keep polygons intact).
  const aiScene* scene = aiImportFileExWithProperties(
      filepathString.c_str(),
      aiProcess_GenNormals | aiProcess_JoinIdenticalVertices
          | aiProcess_SortByPType | aiProcess_OptimizeMeshes,
      retriever ? &fileIO : nullptr,
      propertyStore);

  // If loading failed, clean up and return
  if (!scene) {
    DART_WARN(
        "[MeshLoader::loadScene] Failed to import mesh from: {}",
        filepathString);
    DART_WARN("  Assimp error: {}", aiGetErrorString());
    aiReleasePropertyStore(propertyStore);
    return nullptr;
  }

#if !defined(AI_CONFIG_IMPORT_COLLADA_IGNORE_UP_DIRECTION)
  // Assimp rotates Collada files to align the up-axis with y-axis.
  // Revert this transformation so authored coordinates remain consistent.
  if (isCollada && scene->mRootNode) {
    const_cast<aiScene*>(scene)->mRootNode->mTransformation = aiMatrix4x4();
  }
#endif

  // Apply vertex pre-transformation
  // We can't do this as part of the import process because we may have
  // changed mTransformation above
  scene = aiApplyPostProcessing(scene, aiProcess_PreTransformVertices);
  if (!scene) {
    DART_WARN("[MeshLoader::loadScene] Failed to pre-transform vertices.");
  }

  aiReleasePropertyStore(propertyStore);

  return aiScenePtr(scene);
}

} // namespace utils
} // namespace dart

#endif // DART_UTILS_MESHLOADER_HPP_
