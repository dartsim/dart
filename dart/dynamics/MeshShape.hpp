/*
 * Copyright (c) 2011-2019, The DART development contributors
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

#ifndef DART_DYNAMICS_MESHSHAPE_HPP_
#define DART_DYNAMICS_MESHSHAPE_HPP_

#include <string>

#include <assimp/scene.h>

#include "dart/dynamics/Shape.hpp"
#include "dart/common/ResourceRetriever.hpp"

namespace Assimp {

class IOSystem;

} // namespace Assimp

namespace dart {
namespace dynamics {

class MeshShape : public Shape
{
public:
  enum ColorMode
  {
    MATERIAL_COLOR = 0, ///< Use the colors specified by the Mesh's material
    COLOR_INDEX,        ///< Use the colors specified by aiMesh::mColor
    SHAPE_COLOR,        ///< Use the color specified by the Shape base class
  };

  /// Constructor.
  MeshShape(const Eigen::Vector3d& scale,
    const aiScene* mesh,
    const common::Uri& uri = "",
    common::ResourceRetrieverPtr resourceRetriever = nullptr);

  /// Destructor.
  virtual ~MeshShape();

  // Documentation inherited.
  const std::string& getType() const override;

  /// Returns shape type for this class
  static const std::string& getStaticType();

  const aiScene* getMesh() const;

  /// Updates positions of the vertices or the elements. By default, this does
  /// nothing; you must extend the MeshShape class and implement your own
  /// version of this function if you want the mesh data to get updated before
  /// rendering
  virtual void update();

  // Documentation inherited
  void notifyAlphaUpdated(double alpha) override;

  void setMesh(
    const aiScene* mesh,
    const std::string& path = "",
    common::ResourceRetrieverPtr resourceRetriever = nullptr);

  void setMesh(
    const aiScene* mesh,
    const common::Uri& path,
    common::ResourceRetrieverPtr resourceRetriever = nullptr);

  /// Returns URI to the mesh as std::string; an empty string if unavailable.
  std::string getMeshUri() const;
  // TODO(DART 7): Replace with getMeshUri2().

  /// Returns URI to the mesh; an empty string if unavailable.
  const common::Uri& getMeshUri2() const;
  // TODO(DART 7): Remove.

  /// Returns path to the mesh on disk; an empty string if unavailable.
  const std::string& getMeshPath() const;

  common::ResourceRetrieverPtr getResourceRetriever();

  void setScale(const Eigen::Vector3d& scale);

  const Eigen::Vector3d& getScale() const;

  /// Set how the color of this mesh should be determined
  void setColorMode(ColorMode mode);

  /// Get the coloring mode that this mesh is using
  ColorMode getColorMode() const;

  /// Set which entry in aiMesh::mColor should be used when the color mode is
  /// COLOR_INDEX. This value must be smaller than AI_MAX_NUMBER_OF_COLOR_SETS.
  /// If the color index is higher than what the mesh has available, then we
  /// will use the highest index possible.
  void setColorIndex(int index);

  /// Get the index that will be used when the ColorMode is set to COLOR_INDEX
  int getColorIndex() const;

  int getDisplayList() const;

  void setDisplayList(int index);

  static const aiScene* loadMesh(const std::string& filePath);

  static const aiScene* loadMesh(
    const std::string& _uri, const common::ResourceRetrieverPtr& retriever);

  static const aiScene* loadMesh(
    const common::Uri& uri, const common::ResourceRetrieverPtr& retriever);

  // Documentation inherited.
  Eigen::Matrix3d computeInertia(double mass) const override;

protected:
  // Documentation inherited.
  void updateBoundingBox() const override;

  // Documentation inherited.
  void updateVolume() const override;

  const aiScene* mMesh;

  /// URI the mesh, if available).
  common::Uri mMeshUri;

  /// Path the mesh on disk, if available.
  std::string mMeshPath;

  /// Optional method of loading resources by URI.
  common::ResourceRetrieverPtr mResourceRetriever;

  /// OpenGL DisplayList id for rendering
  int mDisplayList;

  /// Scale
  Eigen::Vector3d mScale;

  /// Specifies how the color of this mesh should be determined
  ColorMode mColorMode;

  /// Specifies which color index should be used when mColorMode is COLOR_INDEX
  int mColorIndex;
};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_MESHSHAPE_HPP_
