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

#ifndef DART_COLLISION_ODE_DETAIL_ODEMESH_HPP_
#define DART_COLLISION_ODE_DETAIL_ODEMESH_HPP_

#include <dart/collision/ode/detail/OdeGeom.hpp>

#include <assimp/scene.h>
#include <ode/ode.h>

namespace dart {
namespace collision {
namespace detail {

class OdeMesh : public OdeGeom
{
public:
  /// Constructor
  OdeMesh(
      const OdeCollisionObject* parent,
      const aiScene* scene,
      const Eigen::Vector3d& scale = Eigen::Vector3d::Ones());

  /// Destructor
  virtual ~OdeMesh();

  // Documentation inherited
  void updateEngineData() override;

private:
  void fillArrays(
      const aiScene* scene,
      const Eigen::Vector3d& scale = Eigen::Vector3d::Ones());

private:
  /// Array of vertex values.
  std::vector<double> mVertices;

  /// Array of normals values.
  std::vector<double> mNormals;

  /// Array of index values.
  std::vector<int> mIndices;

  /// ODE trimesh data.
  dTriMeshDataID mOdeTriMeshDataId;
};

} // namespace detail
} // namespace collision
} // namespace dart

#endif // DART_COLLISION_ODE_DETAIL_ODEMESH_HPP_
