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

#ifndef DART_GUI_EXPERIMENTAL_GEOMETRY_HPP_
#define DART_GUI_EXPERIMENTAL_GEOMETRY_HPP_

#include <dart/gui/export.hpp>

#include <Eigen/Core>

#include <vector>

#include <cstddef>
#include <cstdint>

namespace dart::gui::experimental {

struct MeshVertex
{
  Eigen::Vector3f position = Eigen::Vector3f::Zero();
  Eigen::Vector3f normal = Eigen::Vector3f::UnitZ();
  Eigen::Vector2f uv = Eigen::Vector2f::Zero();
};

struct MeshTriangle
{
  std::uint32_t a = 0u;
  std::uint32_t b = 0u;
  std::uint32_t c = 0u;
};

struct MeshIndexRange
{
  std::size_t indexOffset = 0u;
  std::size_t indexCount = 0u;
};

struct MeshGeometry
{
  std::vector<MeshVertex> vertices;
  std::vector<std::uint32_t> indices;
  std::vector<MeshTriangle> triangles;
  Eigen::Vector3f boundsMin = Eigen::Vector3f::Zero();
  Eigen::Vector3f boundsMax = Eigen::Vector3f::Zero();
  bool hasBounds = false;
};

DART_GUI_API void appendTriangle(
    MeshGeometry& mesh, std::uint32_t a, std::uint32_t b, std::uint32_t c);

DART_GUI_API MeshIndexRange appendBoxMeshGeometry(
    MeshGeometry& mesh,
    const Eigen::Vector3d& center,
    const Eigen::Vector3d& size);

DART_GUI_API void appendEllipsoidMeshGeometry(
    MeshGeometry& mesh,
    const Eigen::Vector3d& center,
    const Eigen::Vector3d& radii);

DART_GUI_API MeshGeometry makeBoxMeshGeometry(const Eigen::Vector3d& size);

DART_GUI_API MeshGeometry
makeEllipsoidMeshGeometry(const Eigen::Vector3d& radii);

DART_GUI_API MeshGeometry makeMultiSphereMeshGeometry(
    const std::vector<Eigen::Vector3d>& centers,
    const std::vector<double>& radii);

DART_GUI_API MeshGeometry
makeCylinderMeshGeometry(double radius, double height);

DART_GUI_API MeshGeometry makeConeMeshGeometry(double radius, double height);

DART_GUI_API MeshGeometry makePyramidMeshGeometry(const Eigen::Vector3d& size);

DART_GUI_API MeshGeometry makeCapsuleMeshGeometry(double radius, double height);

} // namespace dart::gui::experimental

#endif // DART_GUI_EXPERIMENTAL_GEOMETRY_HPP_
