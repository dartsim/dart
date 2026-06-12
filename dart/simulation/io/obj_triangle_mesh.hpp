/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary form, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice and this list of conditions in the documentation
 *     and/or other materials provided with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR
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

#include <dart/simulation/export.hpp>

#include <Eigen/Core>

#include <array>
#include <filesystem>
#include <iosfwd>
#include <vector>

#include <cstddef>

namespace dart::simulation::io {

/// A triangle surface mesh: vertex positions and 0-based triangle connectivity.
/// This is the geometry a deformable membrane/cloth body or a codimensional
/// triangle collision object is built from.
struct TriangleMesh
{
  std::vector<Eigen::Vector3d> positions;
  std::vector<std::array<std::size_t, 3>> triangles;
};

/// Parse a Wavefront ``.obj`` triangle surface mesh from a stream.
///
/// Reads ``v`` vertex positions and ``f`` faces, ignoring everything else
/// (``vn`` / ``vt`` / ``vp`` / ``o`` / ``g`` / ``s`` / ``usemtl`` / ``mtllib``
/// and ``#`` comments). Each face vertex token may be ``v``, ``v/vt``,
/// ``v/vt/vn``, or ``v//vn``; only the position index is used. Indices are
/// 1-based; negative indices count back from the current vertex list. Polygon
/// faces (more than three vertices) are fan-triangulated. Throws
/// ``InvalidArgumentException`` on a malformed vertex/face or an out-of-range
/// index.
DART_SIMULATION_API TriangleMesh loadObjTriangleMesh(std::istream& input);

/// Same as ``loadObjTriangleMesh``, reading from a file path.
DART_SIMULATION_API TriangleMesh
loadObjTriangleMeshFile(const std::filesystem::path& path);

} // namespace dart::simulation::io
