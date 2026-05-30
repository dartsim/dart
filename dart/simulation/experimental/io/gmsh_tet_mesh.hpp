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

#include <Eigen/Core>

#include <array>
#include <filesystem>
#include <iosfwd>
#include <vector>

#include <cstddef>

namespace dart::simulation::experimental::io {

/// A tetrahedral volume mesh: node positions and 0-based tetrahedron
/// connectivity. This is the geometry a deformable FEM body is built from
/// (``DeformableBodyOptions.positions`` / ``.tetrahedra``).
struct TetMesh
{
  std::vector<Eigen::Vector3d> positions;
  std::vector<std::array<std::size_t, 4>> tetrahedra;
};

/// Parse a GMSH ASCII ``.msh`` (format version 2.x or 4.x) tetrahedral mesh
/// from a stream. Reads the ``$Nodes`` and the 4-node tetrahedron
/// (``$Elements`` element type 4) connectivity for both the legacy (2.x) and
/// entity-block (4.x) layouts, remapping GMSH node ids to 0-based indices;
/// other element types (points, lines, triangles) are ignored. Throws
/// ``InvalidArgumentException`` on malformed input or an unsupported (binary,
/// or version &lt; 2 / &ge; 5) format.
TetMesh loadGmshTetMesh(std::istream& input);

/// Same as ``loadGmshTetMesh``, reading from a file path.
TetMesh loadGmshTetMeshFile(const std::filesystem::path& path);

} // namespace dart::simulation::experimental::io
