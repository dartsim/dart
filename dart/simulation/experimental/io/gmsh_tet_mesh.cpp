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

#include "dart/simulation/experimental/io/gmsh_tet_mesh.hpp"

#include "dart/simulation/experimental/common/exceptions.hpp"

#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>

namespace dart::simulation::experimental::io {

namespace {

// GMSH element type id for a 4-node (linear) tetrahedron.
constexpr int kTetrahedronElementType = 4;

// Strip surrounding whitespace and a trailing CR (Windows line endings).
std::string trimmed(const std::string& line)
{
  const std::size_t begin = line.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return {};
  }
  const std::size_t end = line.find_last_not_of(" \t\r\n");
  return line.substr(begin, end - begin + 1);
}

} // namespace

//==============================================================================
TetMesh loadGmshTetMesh(std::istream& input)
{
  TetMesh mesh;
  std::unordered_map<long long, std::size_t> idToIndex;
  bool sawFormat = false;
  bool sawNodes = false;
  bool sawElements = false;

  std::string raw;
  while (std::getline(input, raw)) {
    const std::string line = trimmed(raw);

    if (line == "$MeshFormat") {
      std::getline(input, raw);
      std::istringstream header(trimmed(raw));
      double version = 0.0;
      int fileType = -1;
      header >> version >> fileType;
      DART_EXPERIMENTAL_THROW_T_IF(
          !header || version < 2.0 || version >= 3.0,
          InvalidArgumentException,
          "GMSH .msh: unsupported format version (only ASCII 2.x is "
          "supported)");
      DART_EXPERIMENTAL_THROW_T_IF(
          fileType != 0,
          InvalidArgumentException,
          "GMSH .msh: binary format is not supported (expected ASCII)");
      sawFormat = true;
    } else if (line == "$Nodes") {
      std::getline(input, raw);
      std::istringstream countStream(trimmed(raw));
      long long count = -1;
      countStream >> count;
      DART_EXPERIMENTAL_THROW_T_IF(
          !countStream || count < 0,
          InvalidArgumentException,
          "GMSH .msh: invalid $Nodes count");
      for (long long i = 0; i < count; ++i) {
        DART_EXPERIMENTAL_THROW_T_IF(
            !std::getline(input, raw),
            InvalidArgumentException,
            "GMSH .msh: truncated $Nodes block");
        std::istringstream node(trimmed(raw));
        long long id = 0;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        node >> id >> x >> y >> z;
        DART_EXPERIMENTAL_THROW_T_IF(
            !node, InvalidArgumentException, "GMSH .msh: malformed node line");
        idToIndex[id] = mesh.positions.size();
        mesh.positions.emplace_back(x, y, z);
      }
      sawNodes = true;
    } else if (line == "$Elements") {
      std::getline(input, raw);
      std::istringstream countStream(trimmed(raw));
      long long count = -1;
      countStream >> count;
      DART_EXPERIMENTAL_THROW_T_IF(
          !countStream || count < 0,
          InvalidArgumentException,
          "GMSH .msh: invalid $Elements count");
      for (long long i = 0; i < count; ++i) {
        DART_EXPERIMENTAL_THROW_T_IF(
            !std::getline(input, raw),
            InvalidArgumentException,
            "GMSH .msh: truncated $Elements block");
        std::istringstream element(trimmed(raw));
        long long id = 0;
        int type = 0;
        int tagCount = 0;
        element >> id >> type >> tagCount;
        DART_EXPERIMENTAL_THROW_T_IF(
            !element || tagCount < 0,
            InvalidArgumentException,
            "GMSH .msh: malformed element header");
        for (int t = 0; t < tagCount; ++t) {
          long long tag = 0;
          element >> tag;
        }
        if (type != kTetrahedronElementType) {
          continue; // ignore points, lines, triangles, etc.
        }
        std::array<std::size_t, 4> tetrahedron{};
        for (int k = 0; k < 4; ++k) {
          long long nodeId = 0;
          element >> nodeId;
          DART_EXPERIMENTAL_THROW_T_IF(
              !element,
              InvalidArgumentException,
              "GMSH .msh: malformed tetrahedron connectivity");
          const auto found = idToIndex.find(nodeId);
          DART_EXPERIMENTAL_THROW_T_IF(
              found == idToIndex.end(),
              InvalidArgumentException,
              "GMSH .msh: tetrahedron references unknown node id {}",
              nodeId);
          tetrahedron[static_cast<std::size_t>(k)] = found->second;
        }
        mesh.tetrahedra.push_back(tetrahedron);
      }
      sawElements = true;
    }
  }

  DART_EXPERIMENTAL_THROW_T_IF(
      !sawFormat || !sawNodes || !sawElements,
      InvalidArgumentException,
      "GMSH .msh: missing $MeshFormat, $Nodes, or $Elements section");
  DART_EXPERIMENTAL_THROW_T_IF(
      mesh.positions.empty(), InvalidArgumentException, "GMSH .msh: no nodes");
  DART_EXPERIMENTAL_THROW_T_IF(
      mesh.tetrahedra.empty(),
      InvalidArgumentException,
      "GMSH .msh: no tetrahedra (element type 4)");
  return mesh;
}

//==============================================================================
TetMesh loadGmshTetMeshFile(const std::filesystem::path& path)
{
  std::ifstream input(path);
  DART_EXPERIMENTAL_THROW_T_IF(
      !input,
      InvalidArgumentException,
      "GMSH .msh: cannot open file '{}'",
      path.string());
  return loadGmshTetMesh(input);
}

} // namespace dart::simulation::experimental::io
