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

#include "dart/simulation/experimental/io/obj_triangle_mesh.hpp"

#include "dart/simulation/experimental/common/exceptions.hpp"

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace dart::simulation::experimental::io {

namespace {

// Parse one face-vertex token (``v``, ``v/vt``, ``v/vt/vn`` or ``v//vn``) and
// resolve its (possibly negative / 1-based) position index to a 0-based index
// into the ``vertexCount`` vertices read so far.
std::size_t parseFaceVertex(const std::string& token, std::size_t vertexCount)
{
  DART_EXPERIMENTAL_THROW_T_IF(
      token.empty(), InvalidArgumentException, ".obj: empty face vertex");
  const std::string indexText = token.substr(0, token.find('/'));
  long long index = 0;
  std::istringstream stream(indexText);
  stream >> index;
  DART_EXPERIMENTAL_THROW_T_IF(
      !stream || index == 0,
      InvalidArgumentException,
      ".obj: malformed face vertex index '{}'",
      token);
  // Negative indices are relative to the end of the current vertex list.
  const long long resolved
      = (index < 0) ? static_cast<long long>(vertexCount) + index : index - 1;
  DART_EXPERIMENTAL_THROW_T_IF(
      resolved < 0 || resolved >= static_cast<long long>(vertexCount),
      InvalidArgumentException,
      ".obj: face vertex index out of range '{}'",
      token);
  return static_cast<std::size_t>(resolved);
}

} // namespace

//==============================================================================
TriangleMesh loadObjTriangleMesh(std::istream& input)
{
  TriangleMesh mesh;
  std::string line;
  while (std::getline(input, line)) {
    // Strip a trailing CR (Windows line endings) and skip blank/comment lines.
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    std::istringstream stream(line);
    std::string keyword;
    if (!(stream >> keyword) || keyword.empty() || keyword[0] == '#') {
      continue;
    }

    if (keyword == "v") {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      stream >> x >> y >> z;
      DART_EXPERIMENTAL_THROW_T_IF(
          !stream,
          InvalidArgumentException,
          ".obj: malformed vertex line '{}'",
          line);
      mesh.positions.emplace_back(x, y, z);
    } else if (keyword == "f") {
      std::vector<std::size_t> face;
      std::string token;
      while (stream >> token) {
        face.push_back(parseFaceVertex(token, mesh.positions.size()));
      }
      DART_EXPERIMENTAL_THROW_T_IF(
          face.size() < 3,
          InvalidArgumentException,
          ".obj: face with fewer than three vertices '{}'",
          line);
      // Fan-triangulate the (possibly polygonal) face around its first vertex.
      for (std::size_t i = 1; i + 1 < face.size(); ++i) {
        mesh.triangles.push_back({face[0], face[i], face[i + 1]});
      }
    }
    // All other keywords (vn, vt, vp, o, g, s, usemtl, mtllib, ...) are
    // ignored.
  }

  DART_EXPERIMENTAL_THROW_T_IF(
      mesh.positions.empty(),
      InvalidArgumentException,
      ".obj: no vertices found");
  DART_EXPERIMENTAL_THROW_T_IF(
      mesh.triangles.empty(), InvalidArgumentException, ".obj: no faces found");
  return mesh;
}

//==============================================================================
TriangleMesh loadObjTriangleMeshFile(const std::filesystem::path& path)
{
  std::ifstream file(path);
  DART_EXPERIMENTAL_THROW_T_IF(
      !file,
      InvalidArgumentException,
      ".obj: cannot open file '{}'",
      path.string());
  return loadObjTriangleMesh(file);
}

} // namespace dart::simulation::experimental::io
