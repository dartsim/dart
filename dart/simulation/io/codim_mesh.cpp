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

#include "dart/simulation/io/codim_mesh.hpp"

#include "dart/simulation/common/exceptions.hpp"

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace dart::simulation::io {

namespace {

// Resolve a (possibly negative / 1-based) vertex index to a 0-based index into
// the ``vertexCount`` vertices read so far.
std::size_t resolveVertexIndex(long long index, std::size_t vertexCount)
{
  DART_SIMULATION_THROW_T_IF(
      index == 0,
      InvalidArgumentException,
      ".seg/.pt: vertex index 0 (indices are 1-based)");
  const long long resolved
      = (index < 0) ? static_cast<long long>(vertexCount) + index : index - 1;
  DART_SIMULATION_THROW_T_IF(
      resolved < 0 || resolved >= static_cast<long long>(vertexCount),
      InvalidArgumentException,
      ".seg: vertex index out of range");
  return static_cast<std::size_t>(resolved);
}

// Drop a trailing CR (Windows line endings).
void stripCarriageReturn(std::string& line)
{
  if (!line.empty() && line.back() == '\r') {
    line.pop_back();
  }
}

} // namespace

//==============================================================================
SegmentMesh loadSegLineMesh(std::istream& input)
{
  SegmentMesh mesh;
  std::string line;
  while (std::getline(input, line)) {
    stripCarriageReturn(line);
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
      DART_SIMULATION_THROW_T_IF(
          !stream,
          InvalidArgumentException,
          ".seg: malformed vertex line '{}'",
          line);
      mesh.positions.emplace_back(x, y, z);
    } else if (keyword == "l") {
      std::vector<std::size_t> polyline;
      long long index = 0;
      while (stream >> index) {
        polyline.push_back(resolveVertexIndex(index, mesh.positions.size()));
      }
      DART_SIMULATION_THROW_T_IF(
          polyline.size() < 2,
          InvalidArgumentException,
          ".seg: line element with fewer than two vertices '{}'",
          line);
      for (std::size_t i = 0; i + 1 < polyline.size(); ++i) {
        mesh.segments.push_back({polyline[i], polyline[i + 1]});
      }
    }
    // All other keywords (vn, vt, ...) are ignored.
  }

  DART_SIMULATION_THROW_T_IF(
      mesh.positions.empty(),
      InvalidArgumentException,
      ".seg: no vertices found");
  DART_SIMULATION_THROW_T_IF(
      mesh.segments.empty(),
      InvalidArgumentException,
      ".seg: no segments found");
  return mesh;
}

//==============================================================================
SegmentMesh loadSegLineMeshFile(const std::filesystem::path& path)
{
  std::ifstream file(path);
  DART_SIMULATION_THROW_T_IF(
      !file,
      InvalidArgumentException,
      ".seg: cannot open file '{}'",
      path.string());
  return loadSegLineMesh(file);
}

//==============================================================================
PointSet loadPointSet(std::istream& input)
{
  PointSet points;
  std::string line;
  while (std::getline(input, line)) {
    stripCarriageReturn(line);
    std::istringstream stream(line);
    std::string first;
    if (!(stream >> first) || first.empty() || first[0] == '#') {
      continue;
    }

    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    if (first == "v") {
      // Wavefront vertex line: the coordinates follow the keyword.
      stream >> x >> y >> z;
      DART_SIMULATION_THROW_T_IF(
          !stream,
          InvalidArgumentException,
          ".pt: malformed vertex line '{}'",
          line);
    } else {
      // Bare "x y z" line: the first token is already the x coordinate.
      std::istringstream coords(first);
      coords >> x;
      stream >> y >> z;
      DART_SIMULATION_THROW_T_IF(
          !coords || !stream,
          InvalidArgumentException,
          ".pt: malformed point line '{}'",
          line);
    }
    points.positions.emplace_back(x, y, z);
  }

  DART_SIMULATION_THROW_T_IF(
      points.positions.empty(),
      InvalidArgumentException,
      ".pt: no points found");
  return points;
}

//==============================================================================
PointSet loadPointSetFile(const std::filesystem::path& path)
{
  std::ifstream file(path);
  DART_SIMULATION_THROW_T_IF(
      !file,
      InvalidArgumentException,
      ".pt: cannot open file '{}'",
      path.string());
  return loadPointSet(file);
}

} // namespace dart::simulation::io
