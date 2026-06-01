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

#include <dart/simulation/experimental/export.hpp>

#include <Eigen/Core>

#include <array>
#include <filesystem>
#include <iosfwd>
#include <vector>

#include <cstddef>

namespace dart::simulation::experimental::io {

/// A codimensional segment (edge) mesh: vertex positions and 0-based segment
/// connectivity. This is the geometry a one-dimensional deformable strand
/// (mass-spring chain) -- or, later, a codimensional edge collision object --
/// is built from.
struct SegmentMesh
{
  std::vector<Eigen::Vector3d> positions;
  std::vector<std::array<std::size_t, 2>> segments;
};

/// A codimensional point set: just vertex positions, the geometry a cloud of
/// free deformable particles -- or a codimensional point collision object -- is
/// built from.
struct PointSet
{
  std::vector<Eigen::Vector3d> positions;
};

/// Parse a ``.seg`` segment mesh from a stream.
///
/// Reads Wavefront-style ``v`` vertex positions and ``l`` line elements; an
/// ``l v1 v2 ... vk`` polyline contributes the k-1 consecutive segments
/// ``(v1,v2), (v2,v3), ...``. Indices are 1-based; negative indices count back
/// from the current vertex list. ``vn`` / ``vt`` / comments and other keywords
/// are ignored. Throws ``InvalidArgumentException`` on a malformed or
/// out-of-range index, or when no vertices or segments are found.
DART_EXPERIMENTAL_API SegmentMesh loadSegLineMesh(std::istream& input);

/// Same as ``loadSegLineMesh``, reading from a file path.
DART_EXPERIMENTAL_API SegmentMesh
loadSegLineMeshFile(const std::filesystem::path& path);

/// Parse a ``.pt`` point set from a stream. Each non-empty, non-comment line is
/// three whitespace-separated coordinates (an optional leading ``v`` keyword is
/// accepted, so a Wavefront vertex list also loads). Throws
/// ``InvalidArgumentException`` on a malformed line or an empty set.
DART_EXPERIMENTAL_API PointSet loadPointSet(std::istream& input);

/// Same as ``loadPointSet``, reading from a file path.
DART_EXPERIMENTAL_API PointSet
loadPointSetFile(const std::filesystem::path& path);

} // namespace dart::simulation::experimental::io
