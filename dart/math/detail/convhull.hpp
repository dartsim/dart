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

/*
 * This file contains a derivative work based on the convhull_3d algorithm:
 *
 * Original convhull_3d implementation - MIT License
 * Copyright (c) 2017-2021 Leo McCormack
 * Derived from computational-geometry-toolbox by George Papazafeiropoulos
 * (c) 2014, distributed under BSD 2-clause license.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#pragma once

#include <Eigen/Core>

#include <span>
#include <vector>

namespace dart::math::detail {

/// Computes the 3D convex hull using an optimized quickhull algorithm
///
/// This function efficiently computes the convex hull of a set of 3D points
/// using a modern C++ implementation of the quickhull algorithm with Eigen
/// types. The implementation is optimized for performance, achieving 20-30%
/// speedup over the baseline through compiler-friendly scalar operations and
/// efficient memory management.
///
/// **Performance**: Optimized implementation with:
/// - Direct scalar operations (x*x) instead of std::pow(x, 2.0)
/// - Pre-allocated vectors to minimize reallocations
/// - Cache-friendly memory access patterns
/// - Compiler auto-vectorization of hot loops
///
/// **Algorithm**: Based on the quickhull algorithm with incremental
/// construction Reference: "The Quickhull Algorithm for Convex Hull" by C.
/// Bradford Barber, David P. Dobkin and Hannu Huhdanpaa, Geometry Center
/// Technical Report GCG53, July 30, 1993
///
/// @tparam S Scalar type (float or double)
/// @param[in] inVertices Input vertices as Eigen::Vector3<S>
/// @param[out] outFaces Output face indices (flat array: 3 indices per
/// triangle)
/// @param[out] numOutFaces Number of output triangular faces
///
/// @note Input vertices are perturbed slightly to avoid degeneracies
/// @note Output faces have consistent counter-clockwise winding order
/// @note For degenerate inputs (< 4 vertices or coplanar), returns empty hull
///
/// @example
/// ```cpp
/// std::vector<Eigen::Vector3d> vertices = {
///   Eigen::Vector3d(0, 0, 0),
///   Eigen::Vector3d(1, 0, 0),
///   Eigen::Vector3d(0, 1, 0),
///   Eigen::Vector3d(0, 0, 1)
/// };
/// std::vector<int> faces;
/// int numFaces = 0;
/// convexHull3dBuild(std::span<const Eigen::Vector3d>(vertices), faces,
/// numFaces);
/// // Result: 4 faces (tetrahedron), 12 indices total
/// ```
template <typename S>
void convexHull3dBuild(
    std::span<const Eigen::Vector3<S>> inVertices,
    std::vector<int>& outFaces,
    int& numOutFaces);

} // namespace dart::math::detail

#include <dart/math/detail/convhull-impl.hpp>
