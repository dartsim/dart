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

#include <dart/dynamics/fem/TetMesh.hpp>

#include <dart/common/Macros.hpp>

#include <stdexcept>
#include <utility>

namespace dart {
namespace dynamics {
namespace fem {

namespace {

/// Smallest rest volume a tetrahedron may have relative to the cube of its
/// longest edge. A regular tetrahedron sits near 0.118, so this bound rejects
/// only elements that have collapsed to a sliver.
constexpr double kMinVolumeToEdgeCubed = 1e-12;

/// Kuhn decomposition of one grid cell into six tetrahedra, written with the
/// cell-corner bit index i + 2 * j + 4 * k. The six tetrahedra correspond to
/// the six orderings in which the three axes can be traversed from corner 0 to
/// corner 7, so they tile the cell exactly and share conforming faces with
/// neighboring cells. Three of the six come out negatively oriented and are
/// corrected when the box is built.
constexpr int kCellTets[6][4] = {
    {0, 1, 3, 7},
    {0, 1, 5, 7},
    {0, 2, 3, 7},
    {0, 2, 6, 7},
    {0, 4, 5, 7},
    {0, 4, 6, 7},
};

//==============================================================================
Eigen::Matrix3d computeShapeMatrix(
    const Eigen::Vector3d& x0,
    const Eigen::Vector3d& x1,
    const Eigen::Vector3d& x2,
    const Eigen::Vector3d& x3)
{
  Eigen::Matrix3d shape;
  shape.col(0) = x1 - x0;
  shape.col(1) = x2 - x0;
  shape.col(2) = x3 - x0;
  return shape;
}

} // namespace

//==============================================================================
TetMesh::TetMesh(
    std::vector<Eigen::Vector3d> restPositions,
    common::aligned_vector<Eigen::Vector4i> tets)
  : mRestPositions(std::move(restPositions)), mTets(std::move(tets))
{
  computeRestQuantities();
}

//==============================================================================
TetMesh TetMesh::createBox(
    const Eigen::Vector3d& size, const Eigen::Vector3i& divisions)
{
  if ((size.array() <= 0.0).any()) {
    throw std::invalid_argument(
        "TetMesh::createBox: box size must be positive along every axis");
  }

  if ((divisions.array() < 1).any()) {
    throw std::invalid_argument(
        "TetMesh::createBox: divisions must be at least one along every axis");
  }

  const int nx = divisions[0];
  const int ny = divisions[1];
  const int nz = divisions[2];

  std::vector<Eigen::Vector3d> restPositions;
  restPositions.reserve(static_cast<std::size_t>(nx + 1) * (ny + 1) * (nz + 1));

  for (int k = 0; k <= nz; ++k) {
    for (int j = 0; j <= ny; ++j) {
      for (int i = 0; i <= nx; ++i) {
        restPositions.emplace_back(
            -0.5 * size[0] + size[0] * i / nx,
            -0.5 * size[1] + size[1] * j / ny,
            -0.5 * size[2] + size[2] * k / nz);
      }
    }
  }

  const auto nodeIndex = [nx, ny](int i, int j, int k) {
    return (k * (ny + 1) + j) * (nx + 1) + i;
  };

  common::aligned_vector<Eigen::Vector4i> tets;
  tets.reserve(static_cast<std::size_t>(nx) * ny * nz * 6);

  for (int k = 0; k < nz; ++k) {
    for (int j = 0; j < ny; ++j) {
      for (int i = 0; i < nx; ++i) {
        int cellCorner[8];
        for (int kk = 0; kk < 2; ++kk) {
          for (int jj = 0; jj < 2; ++jj) {
            for (int ii = 0; ii < 2; ++ii) {
              cellCorner[ii + 2 * jj + 4 * kk]
                  = nodeIndex(i + ii, j + jj, k + kk);
            }
          }
        }

        for (const auto& cellTet : kCellTets) {
          Eigen::Vector4i tet(
              cellCorner[cellTet[0]],
              cellCorner[cellTet[1]],
              cellCorner[cellTet[2]],
              cellCorner[cellTet[3]]);

          // Half of the Kuhn tetrahedra are negatively oriented. Swapping the
          // last two nodes flips the orientation without changing which region
          // the tetrahedron covers.
          const Eigen::Matrix3d shape = computeShapeMatrix(
              restPositions[tet[0]],
              restPositions[tet[1]],
              restPositions[tet[2]],
              restPositions[tet[3]]);
          if (shape.determinant() < 0.0) {
            std::swap(tet[2], tet[3]);
          }

          tets.push_back(tet);
        }
      }
    }
  }

  return TetMesh(std::move(restPositions), std::move(tets));
}

//==============================================================================
void TetMesh::computeRestQuantities()
{
  const auto numNodes = static_cast<int>(mRestPositions.size());

  mRestVolumes.clear();
  mRestVolumes.reserve(mTets.size());
  mInverseRestShapes.clear();
  mInverseRestShapes.reserve(mTets.size());
  mTotalRestVolume = 0.0;

  for (const auto& tet : mTets) {
    for (int corner = 0; corner < 4; ++corner) {
      if (tet[corner] < 0 || tet[corner] >= numNodes) {
        throw std::invalid_argument(
            "TetMesh: tetrahedron node index is out of range");
      }
    }

    const Eigen::Matrix3d shape = computeShapeMatrix(
        mRestPositions[tet[0]],
        mRestPositions[tet[1]],
        mRestPositions[tet[2]],
        mRestPositions[tet[3]]);

    const double restVolume = shape.determinant() / 6.0;
    if (!(restVolume > 0.0)) {
      throw std::invalid_argument(
          "TetMesh: tetrahedron is degenerate or inverted; node ordering must "
          "give a positive rest volume");
    }

    // A tetrahedron flattened almost to a plane still has a positive volume and
    // would pass the check above, but its inverse rest shape is then so
    // ill-conditioned that it amplifies any perturbation by the same factor.
    // That is harmless while only gravity acts, and ruinous as soon as
    // deformation gradients F = Ds * inverseRestShape are formed, so reject
    // slivers here instead of letting them reach the elastic work. The bound is
    // eleven orders below a regular tetrahedron's volume-to-edge ratio, so it
    // only catches genuinely collapsed elements.
    const double edgeScale = shape.colwise().norm().maxCoeff();
    if (!(restVolume
          > kMinVolumeToEdgeCubed * edgeScale * edgeScale * edgeScale)) {
      throw std::invalid_argument(
          "TetMesh: tetrahedron is a sliver; its rest volume is vanishing "
          "relative to its edge lengths, which makes the inverse rest shape "
          "numerically unusable");
    }

    mRestVolumes.push_back(restVolume);
    mInverseRestShapes.push_back(shape.inverse());
    mTotalRestVolume += restVolume;
  }
}

//==============================================================================
std::size_t TetMesh::getNumNodes() const
{
  return mRestPositions.size();
}

//==============================================================================
std::size_t TetMesh::getNumTets() const
{
  return mTets.size();
}

//==============================================================================
bool TetMesh::isEmpty() const
{
  return mTets.empty();
}

//==============================================================================
const Eigen::Vector3d& TetMesh::getRestPosition(std::size_t nodeIndex) const
{
  DART_ASSERT(nodeIndex < mRestPositions.size());
  return mRestPositions[nodeIndex];
}

//==============================================================================
Eigen::Vector4i TetMesh::getTet(std::size_t tetIndex) const
{
  DART_ASSERT(tetIndex < mTets.size());
  return mTets[tetIndex];
}

//==============================================================================
double TetMesh::getRestVolume(std::size_t tetIndex) const
{
  DART_ASSERT(tetIndex < mRestVolumes.size());
  return mRestVolumes[tetIndex];
}

//==============================================================================
const Eigen::Matrix3d& TetMesh::getInverseRestShape(std::size_t tetIndex) const
{
  DART_ASSERT(tetIndex < mInverseRestShapes.size());
  return mInverseRestShapes[tetIndex];
}

//==============================================================================
double TetMesh::getTotalRestVolume() const
{
  return mTotalRestVolume;
}

} // namespace fem
} // namespace dynamics
} // namespace dart
