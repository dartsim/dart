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

#pragma once

#include <dart/collision/experimental/broad_phase/broad_phase.hpp>

#include <array>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace dart::collision::experimental {

/// Spatial Hash broad-phase for O(1) average query time with uniform
/// object distributions.
///
/// Objects are hashed into a 3D grid based on their AABB. Each object is
/// stored in all cells it overlaps. Collision pairs are found by checking
/// objects that share at least one cell.
///
/// Best for:
/// - Uniform object size distributions
/// - Dense scenes where objects are clustered
/// - Scenes where average object size is known beforehand
///
/// Trade-offs:
/// - Cell size must be tuned for best performance
/// - Large objects spanning many cells increase memory and query cost
/// - Not ideal for very sparse scenes or objects with highly varying sizes
class DART_COLLISION_EXPERIMENTAL_API SpatialHashBroadPhase : public BroadPhase
{
public:
  /// Default cell size (1.0 meter)
  static constexpr double kDefaultCellSize = 1.0;

  /// Constructor with configurable cell size
  /// @param cellSize Size of each cell in the spatial hash grid
  explicit SpatialHashBroadPhase(double cellSize = kDefaultCellSize);

  void clear() override;
  void add(std::size_t id, const Aabb& aabb) override;
  void update(std::size_t id, const Aabb& aabb) override;
  void remove(std::size_t id) override;

  [[nodiscard]] std::vector<BroadPhasePair> queryPairs() const override;
  [[nodiscard]] std::vector<std::size_t> queryOverlapping(
      const Aabb& aabb) const override;
  [[nodiscard]] std::size_t size() const override;

  using BroadPhase::queryPairs;
  using BroadPhase::build;
  using BroadPhase::updateRange;

  /// Get the cell size
  [[nodiscard]] double getCellSize() const
  {
    return cellSize_;
  }

  /// Set the cell size (requires rebuild)
  /// @note This clears and rebuilds the entire structure
  void setCellSize(double cellSize);

  /// Get the number of non-empty cells
  [[nodiscard]] std::size_t numCells() const
  {
    return grid_.size();
  }

  /// Get the average objects per cell (for tuning)
  [[nodiscard]] double averageObjectsPerCell() const;

private:
  /// Cell coordinate type (signed to handle negative positions)
  using CellCoord = std::array<int, 3>;

  /// Hash functor for CellCoord
  struct CellHash
  {
    std::size_t operator()(const CellCoord& coord) const noexcept
    {
      // Prime-based spatial hash for good distribution
      constexpr std::size_t p1 = 73856093;
      constexpr std::size_t p2 = 19349663;
      constexpr std::size_t p3 = 83492791;

      return static_cast<std::size_t>(coord[0]) * p1
             ^ static_cast<std::size_t>(coord[1]) * p2
             ^ static_cast<std::size_t>(coord[2]) * p3;
    }
  };

  /// Object entry stored in the main map
  struct ObjectEntry
  {
    Aabb aabb;
    std::vector<CellCoord> cells; // Cells this object occupies
  };

  /// Compute the cell coordinate for a point
  [[nodiscard]] CellCoord computeCell(const Eigen::Vector3d& point) const;

  /// Compute all cells an AABB overlaps
  [[nodiscard]] std::vector<CellCoord> computeCells(const Aabb& aabb) const;

  /// Add object to grid cells
  void addToGrid(std::size_t id, const std::vector<CellCoord>& cells);

  /// Remove object from grid cells
  void removeFromGrid(std::size_t id, const std::vector<CellCoord>& cells);

  double cellSize_;
  double inverseCellSize_; // Cached 1.0 / cellSize for performance

  /// Main object storage: id -> (aabb, cells)
  std::unordered_map<std::size_t, ObjectEntry> objects_;

  /// Spatial grid: cell -> set of object ids
  std::unordered_map<CellCoord, std::unordered_set<std::size_t>, CellHash>
      grid_;

  /// Ordered object IDs for deterministic iteration
  std::vector<std::size_t> orderedIds_;

  /// Rebuild ordered IDs after add/remove
  void rebuildOrderedIds();
};

} // namespace dart::collision::experimental
