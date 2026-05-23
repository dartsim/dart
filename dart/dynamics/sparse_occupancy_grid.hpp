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

#ifndef DART_DYNAMICS_SPARSEOCCUPANCYGRID_HPP_
#define DART_DYNAMICS_SPARSEOCCUPANCYGRID_HPP_

#include <dart/export.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <functional>
#include <span>
#include <unordered_map>
#include <vector>

#include <cstdint>

namespace dart {
namespace dynamics {

/// Sparse probabilistic occupancy grid keyed by integer cell coordinates.
class DART_API SparseOccupancyGrid
{
public:
  /// Integer key for a grid cell.
  struct CellKey
  {
    std::int64_t x = 0;
    std::int64_t y = 0;
    std::int64_t z = 0;

    bool operator==(const CellKey& other) const = default;
  };

  /// Hash for cell keys.
  struct CellKeyHash
  {
    std::size_t operator()(const CellKey& key) const;
  };

  /// Occupied cell extracted for collision, rendering, or serialization.
  struct OccupiedCell
  {
    CellKey key;
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    double size = 0.0;
    double occupancy = 0.0;
  };

  /// Creates an empty grid with the given positive resolution.
  explicit SparseOccupancyGrid(double resolution = 0.01);

  /// Returns the cell edge length.
  double getResolution() const;

  /// Sets the probability threshold used by isOccupied().
  void setOccupancyThreshold(double probability);

  /// Returns the probability threshold used by isOccupied().
  double getOccupancyThreshold() const;

  /// Sets the probability used for positive occupancy updates.
  void setHitProbability(double probability);

  /// Returns the probability used for positive occupancy updates.
  double getHitProbability() const;

  /// Sets the probability used for free-space updates.
  void setMissProbability(double probability);

  /// Returns the probability used for free-space updates.
  double getMissProbability() const;

  /// Removes all known cells.
  void clear();

  /// Returns the number of known cells, including free cells.
  std::size_t getNumCells() const;

  /// Returns the number of cells whose occupancy is at or above the threshold.
  std::size_t getNumOccupiedCells() const;

  /// Converts a world-space point to a grid cell key.
  CellKey worldToCell(const Eigen::Vector3d& point) const;

  /// Returns the world-space center of a cell.
  Eigen::Vector3d getCellCenter(const CellKey& key) const;

  /// Returns the occupancy probability at a world-space point.
  double getOccupancy(const Eigen::Vector3d& point) const;

  /// Returns the occupancy probability of a cell.
  double getOccupancy(const CellKey& key) const;

  /// Sets the occupancy probability of a cell.
  void setOccupancy(const CellKey& key, double probability);

  /// Applies one occupied/free update at a world-space point.
  void updateOccupancy(const Eigen::Vector3d& point, bool occupied = true);

  /// Applies one occupied/free update to a cell.
  void updateOccupancy(const CellKey& key, bool occupied = true);

  /// Inserts a ray: traversed cells become more free, endpoint more occupied.
  void insertRay(const Eigen::Vector3d& from, const Eigen::Vector3d& to);

  /// Inserts a point cloud relative to an optional transform.
  /// @param numThreads Number of worker threads used for ray classification.
  /// Use 1 for deterministic single-threaded insertion, or 0 to use hardware
  /// concurrency.
  void insertPointCloud(
      std::span<const Eigen::Vector3d> pointCloud,
      const Eigen::Vector3d& sensorOrigin = Eigen::Vector3d::Zero(),
      const Eigen::Isometry3d& relativeTo = Eigen::Isometry3d::Identity(),
      std::size_t numThreads = 1);

  /// Returns the grid cells traversed by a ray, including the endpoint cell.
  std::vector<CellKey> traceRay(
      const Eigen::Vector3d& from, const Eigen::Vector3d& to) const;

  /// Returns true if a known cell is occupied.
  bool isOccupied(const CellKey& key) const;

  /// Returns occupied cells with centers and probabilities.
  std::vector<OccupiedCell> getOccupiedCells() const;

private:
  static double probabilityToLogOdds(double probability);
  static double logOddsToProbability(double logOdds);
  static void validateProbability(double probability, const char* name);
  void markOccupiedCellsDirty();
  void rebuildOccupiedCellsCache() const;

  double mResolution;
  double mOccupancyThresholdLogOdds;
  double mHitLogOdds;
  double mMissLogOdds;
  double mMinLogOdds;
  double mMaxLogOdds;
  std::unordered_map<CellKey, double, CellKeyHash> mLogOdds;
  mutable bool mOccupiedCellsDirty;
  mutable std::vector<OccupiedCell> mOccupiedCellsCache;
};

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_SPARSEOCCUPANCYGRID_HPP_
