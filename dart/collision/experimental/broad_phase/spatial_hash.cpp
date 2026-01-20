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

#include <dart/collision/experimental/broad_phase/spatial_hash.hpp>

#include <algorithm>
#include <cmath>

namespace dart::collision::experimental {

SpatialHashBroadPhase::SpatialHashBroadPhase(double cellSize)
  : cellSize_(cellSize), inverseCellSize_(1.0 / cellSize)
{
}

void SpatialHashBroadPhase::clear()
{
  objects_.clear();
  grid_.clear();
  orderedIds_.clear();
}

void SpatialHashBroadPhase::add(std::size_t id, const Aabb& aabb)
{
  auto cells = computeCells(aabb);

  ObjectEntry entry;
  entry.aabb = aabb;
  entry.cells = cells;

  objects_[id] = std::move(entry);
  addToGrid(id, cells);
  rebuildOrderedIds();
}

void SpatialHashBroadPhase::update(std::size_t id, const Aabb& aabb)
{
  auto it = objects_.find(id);
  if (it == objects_.end()) {
    return;
  }

  auto newCells = computeCells(aabb);
  auto& entry = it->second;

  if (entry.cells != newCells) {
    removeFromGrid(id, entry.cells);
    entry.cells = newCells;
    addToGrid(id, newCells);
  }

  entry.aabb = aabb;
}

void SpatialHashBroadPhase::remove(std::size_t id)
{
  auto it = objects_.find(id);
  if (it == objects_.end()) {
    return;
  }

  removeFromGrid(id, it->second.cells);
  objects_.erase(it);
  rebuildOrderedIds();
}

std::vector<BroadPhasePair> SpatialHashBroadPhase::queryPairs() const
{
  std::set<BroadPhasePair> uniquePairs;

  for (const auto& [cellCoord, objectIds] : grid_) {
    if (objectIds.size() < 2) {
      continue;
    }

    std::vector<std::size_t> ids(objectIds.begin(), objectIds.end());
    std::sort(ids.begin(), ids.end());

    for (std::size_t i = 0; i < ids.size(); ++i) {
      for (std::size_t j = i + 1; j < ids.size(); ++j) {
        const std::size_t id1 = ids[i];
        const std::size_t id2 = ids[j];

        const auto& aabb1 = objects_.at(id1).aabb;
        const auto& aabb2 = objects_.at(id2).aabb;

        if (aabb1.overlaps(aabb2)) {
          uniquePairs.emplace(id1, id2);
        }
      }
    }
  }

  return std::vector<BroadPhasePair>(uniquePairs.begin(), uniquePairs.end());
}

std::vector<std::size_t> SpatialHashBroadPhase::queryOverlapping(
    const Aabb& aabb) const
{
  auto cells = computeCells(aabb);
  std::set<std::size_t> candidates;

  for (const auto& cell : cells) {
    auto it = grid_.find(cell);
    if (it != grid_.end()) {
      for (std::size_t id : it->second) {
        candidates.insert(id);
      }
    }
  }

  std::vector<std::size_t> result;
  for (std::size_t id : candidates) {
    if (objects_.at(id).aabb.overlaps(aabb)) {
      result.push_back(id);
    }
  }

  return result;
}

std::size_t SpatialHashBroadPhase::size() const
{
  return objects_.size();
}

void SpatialHashBroadPhase::setCellSize(double cellSize)
{
  cellSize_ = cellSize;
  inverseCellSize_ = 1.0 / cellSize;

  std::vector<std::pair<std::size_t, Aabb>> backup;
  backup.reserve(objects_.size());
  for (const auto& [id, entry] : objects_) {
    backup.emplace_back(id, entry.aabb);
  }

  clear();

  for (const auto& [id, aabb] : backup) {
    add(id, aabb);
  }
}

double SpatialHashBroadPhase::averageObjectsPerCell() const
{
  if (grid_.empty()) {
    return 0.0;
  }

  std::size_t totalObjects = 0;
  for (const auto& [cell, ids] : grid_) {
    totalObjects += ids.size();
  }

  return static_cast<double>(totalObjects) / static_cast<double>(grid_.size());
}

SpatialHashBroadPhase::CellCoord SpatialHashBroadPhase::computeCell(
    const Eigen::Vector3d& point) const
{
  return {
      static_cast<int>(std::floor(point.x() * inverseCellSize_)),
      static_cast<int>(std::floor(point.y() * inverseCellSize_)),
      static_cast<int>(std::floor(point.z() * inverseCellSize_))};
}

std::vector<SpatialHashBroadPhase::CellCoord> SpatialHashBroadPhase::computeCells(
    const Aabb& aabb) const
{
  CellCoord minCell = computeCell(aabb.min);
  CellCoord maxCell = computeCell(aabb.max);

  std::vector<CellCoord> cells;
  cells.reserve(static_cast<std::size_t>(
      (maxCell[0] - minCell[0] + 1) * (maxCell[1] - minCell[1] + 1)
      * (maxCell[2] - minCell[2] + 1)));

  for (int x = minCell[0]; x <= maxCell[0]; ++x) {
    for (int y = minCell[1]; y <= maxCell[1]; ++y) {
      for (int z = minCell[2]; z <= maxCell[2]; ++z) {
        cells.push_back({x, y, z});
      }
    }
  }

  return cells;
}

void SpatialHashBroadPhase::addToGrid(
    std::size_t id,
    const std::vector<CellCoord>& cells)
{
  for (const auto& cell : cells) {
    grid_[cell].insert(id);
  }
}

void SpatialHashBroadPhase::removeFromGrid(
    std::size_t id,
    const std::vector<CellCoord>& cells)
{
  for (const auto& cell : cells) {
    auto it = grid_.find(cell);
    if (it != grid_.end()) {
      it->second.erase(id);
      if (it->second.empty()) {
        grid_.erase(it);
      }
    }
  }
}

void SpatialHashBroadPhase::rebuildOrderedIds()
{
  orderedIds_.clear();
  orderedIds_.reserve(objects_.size());

  for (const auto& [id, entry] : objects_) {
    orderedIds_.push_back(id);
  }

  std::sort(orderedIds_.begin(), orderedIds_.end());
}

}  // namespace dart::collision::experimental
