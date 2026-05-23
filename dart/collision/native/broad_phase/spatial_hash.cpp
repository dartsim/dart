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

#include <dart/collision/native/broad_phase/spatial_hash.hpp>

#include <algorithm>

#include <cmath>

namespace dart::collision::native {

SpatialHashBroadPhase::SpatialHashBroadPhase(double cellSize)
  : cellSize_(cellSize), inverseCellSize_(1.0 / cellSize)
{
}

void SpatialHashBroadPhase::clear()
{
  objects_.clear();
  grid_.clear();
  unboundedIds_.clear();
  orderedIds_.clear();
}

void SpatialHashBroadPhase::add(std::size_t id, const Aabb& aabb)
{
  ObjectEntry entry;
  entry.aabb = aabb;

  if (isFinite(aabb)) {
    entry.cells = computeCells(aabb);
    objects_[id] = std::move(entry);
    addToGrid(id, objects_.at(id).cells);
  } else {
    objects_[id] = std::move(entry);
    unboundedIds_.insert(id);
  }

  rebuildOrderedIds();
}

void SpatialHashBroadPhase::update(std::size_t id, const Aabb& aabb)
{
  auto it = objects_.find(id);
  if (it == objects_.end()) {
    return;
  }

  auto& entry = it->second;
  const bool wasFinite = unboundedIds_.find(id) == unboundedIds_.end();
  const bool isNowFinite = isFinite(aabb);

  if (isNowFinite) {
    auto newCells = computeCells(aabb);
    if (!wasFinite) {
      unboundedIds_.erase(id);
      entry.cells = std::move(newCells);
      addToGrid(id, entry.cells);
    } else if (entry.cells != newCells) {
      removeFromGrid(id, entry.cells);
      entry.cells = std::move(newCells);
      addToGrid(id, entry.cells);
    }
  } else if (wasFinite) {
    removeFromGrid(id, entry.cells);
    entry.cells.clear();
    unboundedIds_.insert(id);
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
  unboundedIds_.erase(id);
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

  for (std::size_t id1 : orderedIds_) {
    if (unboundedIds_.find(id1) == unboundedIds_.end()) {
      continue;
    }

    const auto& aabb1 = objects_.at(id1).aabb;
    for (std::size_t id2 : orderedIds_) {
      if (id1 == id2) {
        continue;
      }

      const auto& aabb2 = objects_.at(id2).aabb;
      if (aabb1.overlaps(aabb2)) {
        uniquePairs.emplace(std::min(id1, id2), std::max(id1, id2));
      }
    }
  }

  return std::vector<BroadPhasePair>(uniquePairs.begin(), uniquePairs.end());
}

std::vector<std::size_t> SpatialHashBroadPhase::queryOverlapping(
    const Aabb& aabb) const
{
  std::set<std::size_t> candidates;

  if (isFinite(aabb)) {
    auto cells = computeCells(aabb);
    for (const auto& cell : cells) {
      auto it = grid_.find(cell);
      if (it != grid_.end()) {
        for (std::size_t id : it->second) {
          candidates.insert(id);
        }
      }
    }

    candidates.insert(unboundedIds_.begin(), unboundedIds_.end());
  } else {
    candidates.insert(orderedIds_.begin(), orderedIds_.end());
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

void SpatialHashBroadPhase::buildDebugSnapshot(
    BroadPhaseDebugSnapshot& out) const
{
  out.clear();
  out.candidatePairs = queryPairs();
  out.numObjects = size();
  out.spatialHashCellSize = cellSize_;
  out.hasSpatialHashCells = true;
  out.cells.reserve(grid_.size());

  for (const auto& [coord, objectIds] : grid_) {
    BroadPhaseDebugCell cell;
    cell.coordinates = coord;
    cell.hash = CellHash{}(coord);
    cell.aabb = Aabb(
        Eigen::Vector3d(
            static_cast<double>(coord[0]) * cellSize_,
            static_cast<double>(coord[1]) * cellSize_,
            static_cast<double>(coord[2]) * cellSize_),
        Eigen::Vector3d(
            static_cast<double>(coord[0] + 1) * cellSize_,
            static_cast<double>(coord[1] + 1) * cellSize_,
            static_cast<double>(coord[2] + 1) * cellSize_));
    cell.objectIds.assign(objectIds.begin(), objectIds.end());
    std::sort(cell.objectIds.begin(), cell.objectIds.end());
    out.cells.push_back(std::move(cell));
  }

  std::sort(
      out.cells.begin(),
      out.cells.end(),
      [](const BroadPhaseDebugCell& lhs, const BroadPhaseDebugCell& rhs) {
        return lhs.coordinates < rhs.coordinates;
      });
}

bool SpatialHashBroadPhase::isFinite(const Aabb& aabb)
{
  return aabb.min.allFinite() && aabb.max.allFinite();
}

SpatialHashBroadPhase::CellCoord SpatialHashBroadPhase::computeCell(
    const Eigen::Vector3d& point) const
{
  return {
      static_cast<int>(std::floor(point.x() * inverseCellSize_)),
      static_cast<int>(std::floor(point.y() * inverseCellSize_)),
      static_cast<int>(std::floor(point.z() * inverseCellSize_))};
}

std::vector<SpatialHashBroadPhase::CellCoord>
SpatialHashBroadPhase::computeCells(const Aabb& aabb) const
{
  CellCoord minCell = computeCell(aabb.min);
  CellCoord maxCell = computeCell(aabb.max);

  std::vector<CellCoord> cells;
  cells.reserve(
      static_cast<std::size_t>(
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
    std::size_t id, const std::vector<CellCoord>& cells)
{
  for (const auto& cell : cells) {
    grid_[cell].insert(id);
  }
}

void SpatialHashBroadPhase::removeFromGrid(
    std::size_t id, const std::vector<CellCoord>& cells)
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

} // namespace dart::collision::native
