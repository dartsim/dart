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

#include "dart/dynamics/sparse_occupancy_grid.hpp"

#include "dart/common/exception.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <unordered_set>

#include <cmath>

namespace dart {
namespace dynamics {

namespace {

//==============================================================================
std::int64_t floorToInteger(double value)
{
  return static_cast<std::int64_t>(std::floor(value));
}

//==============================================================================
int sign(double value)
{
  if (value > 0.0) {
    return 1;
  }

  if (value < 0.0) {
    return -1;
  }

  return 0;
}

//==============================================================================
std::size_t hashCombine(std::size_t seed, std::size_t value)
{
  return seed ^ (value + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2));
}

//==============================================================================
void validateResolution(double resolution)
{
  if (!(resolution > 0.0) || !std::isfinite(resolution)) {
    DART_THROW_T(
        ::dart::common::InvalidArgumentException,
        "SparseOccupancyGrid resolution must be positive and finite.");
  }
}

//==============================================================================
double clamp(double value, double lower, double upper)
{
  return std::min(std::max(value, lower), upper);
}

} // namespace

//==============================================================================
std::size_t SparseOccupancyGrid::CellKeyHash::operator()(
    const CellKey& key) const
{
  std::size_t seed = 0;
  seed = hashCombine(seed, std::hash<std::int64_t>{}(key.x));
  seed = hashCombine(seed, std::hash<std::int64_t>{}(key.y));
  return hashCombine(seed, std::hash<std::int64_t>{}(key.z));
}

//==============================================================================
SparseOccupancyGrid::SparseOccupancyGrid(double resolution)
  : mResolution(resolution),
    mOccupancyThresholdLogOdds(probabilityToLogOdds(0.5)),
    mHitLogOdds(probabilityToLogOdds(0.7)),
    mMissLogOdds(probabilityToLogOdds(0.4)),
    mMinLogOdds(probabilityToLogOdds(0.1192)),
    mMaxLogOdds(probabilityToLogOdds(0.971))
{
  validateResolution(resolution);
}

//==============================================================================
double SparseOccupancyGrid::getResolution() const
{
  return mResolution;
}

//==============================================================================
void SparseOccupancyGrid::setOccupancyThreshold(double probability)
{
  validateProbability(probability, "occupancy threshold");
  mOccupancyThresholdLogOdds = probabilityToLogOdds(probability);
}

//==============================================================================
double SparseOccupancyGrid::getOccupancyThreshold() const
{
  return logOddsToProbability(mOccupancyThresholdLogOdds);
}

//==============================================================================
void SparseOccupancyGrid::setHitProbability(double probability)
{
  validateProbability(probability, "hit probability");
  mHitLogOdds = probabilityToLogOdds(probability);
}

//==============================================================================
double SparseOccupancyGrid::getHitProbability() const
{
  return logOddsToProbability(mHitLogOdds);
}

//==============================================================================
void SparseOccupancyGrid::setMissProbability(double probability)
{
  validateProbability(probability, "miss probability");
  mMissLogOdds = probabilityToLogOdds(probability);
}

//==============================================================================
double SparseOccupancyGrid::getMissProbability() const
{
  return logOddsToProbability(mMissLogOdds);
}

//==============================================================================
void SparseOccupancyGrid::clear()
{
  mLogOdds.clear();
}

//==============================================================================
std::size_t SparseOccupancyGrid::getNumCells() const
{
  return mLogOdds.size();
}

//==============================================================================
std::size_t SparseOccupancyGrid::getNumOccupiedCells() const
{
  std::size_t count = 0;
  for (const auto& item : mLogOdds) {
    if (item.second >= mOccupancyThresholdLogOdds) {
      ++count;
    }
  }

  return count;
}

//==============================================================================
SparseOccupancyGrid::CellKey SparseOccupancyGrid::worldToCell(
    const Eigen::Vector3d& point) const
{
  return CellKey{
      floorToInteger(point.x() / mResolution),
      floorToInteger(point.y() / mResolution),
      floorToInteger(point.z() / mResolution)};
}

//==============================================================================
Eigen::Vector3d SparseOccupancyGrid::getCellCenter(const CellKey& key) const
{
  return Eigen::Vector3d(
      (static_cast<double>(key.x) + 0.5) * mResolution,
      (static_cast<double>(key.y) + 0.5) * mResolution,
      (static_cast<double>(key.z) + 0.5) * mResolution);
}

//==============================================================================
double SparseOccupancyGrid::getOccupancy(const Eigen::Vector3d& point) const
{
  return getOccupancy(worldToCell(point));
}

//==============================================================================
double SparseOccupancyGrid::getOccupancy(const CellKey& key) const
{
  const auto it = mLogOdds.find(key);
  if (it == mLogOdds.end()) {
    return 0.0;
  }

  return logOddsToProbability(it->second);
}

//==============================================================================
void SparseOccupancyGrid::setOccupancy(const CellKey& key, double probability)
{
  validateProbability(probability, "occupancy probability");
  mLogOdds[key] = probabilityToLogOdds(probability);
}

//==============================================================================
void SparseOccupancyGrid::updateOccupancy(
    const Eigen::Vector3d& point, bool occupied)
{
  updateOccupancy(worldToCell(point), occupied);
}

//==============================================================================
void SparseOccupancyGrid::updateOccupancy(const CellKey& key, bool occupied)
{
  auto& logOdds = mLogOdds[key];
  logOdds += occupied ? mHitLogOdds : mMissLogOdds;
  logOdds = clamp(logOdds, mMinLogOdds, mMaxLogOdds);
}

//==============================================================================
void SparseOccupancyGrid::insertRay(
    const Eigen::Vector3d& from, const Eigen::Vector3d& to)
{
  CellKey key = worldToCell(from);
  const CellKey end = worldToCell(to);

  if (key == end) {
    updateOccupancy(key, true);
    return;
  }

  const Eigen::Vector3d direction = to - from;
  const std::array<int, 3> step{
      sign(direction.x()), sign(direction.y()), sign(direction.z())};

  std::array<double, 3> tMax;
  std::array<double, 3> tDelta;
  const auto inf = std::numeric_limits<double>::infinity();

  const std::array<std::int64_t, 3> startKey{key.x, key.y, key.z};
  for (auto axis = 0u; axis < 3u; ++axis) {
    if (step[axis] == 0) {
      tMax[axis] = inf;
      tDelta[axis] = inf;
      continue;
    }

    const double boundary
        = (static_cast<double>(startKey[axis] + (step[axis] > 0 ? 1 : 0))
           * mResolution);
    tMax[axis] = (boundary - from[static_cast<Eigen::Index>(axis)])
                 / direction[static_cast<Eigen::Index>(axis)];
    tDelta[axis]
        = mResolution / std::abs(direction[static_cast<Eigen::Index>(axis)]);
  }

  const auto maxSteps = static_cast<std::size_t>(
      std::abs(end.x - key.x) + std::abs(end.y - key.y)
      + std::abs(end.z - key.z) + 3);

  for (std::size_t steps = 0; steps < maxSteps && !(key == end); ++steps) {
    updateOccupancy(key, false);

    const double nextT = std::min({tMax[0], tMax[1], tMax[2]});
    constexpr double tolerance = 1e-12;

    if (tMax[0] <= nextT + tolerance) {
      key.x += step[0];
      tMax[0] += tDelta[0];
    }
    if (tMax[1] <= nextT + tolerance) {
      key.y += step[1];
      tMax[1] += tDelta[1];
    }
    if (tMax[2] <= nextT + tolerance) {
      key.z += step[2];
      tMax[2] += tDelta[2];
    }
  }

  updateOccupancy(end, true);
}

//==============================================================================
void SparseOccupancyGrid::insertPointCloud(
    std::span<const Eigen::Vector3d> pointCloud,
    const Eigen::Vector3d& sensorOrigin,
    const Eigen::Isometry3d& relativeTo)
{
  const Eigen::Vector3d origin = relativeTo * sensorOrigin;
  std::unordered_set<CellKey, CellKeyHash> freeCells;
  std::unordered_set<CellKey, CellKeyHash> occupiedCells;
  freeCells.reserve(pointCloud.size() * 16);
  occupiedCells.reserve(pointCloud.size());

  for (const auto& point : pointCloud) {
    CellKey key = worldToCell(origin);
    const Eigen::Vector3d endpoint = relativeTo * point;
    const CellKey end = worldToCell(endpoint);
    occupiedCells.insert(end);

    if (key == end) {
      continue;
    }

    const Eigen::Vector3d direction = endpoint - origin;
    const std::array<int, 3> step{
        sign(direction.x()), sign(direction.y()), sign(direction.z())};

    std::array<double, 3> tMax;
    std::array<double, 3> tDelta;
    const auto inf = std::numeric_limits<double>::infinity();

    const std::array<std::int64_t, 3> startKey{key.x, key.y, key.z};
    for (auto axis = 0u; axis < 3u; ++axis) {
      if (step[axis] == 0) {
        tMax[axis] = inf;
        tDelta[axis] = inf;
        continue;
      }

      const double boundary
          = (static_cast<double>(startKey[axis] + (step[axis] > 0 ? 1 : 0))
             * mResolution);
      tMax[axis] = (boundary - origin[static_cast<Eigen::Index>(axis)])
                   / direction[static_cast<Eigen::Index>(axis)];
      tDelta[axis]
          = mResolution / std::abs(direction[static_cast<Eigen::Index>(axis)]);
    }

    const auto maxSteps = static_cast<std::size_t>(
        std::abs(end.x - key.x) + std::abs(end.y - key.y)
        + std::abs(end.z - key.z) + 3);

    for (std::size_t steps = 0; steps < maxSteps && !(key == end); ++steps) {
      freeCells.insert(key);

      const double nextT = std::min({tMax[0], tMax[1], tMax[2]});
      constexpr double tolerance = 1e-12;

      if (tMax[0] <= nextT + tolerance) {
        key.x += step[0];
        tMax[0] += tDelta[0];
      }
      if (tMax[1] <= nextT + tolerance) {
        key.y += step[1];
        tMax[1] += tDelta[1];
      }
      if (tMax[2] <= nextT + tolerance) {
        key.z += step[2];
        tMax[2] += tDelta[2];
      }
    }
  }

  for (const auto& key : occupiedCells) {
    freeCells.erase(key);
  }

  for (const auto& key : freeCells) {
    updateOccupancy(key, false);
  }

  for (const auto& key : occupiedCells) {
    updateOccupancy(key, true);
  }
}

//==============================================================================
std::vector<SparseOccupancyGrid::CellKey> SparseOccupancyGrid::traceRay(
    const Eigen::Vector3d& from, const Eigen::Vector3d& to) const
{
  CellKey key = worldToCell(from);
  const CellKey end = worldToCell(to);

  std::vector<CellKey> cells;
  cells.push_back(key);

  if (key == end) {
    return cells;
  }

  const Eigen::Vector3d direction = to - from;
  const std::array<int, 3> step{
      sign(direction.x()), sign(direction.y()), sign(direction.z())};

  std::array<double, 3> tMax;
  std::array<double, 3> tDelta;
  const auto inf = std::numeric_limits<double>::infinity();

  const std::array<std::int64_t, 3> startKey{key.x, key.y, key.z};
  for (auto axis = 0u; axis < 3u; ++axis) {
    if (step[axis] == 0) {
      tMax[axis] = inf;
      tDelta[axis] = inf;
      continue;
    }

    const double boundary
        = (static_cast<double>(startKey[axis] + (step[axis] > 0 ? 1 : 0))
           * mResolution);
    tMax[axis] = (boundary - from[static_cast<Eigen::Index>(axis)])
                 / direction[static_cast<Eigen::Index>(axis)];
    tDelta[axis]
        = mResolution / std::abs(direction[static_cast<Eigen::Index>(axis)]);
  }

  const auto maxSteps = static_cast<std::size_t>(
      std::abs(end.x - key.x) + std::abs(end.y - key.y)
      + std::abs(end.z - key.z) + 3);

  for (std::size_t steps = 0; steps < maxSteps && !(key == end); ++steps) {
    const double nextT = std::min({tMax[0], tMax[1], tMax[2]});
    constexpr double tolerance = 1e-12;

    if (tMax[0] <= nextT + tolerance) {
      key.x += step[0];
      tMax[0] += tDelta[0];
    }
    if (tMax[1] <= nextT + tolerance) {
      key.y += step[1];
      tMax[1] += tDelta[1];
    }
    if (tMax[2] <= nextT + tolerance) {
      key.z += step[2];
      tMax[2] += tDelta[2];
    }

    cells.push_back(key);
  }

  if (!(cells.back() == end)) {
    cells.push_back(end);
  }

  return cells;
}

//==============================================================================
bool SparseOccupancyGrid::isOccupied(const CellKey& key) const
{
  const auto it = mLogOdds.find(key);
  return it != mLogOdds.end() && it->second >= mOccupancyThresholdLogOdds;
}

//==============================================================================
std::vector<SparseOccupancyGrid::OccupiedCell>
SparseOccupancyGrid::getOccupiedCells() const
{
  std::vector<OccupiedCell> cells;
  cells.reserve(getNumOccupiedCells());

  for (const auto& [key, logOdds] : mLogOdds) {
    if (logOdds < mOccupancyThresholdLogOdds) {
      continue;
    }

    cells.push_back(
        OccupiedCell{
            key,
            getCellCenter(key),
            mResolution,
            logOddsToProbability(logOdds)});
  }

  return cells;
}

//==============================================================================
double SparseOccupancyGrid::probabilityToLogOdds(double probability)
{
  return std::log(probability / (1.0 - probability));
}

//==============================================================================
double SparseOccupancyGrid::logOddsToProbability(double logOdds)
{
  return 1.0 - (1.0 / (1.0 + std::exp(logOdds)));
}

//==============================================================================
void SparseOccupancyGrid::validateProbability(
    double probability, const char* name)
{
  if (!(probability > 0.0 && probability < 1.0)
      || !std::isfinite(probability)) {
    DART_THROW_T(
        ::dart::common::InvalidArgumentException,
        "SparseOccupancyGrid {} must be finite and between 0 and 1.",
        name);
  }
}

} // namespace dynamics
} // namespace dart
