/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include "dart7/common/profiling.hpp"

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <limits>
#include <unordered_map>
#include <vector>

namespace dart7::common {

void ScopedTimer::reportDuration(std::string_view name, long long microseconds)
{
  ProfileStats::record(name, microseconds);
}

void ProfileStats::record(std::string_view name, long long microseconds)
{
  auto& entries = entriesInternal();
  auto& entry = entries[std::string(name)];

  if (entry.name.empty()) {
    entry.name = name;
  }

  entry.totalMicroseconds += microseconds;
  entry.count++;
  entry.minMicroseconds = std::min(entry.minMicroseconds, microseconds);
  entry.maxMicroseconds = std::max(entry.maxMicroseconds, microseconds);
}

const std::unordered_map<std::string, ProfileStats::Entry>&
ProfileStats::entries()
{
  return entriesInternal();
}

void ProfileStats::reset()
{
  entriesInternal().clear();
}

void ProfileStats::printSummary()
{
  const auto& entries = entriesInternal();

  if (entries.empty()) {
    std::cout << "No profiling data collected.\n";
    return;
  }

  // Convert to vector for sorting
  std::vector<Entry> sortedEntries;
  sortedEntries.reserve(entries.size());
  for (const auto& [_, entry] : entries) {
    sortedEntries.push_back(entry);
  }

  // Sort by total time (descending)
  std::sort(
      sortedEntries.begin(),
      sortedEntries.end(),
      [](const Entry& a, const Entry& b) {
        return a.totalMicroseconds > b.totalMicroseconds;
      });

  // Print header
  std::cout << "\n=== Profiling Summary ===\n\n";
  std::cout << std::left << std::setw(40) << "Name" << std::right
            << std::setw(12) << "Calls" << std::setw(15) << "Total (ms)"
            << std::setw(15) << "Avg (ms)" << std::setw(15) << "Min (ms)"
            << std::setw(15) << "Max (ms)" << '\n';
  std::cout << std::string(112, '-') << '\n';

  // Print entries
  for (const auto& entry : sortedEntries) {
    std::cout << std::left << std::setw(40) << entry.name << std::right
              << std::setw(12) << entry.count << std::setw(15) << std::fixed
              << std::setprecision(3) << (entry.totalMicroseconds / 1000.0)
              << std::setw(15) << (entry.averageMicroseconds() / 1000.0)
              << std::setw(15) << (entry.minMicroseconds / 1000.0)
              << std::setw(15) << (entry.maxMicroseconds / 1000.0) << '\n';
  }

  std::cout << '\n';
}

std::unordered_map<std::string, ProfileStats::Entry>&
ProfileStats::entriesInternal()
{
  static std::unordered_map<std::string, Entry> s_entries;
  return s_entries;
}

} // namespace dart7::common
