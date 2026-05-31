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

#ifndef DART_GUI_DETAIL_APPLICATION_HPP_
#define DART_GUI_DETAIL_APPLICATION_HPP_

#include <dart/gui/application.hpp>

#include <algorithm>
#include <string>
#include <string_view>
#include <vector>

#include <cctype>
#include <cstddef>

namespace dart::gui {
struct ApplicationOptions;
} // namespace dart::gui

namespace dart::gui::detail {

struct DemoCategoryGroup
{
  std::string category;
  std::vector<std::size_t> sceneIndices;
};

inline std::string normalizedDemoSearchText(std::string_view value)
{
  std::string normalized;
  normalized.reserve(value.size());
  for (const unsigned char ch : value) {
    normalized.push_back(static_cast<char>(std::tolower(ch)));
  }
  return normalized;
}

inline bool demoCatalogContainsSearchText(
    std::string_view haystack, const std::string& needle)
{
  if (needle.empty()) {
    return true;
  }
  return normalizedDemoSearchText(haystack).find(needle) != std::string::npos;
}

inline bool demoSceneMatchesSearch(
    const dart::gui::DemoSceneEntry& scene, const std::string& searchText)
{
  return demoCatalogContainsSearchText(scene.id, searchText)
         || demoCatalogContainsSearchText(scene.title, searchText)
         || demoCatalogContainsSearchText(scene.category, searchText)
         || demoCatalogContainsSearchText(scene.summary, searchText);
}

inline bool demoSceneMatchesExperimentalFocus(
    const dart::gui::DemoSceneEntry& scene)
{
  const std::string category = normalizedDemoSearchText(scene.category);
  return category.find("(sx)") != std::string::npos
         || category.find("experimental") != std::string::npos;
}

inline bool demoSceneVisibleInNavigator(
    const dart::gui::DemoSceneEntry& scene,
    const std::string& searchText,
    bool experimentalFocus)
{
  return (!experimentalFocus || demoSceneMatchesExperimentalFocus(scene))
         && demoSceneMatchesSearch(scene, searchText);
}

inline std::vector<DemoCategoryGroup> groupDemoScenesByCategory(
    const std::vector<dart::gui::DemoSceneEntry>& scenes)
{
  std::vector<DemoCategoryGroup> groups;
  for (std::size_t i = 0; i < scenes.size(); ++i) {
    const auto& scene = scenes[i];
    auto group = std::find_if(
        groups.begin(), groups.end(), [&scene](const DemoCategoryGroup& entry) {
          return entry.category == scene.category;
        });
    if (group == groups.end()) {
      groups.push_back({scene.category, {i}});
      continue;
    }
    group->sceneIndices.push_back(i);
  }
  return groups;
}

int runGuiBackendApplication(int argc, char* argv[]);

int runGuiBackendApplication(
    int argc, char* argv[], const dart::gui::ApplicationOptions& options);

} // namespace dart::gui::detail

#endif // DART_GUI_DETAIL_APPLICATION_HPP_
