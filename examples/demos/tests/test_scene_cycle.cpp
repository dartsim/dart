/* Copyright (c) 2026, The DART development contributors.
 * All rights reserved. See LICENSE for the BSD-style license.
 */

#include "scenes/Scenes.hpp"

#include <dart/gui/app/App.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <optional>
#include <stdexcept>

#include <cstdlib>

namespace dart_demos {
namespace {

#if defined(__linux__) && defined(DART_DEMOS_HAVE_MODEL_ASSETS)
TEST(DemoSceneCycle, EmptyPrefetchedCacheSkipsModernHumanoids)
{
  const char* previous = std::getenv("XDG_CACHE_HOME");
  const std::optional<std::string> previousCache
      = previous ? std::make_optional<std::string>(previous) : std::nullopt;
  const auto cache
      = std::filesystem::temp_directory_path()
        / ("dart-scene-cycle-"
           + std::to_string(
               std::chrono::steady_clock::now().time_since_epoch().count()));
  std::filesystem::create_directory(cache);
  ASSERT_EQ(setenv("XDG_CACHE_HOME", cache.c_str(), 1), 0);
  {
    dart::gui::app::App host(
        {makeAtlasV5Scene(), makeUnitreeG1Scene()}, std::nullopt);
    testing::internal::CaptureStdout();
    EXPECT_EQ(host.cycleScenes(0), 0);
    const auto output = testing::internal::GetCapturedStdout();
    EXPECT_NE(output.find("demo 'atlas_v5' skipped:"), std::string::npos);
    EXPECT_NE(output.find("demo 'unitree_g1' skipped:"), std::string::npos);
    EXPECT_NE(output.find("4 skipped. OK"), std::string::npos);
  }
  if (previousCache)
    EXPECT_EQ(setenv("XDG_CACHE_HOME", previousCache->c_str(), 1), 0);
  else
    EXPECT_EQ(unsetenv("XDG_CACHE_HOME"), 0);
  EXPECT_TRUE(std::filesystem::is_empty(cache));
  std::filesystem::remove(cache);
}
#endif

} // namespace
} // namespace dart_demos
