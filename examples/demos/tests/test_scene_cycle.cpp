/* Copyright (c) 2026, The DART development contributors.
 * All rights reserved. See LICENSE for the BSD-style license.
 */

#include "demo_models/HeightmapModel.hpp"
#include "scenes/Scenes.hpp"

#include <dart/gui/app/App.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <limits>
#include <optional>
#include <stdexcept>

#include <cstdlib>

namespace dart_demos {
namespace {

TEST(HeightmapScene, RejectsOverflowingRangeAndAcceptsSubsequentEdit)
{
  float zMin = 0.02f;
  float zMax = 0.04f;
  auto shape = createHeightmapShape(5, 5, 2.f, 2.f, zMin, zMax);
  const auto original = shape;
  const auto editRange = [&](float low, float high) {
    if (updateHeightmapRange(zMin, zMax, low, high))
      shape = createHeightmapShape(5, 5, 2.f, 2.f, zMin, zMax);
  };

  EXPECT_NO_THROW(editRange(-3e38f, 3e38f));
  EXPECT_FLOAT_EQ(zMin, 0.02f);
  EXPECT_FLOAT_EQ(zMax, 0.04f);
  EXPECT_EQ(shape, original);
  EXPECT_NO_THROW(editRange(3e38f, -3e38f));
  EXPECT_FLOAT_EQ(zMin, 0.02f);
  EXPECT_FLOAT_EQ(zMax, 0.04f);
  EXPECT_EQ(shape, original);

  EXPECT_NO_THROW(editRange(0.05f, 0.1f));
  EXPECT_NE(shape, original);
  EXPECT_FLOAT_EQ(zMin, 0.05f);
  EXPECT_FLOAT_EQ(zMax, 0.1f);
  EXPECT_GE(shape->getMinHeight(), zMin);
  EXPECT_LE(shape->getMaxHeight(), zMax);
}

TEST(HeightmapScene, PreservesFiniteAndOrderedRangeEdits)
{
  float zMin = 0.f;
  float zMax = 0.1f;
  EXPECT_FALSE(updateHeightmapRange(zMin, zMax, zMin, zMax));
  EXPECT_TRUE(updateHeightmapRange(zMin, zMax, 0.3f, 0.2f));
  EXPECT_FLOAT_EQ(zMin, 0.2f);
  EXPECT_FLOAT_EQ(zMax, 0.3f);
  EXPECT_TRUE(updateHeightmapRange(
      zMin, zMax, std::numeric_limits<float>::quiet_NaN(), 0.4f));
  EXPECT_FLOAT_EQ(zMin, 0.2f);
  EXPECT_FLOAT_EQ(zMax, 0.4f);
  EXPECT_FALSE(updateHeightmapRange(
      zMin, zMax, zMin, std::numeric_limits<float>::infinity()));
  EXPECT_FLOAT_EQ(zMin, 0.2f);
  EXPECT_FLOAT_EQ(zMax, 0.4f);
}

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
