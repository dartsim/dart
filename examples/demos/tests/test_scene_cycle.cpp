/* Copyright (c) 2026, The DART development contributors.
 * All rights reserved. See LICENSE for the BSD-style license.
 */

#include "DemoHost.hpp"
#include "scenes/Scenes.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <optional>
#include <stdexcept>

#include <cstdlib>

namespace dart_demos {
namespace {

TEST(DemoSceneCycle, MissingAssetsAreSkippedAndPreviousSceneIsPreserved)
{
  DemoScene missing;
  missing.id = "missing";
  missing.factory = []() -> DemoSceneSetup {
    throw AssetsNotPrefetched("Prefetch test assets");
  };
  DemoScene valid;
  valid.id = "valid";
  int activations = 0;
  int teardowns = 0;
  valid.factory = [&] {
    DemoSceneSetup setup;
    setup.world = dart::simulation::World::create();
    setup.onActivate = [&](DemoHostContext& context) {
      ++activations;
      context.addTeardown([&] { ++teardowns; });
    };
    return setup;
  };
  {
    // The first miss installs a fallback; the second preserves the valid scene.
    DemoHost host({missing, valid}, std::nullopt);
    testing::internal::CaptureStdout();
    EXPECT_EQ(host.cycleScenes(0), 0);
    const auto output = testing::internal::GetCapturedStdout();
    EXPECT_NE(
        output.find(
            "[cycle-scenes] demo 'missing' skipped: Prefetch test assets"),
        std::string::npos);
    EXPECT_NE(output.find("2 skipped. OK"), std::string::npos);
    EXPECT_EQ(host.getActiveWorldNodeCount(), 1u);
    EXPECT_EQ(activations, 2);
    EXPECT_EQ(teardowns, 1);
  }
  EXPECT_EQ(teardowns, 2);
}

TEST(DemoSceneCycle, OtherFactoryFailuresStillFail)
{
  DemoScene missing;
  missing.id = "missing";
  missing.factory = []() -> DemoSceneSetup {
    throw AssetsNotPrefetched("Prefetch test assets");
  };
  for (int failure = 0; failure < 3; ++failure) {
    DemoScene broken;
    broken.id = "broken";
    broken.factory = [failure]() -> DemoSceneSetup {
      if (failure == 0)
        throw std::runtime_error("Corrupt cached model");
      if (failure == 1)
        throw 42;
      return {};
    };
    DemoHost host({missing, broken}, std::nullopt);
    EXPECT_EQ(host.cycleScenes(0), 1);
    EXPECT_EQ(host.getActiveWorldNodeCount(), 1u);
  }
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
    DemoHost host({makeAtlasV5Scene(), makeUnitreeG1Scene()}, std::nullopt);
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
