// C++ side of the dart-demos golden-set parity smoke (PLAN-103 Phase 2).
//
// Mirrors python/tests/unit/test_golden_parity.py: builds every golden scene
// from examples/demos/scenes/, steps the deterministic World the same number
// of steps, and asserts each scene's skeleton positions stay within tight
// tolerance of the shared fixture values (hardcoded here; regenerate via
// python/examples/demos/golden/_generate.py and update both sides).
//
// Plain main() avoids depending on GTest from the examples subdir scope.

#include <scenes.hpp>

#include <dart/dynamics/skeleton.hpp>
#include <dart/gui/application.hpp>
#include <dart/simulation/world.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

namespace {

constexpr int kGoldenSteps = 60;
constexpr double kGoldenAbsTol = 1e-9;

struct GoldenSkeletonState
{
  std::string name;
  std::vector<double> positions;
};

struct GoldenScene
{
  std::string id;
  std::vector<GoldenSkeletonState> expected;
};

// Hardcoded from python/examples/demos/golden/<scene>.json. When the Python
// generator regenerates a fixture, update the matching entry here. Per-box
// noise across the boxes scene stays below 1e-14, so a single reference value
// is well within ``kGoldenAbsTol`` for all 27 boxes.
std::vector<GoldenScene> goldenScenes()
{
  std::vector<GoldenScene> scenes;

  scenes.push_back({
      "hello_world",
      {{"falling_box",
        {-6.938893903907231e-18,
         0.0,
         0.0,
         -0.008630151867373204,
         -0.004149673879427912,
         -0.015185050567265601}}}});

  GoldenScene boxes{"boxes", {}};
  const std::vector<double> boxExpected
      = {0.0, 0.0, 0.0, 0.0, 0.0, -0.017952299999999977};
  for (int i = 0; i < 27; ++i) {
    boxes.expected.push_back({"box_" + std::to_string(i), boxExpected});
  }
  scenes.push_back(std::move(boxes));

  return scenes;
}

int fail(const std::string& message)
{
  std::cerr << "golden parity failure: " << message << "\n";
  return EXIT_FAILURE;
}

int verifyScene(
    const std::vector<dart::gui::DemoSceneEntry>& entries,
    const GoldenScene& golden)
{
  auto it = std::find_if(
      entries.begin(), entries.end(),
      [&golden](const dart::gui::DemoSceneEntry& entry) {
        return entry.id == golden.id;
      });
  if (it == entries.end()) {
    return fail("golden scene '" + golden.id + "' missing from registry");
  }

  const dart::gui::ApplicationOptions options = it->factory();
  if (options.world == nullptr) {
    return fail("scene '" + golden.id + "' factory produced a null world");
  }

  for (int step = 0; step < kGoldenSteps; ++step) {
    options.world->step();
  }

  for (const auto& expected : golden.expected) {
    auto skeleton = options.world->getSkeleton(expected.name);
    if (skeleton == nullptr) {
      return fail(
          "scene '" + golden.id + "': skeleton '" + expected.name
          + "' missing after stepping");
    }

    const auto positions = skeleton->getPositions();
    if (static_cast<std::size_t>(positions.size())
        != expected.positions.size()) {
      return fail(
          "scene '" + golden.id + "' skeleton '" + expected.name
          + "' position size mismatch: actual="
          + std::to_string(positions.size()) + " expected="
          + std::to_string(expected.positions.size()));
    }

    for (Eigen::Index i = 0; i < positions.size(); ++i) {
      const double diff = std::abs(
          positions(i) - expected.positions[static_cast<std::size_t>(i)]);
      if (diff > kGoldenAbsTol) {
        return fail(
            "scene '" + golden.id + "' skeleton '" + expected.name
            + "' position[" + std::to_string(i) + "] mismatch: actual="
            + std::to_string(positions(i)) + " expected="
            + std::to_string(
                expected.positions[static_cast<std::size_t>(i)])
            + " diff=" + std::to_string(diff));
      }
    }
  }

  std::cout << "golden parity OK: " << golden.id << " matches the fixture after "
            << kGoldenSteps << " steps.\n";
  return EXIT_SUCCESS;
}

} // namespace

int main()
{
  const auto entries = dart::examples::demos::makeDemoScenes();
  for (const auto& golden : goldenScenes()) {
    if (const int status = verifyScene(entries, golden); status != EXIT_SUCCESS) {
      return status;
    }
  }
  return EXIT_SUCCESS;
}
