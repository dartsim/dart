// C++ side of the dart-demos golden-set parity smoke (PLAN-103 Phase 2).
//
// Mirrors python/tests/unit/test_golden_parity.py: builds the matching
// hello_world scene from examples/demos/scenes/, steps the deterministic
// World the same number of steps, and asserts positions within tight
// tolerance against the shared fixture values (hardcoded here; regenerate via
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
#include <vector>

namespace {

constexpr int kGoldenSteps = 60;
constexpr double kGoldenAbsTol = 1e-9;

// Hardcoded from python/examples/demos/golden/hello_world.json. When the
// Python generator regenerates the fixture, update these values to match.
const std::vector<double> kHelloWorldFallingBoxExpected = {
    -6.938893903907231e-18,
    0.0,
    0.0,
    -0.008630151867373204,
    -0.004149673879427912,
    -0.015185050567265601,
};

int fail(const std::string& message)
{
  std::cerr << "golden parity failure: " << message << "\n";
  return EXIT_FAILURE;
}

} // namespace

int main()
{
  const auto entries = dart::examples::demos::makeDemoScenes();
  auto it = std::find_if(
      entries.begin(), entries.end(),
      [](const dart::gui::DemoSceneEntry& entry) {
        return entry.id == "hello_world";
      });
  if (it == entries.end()) {
    return fail("golden scene 'hello_world' missing from registry");
  }

  const dart::gui::ApplicationOptions options = it->factory();
  if (options.world == nullptr) {
    return fail("scene factory produced a null world");
  }

  for (int step = 0; step < kGoldenSteps; ++step) {
    options.world->step();
  }

  auto skeleton = options.world->getSkeleton("falling_box");
  if (skeleton == nullptr) {
    return fail("skeleton 'falling_box' missing after stepping");
  }

  const auto positions = skeleton->getPositions();
  if (static_cast<std::size_t>(positions.size())
      != kHelloWorldFallingBoxExpected.size()) {
    return fail(
        "falling_box position size mismatch: actual="
        + std::to_string(positions.size()) + " expected="
        + std::to_string(kHelloWorldFallingBoxExpected.size()));
  }

  for (Eigen::Index i = 0; i < positions.size(); ++i) {
    const double diff = std::abs(positions(i) - kHelloWorldFallingBoxExpected[
        static_cast<std::size_t>(i)]);
    if (diff > kGoldenAbsTol) {
      return fail(
          "falling_box position[" + std::to_string(i) + "] mismatch: actual="
          + std::to_string(positions(i)) + " expected="
          + std::to_string(kHelloWorldFallingBoxExpected[
              static_cast<std::size_t>(i)]) + " diff=" + std::to_string(diff));
    }
  }

  std::cout << "golden parity OK: hello_world matches the fixture after "
            << kGoldenSteps << " steps.\n";
  return EXIT_SUCCESS;
}
