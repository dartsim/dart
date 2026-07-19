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

// dart-demos: a single-window host for browsing and running DART example
// scenes. See examples/demos/README.md for the host architecture.

#include "DemoHost.hpp"
#include "Registry.hpp"
#include "scenes/Scenes.hpp"

#include <dart/gui/osg/osg.hpp>

#include <algorithm>
#include <exception>
#include <iostream>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <cctype>
#include <cerrno>
#include <cstdlib>
#include <cstring>

#ifndef DART_DEMOS_DEFAULT_SCENE
  #define DART_DEMOS_DEFAULT_SCENE ""
#endif

namespace {

constexpr int kDefaultWindowWidth = 1600;
constexpr int kDefaultWindowHeight = 1000;

//==============================================================================
enum class ParseResult
{
  Run,
  Help,
  Error
};

//==============================================================================
struct Options
{
  double guiScale = 1.0;
  bool listScenes = false;
  bool verifyFbfSceneDocs = false;
  std::string fbfAuthorTurntableContractScene;
  std::string fbfAuthorCardHouseContractScene;
  bool cycleScenes = false;
  int cycleFrames = 30;
  bool headless = false;
  std::string shotPath = "dart-demos.png";
  bool shotPathExplicit = false;
  int steps = 150;
  std::string sceneId = DART_DEMOS_DEFAULT_SCENE;
  int width = kDefaultWindowWidth;
  int height = kDefaultWindowHeight;
  std::string collisionDetectorName;
  std::size_t simulationThreads = 1u;
  std::vector<int> headlessActionKeys;
  dart_demos::HeadlessTimeline headlessTimeline;

  // Hidden test/debug hooks (undocumented -- not in printUsage()): let a
  // headless --shot capture exercise UI state that normally requires
  // interactive input (a body selected in the Inspector; the Profiler
  // actively recording).
  std::string debugSelectBody;
  bool debugRecordProfile = false;
};

//==============================================================================
void printUsage(const char* prog)
{
  std::cout
      << "Usage: " << prog << " [options]\n"
      << "  --list-scenes   Print the demo catalog grouped by category and "
         "exit.\n"
      << "  --verify-fbf-scene-docs\n"
         "                  Verify FBF paper scenes have self-contained "
         "Scene-tab text and exit.\n"
      << "  --fbf-author-turntable-contract <scene-id>\n"
         "                  Print the runtime-inspected author-turntable "
         "physics/control contract and exit.\n"
      << "  --fbf-author-card-house-contract <scene-id>\n"
         "                  Print the runtime-inspected author-card-house "
         "configuration contract and exit.\n"
      << "  --scene <id>    Select the initial demo (default: first in the "
         "catalog).\n"
      << "  --cycle-scenes  Advance through every demo without opening a "
         "window; smoke test.\n"
      << "  --frames <n>    Steps per demo with --cycle-scenes (default "
         "30).\n"
      << "  --headless      Render one frame off-screen to a PNG and exit "
         "(no window).\n"
      << "  --headless-action <key>\n"
         "                  Invoke a scene key action before --headless "
         "capture "
         "steps; may be repeated.\n"
      << "  --headless-shot-at <step:path>\n"
         "                  Capture a completed simulation step; may be "
         "repeated (step 0 is the initial state).\n"
      << "  --headless-action-at <step:key>\n"
         "                  Invoke a scene key action after same-step "
         "captures; may be repeated.\n"
      << "  --headless-sidecar <path>\n"
         "                  Write the headless timeline and solver "
         "diagnostics as JSON.\n"
      << "  --shot <path>   Output PNG path for --headless (default "
         "dart-demos.png).\n"
      << "  --steps <n>     Sim steps to settle before the --headless shot "
         "(default 150).\n"
      << "  --width <w> --height <h>  Render/window size (default "
      << kDefaultWindowWidth << "x" << kDefaultWindowHeight << ").\n"
      << "  --gui-scale <f> Scale the ImGui panels and initial window size "
         "(default 1.0).\n"
      << "  --collision-detector <name>  Initial collision backend "
         "(e.g. fcl, dart, native, bullet, ode when available).\n"
      << "  --threads <n>   Initial simulation worker threads (0 selects "
         "hardware concurrency).\n"
      << "  -h, --help      Show this help.\n";
}

//==============================================================================
std::size_t parseThreadCount(const char* value, std::size_t fallback)
{
  if (value == nullptr || value[0] == '\0')
    return fallback;

  char* end = nullptr;
  const auto parsed = std::strtoull(value, &end, 10);
  if (end == value)
    return fallback;

  return static_cast<std::size_t>(parsed);
}

//==============================================================================
int parseHeadlessActionKey(const char* value)
{
  if (value == nullptr || std::strlen(value) != 1u)
    return -1;

  return static_cast<unsigned char>(value[0]);
}

//==============================================================================
bool parseScheduledSpec(
    const char* value, std::size_t& step, std::string& payload)
{
  if (value == nullptr)
    return false;

  const std::string spec(value);
  const std::size_t separator = spec.find(':');
  if (separator == std::string::npos || separator == 0u
      || separator + 1u >= spec.size()) {
    return false;
  }

  const std::string stepText = spec.substr(0u, separator);
  if (!std::all_of(stepText.begin(), stepText.end(), [](unsigned char c) {
        return std::isdigit(c);
      })) {
    return false;
  }

  errno = 0;
  char* end = nullptr;
  const unsigned long long parsed = std::strtoull(stepText.c_str(), &end, 10);
  if (errno == ERANGE || end == stepText.c_str() || *end != '\0'
      || parsed > std::numeric_limits<std::size_t>::max()) {
    return false;
  }

  step = static_cast<std::size_t>(parsed);
  payload = spec.substr(separator + 1u);
  return true;
}

//==============================================================================
std::string quoteCommandArgument(const char* value)
{
  const std::string argument = value ? value : "";
  const bool plain
      = !argument.empty()
        && std::all_of(argument.begin(), argument.end(), [](unsigned char c) {
             return std::isalnum(c) || c == '_' || c == '-' || c == '.'
                    || c == '/' || c == ':' || c == '=' || c == ',';
           });
  if (plain)
    return argument;

  std::string quoted = "'";
  for (const char c : argument) {
    if (c == '\'')
      quoted += "'\\''";
    else
      quoted += c;
  }
  quoted += "'";
  return quoted;
}

//==============================================================================
std::string buildRuntimeCommand(int argc, char** argv)
{
  std::string command;
  for (int i = 0; i < argc; ++i) {
    if (!command.empty())
      command += ' ';
    command += quoteCommandArgument(argv[i]);
  }
  return command;
}

//==============================================================================
/// Parses command-line options.
ParseResult parseArgs(int argc, char** argv, Options& opt)
{
  if (const char* detectorEnv = std::getenv("COLLISION_DETECTOR"))
    opt.collisionDetectorName = detectorEnv;
  if (const char* threadsEnv = std::getenv("THREADS"))
    opt.simulationThreads = parseThreadCount(threadsEnv, opt.simulationThreads);

  auto needsValue = [&](int i) {
    if (i + 1 >= argc) {
      std::cerr << "Missing value for " << argv[i] << "\n";
      return ParseResult::Error;
    }
    return ParseResult::Run;
  };

  for (int i = 1; i < argc; ++i) {
    const char* a = argv[i];
    if (std::strcmp(a, "--list-scenes") == 0) {
      opt.listScenes = true;
    } else if (std::strcmp(a, "--verify-fbf-scene-docs") == 0) {
      opt.verifyFbfSceneDocs = true;
    } else if (std::strcmp(a, "--fbf-author-turntable-contract") == 0) {
      if (needsValue(i) == ParseResult::Error)
        return ParseResult::Error;
      opt.fbfAuthorTurntableContractScene = argv[++i];
    } else if (std::strcmp(a, "--fbf-author-card-house-contract") == 0) {
      if (needsValue(i) == ParseResult::Error)
        return ParseResult::Error;
      opt.fbfAuthorCardHouseContractScene = argv[++i];
    } else if (std::strcmp(a, "--scene") == 0) {
      if (needsValue(i) == ParseResult::Error)
        return ParseResult::Error;
      opt.sceneId = argv[++i];
    } else if (std::strcmp(a, "--cycle-scenes") == 0) {
      opt.cycleScenes = true;
    } else if (std::strcmp(a, "--frames") == 0) {
      if (needsValue(i) == ParseResult::Error)
        return ParseResult::Error;
      opt.cycleFrames = std::stoi(argv[++i]);
    } else if (std::strcmp(a, "--headless") == 0) {
      opt.headless = true;
    } else if (std::strcmp(a, "--headless-action") == 0) {
      if (needsValue(i) == ParseResult::Error)
        return ParseResult::Error;
      const int key = parseHeadlessActionKey(argv[++i]);
      if (key < 0) {
        std::cerr << "--headless-action expects a single key character.\n";
        return ParseResult::Error;
      }
      opt.headlessActionKeys.push_back(key);
    } else if (std::strcmp(a, "--headless-shot-at") == 0) {
      if (needsValue(i) == ParseResult::Error)
        return ParseResult::Error;
      std::size_t step = 0u;
      std::string path;
      if (!parseScheduledSpec(argv[++i], step, path)) {
        std::cerr << "--headless-shot-at expects STEP:PATH with a nonnegative "
                     "integer step and nonempty path.\n";
        return ParseResult::Error;
      }
      opt.headlessTimeline.shots.push_back({step, std::move(path)});
    } else if (std::strcmp(a, "--headless-action-at") == 0) {
      if (needsValue(i) == ParseResult::Error)
        return ParseResult::Error;
      std::size_t step = 0u;
      std::string keyText;
      if (!parseScheduledSpec(argv[++i], step, keyText)
          || keyText.size() != 1u) {
        std::cerr << "--headless-action-at expects STEP:KEY with a "
                     "nonnegative integer step and one key character.\n";
        return ParseResult::Error;
      }
      opt.headlessTimeline.actions.push_back(
          {step, static_cast<unsigned char>(keyText[0])});
    } else if (std::strcmp(a, "--headless-sidecar") == 0) {
      if (needsValue(i) == ParseResult::Error)
        return ParseResult::Error;
      opt.headlessTimeline.sidecarPath = argv[++i];
      if (opt.headlessTimeline.sidecarPath.empty()) {
        std::cerr << "--headless-sidecar expects a nonempty path.\n";
        return ParseResult::Error;
      }
    } else if (std::strcmp(a, "--shot") == 0) {
      if (needsValue(i) == ParseResult::Error)
        return ParseResult::Error;
      opt.shotPath = argv[++i];
      opt.shotPathExplicit = true;
    } else if (std::strcmp(a, "--steps") == 0) {
      if (needsValue(i) == ParseResult::Error)
        return ParseResult::Error;
      opt.steps = std::stoi(argv[++i]);
    } else if (std::strcmp(a, "--width") == 0) {
      if (needsValue(i) == ParseResult::Error)
        return ParseResult::Error;
      opt.width = std::stoi(argv[++i]);
    } else if (std::strcmp(a, "--height") == 0) {
      if (needsValue(i) == ParseResult::Error)
        return ParseResult::Error;
      opt.height = std::stoi(argv[++i]);
    } else if (std::strcmp(a, "--gui-scale") == 0) {
      if (needsValue(i) == ParseResult::Error)
        return ParseResult::Error;
      opt.guiScale
          = dart::gui::osg::parseGuiScale(argv[++i], opt.guiScale, &std::cerr);
    } else if (std::strcmp(a, "--collision-detector") == 0) {
      if (needsValue(i) == ParseResult::Error)
        return ParseResult::Error;
      opt.collisionDetectorName = argv[++i];
    } else if (std::strcmp(a, "--threads") == 0) {
      if (needsValue(i) == ParseResult::Error)
        return ParseResult::Error;
      opt.simulationThreads
          = parseThreadCount(argv[++i], opt.simulationThreads);
    } else if (std::strcmp(a, "--debug-select-body") == 0) {
      if (needsValue(i) == ParseResult::Error)
        return ParseResult::Error;
      opt.debugSelectBody = argv[++i];
    } else if (std::strcmp(a, "--debug-record-profile") == 0) {
      opt.debugRecordProfile = true;
    } else if (std::strcmp(a, "-h") == 0 || std::strcmp(a, "--help") == 0) {
      printUsage(argv[0]);
      return ParseResult::Help;
    } else {
      std::cerr << "Unknown option: " << a << "\n";
      printUsage(argv[0]);
      return ParseResult::Error;
    }
  }

  const bool timelineRequested = !opt.headlessTimeline.shots.empty()
                                 || !opt.headlessTimeline.actions.empty()
                                 || !opt.headlessTimeline.sidecarPath.empty();
  if (timelineRequested && !opt.headless) {
    std::cerr << "--headless-shot-at, --headless-action-at, and "
                 "--headless-sidecar require --headless.\n";
    return ParseResult::Error;
  }
  if (timelineRequested && opt.steps < 0) {
    std::cerr << "--steps must be nonnegative for a headless timeline.\n";
    return ParseResult::Error;
  }
  if (timelineRequested) {
    const std::size_t totalSteps = static_cast<std::size_t>(opt.steps);
    for (const auto& shot : opt.headlessTimeline.shots) {
      if (shot.step > totalSteps) {
        std::cerr << "--headless-shot-at step " << shot.step
                  << " exceeds --steps " << totalSteps << ".\n";
        return ParseResult::Error;
      }
    }
    for (const auto& action : opt.headlessTimeline.actions) {
      if (action.step > totalSteps) {
        std::cerr << "--headless-action-at step " << action.step
                  << " exceeds --steps " << totalSteps << ".\n";
        return ParseResult::Error;
      }
    }
  }
  return ParseResult::Run;
}

} // namespace

//==============================================================================
int main(int argc, char** argv)
{
  Options opt;
  const ParseResult parseResult = parseArgs(argc, argv, opt);
  if (parseResult == ParseResult::Help)
    return 0;
  if (parseResult == ParseResult::Error)
    return 1;

  if (!opt.fbfAuthorTurntableContractScene.empty()) {
    try {
      std::cout << dart_demos::fbfAuthorTurntablePhysicsContractJson(
          opt.fbfAuthorTurntableContractScene)
                << '\n';
      return 0;
    } catch (const std::exception& error) {
      std::cerr << error.what() << '\n';
      return 1;
    }
  }

  if (!opt.fbfAuthorCardHouseContractScene.empty()) {
    try {
      std::cout << dart_demos::fbfAuthorCardHouseConfigurationContractJson(
          opt.fbfAuthorCardHouseContractScene)
                << '\n';
      return 0;
    } catch (const std::exception& error) {
      std::cerr << error.what() << '\n';
      return 1;
    }
  }

  dart_demos::DemoHost host(
      dart_demos::makeDemoScenes(),
      opt.guiScale,
      opt.collisionDetectorName,
      opt.simulationThreads);
  if (!opt.sceneId.empty())
    host.setInitialScene(opt.sceneId);
  if (!opt.debugSelectBody.empty())
    host.setDebugSelectBodyName(opt.debugSelectBody);
  if (opt.debugRecordProfile)
    host.setDebugRecordProfile(true);
  for (const int key : opt.headlessActionKeys)
    host.addHeadlessActionKey(key);

  if (opt.listScenes)
    return host.listScenes();

  if (opt.verifyFbfSceneDocs)
    return host.verifyFbfSceneDocs();

  if (opt.cycleScenes)
    return host.cycleScenes(opt.cycleFrames);

  if (opt.headless) {
    const bool timelineRequested = !opt.headlessTimeline.shots.empty()
                                   || !opt.headlessTimeline.actions.empty()
                                   || !opt.headlessTimeline.sidecarPath.empty();
    if (timelineRequested) {
      opt.headlessTimeline.runtimeCommand = buildRuntimeCommand(argc, argv);
      if (opt.shotPathExplicit || opt.headlessTimeline.shots.empty()) {
        opt.headlessTimeline.shots.push_back(
            {static_cast<std::size_t>(opt.steps), opt.shotPath});
      }
      return host.runHeadlessTimeline(
          opt.headlessTimeline, opt.steps, opt.sceneId, opt.width, opt.height);
    }
    return host.runHeadlessShot(
        opt.shotPath, opt.steps, opt.sceneId, opt.width, opt.height);
  }

  return host.run();
}
