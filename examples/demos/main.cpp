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
// scenes. See docs/dev_tasks/dart6_consolidated_demos/BRIEF-phase1.md.

#include "DemoHost.hpp"
#include "Registry.hpp"

#include <dart/gui/osg/osg.hpp>

#include <iostream>
#include <string>

#include <cstring>

namespace {

//==============================================================================
struct Options
{
  double guiScale = 1.0;
  bool listScenes = false;
  bool cycleScenes = false;
  int cycleFrames = 30;
  bool headless = false;
  std::string shotPath = "dart-demos.png";
  int steps = 150;
  std::string sceneId;
  int width = 1280;
  int height = 800;
};

//==============================================================================
void printUsage(const char* prog)
{
  std::cout
      << "Usage: " << prog << " [options]\n"
      << "  --list-scenes   Print the demo catalog grouped by category and "
         "exit.\n"
      << "  --scene <id>    Select the initial demo (default: first in the "
         "catalog).\n"
      << "  --cycle-scenes  Advance through every demo without opening a "
         "window; smoke test.\n"
      << "  --frames <n>    Steps per demo with --cycle-scenes (default "
         "30).\n"
      << "  --headless      Render one frame off-screen to a PNG and exit "
         "(no window).\n"
      << "  --shot <path>   Output PNG path for --headless (default "
         "dart-demos.png).\n"
      << "  --steps <n>     Sim steps to settle before the --headless shot "
         "(default 150).\n"
      << "  --width <w> --height <h>  Render/window size (default "
         "1280x800).\n"
      << "  --gui-scale <f> Scale the ImGui panels and initial window size "
         "(default 1.0).\n"
      << "  -h, --help      Show this help.\n";
}

//==============================================================================
/// Parses command-line options. Returns false if the program should exit
/// immediately (e.g. after --help or on a parse error).
bool parseArgs(int argc, char** argv, Options& opt)
{
  auto needsValue = [&](int i) {
    if (i + 1 >= argc) {
      std::cerr << "Missing value for " << argv[i] << "\n";
      return false;
    }
    return true;
  };

  for (int i = 1; i < argc; ++i) {
    const char* a = argv[i];
    if (std::strcmp(a, "--list-scenes") == 0) {
      opt.listScenes = true;
    } else if (std::strcmp(a, "--scene") == 0) {
      if (!needsValue(i))
        return false;
      opt.sceneId = argv[++i];
    } else if (std::strcmp(a, "--cycle-scenes") == 0) {
      opt.cycleScenes = true;
    } else if (std::strcmp(a, "--frames") == 0) {
      if (!needsValue(i))
        return false;
      opt.cycleFrames = std::stoi(argv[++i]);
    } else if (std::strcmp(a, "--headless") == 0) {
      opt.headless = true;
    } else if (std::strcmp(a, "--shot") == 0) {
      if (!needsValue(i))
        return false;
      opt.shotPath = argv[++i];
    } else if (std::strcmp(a, "--steps") == 0) {
      if (!needsValue(i))
        return false;
      opt.steps = std::stoi(argv[++i]);
    } else if (std::strcmp(a, "--width") == 0) {
      if (!needsValue(i))
        return false;
      opt.width = std::stoi(argv[++i]);
    } else if (std::strcmp(a, "--height") == 0) {
      if (!needsValue(i))
        return false;
      opt.height = std::stoi(argv[++i]);
    } else if (std::strcmp(a, "--gui-scale") == 0) {
      if (!needsValue(i))
        return false;
      opt.guiScale
          = dart::gui::osg::parseGuiScale(argv[++i], opt.guiScale, &std::cerr);
    } else if (std::strcmp(a, "-h") == 0 || std::strcmp(a, "--help") == 0) {
      printUsage(argv[0]);
      return false;
    } else {
      std::cerr << "Unknown option: " << a << "\n";
      printUsage(argv[0]);
      return false;
    }
  }
  return true;
}

} // namespace

//==============================================================================
int main(int argc, char** argv)
{
  Options opt;
  if (!parseArgs(argc, argv, opt))
    return 0;

  dart_demos::DemoHost host(dart_demos::makeDemoScenes(), opt.guiScale);
  if (!opt.sceneId.empty())
    host.setInitialScene(opt.sceneId);

  if (opt.listScenes)
    return host.listScenes();

  if (opt.cycleScenes)
    return host.cycleScenes(opt.cycleFrames);

  if (opt.headless) {
    return host.runHeadlessShot(
        opt.shotPath, opt.steps, opt.sceneId, opt.width, opt.height);
  }

  return host.run();
}
