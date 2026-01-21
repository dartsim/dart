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

#include <dart/gui/all.hpp>

#include <dart/utils/All.hpp>

#include <dart/all.hpp>
#include <dart/io/read.hpp>

#include <filesystem>
#include <iostream>
#include <string>
#include <string_view>

#include <cstdlib>

using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::gui;
using namespace dart::utils;
using namespace dart::math;

namespace {

//==============================================================================
// Command-line options
//==============================================================================
struct Options
{
  bool headless = false;
  int frames = -1;
  std::string outDir;
  int width = 640;
  int height = 480;
  bool help = false;
};

//==============================================================================
void printUsage(const char* argv0)
{
  std::cout
      << "Usage: " << argv0 << " [options]\n\n"
      << "Options:\n"
      << "  --headless        Run without display window (for CI/testing)\n"
      << "  --frames <n>      Run for n frames then exit (default: "
         "interactive)\n"
      << "  --out <dir>       Output directory for captured frames\n"
      << "  --width <n>       Viewport width (default: 640)\n"
      << "  --height <n>      Viewport height (default: 480)\n"
      << "  -h, --help        Show this help\n\n"
      << "Interactive controls (when not headless):\n"
      << "  space bar: simulation on/off\n"
      << "  'p': playback/stop\n"
      << "  'v': visualization on/off\n"
      << "  '1'-'4': apply directional forces\n";
}

//==============================================================================
bool parseInt(std::string_view value, int& output)
{
  if (value.empty())
    return false;

  const std::string str(value);
  char* end = nullptr;
  const long result = std::strtol(str.c_str(), &end, 10);
  if (!end || *end != '\0')
    return false;

  output = static_cast<int>(result);
  return true;
}

//==============================================================================
enum class ParseResult
{
  Ok,
  Help,
  Error
};

//==============================================================================
ParseResult parseArgs(int argc, char* argv[], Options& options)
{
  for (int i = 1; i < argc; ++i) {
    const std::string_view arg(argv[i]);

    if (arg == "-h" || arg == "--help") {
      printUsage(argv[0]);
      return ParseResult::Help;
    }

    if (arg == "--headless") {
      options.headless = true;
      continue;
    }

    if (arg == "--frames" && i + 1 < argc) {
      if (!parseInt(argv[++i], options.frames) || options.frames < 0) {
        std::cerr << "Invalid frames value: " << argv[i] << "\n";
        printUsage(argv[0]);
        return ParseResult::Error;
      }
      continue;
    }

    if (arg == "--out" && i + 1 < argc) {
      options.outDir = argv[++i];
      continue;
    }

    if (arg == "--width" && i + 1 < argc) {
      if (!parseInt(argv[++i], options.width) || options.width <= 0) {
        std::cerr << "Invalid width: " << argv[i] << "\n";
        printUsage(argv[0]);
        return ParseResult::Error;
      }
      continue;
    }

    if (arg == "--height" && i + 1 < argc) {
      if (!parseInt(argv[++i], options.height) || options.height <= 0) {
        std::cerr << "Invalid height: " << argv[i] << "\n";
        printUsage(argv[0]);
        return ParseResult::Error;
      }
      continue;
    }

    std::cerr << "Unknown argument: " << arg << "\n";
    printUsage(argv[0]);
    return ParseResult::Error;
  }

  return ParseResult::Ok;
}

} // namespace

//==============================================================================
class RigidCubesWorldNode : public dart::gui::RealTimeWorldNode
{
public:
  RigidCubesWorldNode(
      dart::simulation::WorldPtr world,
      ::osg::ref_ptr<osgShadow::ShadowTechnique> shadow = nullptr)
    : dart::gui::RealTimeWorldNode(std::move(world), std::move(shadow)),
      mForce(Eigen::Vector3d::Zero())
  {
  }

  void customPreStep() override
  {
    // Apply the force to the second skeleton (index 1), first body node
    if (mWorld->getNumSkeletons() > 1) {
      mWorld->getSkeleton(1)->getBodyNode(0)->addExtForce(mForce);
    }

    // Decay the force by half each step (as in original)
    mForce /= 2.0;
  }

  void setForce(const Eigen::Vector3d& force)
  {
    mForce = force;
  }

protected:
  Eigen::Vector3d mForce;
};

//==============================================================================
class RigidCubesEventHandler : public ::osgGA::GUIEventHandler
{
public:
  RigidCubesEventHandler(
      dart::gui::Viewer* viewer,
      RigidCubesWorldNode* worldNode,
      const WorldPtr& world)
    : mViewer(viewer), mWorldNode(worldNode), mWorld(world)
  {
  }

  virtual bool handle(
      const ::osgGA::GUIEventAdapter& ea, ::osgGA::GUIActionAdapter&) override
  {
    if (::osgGA::GUIEventAdapter::KEYDOWN == ea.getEventType()) {
      static bool eventHandlerOn = true;
      switch (ea.getKey()) {
        case ' ': // Space bar: toggle simulation
          mViewer->simulate(!mViewer->isSimulating());
          return true;
        case 'p': // Toggle playback mode
          eventHandlerOn = !eventHandlerOn;
          mViewer->switchDefaultEventHandler(eventHandlerOn);
          return true;
        case 'v': // Toggle visualization markers
          if (mWorldNode) {
            // Toggle marker visibility (implementation would depend on specific
            // markers)
            std::cout << "Toggling visualization markers\n";
          }
          return true;
        case '1': // Apply negative X force
          if (mWorldNode) {
            mWorldNode->setForce(Eigen::Vector3d(-500, 0, 0));
            std::cout << "Applied -X force\n";
          }
          return true;
        case '2': // Apply positive X force
          if (mWorldNode) {
            mWorldNode->setForce(Eigen::Vector3d(500, 0, 0));
            std::cout << "Applied +X force\n";
          }
          return true;
        case '3': // Apply negative Z force
          if (mWorldNode) {
            mWorldNode->setForce(Eigen::Vector3d(0, 0, -500));
            std::cout << "Applied -Z force\n";
          }
          return true;
        case '4': // Apply positive Z force
          if (mWorldNode) {
            mWorldNode->setForce(Eigen::Vector3d(0, 0, 500));
            std::cout << "Applied +Z force\n";
          }
          return true;
        default:
          return false;
      }
    }
    return false;
  }

protected:
  dart::gui::Viewer* mViewer;
  RigidCubesWorldNode* mWorldNode;
  WorldPtr mWorld;
};

//==============================================================================
int main(int argc, char* argv[])
{
  // Parse command-line arguments
  Options options;
  const ParseResult parseResult = parseArgs(argc, argv, options);
  if (parseResult == ParseResult::Help)
    return EXIT_SUCCESS;
  if (parseResult == ParseResult::Error)
    return EXIT_FAILURE;

  // Validate headless options
  if (options.headless && options.frames < 0) {
    std::cerr << "Error: --headless requires --frames to be specified\n";
    printUsage(argv[0]);
    return EXIT_FAILURE;
  }

  // Create output directory if specified
  if (!options.outDir.empty()) {
    std::error_code ec;
    std::filesystem::create_directories(options.outDir, ec);
    if (ec) {
      std::cerr << "Failed to create output directory '" << options.outDir
                << "': " << ec.message() << "\n";
      return EXIT_FAILURE;
    }
  }

  // Create and initialize the world
  auto world = dart::io::readWorld("dart://sample/skel/cubes.skel");
  if (!world) {
    DART_ERROR("Failed to load world.");
    return EXIT_FAILURE;
  }
  world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));

  std::unique_ptr<dart::gui::Viewer> viewerPtr;
  if (options.headless) {
    viewerPtr = std::make_unique<dart::gui::Viewer>(
        dart::gui::ViewerConfig::headless(options.width, options.height));
  } else {
    viewerPtr = std::make_unique<dart::gui::Viewer>();
  }
  dart::gui::Viewer& viewer = *viewerPtr;

  auto shadow = dart::gui::WorldNode::createDefaultShadowTechnique(&viewer);

  ::osg::ref_ptr<RigidCubesWorldNode> worldNode
      = new RigidCubesWorldNode(world, shadow);
  viewer.addWorldNode(worldNode);

  ::osg::ref_ptr<RigidCubesEventHandler> eventHandler
      = new RigidCubesEventHandler(&viewer, worldNode, world);
  viewer.addEventHandler(eventHandler);

  if (!options.headless) {
    viewer.setUpViewInWindow(0, 0, options.width, options.height);
  }

  // Set up camera position
  viewer.getCameraManipulator()->setHomePosition(
      ::osg::Vec3(5.0f, 5.0f, 5.0f),
      ::osg::Vec3(0.0f, 0.0f, 0.0f),
      ::osg::Vec3(0.0f, 0.0f, 1.0f));
  viewer.setCameraManipulator(viewer.getCameraManipulator());

  if (options.headless) {
    std::cout << "Running in headless mode for " << options.frames
              << " frames\n";
    if (!options.outDir.empty()) {
      std::cout << "Saving frames to: " << options.outDir << "\n";
    }

    // Start recording if output directory specified
    if (!options.outDir.empty()) {
      viewer.record(options.outDir, "frame_", false, 6);
    }

    // Enable simulation
    viewer.simulate(true);

    // Run the specified number of frames
    for (int i = 0; i < options.frames; ++i) {
      viewer.frame();
    }

    std::cout << "Headless rendering complete.\n";
    return EXIT_SUCCESS;
  }

  // Interactive mode: add instruction text and run viewer loop
  viewer.addInstructionText("Rigid Cubes Example\n");
  viewer.addInstructionText("Controls:\n");
  viewer.addInstructionText("  space bar: simulation on/off\n");
  viewer.addInstructionText("  'p': playback/stop\n");
  viewer.addInstructionText("  'v': visualization on/off\n");
  viewer.addInstructionText("  '1'-'4': apply directional forces\n");
  viewer.addInstructionText("    '1': -X force    '2': +X force\n");
  viewer.addInstructionText("    '3': -Z force    '4': +Z force\n");

  // Print instructions to console
  std::cout << viewer.getInstructions() << std::endl;

  // Run the simulation
  return viewer.run();
}
