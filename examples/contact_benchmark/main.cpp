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

#include "ContactContainerScene.hpp"

#include <dart/gui/osg/ImGuiHandler.hpp>
#include <dart/gui/osg/ImGuiViewer.hpp>
#include <dart/gui/osg/ImGuiWidget.hpp>
#include <dart/gui/osg/IncludeImGui.hpp>
#include <dart/gui/osg/Utils.hpp>
#include <dart/gui/osg/osg.hpp>

#include <dart/utils/sdf/SdfParser.hpp>

#include <dart/constraint/BoxedLcpConstraintSolver.hpp>
#include <dart/constraint/ConstraintSolver.hpp>
#include <dart/constraint/ContactConstraint.hpp>
#include <dart/constraint/DantzigBoxedLcpSolver.hpp>
#include <dart/constraint/PgsBoxedLcpSolver.hpp>

#include <dart/collision/bullet/BulletCollisionDetector.hpp>
#include <dart/collision/dart/DARTCollisionDetector.hpp>
#include <dart/collision/fcl/FCLCollisionDetector.hpp>
#include <dart/collision/ode/OdeCollisionDetector.hpp>

#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/CapsuleShape.hpp>
#include <dart/dynamics/CylinderShape.hpp>
#include <dart/dynamics/EllipsoidShape.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/Inertia.hpp>
#include <dart/dynamics/PlaneShape.hpp>
#include <dart/dynamics/SphereShape.hpp>
#include <dart/dynamics/WeldJoint.hpp>

#include <dart/common/Profile.hpp>

#include <dart/dart.hpp>

#include <osg/Camera>
#include <osgGA/GUIActionAdapter>
#include <osgGA/GUIEventAdapter>
#include <osgGA/GUIEventHandler>
#include <osgViewer/ViewerBase>

#include <algorithm>
#include <array>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>

namespace {

namespace contact_scene = dart::examples::contact_benchmark;

constexpr std::size_t kDefaultGeneratedObjects = 30;
constexpr double kDefaultGeneratedSpacing = 1.1;
constexpr double kDefaultGeneratedContainerSpacing
    = contact_scene::kDefaultContactContainerSpacing;
constexpr std::size_t kDefaultGeneratedContainerLayers
    = contact_scene::kDefaultContactContainerLayers;
constexpr double kDefaultGeneratedContainerJitter
    = contact_scene::kDefaultContactContainerJitter;
constexpr double kDefaultGeneratedContainerWallThickness
    = contact_scene::kDefaultContactContainerWallThickness;
constexpr double kDefaultGeneratedContainerDropHeight
    = contact_scene::kDefaultContactContainerDropHeight;
constexpr std::uint32_t kDefaultGeneratedContainerSeed
    = contact_scene::kDefaultContactContainerSeed;
constexpr double kDefaultGeneratedContainerLinearDamping
    = contact_scene::kDefaultContactContainerLinearDamping;
constexpr double kDefaultGeneratedContainerAngularDamping
    = contact_scene::kDefaultContactContainerAngularDamping;
constexpr double kDefaultDropHeight = 0.2;

enum class CollisionEngine
{
  Default,
  Dart,
  Fcl,
  Bullet,
  Ode,
};

enum class BoxedLcpSolverKind
{
  Default,
  Dantzig,
  Pgs,
};

struct Options
{
  std::optional<std::string> sdfPath;
  std::size_t steps = 1000;
  std::size_t warmup = 0;
  std::size_t checkpoint = 100;
  bool gui = false;
  bool quiet = false;
  bool profile = false;
  bool primitiveShapes = false;
  bool sdfPlaneShapes = false;
  bool disableDeactivation = false;
  bool disableSecondaryLcp = false;
  bool sleepStateColors = false;
  bool guiStart = false;
  bool guiCaptureExerciseWidget = false;
  double guiTargetRtf = 1.0;
  double guiScale = dart::gui::osg::getDefaultGuiScale();
  std::optional<std::string> guiCapturePath;
  std::size_t guiCaptureSteps = 0;
  std::size_t worldThreads = 1;
  double dropHeight = 0.0;
  std::optional<double> sleepLinearThreshold;
  std::optional<double> sleepAngularThreshold;
  std::optional<double> sleepTimeUntilSleep;
  std::optional<double> sleepContactPenetrationTolerance;
  std::optional<double> contactMaxErrorReductionVelocity;
  std::optional<std::size_t> generatedObjects;
  bool generateCapsules = false;
  bool generateContainer = false;
  double generatedSpacing = kDefaultGeneratedSpacing;
  bool generatedSpacingSpecified = false;
  std::size_t generatedContainerLayers = kDefaultGeneratedContainerLayers;
  std::optional<double> generatedContainerSize;
  std::optional<double> generatedContainerWallHeight;
  double generatedContainerWallThickness
      = kDefaultGeneratedContainerWallThickness;
  double generatedContainerJitter = kDefaultGeneratedContainerJitter;
  std::uint32_t generatedContainerSeed = kDefaultGeneratedContainerSeed;
  double generatedContainerInitialSpeed = 0.0;
  double generatedContainerInitialAngularSpeed = 0.0;
  double generatedContainerLinearDamping
      = kDefaultGeneratedContainerLinearDamping;
  double generatedContainerAngularDamping
      = kDefaultGeneratedContainerAngularDamping;
  std::optional<std::string> dumpFinalScenePath;
  std::optional<std::size_t> maxContacts;
  std::optional<std::size_t> maxContactsPerPair;
  CollisionEngine collisionEngine = CollisionEngine::Default;
  BoxedLcpSolverKind boxedLcpSolver = BoxedLcpSolverKind::Default;
  bool dropHeightSpecified = false;
};

class ScopedProfileRecording
{
public:
  explicit ScopedProfileRecording(bool enabled)
    : mPrevious(dart::common::profile::setProfileRecordingEnabled(enabled))
  {
    // Do nothing
  }

  ~ScopedProfileRecording()
  {
    dart::common::profile::setProfileRecordingEnabled(mPrevious);
  }

  ScopedProfileRecording(const ScopedProfileRecording&) = delete;
  ScopedProfileRecording& operator=(const ScopedProfileRecording&) = delete;

private:
  bool mPrevious;
};

struct ContactPair
{
  std::uintptr_t a = 0;
  std::uintptr_t b = 0;

  ContactPair(const void* first, const void* second)
  {
    const auto firstAddress = reinterpret_cast<std::uintptr_t>(first);
    const auto secondAddress = reinterpret_cast<std::uintptr_t>(second);
    a = std::min(firstAddress, secondAddress);
    b = std::max(firstAddress, secondAddress);
  }

  bool operator==(const ContactPair& other) const
  {
    return a == other.a && b == other.b;
  }
};

struct ContactPairHash
{
  std::size_t operator()(const ContactPair& pair) const
  {
    const auto h1 = std::hash<std::uintptr_t>()(pair.a);
    const auto h2 = std::hash<std::uintptr_t>()(pair.b);
    return h1 ^ (h2 + 0x9e3779b97f4a7c15ULL + (h1 << 6) + (h1 >> 2));
  }
};

struct ContactStats
{
  std::size_t contacts = 0;
  std::size_t pairs = 0;
  std::size_t maxPairContacts = 0;
  std::size_t contactsOverSleepTolerance = 0;
  std::size_t zeroNormalContacts = 0;
  double maxPenetration = 0.0;
};

struct SleepStats
{
  std::size_t skeletons = 0;
  std::size_t mobile = 0;
  std::size_t islands = 0;
  std::size_t maxIslandMobile = 0;
  std::size_t islanded = 0;
  std::size_t candidates = 0;
  std::size_t resting = 0;
  std::size_t disturbed = 0;
  std::size_t belowSleepSpeed = 0;
  std::size_t belowFinalSpeed = 0;
  std::size_t belowWakeSpeed = 0;
  std::size_t dwellReady = 0;
  std::size_t dwellAndSleepReady = 0;
  std::size_t dwellAndFinalReady = 0;
  std::size_t dwellAndWakeReady = 0;
  double maxLinearSpeed = 0.0;
  double maxAngularSpeed = 0.0;
  double maxSmoothedLinearSpeed = 0.0;
  double maxSmoothedAngularSpeed = 0.0;
  double maxRestDwellTime = 0.0;
};

struct FinalStateDigest
{
  bool finite = true;
  std::size_t skeletons = 0;
  std::size_t mobile = 0;
  std::size_t bodyNodes = 0;
  std::size_t dofs = 0;
  double worldTime = 0.0;
  int simFrames = 0;
  double positionAbsSum = 0.0;
  double velocityAbsSum = 0.0;
  double bodyTranslationAbsSum = 0.0;
  double maxAbsVelocity = 0.0;
  std::uint64_t hash = 1469598103934665603ULL;
};

void printUsage(const std::string& programName)
{
  std::cout
      << "Usage: " << programName << " [options]\n"
      << "       " << programName << " <sdf_file_path> [num_steps] [options]\n"
      << "       " << programName << " --generate-objects N [options]\n"
      << "With no scene arguments, opens a paused generated contact scene.\n"
      << "Options:\n"
      << "  --steps N                 Number of simulation steps.\n"
      << "  --warmup N                Steps to run before timed measurement.\n"
      << "  --checkpoint N            Diagnostic print interval; 0 disables.\n"
      << "  --quiet, --headless       Print only setup and final results.\n"
      << "  --help, -h                Print this help text.\n"
      << "  --gui                     Open an OSG viewer instead of "
         "benchmarking.\n"
      << "  --profile                 Dump DART text profiler summary.\n"
      << "  --collision default|dart|fcl|bullet|ode\n"
      << "  --lcp-solver default|dantzig|pgs\n"
      << "                            Set the boxed-LCP primary solver for "
         "measurement.\n"
      << "  --max-contacts N          Set global collision contact cap.\n"
      << "  --max-contacts-per-pair N Set per-pair contact cap; 0 preserves "
         "legacy behavior.\n"
      << "  --primitive-shapes        Use FCL primitive shapes when FCL is "
         "used.\n"
      << "  --sdf-plane-shapes        Parse SDF <plane> geometries as "
         "PlaneShape.\n"
      << "  --disable-deactivation    Disable automatic "
         "sleeping/deactivation.\n"
      << "  --disable-secondary-lcp   Disable the boxed-LCP fallback solver "
         "for measurement.\n"
      << "  --sleep-state-colors      In GUI mode, color mobile objects by "
         "sleep state.\n"
      << "  --gui-target-rtf N        Target GUI playback RTF; default 1.0.\n"
      << "  --gui-scale N             Scale the GUI panel, HUD, and initial "
         "viewer window.\n"
      << "  --gui-scale=N             Same as --gui-scale N; DART_GUI_SCALE "
         "is also honored.\n"
      << "  --gui-start               Start GUI simulation immediately; "
         "default is paused.\n"
      << "  --gui-capture PATH        Run the OSG GUI path, save a PNG, and "
         "exit.\n"
      << "  --gui-capture-steps N     Simulation steps before --gui-capture; "
         "default uses --steps.\n"
      << "  --gui-capture-exercise-widget\n"
      << "                            With --gui-capture, exercise generated "
         "scene rebuild controls before capture.\n"
      << "  --world-threads N         Worker threads for independent "
         "simulation "
         "work; 0 uses hardware concurrency.\n"
      << "  --drop-height Z           Raise mobile objects by Z meters before "
         "simulation; 0 is allowed.\n"
      << "  --sleep-linear-threshold V\n"
      << "                            Override automatic sleep linear speed "
         "threshold.\n"
      << "  --sleep-angular-threshold V\n"
      << "                            Override automatic sleep angular speed "
         "threshold.\n"
      << "  --sleep-time T            Override quiet dwell time before sleep.\n"
      << "  --sleep-contact-penetration-tolerance D\n"
      << "                            Override automatic sleep contact "
         "penetration tolerance.\n"
      << "  --contact-max-erv V       Override rigid-contact maximum error "
         "reduction velocity.\n"
      << "  --generate-objects N      Generate an in-memory 3-lane "
         "sphere/box/cylinder scene instead of loading SDF.\n"
      << "  --generate-capsules N     Generate an in-memory capsule-only "
         "scene instead of loading SDF.\n"
      << "  --generate-container N    Generate a dense mixed-shape open-box "
         "scene for active contact benchmarking;\n"
      << "                            defaults contact caps to 20000 total / "
         "4 per pair unless overridden.\n"
      << "  --generated-spacing X     Center spacing for generated objects; "
         "default 1.1 m.\n"
      << "  --container-layers N      Target vertical layers for "
         "--generate-container; default 4.\n"
      << "  --container-size X        Inner square side length for "
         "--generate-container; default fits generated grid.\n"
      << "  --container-wall-height Z Wall height for --generate-container; "
         "default fits generated layers.\n"
      << "  --container-wall-thickness X\n"
      << "                            Wall and floor thickness for "
         "--generate-container; default 0.2 m.\n"
      << "  --container-jitter X      Deterministic placement jitter for "
         "--generate-container; default 0.08 m.\n"
      << "  --container-drop-height Z Default drop height for "
         "--generate-container; same as --drop-height.\n"
      << "  --container-seed N        Deterministic seed for "
         "--generate-container; default 3056.\n"
      << "  --container-initial-speed V\n"
      << "                            Deterministic horizontal initial speed "
         "for container bodies.\n"
      << "  --container-initial-angular-speed V\n"
      << "                            Deterministic initial angular speed for "
         "container bodies.\n"
      << "  --container-linear-damping C\n"
      << "                            Joint-space linear drag for container "
         "bodies; default 0.02.\n"
      << "  --container-angular-damping C\n"
      << "                            Joint-space angular drag for container "
         "bodies (rolling decay); default 0.1.\n"
      << "  --dump-final-scene PATH   Write final collision geometry JSONL "
         "for visual verification.\n";
}

bool isHelpRequest(int argc, char* argv[])
{
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h")
      return true;
  }
  return false;
}

std::size_t parseSize(const std::string& value, const std::string& name)
{
  try {
    if (!value.empty() && (value.front() == '-' || value.front() == '+'))
      throw std::invalid_argument("signed value");

    std::size_t consumed = 0;
    const auto parsed = std::stoull(value, &consumed);
    if (consumed != value.size())
      throw std::invalid_argument("trailing characters");
    return static_cast<std::size_t>(parsed);
  } catch (const std::exception&) {
    throw std::invalid_argument("Invalid " + name + ": " + value);
  }
}

double parsePositiveDouble(const std::string& value, const std::string& name)
{
  try {
    std::size_t consumed = 0;
    const double parsed = std::stod(value, &consumed);
    if (consumed != value.size() || !std::isfinite(parsed) || parsed <= 0.0)
      throw std::invalid_argument("invalid value");
    return parsed;
  } catch (const std::exception&) {
    throw std::invalid_argument("Invalid " + name + ": " + value);
  }
}

double parseNonNegativeDouble(const std::string& value, const std::string& name)
{
  try {
    std::size_t consumed = 0;
    const double parsed = std::stod(value, &consumed);
    if (consumed != value.size() || !std::isfinite(parsed) || parsed < 0.0)
      throw std::invalid_argument("invalid value");
    return parsed;
  } catch (const std::exception&) {
    throw std::invalid_argument("Invalid " + name + ": " + value);
  }
}

CollisionEngine parseCollisionEngine(const std::string& value)
{
  if (value == "default")
    return CollisionEngine::Default;
  if (value == "dart")
    return CollisionEngine::Dart;
  if (value == "fcl")
    return CollisionEngine::Fcl;
  if (value == "bullet")
    return CollisionEngine::Bullet;
  if (value == "ode")
    return CollisionEngine::Ode;

  throw std::invalid_argument("Invalid --collision value: " + value);
}

const char* collisionEngineName(CollisionEngine engine)
{
  switch (engine) {
    case CollisionEngine::Default:
      return "default";
    case CollisionEngine::Dart:
      return "dart";
    case CollisionEngine::Fcl:
      return "fcl";
    case CollisionEngine::Bullet:
      return "bullet";
    case CollisionEngine::Ode:
      return "ode";
  }

  return "unknown";
}

BoxedLcpSolverKind parseBoxedLcpSolver(const std::string& value)
{
  if (value == "default")
    return BoxedLcpSolverKind::Default;
  if (value == "dantzig")
    return BoxedLcpSolverKind::Dantzig;
  if (value == "pgs")
    return BoxedLcpSolverKind::Pgs;

  throw std::invalid_argument("Invalid --lcp-solver value: " + value);
}

void normalizeGeneratedCapsuleCollisionOptions(Options& options)
{
  if (!options.generateCapsules)
    return;

  if (options.collisionEngine == CollisionEngine::Default)
    options.collisionEngine = CollisionEngine::Dart;

  if (options.collisionEngine == CollisionEngine::Fcl
      || options.primitiveShapes) {
    throw std::invalid_argument(
        "--generate-capsules cannot use the DART 6 FCL backend because FCL "
        "capsule geometry is not wired through this backend");
  }
}

// Match the BM_INTEGRATION_contact_container caps for every container scene
// build (CLI parsing, GUI rebuilds, and the capture-time widget exercise).
// The world default of 1000 total contacts overflows when a detector produces
// very dense per-pair manifolds (the ODE detector's mesh-based cylinder
// fallback reports 100+ contacts per pair); dropped contacts then let bodies
// tunnel out of the container.
void normalizeContainerContactCaps(Options& options)
{
  if (!options.generateContainer)
    return;

  if (!options.maxContacts.has_value())
    options.maxContacts = 20000;
  if (!options.maxContactsPerPair.has_value())
    options.maxContactsPerPair = 4;
}

Options parseOptions(int argc, char* argv[])
{
  Options options;
  if (const char* envScale = std::getenv("DART_GUI_SCALE")) {
    options.guiScale = dart::gui::osg::parseGuiScale(
        envScale, dart::gui::osg::getDefaultGuiScale(), &std::cerr);
  }

  bool consumedPositionalSteps = false;
  const std::string guiScalePrefix = "--gui-scale=";
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];

    auto needValue = [&](const std::string& name) -> std::string {
      if (i + 1 >= argc)
        throw std::invalid_argument("Missing value for " + name);
      return argv[++i];
    };

    if (arg == "--steps") {
      options.steps = parseSize(needValue(arg), arg);
    } else if (arg == "--warmup") {
      options.warmup = parseSize(needValue(arg), arg);
    } else if (arg == "--checkpoint") {
      options.checkpoint = parseSize(needValue(arg), arg);
    } else if (arg == "--quiet" || arg == "--headless") {
      options.quiet = true;
    } else if (arg == "--gui") {
      options.gui = true;
    } else if (arg == "--profile") {
      options.profile = true;
    } else if (arg == "--primitive-shapes") {
      options.primitiveShapes = true;
    } else if (arg == "--sdf-plane-shapes") {
      options.sdfPlaneShapes = true;
    } else if (arg == "--disable-deactivation") {
      options.disableDeactivation = true;
    } else if (arg == "--disable-secondary-lcp") {
      options.disableSecondaryLcp = true;
    } else if (arg == "--sleep-state-colors") {
      options.sleepStateColors = true;
    } else if (arg == "--gui-target-rtf") {
      options.guiTargetRtf = parsePositiveDouble(needValue(arg), arg);
    } else if (arg == "--gui-scale") {
      options.guiScale = dart::gui::osg::parseGuiScale(
          needValue(arg), options.guiScale, &std::cerr);
    } else if (arg.compare(0, guiScalePrefix.size(), guiScalePrefix) == 0) {
      options.guiScale = dart::gui::osg::parseGuiScale(
          arg.substr(guiScalePrefix.size()), options.guiScale, &std::cerr);
    } else if (arg == "--gui-start") {
      options.guiStart = true;
    } else if (arg == "--gui-capture") {
      options.guiCapturePath = needValue(arg);
    } else if (arg == "--gui-capture-steps") {
      options.guiCaptureSteps = parseSize(needValue(arg), arg);
    } else if (arg == "--gui-capture-exercise-widget") {
      options.guiCaptureExerciseWidget = true;
    } else if (arg == "--world-threads") {
      options.worldThreads = parseSize(needValue(arg), arg);
    } else if (arg == "--drop-height") {
      options.dropHeight = parseNonNegativeDouble(needValue(arg), arg);
      options.dropHeightSpecified = true;
    } else if (arg == "--container-drop-height") {
      options.dropHeight = parseNonNegativeDouble(needValue(arg), arg);
      options.dropHeightSpecified = true;
    } else if (arg == "--sleep-linear-threshold") {
      options.sleepLinearThreshold = parsePositiveDouble(needValue(arg), arg);
    } else if (arg == "--sleep-angular-threshold") {
      options.sleepAngularThreshold = parsePositiveDouble(needValue(arg), arg);
    } else if (arg == "--sleep-time") {
      options.sleepTimeUntilSleep = parsePositiveDouble(needValue(arg), arg);
    } else if (arg == "--sleep-contact-penetration-tolerance") {
      options.sleepContactPenetrationTolerance
          = parseNonNegativeDouble(needValue(arg), arg);
    } else if (
        arg == "--contact-max-erv"
        || arg == "--contact-max-error-reduction-velocity") {
      options.contactMaxErrorReductionVelocity
          = parseNonNegativeDouble(needValue(arg), arg);
    } else if (arg == "--generate-objects" || arg == "--num-objects") {
      options.generatedObjects = parseSize(needValue(arg), arg);
      options.generateCapsules = false;
      options.generateContainer = false;
    } else if (arg == "--generate-capsules") {
      options.generatedObjects = parseSize(needValue(arg), arg);
      options.generateCapsules = true;
      options.generateContainer = false;
    } else if (arg == "--generate-container") {
      options.generatedObjects = parseSize(needValue(arg), arg);
      options.generateCapsules = false;
      options.generateContainer = true;
    } else if (arg == "--generated-spacing") {
      options.generatedSpacing = parsePositiveDouble(needValue(arg), arg);
      options.generatedSpacingSpecified = true;
    } else if (arg == "--container-layers") {
      options.generatedContainerLayers = parseSize(needValue(arg), arg);
    } else if (arg == "--container-size") {
      options.generatedContainerSize = parsePositiveDouble(needValue(arg), arg);
    } else if (arg == "--container-wall-height") {
      options.generatedContainerWallHeight
          = parsePositiveDouble(needValue(arg), arg);
    } else if (arg == "--container-wall-thickness") {
      options.generatedContainerWallThickness
          = parsePositiveDouble(needValue(arg), arg);
    } else if (arg == "--container-jitter") {
      options.generatedContainerJitter
          = parseNonNegativeDouble(needValue(arg), arg);
    } else if (arg == "--container-seed") {
      const auto seed = parseSize(needValue(arg), arg);
      if (seed > std::numeric_limits<std::uint32_t>::max()) {
        throw std::invalid_argument("--container-seed exceeds uint32_t range");
      }
      options.generatedContainerSeed = static_cast<std::uint32_t>(seed);
    } else if (arg == "--container-initial-speed") {
      options.generatedContainerInitialSpeed
          = parseNonNegativeDouble(needValue(arg), arg);
    } else if (arg == "--container-initial-angular-speed") {
      options.generatedContainerInitialAngularSpeed
          = parseNonNegativeDouble(needValue(arg), arg);
    } else if (arg == "--container-linear-damping") {
      options.generatedContainerLinearDamping
          = parseNonNegativeDouble(needValue(arg), arg);
    } else if (arg == "--container-angular-damping") {
      options.generatedContainerAngularDamping
          = parseNonNegativeDouble(needValue(arg), arg);
    } else if (arg == "--dump-final-scene") {
      options.dumpFinalScenePath = needValue(arg);
    } else if (arg == "--max-contacts") {
      options.maxContacts = parseSize(needValue(arg), arg);
    } else if (arg == "--max-contacts-per-pair") {
      options.maxContactsPerPair = parseSize(needValue(arg), arg);
    } else if (arg == "--collision") {
      options.collisionEngine = parseCollisionEngine(needValue(arg));
    } else if (arg == "--lcp-solver") {
      options.boxedLcpSolver = parseBoxedLcpSolver(needValue(arg));
    } else if (!arg.empty() && arg.front() == '-') {
      throw std::invalid_argument("Unknown option: " + arg);
    } else if (!options.sdfPath.has_value()) {
      options.sdfPath = arg;
    } else if (!consumedPositionalSteps) {
      options.steps = parseSize(arg, "num_steps");
      consumedPositionalSteps = true;
    } else {
      throw std::invalid_argument("Unexpected positional argument: " + arg);
    }
  }

  if (options.steps == 0)
    throw std::invalid_argument("Number of simulation steps must be positive");
  if (options.guiCapturePath.has_value() && options.guiCaptureSteps == 0)
    options.guiCaptureSteps = options.steps;

  const bool usingDefaultScene
      = !options.generatedObjects.has_value() && !options.sdfPath.has_value();
  if (usingDefaultScene) {
    options.generatedObjects = kDefaultGeneratedObjects;
    options.sleepStateColors = true;
    if (!options.dropHeightSpecified)
      options.dropHeight = kDefaultDropHeight;
    if (!options.quiet && !options.guiCapturePath.has_value())
      options.gui = true;
  }

  if (options.generatedObjects.has_value() && *options.generatedObjects == 0) {
    throw std::invalid_argument("Generated object count must be positive");
  }

  if (options.generateContainer && options.generatedContainerLayers == 0) {
    throw std::invalid_argument("--container-layers must be positive");
  }

  if (options.generateContainer && !options.generatedSpacingSpecified)
    options.generatedSpacing = kDefaultGeneratedContainerSpacing;

  if (options.generateContainer && !options.dropHeightSpecified)
    options.dropHeight = kDefaultGeneratedContainerDropHeight;

  normalizeContainerContactCaps(options);

  if (options.generatedObjects.has_value() && options.sdfPath.has_value()) {
    throw std::invalid_argument(
        "Generated scenes cannot be combined with an SDF path");
  }

  if (options.sdfPlaneShapes && options.generatedObjects.has_value()) {
    throw std::invalid_argument(
        "--sdf-plane-shapes requires an SDF input scene");
  }

  normalizeGeneratedCapsuleCollisionOptions(options);

  if (options.guiCaptureExerciseWidget && !options.guiCapturePath.has_value()) {
    throw std::invalid_argument(
        "--gui-capture-exercise-widget requires --gui-capture");
  }

  if (options.primitiveShapes
      && options.collisionEngine != CollisionEngine::Default
      && options.collisionEngine != CollisionEngine::Fcl) {
    throw std::invalid_argument(
        "--primitive-shapes requires the FCL collision detector");
  }

  return options;
}

dart::collision::CollisionDetectorPtr makeCollisionDetector(
    const Options& options)
{
  const auto engine = options.collisionEngine;
  if (options.primitiveShapes && engine != CollisionEngine::Default
      && engine != CollisionEngine::Fcl) {
    throw std::runtime_error(
        "--primitive-shapes requires the FCL collision detector");
  }

  if (options.primitiveShapes && engine == CollisionEngine::Default) {
    auto detector = dart::collision::FCLCollisionDetector::create();
    detector->setPrimitiveShapeType(
        dart::collision::FCLCollisionDetector::PRIMITIVE);
    return detector;
  }

  if (engine == CollisionEngine::Dart)
    return dart::collision::DARTCollisionDetector::create();
  if (engine == CollisionEngine::Bullet)
    return dart::collision::BulletCollisionDetector::create();
  if (engine == CollisionEngine::Fcl) {
    auto detector = dart::collision::FCLCollisionDetector::create();
    if (options.primitiveShapes) {
      detector->setPrimitiveShapeType(
          dart::collision::FCLCollisionDetector::PRIMITIVE);
    }
    return detector;
  }
  if (engine == CollisionEngine::Ode)
    return dart::collision::OdeCollisionDetector::create();

  return nullptr;
}

struct GeneratedShapeSpec
{
  dart::dynamics::ShapePtr shape;
  double centerHeight = 0.5;
  Eigen::Vector4d color = Eigen::Vector4d::Ones();
  const char* namePrefix = "object";
};

void setShapeInertia(
    dart::dynamics::BodyNode* body,
    const dart::dynamics::ShapePtr& shape,
    double mass)
{
  dart::dynamics::Inertia inertia;
  inertia.setMass(mass);
  inertia.setMoment(shape->computeInertia(mass));
  body->setInertia(inertia);
}

void addStaticBox(
    dart::dynamics::BodyNode* body,
    const Eigen::Vector3d& dimensions,
    const Eigen::Vector3d& translation,
    const Eigen::Vector4d& color)
{
  auto shape = std::make_shared<dart::dynamics::BoxShape>(dimensions);
  auto* shapeNode = body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(shape);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = translation;
  shapeNode->setRelativeTransform(transform);
  shapeNode->getVisualAspect()->setColor(color);
}

GeneratedShapeSpec makeGeneratedShape(std::size_t index, bool capsulesOnly)
{
  if (capsulesOnly) {
    return {
        std::make_shared<dart::dynamics::CapsuleShape>(0.5, 1.0),
        1.0,
        Eigen::Vector4d(0.65, 0.15, 1.0, 1.0),
        "capsule"};
  }

  switch (index % 3u) {
    case 0u:
      return {
          std::make_shared<dart::dynamics::SphereShape>(0.5),
          0.5,
          Eigen::Vector4d(0.0, 0.15, 1.0, 1.0),
          "sphere"};
    case 1u:
      return {
          std::make_shared<dart::dynamics::BoxShape>(
              Eigen::Vector3d::Constant(1.0)),
          0.5,
          Eigen::Vector4d(1.0, 0.0, 0.0, 1.0),
          "box"};
    default:
      return {
          std::make_shared<dart::dynamics::CylinderShape>(0.5, 1.0),
          0.5,
          Eigen::Vector4d(0.0, 0.75, 0.0, 1.0),
          "cylinder"};
  }
}

void addGeneratedGroundPlane(
    dart::simulation::WorldPtr world,
    double floorLength,
    double floorWidth,
    double floorCenterX)
{
  auto ground = dart::dynamics::Skeleton::create("ground");
  auto* groundBody
      = ground->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(nullptr)
            .second;

  addStaticBox(
      groundBody,
      Eigen::Vector3d(floorLength, floorWidth, 0.002),
      Eigen::Vector3d(floorCenterX, 0.0, -0.001),
      Eigen::Vector4d(0.55, 0.55, 0.55, 1.0));
  ground->setMobile(false);
  world->addSkeleton(ground);
}

contact_scene::ContactContainerOptions makeContactContainerOptions(
    const Options& options)
{
  contact_scene::ContactContainerOptions contactOptions;
  contactOptions.objects = *options.generatedObjects;
  contactOptions.spacing = options.generatedSpacing;
  contactOptions.layers = options.generatedContainerLayers;
  contactOptions.containerSize = options.generatedContainerSize;
  contactOptions.wallHeight = options.generatedContainerWallHeight;
  contactOptions.containerWallThickness
      = options.generatedContainerWallThickness;
  contactOptions.jitter = options.generatedContainerJitter;
  contactOptions.dropHeight = options.dropHeight;
  contactOptions.seed = options.generatedContainerSeed;
  contactOptions.initialSpeed = options.generatedContainerInitialSpeed;
  contactOptions.initialAngularSpeed
      = options.generatedContainerInitialAngularSpeed;
  contactOptions.linearDamping = options.generatedContainerLinearDamping;
  contactOptions.angularDamping = options.generatedContainerAngularDamping;
  contactOptions.collisionDetector = makeCollisionDetector(options);
  return contactOptions;
}

dart::simulation::WorldPtr createGeneratedWorld(const Options& options)
{
  if (options.generateContainer) {
    return contact_scene::createContactContainerWorld(
        makeContactContainerOptions(options));
  }

  const std::size_t numObjects = *options.generatedObjects;
  const std::size_t lanes = options.generateCapsules ? 1u : 3u;

  auto world = dart::simulation::World::create(
      options.generateCapsules ? "generated_capsules"
                               : "generated_3lane_shapes");
  world->setTimeStep(0.001);
  if (auto detector = makeCollisionDetector(options))
    world->getConstraintSolver()->setCollisionDetector(detector);

  const std::size_t rows = (numObjects + lanes - 1u) / lanes;
  const double floorLength = std::max(
      10.0, (static_cast<double>(rows) + 2.0) * options.generatedSpacing);
  const double floorWidth = std::max(
      10.0, static_cast<double>(lanes + 1u) * options.generatedSpacing);
  const double floorCenterX = rows > 0 ? 0.5 * static_cast<double>(rows - 1u)
                                             * options.generatedSpacing
                                       : 0.0;

  addGeneratedGroundPlane(world, floorLength, floorWidth, floorCenterX);

  for (std::size_t i = 0; i < numObjects; ++i) {
    const std::size_t row = i / lanes;
    const std::size_t lane = i % lanes;
    const auto spec = makeGeneratedShape(lane, options.generateCapsules);

    auto skeleton = dart::dynamics::Skeleton::create(
        std::string(spec.namePrefix) + "_" + std::to_string(i));
    auto pair = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
        nullptr);
    auto* body = pair.second;
    auto* shapeNode = body->createShapeNodeWith<
        dart::dynamics::VisualAspect,
        dart::dynamics::CollisionAspect,
        dart::dynamics::DynamicsAspect>(spec.shape);
    shapeNode->getVisualAspect()->setColor(spec.color);
    setShapeInertia(body, spec.shape, 1.0);

    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translation() = Eigen::Vector3d(
        static_cast<double>(row) * options.generatedSpacing,
        (static_cast<double>(lane) - 0.5 * static_cast<double>(lanes - 1u))
            * options.generatedSpacing,
        spec.centerHeight);
    dart::dynamics::FreeJoint::setTransformOf(body, transform);

    world->addSkeleton(skeleton);
  }

  return world;
}

ContactStats collectContactStats(const dart::simulation::WorldPtr& world)
{
  ContactStats stats;
  const double sleepContactPenetrationTolerance = dart::constraint::
      ConstraintSolver::getAutomaticSleepingContactPenetrationTolerance();
  const auto& result = world->getLastCollisionResult();
  stats.contacts = result.getNumContacts();

  std::unordered_map<ContactPair, std::size_t, ContactPairHash> pairs;
  pairs.reserve(stats.contacts);
  for (std::size_t i = 0; i < stats.contacts; ++i) {
    const auto& contact = result.getContact(i);
    auto& count = pairs[ContactPair(
        contact.collisionObject1, contact.collisionObject2)];
    ++count;
    stats.maxPairContacts = std::max(stats.maxPairContacts, count);
    if (contact.penetrationDepth > sleepContactPenetrationTolerance)
      ++stats.contactsOverSleepTolerance;
    if (dart::collision::Contact::isZeroNormal(contact.normal))
      ++stats.zeroNormalContacts;
    stats.maxPenetration
        = std::max(stats.maxPenetration, contact.penetrationDepth);
  }
  stats.pairs = pairs.size();
  return stats;
}

SleepStats collectSleepStats(const dart::simulation::WorldPtr& world)
{
  SleepStats stats;
  stats.skeletons = world->getNumSkeletons();
  const auto& options = world->getDeactivationOptions();
  const double linearSleep = options.mLinearSpeedThreshold;
  const double angularSleep = options.mAngularSpeedThreshold;
  const double linearFinal = 0.1 * linearSleep;
  const double angularFinal = 0.2 * angularSleep;
  const double linearWake = options.mWakeThresholdScale * linearSleep;
  const double angularWake = options.mWakeThresholdScale * angularSleep;
  static thread_local std::unordered_map<int, std::size_t> islandSizes;
  islandSizes.clear();

  for (std::size_t i = 0; i < world->getNumSkeletons(); ++i) {
    const auto skel = world->getSkeleton(i);
    if (!skel || !skel->isMobile())
      continue;

    ++stats.mobile;
    if (skel->getIslandIndex() >= 0) {
      ++stats.islanded;
      ++islandSizes[skel->getIslandIndex()];
    }
    if (skel->isSleepCandidate())
      ++stats.candidates;
    if (skel->isResting())
      ++stats.resting;
    if (skel->hasExternalDisturbance())
      ++stats.disturbed;

    stats.maxLinearSpeed
        = std::max(stats.maxLinearSpeed, skel->computeMaxBodyLinearSpeed());
    stats.maxAngularSpeed
        = std::max(stats.maxAngularSpeed, skel->computeMaxBodyAngularSpeed());
    stats.maxSmoothedLinearSpeed = std::max(
        stats.maxSmoothedLinearSpeed, skel->getSmoothedLinearSpeed());
    stats.maxSmoothedAngularSpeed = std::max(
        stats.maxSmoothedAngularSpeed, skel->getSmoothedAngularSpeed());
    stats.maxRestDwellTime
        = std::max(stats.maxRestDwellTime, skel->getRestDwellTime());

    const bool belowSleep = skel->getSmoothedLinearSpeed() < linearSleep
                            && skel->getSmoothedAngularSpeed() < angularSleep;
    const bool belowFinal = skel->getSmoothedLinearSpeed() < linearFinal
                            && skel->getSmoothedAngularSpeed() < angularFinal;
    const bool belowWake = skel->getSmoothedLinearSpeed() < linearWake
                           && skel->getSmoothedAngularSpeed() < angularWake;
    const bool dwellReady = skel->getRestDwellTime() >= options.mTimeUntilSleep;
    if (belowSleep)
      ++stats.belowSleepSpeed;
    if (belowFinal)
      ++stats.belowFinalSpeed;
    if (belowWake)
      ++stats.belowWakeSpeed;
    if (dwellReady)
      ++stats.dwellReady;
    if (dwellReady && belowSleep)
      ++stats.dwellAndSleepReady;
    if (dwellReady && belowFinal)
      ++stats.dwellAndFinalReady;
    if (dwellReady && belowWake)
      ++stats.dwellAndWakeReady;
  }

  stats.islands = islandSizes.size();
  for (const auto& island : islandSizes)
    stats.maxIslandMobile = std::max(stats.maxIslandMobile, island.second);

  return stats;
}

void mixDigest(FinalStateDigest& digest, std::uint64_t value)
{
  digest.hash ^= value;
  digest.hash *= 1099511628211ULL;
}

void mixDigest(FinalStateDigest& digest, bool value)
{
  mixDigest(digest, static_cast<std::uint64_t>(value ? 1u : 0u));
}

void mixDigest(FinalStateDigest& digest, double value)
{
  if (!std::isfinite(value))
    digest.finite = false;

  std::uint64_t bits = 0;
  static_assert(sizeof(bits) == sizeof(value), "unexpected double size");
  std::memcpy(&bits, &value, sizeof(bits));
  mixDigest(digest, bits);
}

void consumeVector(FinalStateDigest& digest, const Eigen::VectorXd& values)
{
  mixDigest(digest, static_cast<std::uint64_t>(values.size()));
  for (int i = 0; i < values.size(); ++i)
    mixDigest(digest, values[i]);
}

FinalStateDigest collectFinalStateDigest(
    const dart::simulation::WorldPtr& world,
    const ContactStats& contacts,
    const SleepStats& sleep)
{
  FinalStateDigest digest;
  digest.skeletons = world->getNumSkeletons();
  digest.mobile = sleep.mobile;
  digest.worldTime = world->getTime();
  digest.simFrames = world->getSimFrames();
  mixDigest(digest, static_cast<std::uint64_t>(digest.skeletons));
  mixDigest(digest, digest.worldTime);
  mixDigest(
      digest,
      static_cast<std::uint64_t>(static_cast<std::int64_t>(digest.simFrames)));
  mixDigest(digest, static_cast<std::uint64_t>(contacts.contacts));
  mixDigest(digest, static_cast<std::uint64_t>(contacts.pairs));
  mixDigest(digest, static_cast<std::uint64_t>(contacts.maxPairContacts));
  mixDigest(digest, contacts.maxPenetration);
  mixDigest(digest, static_cast<std::uint64_t>(sleep.resting));
  mixDigest(digest, static_cast<std::uint64_t>(sleep.candidates));
  mixDigest(digest, static_cast<std::uint64_t>(sleep.islanded));

  for (std::size_t i = 0; i < world->getNumSkeletons(); ++i) {
    const auto skel = world->getSkeleton(i);
    if (!skel) {
      mixDigest(digest, false);
      continue;
    }

    mixDigest(digest, true);
    mixDigest(digest, skel->isMobile());
    mixDigest(digest, skel->isResting());
    mixDigest(digest, skel->isSleepCandidate());
    mixDigest(
        digest,
        static_cast<std::uint64_t>(
            static_cast<std::int64_t>(skel->getIslandIndex())));

    digest.dofs += skel->getNumDofs();
    digest.bodyNodes += skel->getNumBodyNodes();

    const Eigen::VectorXd positions = skel->getPositions();
    const Eigen::VectorXd velocities = skel->getVelocities();
    consumeVector(digest, positions);
    consumeVector(digest, velocities);
    digest.positionAbsSum += positions.lpNorm<1>();
    digest.velocityAbsSum += velocities.lpNorm<1>();
    if (velocities.size() > 0) {
      digest.maxAbsVelocity
          = std::max(digest.maxAbsVelocity, velocities.cwiseAbs().maxCoeff());
    }

    for (std::size_t j = 0; j < skel->getNumBodyNodes(); ++j) {
      const auto* body = skel->getBodyNode(j);
      if (!body) {
        mixDigest(digest, false);
        continue;
      }

      mixDigest(digest, true);
      const Eigen::Vector3d translation = body->getTransform().translation();
      for (int k = 0; k < translation.size(); ++k)
        mixDigest(digest, translation[k]);
      digest.bodyTranslationAbsSum += translation.lpNorm<1>();
    }
  }

  digest.finite = digest.finite && std::isfinite(digest.positionAbsSum)
                  && std::isfinite(digest.velocityAbsSum)
                  && std::isfinite(digest.bodyTranslationAbsSum)
                  && std::isfinite(digest.maxAbsVelocity)
                  && std::isfinite(digest.worldTime);
  return digest;
}

void printDiagnostics(
    std::size_t step,
    double rtf,
    const ContactStats& contacts,
    const SleepStats& sleep)
{
  std::cout << "step " << step << " rtf " << rtf << " contacts "
            << contacts.contacts << " pairs " << contacts.pairs
            << " max_pair_contacts " << contacts.maxPairContacts
            << " over_sleep_tol " << contacts.contactsOverSleepTolerance
            << " zero_normals " << contacts.zeroNormalContacts
            << " max_penetration " << contacts.maxPenetration << " mobile "
            << sleep.mobile << " islanded " << sleep.islanded << " candidates "
            << sleep.candidates << " resting " << sleep.resting << " max_lin "
            << sleep.maxLinearSpeed << " max_ang " << sleep.maxAngularSpeed
            << " max_smooth_lin " << sleep.maxSmoothedLinearSpeed
            << " max_smooth_ang " << sleep.maxSmoothedAngularSpeed
            << " max_dwell " << sleep.maxRestDwellTime << " dwell_ready "
            << sleep.dwellReady << " dwell_sleep_ready "
            << sleep.dwellAndSleepReady << " dwell_final_ready "
            << sleep.dwellAndFinalReady << " dwell_wake_ready "
            << sleep.dwellAndWakeReady << " below_sleep "
            << sleep.belowSleepSpeed << " below_final " << sleep.belowFinalSpeed
            << " below_wake " << sleep.belowWakeSpeed << " islands "
            << sleep.islands << " max_island_mobile " << sleep.maxIslandMobile
            << " disturbed " << sleep.disturbed << "\n";
}

std::string jsonString(const std::string& value)
{
  std::string out;
  out.reserve(value.size() + 2);
  out.push_back('"');
  for (const char c : value) {
    switch (c) {
      case '\\':
        out += "\\\\";
        break;
      case '"':
        out += "\\\"";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        out.push_back(c);
        break;
    }
  }
  out.push_back('"');
  return out;
}

void writeVector(std::ostream& out, const Eigen::Vector3d& values)
{
  out << '[' << values[0] << ',' << values[1] << ',' << values[2] << ']';
}

void writeQuaternion(std::ostream& out, const Eigen::Quaterniond& quat)
{
  out << '[' << quat.w() << ',' << quat.x() << ',' << quat.y() << ','
      << quat.z() << ']';
}

void writeShapeParameters(std::ostream& out, const dart::dynamics::Shape* shape)
{
  using namespace dart::dynamics;

  if (const auto* box = dynamic_cast<const BoxShape*>(shape)) {
    out << ",\"primitive\":\"box\",\"size\":";
    writeVector(out, box->getSize());
  } else if (const auto* sphere = dynamic_cast<const SphereShape*>(shape)) {
    out << ",\"primitive\":\"sphere\",\"radius\":" << sphere->getRadius();
  } else if (const auto* capsule = dynamic_cast<const CapsuleShape*>(shape)) {
    out << ",\"primitive\":\"capsule\",\"radius\":" << capsule->getRadius()
        << ",\"height\":" << capsule->getHeight();
  } else if (const auto* cylinder = dynamic_cast<const CylinderShape*>(shape)) {
    out << ",\"primitive\":\"cylinder\",\"radius\":" << cylinder->getRadius()
        << ",\"height\":" << cylinder->getHeight();
  } else if (const auto* plane = dynamic_cast<const PlaneShape*>(shape)) {
    out << ",\"primitive\":\"plane\",\"normal\":";
    writeVector(out, plane->getNormal());
    out << ",\"offset\":" << plane->getOffset();
  } else if (
      const auto* ellipsoid = dynamic_cast<const EllipsoidShape*>(shape)) {
    out << ",\"primitive\":\"ellipsoid\",\"radii\":";
    writeVector(out, ellipsoid->getRadii());
  } else {
    out << ",\"primitive\":\"unsupported\"";
  }
}

void writeSceneDump(
    const dart::simulation::WorldPtr& world, const std::string& path)
{
  const std::filesystem::path dumpPath(path);
  if (!dumpPath.parent_path().empty())
    std::filesystem::create_directories(dumpPath.parent_path());

  std::ofstream out(path);
  if (!out)
    throw std::runtime_error("Failed to open scene dump: " + path);

  out << std::setprecision(17);
  out << "{\"kind\":\"metadata\",\"world_time\":" << world->getTime()
      << ",\"sim_frames\":" << world->getSimFrames()
      << ",\"timestep\":" << world->getTimeStep()
      << ",\"skeletons\":" << world->getNumSkeletons() << "}\n";

  for (std::size_t i = 0; i < world->getNumSkeletons(); ++i) {
    const auto skel = world->getSkeleton(i);
    if (!skel)
      continue;

    for (std::size_t j = 0; j < skel->getNumBodyNodes(); ++j) {
      const auto* body = skel->getBodyNode(j);
      if (!body)
        continue;

      body->eachShapeNodeWith<dart::dynamics::CollisionAspect>(
          [&](const dart::dynamics::ShapeNode* shapeNode) {
            if (!shapeNode || !shapeNode->getShape())
              return;

            const auto shape = shapeNode->getShape();
            const Eigen::Isometry3d& transform = shapeNode->getWorldTransform();
            const Eigen::Quaterniond quat(transform.linear());
            const Eigen::Vector3d translation = transform.translation();

            out << "{\"kind\":\"shape\""
                << ",\"world_time\":" << world->getTime()
                << ",\"sim_frames\":" << world->getSimFrames()
                << ",\"skeleton\":" << jsonString(skel->getName())
                << ",\"body\":" << jsonString(body->getName())
                << ",\"shape_node\":" << jsonString(shapeNode->getName())
                << ",\"shape_type\":" << jsonString(shape->getType())
                << ",\"mobile\":" << (skel->isMobile() ? "true" : "false")
                << ",\"resting\":" << (skel->isResting() ? "true" : "false")
                << ",\"sleep_candidate\":"
                << (skel->isSleepCandidate() ? "true" : "false")
                << ",\"island_index\":" << skel->getIslandIndex()
                << ",\"position\":";
            writeVector(out, translation);
            out << ",\"quaternion\":";
            writeQuaternion(out, quat);
            writeShapeParameters(out, shape.get());
            out << "}\n";
          });
    }
  }
}

std::size_t raiseMobileObjects(
    const dart::simulation::WorldPtr& world, double height)
{
  if (height <= 0.0)
    return 0;

  std::size_t raised = 0;
  for (std::size_t i = 0; i < world->getNumSkeletons(); ++i) {
    const auto skel = world->getSkeleton(i);
    if (!skel || !skel->isMobile())
      continue;

    for (std::size_t tree = 0; tree < skel->getNumTrees(); ++tree) {
      auto* rootBody = skel->getRootBodyNode(tree);
      if (!rootBody)
        continue;

      if (dynamic_cast<dart::dynamics::FreeJoint*>(rootBody->getParentJoint())
          == nullptr) {
        throw std::runtime_error(
            "--drop-height requires mobile root bodies with FreeJoint roots");
      }

      Eigen::Isometry3d transform = rootBody->getTransform();
      transform.translation().z() += height;
      dart::dynamics::FreeJoint::setTransformOf(rootBody, transform);
      ++raised;
    }
  }

  return raised;
}

std::size_t wakeMobileObjects(const dart::simulation::WorldPtr& world)
{
  std::size_t awake = 0;
  for (std::size_t i = 0; i < world->getNumSkeletons(); ++i) {
    const auto skel = world->getSkeleton(i);
    if (!skel || !skel->isMobile())
      continue;

    skel->setResting(false);
    skel->setSleepCandidate(false);
    skel->setIslandIndex(-1);
    skel->setRestDwellTime(0.0);
    skel->setSmoothedLinearSpeed(0.0);
    skel->setSmoothedAngularSpeed(0.0);
    ++awake;
  }

  return awake;
}

void setDynamicVisualColors(
    const dart::simulation::WorldPtr& world, bool enabled)
{
  if (!world)
    return;

  for (std::size_t i = 0; i < world->getNumSkeletons(); ++i) {
    const auto skel = world->getSkeleton(i);
    if (!skel)
      continue;

    for (std::size_t j = 0; j < skel->getNumBodyNodes(); ++j) {
      auto* body = skel->getBodyNode(j);
      if (!body)
        continue;

      body->eachShapeNodeWith<dart::dynamics::VisualAspect>(
          [enabled](dart::dynamics::ShapeNode* shapeNode) {
            if (!shapeNode || !shapeNode->getShape())
              return;

            if (enabled) {
              shapeNode->getShape()->addDataVariance(
                  dart::dynamics::Shape::DYNAMIC_COLOR);
            } else {
              shapeNode->getShape()->removeDataVariance(
                  dart::dynamics::Shape::DYNAMIC_COLOR);
            }
          });
    }
  }
}

void enableDynamicVisualColors(const dart::simulation::WorldPtr& world)
{
  setDynamicVisualColors(world, true);
}

void applySleepStateVisualColors(const dart::simulation::WorldPtr& world)
{
  if (!world)
    return;

  const Eigen::Vector4d staticColor(0.55, 0.55, 0.55, 1.0);
  const Eigen::Vector4d awakeColor(1.0, 0.1, 0.05, 1.0);
  const Eigen::Vector4d candidateColor(1.0, 0.85, 0.05, 1.0);
  const Eigen::Vector4d restingColor(0.05, 0.35, 1.0, 1.0);

  for (std::size_t i = 0; i < world->getNumSkeletons(); ++i) {
    const auto skel = world->getSkeleton(i);
    if (!skel)
      continue;

    const Eigen::Vector4d* color = &awakeColor;
    if (!skel->isMobile())
      color = &staticColor;
    else if (skel->isResting())
      color = &restingColor;
    else if (skel->isSleepCandidate())
      color = &candidateColor;

    skel->eachBodyNode([&](dart::dynamics::BodyNode* bodyNode) {
      if (!bodyNode)
        return;

      bodyNode->eachShapeNodeWith<dart::dynamics::VisualAspect>(
          [&](dart::dynamics::ShapeNode* shapeNode) {
            auto* visual = shapeNode->getVisualAspect();
            if (!visual || visual->getRGBA().isApprox(*color, 0.0))
              return;

            visual->setColor(*color);
          });
    });
  }
}

std::size_t replaceGuiPlaneVisualsWithFloorBoxes(
    const dart::simulation::WorldPtr& world)
{
  if (!world)
    return 0;

  bool hasMobileBody = false;
  double minX = 0.0;
  double maxX = 0.0;
  double minY = 0.0;
  double maxY = 0.0;
  for (std::size_t i = 0; i < world->getNumSkeletons(); ++i) {
    const auto skel = world->getSkeleton(i);
    if (!skel || !skel->isMobile())
      continue;

    for (std::size_t j = 0; j < skel->getNumBodyNodes(); ++j) {
      const auto* body = skel->getBodyNode(j);
      if (!body)
        continue;

      const Eigen::Vector3d translation = body->getTransform().translation();
      if (!hasMobileBody) {
        minX = maxX = translation.x();
        minY = maxY = translation.y();
        hasMobileBody = true;
      } else {
        minX = std::min(minX, translation.x());
        maxX = std::max(maxX, translation.x());
        minY = std::min(minY, translation.y());
        maxY = std::max(maxY, translation.y());
      }
    }
  }

  const double margin = 10.0;
  const double floorX
      = hasMobileBody ? std::max(10.0, maxX - minX + margin) : 100.0;
  const double floorY
      = hasMobileBody ? std::max(10.0, maxY - minY + margin) : 100.0;
  const double centerX = hasMobileBody ? 0.5 * (minX + maxX) : 0.0;
  const double centerY = hasMobileBody ? 0.5 * (minY + maxY) : 0.0;

  std::size_t replaced = 0;
  for (std::size_t i = 0; i < world->getNumSkeletons(); ++i) {
    const auto skel = world->getSkeleton(i);
    if (!skel)
      continue;

    for (std::size_t j = 0; j < skel->getNumBodyNodes(); ++j) {
      auto* body = skel->getBodyNode(j);
      if (!body)
        continue;

      body->eachShapeNodeWith<dart::dynamics::VisualAspect>(
          [&](dart::dynamics::ShapeNode* shapeNode) {
            if (!shapeNode || shapeNode->has<dart::dynamics::CollisionAspect>())
              return;

            if (!std::dynamic_pointer_cast<dart::dynamics::PlaneShape>(
                    shapeNode->getShape())) {
              return;
            }

            shapeNode->setShape(std::make_shared<dart::dynamics::BoxShape>(
                Eigen::Vector3d(floorX, floorY, 0.002)));
            Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
            transform.translation() = Eigen::Vector3d(centerX, centerY, -0.001);
            shapeNode->setRelativeTransform(transform);
            ++replaced;
          });
    }
  }

  return replaced;
}

void applyOptions(
    const dart::simulation::WorldPtr& world, const Options& options)
{
  world->setNumSimulationThreads(options.worldThreads);

  auto deactivation = world->getDeactivationOptions();
  deactivation.mEnabled = !options.disableDeactivation;
  if (options.sleepLinearThreshold.has_value())
    deactivation.mLinearSpeedThreshold = *options.sleepLinearThreshold;
  if (options.sleepAngularThreshold.has_value())
    deactivation.mAngularSpeedThreshold = *options.sleepAngularThreshold;
  if (options.sleepTimeUntilSleep.has_value())
    deactivation.mTimeUntilSleep = *options.sleepTimeUntilSleep;
  world->setDeactivationOptions(deactivation);

  if (options.contactMaxErrorReductionVelocity.has_value()) {
    dart::constraint::ContactConstraint::setMaxErrorReductionVelocity(
        *options.contactMaxErrorReductionVelocity);
  }
  if (options.sleepContactPenetrationTolerance.has_value()) {
    dart::constraint::ConstraintSolver::
        setAutomaticSleepingContactPenetrationTolerance(
            *options.sleepContactPenetrationTolerance);
  }

  if (options.boxedLcpSolver != BoxedLcpSolverKind::Default) {
    auto* boxedSolver
        = dynamic_cast<dart::constraint::BoxedLcpConstraintSolver*>(
            world->getConstraintSolver());
    if (boxedSolver == nullptr) {
      throw std::runtime_error(
          "--lcp-solver requires BoxedLcpConstraintSolver");
    }

    if (options.boxedLcpSolver == BoxedLcpSolverKind::Dantzig) {
      boxedSolver->setBoxedLcpSolver(
          std::make_shared<dart::constraint::DantzigBoxedLcpSolver>());
    } else if (options.boxedLcpSolver == BoxedLcpSolverKind::Pgs) {
      boxedSolver->setBoxedLcpSolver(
          std::make_shared<dart::constraint::PgsBoxedLcpSolver>());
    }
  }

  if (options.disableSecondaryLcp) {
    auto* boxedSolver
        = dynamic_cast<dart::constraint::BoxedLcpConstraintSolver*>(
            world->getConstraintSolver());
    if (boxedSolver == nullptr) {
      throw std::runtime_error(
          "--disable-secondary-lcp requires BoxedLcpConstraintSolver");
    }
    boxedSolver->setSecondaryBoxedLcpSolver(nullptr);
  }

  if (!options.generatedObjects.has_value()) {
    if (auto detector = makeCollisionDetector(options)) {
      world->getConstraintSolver()->setCollisionDetector(detector);
    }
  }

  if (options.maxContacts.has_value()) {
    world->getConstraintSolver()->getCollisionOption().maxNumContacts
        = *options.maxContacts;
  }

  if (options.maxContactsPerPair.has_value()) {
    world->getConstraintSolver()->getCollisionOption().maxNumContactsPerPair
        = *options.maxContactsPerPair;
  }
}

void printWorldSummary(
    const dart::simulation::WorldPtr& world, const Options& options)
{
  std::cout << "World loaded successfully.\n";
  std::cout << "  World Name: " << world->getName() << "\n";
  std::cout << "  Gravity: [" << world->getGravity().transpose() << "] m/s^2\n";
  std::cout << "  Time Step: " << world->getTimeStep() << " s\n";
  std::cout << "  Number of Skeletons (Objects): " << world->getNumSkeletons()
            << "\n";
  std::cout << "  Collision max contacts: "
            << world->getConstraintSolver()->getCollisionOption().maxNumContacts
            << "\n";
  std::cout << "  Collision max contacts per pair: "
            << world->getConstraintSolver()
                   ->getCollisionOption()
                   .maxNumContactsPerPair
            << "\n";
  std::cout << "  Deactivation enabled: "
            << (world->getDeactivationOptions().mEnabled ? "true" : "false")
            << "\n";
  std::cout << "  Sleep linear threshold: "
            << world->getDeactivationOptions().mLinearSpeedThreshold
            << " m/s\n";
  std::cout << "  Sleep angular threshold: "
            << world->getDeactivationOptions().mAngularSpeedThreshold
            << " rad/s\n";
  std::cout << "  Sleep quiet time: "
            << world->getDeactivationOptions().mTimeUntilSleep << " s\n";
  std::cout
      << "  Contact max ERV: "
      << dart::constraint::ContactConstraint::getMaxErrorReductionVelocity()
      << " m/s\n";
  std::cout << "  Sleep contact penetration tolerance: "
            << dart::constraint::ConstraintSolver::
                   getAutomaticSleepingContactPenetrationTolerance()
            << " m\n";
  std::cout << "  World simulation threads: "
            << world->getNumSimulationThreads() << "\n";
  std::cout << "  Collision detector requested: "
            << collisionEngineName(options.collisionEngine) << "\n";
  if (options.sdfPath.has_value()) {
    std::cout << "  SDF plane shapes: "
              << (options.sdfPlaneShapes ? "true" : "false") << "\n";
  }
  std::cout << "  Physics pipeline: DART 6 dynamics, constraints, and solver";
  if (options.collisionEngine != CollisionEngine::Default
      || options.primitiveShapes) {
    std::cout << " with only collision detection delegated";
  }
  std::cout << "\n";
  std::cout << "  FCL primitive shapes: "
            << (options.primitiveShapes ? "true" : "false") << "\n";
  if (options.generatedObjects.has_value()) {
    std::cout << "  Generated mobile objects: " << *options.generatedObjects
              << "\n";
    std::cout << "  Generated scene: "
              << (options.generateContainer  ? "contact container"
                  : options.generateCapsules ? "capsule-only"
                                             : "sphere/box/cylinder")
              << "\n";
    std::cout << "  Generated object spacing: " << options.generatedSpacing
              << " m\n";
    if (options.generateContainer) {
      std::cout << "  Contact container layers: "
                << options.generatedContainerLayers << "\n";
      if (options.generatedContainerSize.has_value()) {
        std::cout << "  Contact container size: "
                  << *options.generatedContainerSize << " m\n";
      }
      if (options.generatedContainerWallHeight.has_value()) {
        std::cout << "  Contact container wall height: "
                  << *options.generatedContainerWallHeight << " m\n";
      }
      std::cout << "  Contact container wall thickness: "
                << options.generatedContainerWallThickness << " m\n";
      std::cout << "  Contact container jitter: "
                << options.generatedContainerJitter << " m\n";
      std::cout << "  Contact container seed: "
                << options.generatedContainerSeed << "\n";
      std::cout << "  Contact container damping: linear "
                << options.generatedContainerLinearDamping << ", angular "
                << options.generatedContainerAngularDamping << "\n";
      if (options.generatedContainerInitialSpeed > 0.0) {
        std::cout << "  Contact container initial speed: "
                  << options.generatedContainerInitialSpeed << " m/s\n";
      }
      if (options.generatedContainerInitialAngularSpeed > 0.0) {
        std::cout << "  Contact container initial angular speed: "
                  << options.generatedContainerInitialAngularSpeed
                  << " rad/s\n";
      }
    }
  }
  if (options.dropHeight > 0.0 || options.dropHeightSpecified)
    std::cout << "  Initial drop height: " << options.dropHeight << " m\n";

  std::size_t totalDofs = 0;
  std::size_t totalBodyNodes = 0;
  for (std::size_t i = 0; i < world->getNumSkeletons(); ++i) {
    const auto skel = world->getSkeleton(i);
    if (skel) {
      totalDofs += skel->getNumDofs();
      totalBodyNodes += skel->getNumBodyNodes();
    }
  }
  std::cout << "  Total Bodies: " << totalBodyNodes << "\n";
  std::cout << "  Total Degrees of Freedom (DOFs): " << totalDofs << "\n";
}

void applyCameraPose(
    dart::gui::osg::Viewer& viewer, const dart::simulation::WorldPtr& world)
{
  dart::gui::osg::applyDefaultCameraPose(viewer, world);
}

class ContactBenchmarkGuiWorldNode final
  : public dart::gui::osg::RealTimeWorldNode
{
public:
  ContactBenchmarkGuiWorldNode(
      const dart::simulation::WorldPtr& world, const Options& options)
    : dart::gui::osg::RealTimeWorldNode(
        world, nullptr, 60.0, options.guiTargetRtf),
      mInitialStates(captureInitialStates(world)),
      mOptions(options),
      mSleepStateColors(options.sleepStateColors)
  {
    if (mSleepStateColors)
      enableDynamicVisualColors(world);

    refreshVisualAids();
  }

  double getSmoothedContacts() const
  {
    return mSmoothedContacts;
  }

  double getSmoothedPairs() const
  {
    return mSmoothedPairs;
  }

  const Options& getOptions() const
  {
    return mOptions;
  }

  bool getSleepStateColors() const
  {
    return mSleepStateColors;
  }

  void setSleepStateColors(bool enabled)
  {
    mSleepStateColors = enabled;
    mOptions.sleepStateColors = enabled;
    if (enabled)
      enableDynamicVisualColors(getWorld());
    refreshVisualAids();
  }

  void setGuiTargetRtf(double targetRtf)
  {
    setTargetRealTimeFactor(targetRtf);
    mOptions.guiTargetRtf = targetRtf;
  }

  void replaceWorld(
      const dart::simulation::WorldPtr& world, const Options& options)
  {
    simulate(false);
    setWorld(world);
    mInitialStates = captureInitialStates(world);
    mOptions = options;
    mSleepStateColors = options.sleepStateColors;
    if (mSleepStateColors)
      enableDynamicVisualColors(world);

    resetTimingStats();
    clearChildUtilizationFlags();
    refreshSkeletons();
    refreshSimpleFrames();
    clearUnusedNodes();
    dirtyBound();
    refreshVisualAids();
  }

  void customPostStep() override
  {
    refreshVisualAids();
  }

  void customPostRefresh() override
  {
    refreshVisualAids();
    updateSmoothedStats();
  }

  void resetWorld()
  {
    const auto world = getWorld();
    if (!world)
      return;

    if (!restoreInitialState(world)) {
      std::cerr << "Failed to reset GUI world: structure changed.\n";
      return;
    }

    world->reset();
    resetTimingStats();

    refreshSkeletons();
    refreshVisualAids();
    std::cout
        << "GUI reset in-place to t=0/drop state; mobile objects awake.\n";
  }

private:
  struct SkeletonInitialState
  {
    dart::dynamics::Skeleton* skeleton = nullptr;
    Eigen::VectorXd positions;
    Eigen::VectorXd velocities;
    Eigen::VectorXd accelerations;
    Eigen::VectorXd forces;
  };

  static std::vector<SkeletonInitialState> captureInitialStates(
      const dart::simulation::WorldPtr& world)
  {
    std::vector<SkeletonInitialState> states;
    if (!world)
      return states;

    states.reserve(world->getNumSkeletons());
    for (std::size_t i = 0; i < world->getNumSkeletons(); ++i) {
      const auto skel = world->getSkeleton(i);
      if (!skel)
        continue;

      states.push_back(SkeletonInitialState{
          skel.get(),
          skel->getPositions(),
          skel->getVelocities(),
          skel->getAccelerations(),
          skel->getForces()});
    }

    return states;
  }

  bool restoreInitialState(const dart::simulation::WorldPtr& world)
  {
    if (world->getNumSkeletons() != mInitialStates.size())
      return false;

    for (std::size_t i = 0; i < mInitialStates.size(); ++i) {
      const auto skel = world->getSkeleton(i);
      if (!skel || skel.get() != mInitialStates[i].skeleton)
        return false;

      skel->setPositions(mInitialStates[i].positions);
      skel->setVelocities(mInitialStates[i].velocities);
      skel->setAccelerations(mInitialStates[i].accelerations);
      skel->setForces(mInitialStates[i].forces);
      skel->clearExternalForces();
      skel->clearInternalForces();
      skel->resetCommands();
      skel->clearConstraintImpulses();
      skel->setImpulseApplied(false);
      if (skel->isMobile()) {
        skel->setResting(false);
        skel->setSleepCandidate(false);
        skel->setIslandIndex(-1);
        skel->setRestDwellTime(0.0);
        skel->setSmoothedLinearSpeed(0.0);
        skel->setSmoothedAngularSpeed(0.0);
      }
    }

    return true;
  }

  void resetTimingStats()
  {
    mFirstRefresh = true;
    mSimTimeBudget = 0.0;
    mLastRealTimeFactor = 0.0;
    mLowestRealTimeFactor = std::numeric_limits<double>::infinity();
    mHighestRealTimeFactor = 0.0;
    mRefreshTimer.setStartTick();
    mSmoothedRealTimeFactor = 0.0;
    mHasSmoothedRealTimeFactor = false;
    mHasSmoothedStats = false;
    mSmoothedContacts = 0.0;
    mSmoothedPairs = 0.0;
  }

  void refreshVisualAids()
  {
    if (mSleepStateColors)
      applySleepStateColors();
  }

  void applySleepStateColors()
  {
    applySleepStateVisualColors(getWorld());
  }

  // Blend the fast-changing per-refresh stats with an exponential moving
  // average (about one second of rendered frames) so the status panel stays
  // readable. Updated once per rendered frame while simulating.
  void updateSmoothedStats()
  {
    if (!isSimulating())
      return;

    const auto world = getWorld();
    if (!world)
      return;

    const auto contacts = collectContactStats(world);
    const double alpha = 0.02;
    const auto blend = [&](double smoothed, double instant) {
      return mHasSmoothedStats ? (1.0 - alpha) * smoothed + alpha * instant
                               : instant;
    };
    mSmoothedContacts
        = blend(mSmoothedContacts, static_cast<double>(contacts.contacts));
    mSmoothedPairs = blend(mSmoothedPairs, static_cast<double>(contacts.pairs));
    mHasSmoothedStats = true;
  }

  std::vector<SkeletonInitialState> mInitialStates;
  Options mOptions;
  bool mSleepStateColors = false;
  bool mHasSmoothedStats = false;
  double mSmoothedContacts = 0.0;
  double mSmoothedPairs = 0.0;
};

int collisionEngineToIndex(CollisionEngine engine)
{
  switch (engine) {
    case CollisionEngine::Default:
      return 0;
    case CollisionEngine::Dart:
      return 1;
    case CollisionEngine::Fcl:
      return 2;
    case CollisionEngine::Bullet:
      return 3;
    case CollisionEngine::Ode:
      return 4;
  }

  return 0;
}

CollisionEngine collisionEngineFromIndex(int index)
{
  switch (index) {
    case 1:
      return CollisionEngine::Dart;
    case 2:
      return CollisionEngine::Fcl;
    case 3:
      return CollisionEngine::Bullet;
    case 4:
      return CollisionEngine::Ode;
    default:
      return CollisionEngine::Default;
  }
}

bool collisionEngineAllowsPrimitiveShapes(int index)
{
  return index == 0 || index == 2;
}

class ContactBenchmarkGuiEventHandler final : public ::osgGA::GUIEventHandler
{
public:
  explicit ContactBenchmarkGuiEventHandler(ContactBenchmarkGuiWorldNode* node)
    : mNode(node)
  {
  }

  void setNode(ContactBenchmarkGuiWorldNode* node)
  {
    mNode = node;
  }

  bool handle(
      const ::osgGA::GUIEventAdapter& ea, ::osgGA::GUIActionAdapter&) override
  {
    if (ea.getEventType() != ::osgGA::GUIEventAdapter::KEYDOWN || !mNode)
      return false;

    switch (ea.getUnmodifiedKey()) {
      case ::osgGA::GUIEventAdapter::KEY_R:
      case 'R':
        mNode->resetWorld();
        return true;
      default:
        return false;
    }
  }

private:
  ContactBenchmarkGuiWorldNode* mNode;
};

class ContactBenchmarkGuiWidget final : public dart::gui::osg::ImGuiWidget
{
public:
  ContactBenchmarkGuiWidget(
      dart::gui::osg::ImGuiViewer* viewer,
      ContactBenchmarkGuiWorldNode* node,
      ContactBenchmarkGuiEventHandler* eventHandler,
      double guiScale)
    : mViewer(viewer),
      mNode(node),
      mEventHandler(eventHandler),
      mGuiScale(static_cast<float>(dart::gui::osg::sanitizeGuiScale(guiScale)))
  {
    syncFromNode();
  }

  void render() override
  {
    if (!mViewer || !mNode)
      return;

    const ImGuiIO& io = ImGui::GetIO();
    const float margin = 12.0f * mGuiScale;
    ImGui::SetNextWindowPos(ImVec2(margin, margin), ImGuiCond_FirstUseEver);
    // Auto-size the window to its content so no label is clipped and no
    // scrolling or manual resize is needed; only cap it to the display.
    ImGui::SetNextWindowSizeConstraints(
        ImVec2(0.0f, 0.0f),
        ImVec2(
            std::max(100.0f, io.DisplaySize.x - 2.0f * margin),
            std::max(100.0f, io.DisplaySize.y - 2.0f * margin)));
    ImGui::SetNextWindowBgAlpha(0.94f);

    if (!ImGui::Begin(
            "Contact Benchmark", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
      ImGui::End();
      return;
    }

    ImGui::PushItemWidth(200.0f * mGuiScale);
    renderStatus();
    renderSimulationControls();
    renderSceneControls();

    if (!mStatusMessage.empty()) {
      ImGui::Separator();
      ImGui::TextWrapped("%s", mStatusMessage.c_str());
    }
    ImGui::PopItemWidth();

    ImGui::End();
  }

  bool exerciseSceneRebuildForTest(std::string* error)
  {
    if (!mNode || !mNode->getOptions().generatedObjects.has_value()) {
      if (error)
        *error = "generated scene is required";
      return false;
    }

    const auto original = mNode->getOptions();
    Options changed = original;
    changed.generatedObjects = std::min<std::size_t>(
        original.generatedObjects.value_or(kDefaultGeneratedObjects) + 3u,
        2u * kDefaultGeneratedObjects);
    changed.generatedSpacing = original.generatedSpacing + 0.05;
    changed.dropHeight
        = std::max(original.dropHeight, kDefaultGeneratedContainerDropHeight);

    if (!replaceGeneratedScene(changed, error))
      return false;

    changed.generateContainer = true;
    changed.generateCapsules = false;
    changed.generatedContainerLayers
        = std::max<std::size_t>(2u, changed.generatedContainerLayers + 1u);
    changed.generatedContainerJitter
        = std::min(0.25, changed.generatedContainerJitter + 0.04);
    changed.generatedContainerInitialSpeed = 0.35;
    changed.generatedContainerInitialAngularSpeed = 1.0;
    if (!replaceGeneratedScene(changed, error))
      return false;

    changed.dropHeight = 0.0;
    changed.dropHeightSpecified = true;
    changed.generatedContainerInitialSpeed = 0.0;
    changed.generatedContainerInitialAngularSpeed = 0.0;
    const bool rebuilt = replaceGeneratedScene(changed, error);
    if (rebuilt) {
      syncFromNode();
      mPendingSceneDirty = false;
    }
    return rebuilt;
  }

  ContactBenchmarkGuiWorldNode* getNode() const
  {
    return mNode;
  }

private:
  void syncFromNode()
  {
    if (!mNode)
      return;

    const Options& options = mNode->getOptions();
    mGeneratedObjects = static_cast<int>(
        options.generatedObjects.value_or(kDefaultGeneratedObjects));
    mGeneratedSpacing = static_cast<float>(options.generatedSpacing);
    mGenerateContainer = options.generateContainer;
    mGenerateCapsules = options.generateCapsules;
    mContainerLayers = static_cast<int>(options.generatedContainerLayers);
    mContainerJitter = static_cast<float>(options.generatedContainerJitter);
    mContainerInitialSpeed
        = static_cast<float>(options.generatedContainerInitialSpeed);
    mContainerInitialAngularSpeed
        = static_cast<float>(options.generatedContainerInitialAngularSpeed);
    mDropHeight = static_cast<float>(options.dropHeight);
    mTargetRtf = static_cast<float>(options.guiTargetRtf);
    mSleepStateColors = options.sleepStateColors;
    mPrimitiveShapes = options.primitiveShapes;
    mCollisionIndex = collisionEngineToIndex(options.collisionEngine);
    if (!collisionEngineAllowsPrimitiveShapes(mCollisionIndex))
      mPrimitiveShapes = false;
  }

  Options makePendingOptions() const
  {
    Options options = mNode->getOptions();
    options.sdfPath.reset();
    options.generatedObjects
        = static_cast<std::size_t>(std::max(1, mGeneratedObjects));
    options.generateContainer = mGenerateContainer;
    options.generateCapsules = mGenerateCapsules && !mGenerateContainer;
    if (mGenerateContainer) {
      options.generateCapsules = false;
      options.generatedContainerJitter
          = std::max(0.0, static_cast<double>(mContainerJitter));
      options.generatedContainerInitialSpeed
          = std::max(0.0, static_cast<double>(mContainerInitialSpeed));
      options.generatedContainerInitialAngularSpeed
          = std::max(0.0, static_cast<double>(mContainerInitialAngularSpeed));
    }
    options.generatedContainerLayers
        = static_cast<std::size_t>(std::max(1, mContainerLayers));
    options.generatedSpacing
        = std::max(0.05, static_cast<double>(mGeneratedSpacing));
    options.dropHeight = std::max(0.0, static_cast<double>(mDropHeight));
    options.dropHeightSpecified = true;
    options.generatedSpacingSpecified = true;
    options.guiTargetRtf = std::max(0.05, static_cast<double>(mTargetRtf));
    options.sleepStateColors = mSleepStateColors;
    options.collisionEngine = collisionEngineFromIndex(mCollisionIndex);
    options.primitiveShapes
        = mPrimitiveShapes
          && collisionEngineAllowsPrimitiveShapes(mCollisionIndex);
    return options;
  }

  bool replaceGeneratedScene(Options options, std::string* error)
  {
    if (!mViewer || !mNode || !mEventHandler) {
      if (error)
        *error = "GUI viewer is not available";
      return false;
    }

    try {
      normalizeGeneratedCapsuleCollisionOptions(options);
      normalizeContainerContactCaps(options);
      auto newWorld = createGeneratedWorld(options);
      applyOptions(newWorld, options);
      if (options.dropHeight > 0.0 && !options.generateContainer)
        raiseMobileObjects(newWorld, options.dropHeight);
      wakeMobileObjects(newWorld);
      replaceGuiPlaneVisualsWithFloorBoxes(newWorld);
      if (options.sleepStateColors)
        enableDynamicVisualColors(newWorld);

      const bool wasSimulating = mViewer->isSimulating();
      mViewer->simulate(false);
      mNode->replaceWorld(newWorld, options);
      mEventHandler->setNode(mNode);
      applyCameraPose(*mViewer, newWorld);
      if (wasSimulating)
        mViewer->simulate(true);
    } catch (const std::exception& e) {
      if (error)
        *error = e.what();
      return false;
    }

    if (error)
      error->clear();
    return true;
  }

  void renderStatus()
  {
    const auto world = mNode->getWorld();
    const auto sleep = world ? collectSleepStats(world) : SleepStats();

    if (ImGui::CollapsingHeader("Status", ImGuiTreeNodeFlags_DefaultOpen)) {
      ImGui::Text(
          "Time %.3f s   Frame %d",
          world ? world->getTime() : 0.0,
          world ? world->getSimFrames() : 0);
      const double lowestRtf = std::isfinite(mNode->getLowestRealTimeFactor())
                                   ? mNode->getLowestRealTimeFactor()
                                   : 0.0;
      ImGui::Text(
          "RTF %.2f / target %.2f   (lo %.2f, hi %.2f)",
          mNode->getSmoothedRealTimeFactor(),
          mNode->getTargetRealTimeFactor(),
          lowestRtf,
          mNode->getHighestRealTimeFactor());
      ImGui::Text(
          "Contacts %.0f   Pairs %.0f",
          mNode->getSmoothedContacts(),
          mNode->getSmoothedPairs());
      ImGui::Text(
          "Resting %zu / %zu   Candidates %zu   Disturbed %zu",
          sleep.resting,
          sleep.mobile,
          sleep.candidates,
          sleep.disturbed);
      ImGui::Text(
          "Smoothed max v %.3f m/s   w %.3f rad/s",
          sleep.maxSmoothedLinearSpeed,
          sleep.maxSmoothedAngularSpeed);
      ImGui::Text("Rest dwell %.2f s", sleep.maxRestDwellTime);

      const Options& options = mNode->getOptions();
      std::ostringstream pipeline;
      pipeline << "Pipeline DART 6, collision detector "
               << collisionEngineName(options.collisionEngine);
      if (options.generatedObjects.has_value()) {
        pipeline << ", generated "
                 << (options.generateContainer  ? "container "
                     : options.generateCapsules ? "capsules "
                                                : "objects ")
                 << *options.generatedObjects;
      }
      if (options.dropHeight > 0.0) {
        pipeline << ", drop height " << std::fixed << std::setprecision(2)
                 << options.dropHeight << " m";
      }
      ImGui::TextWrapped("%s", pipeline.str().c_str());
      ImGui::TextWrapped("Key R resets to the t=0/drop state.");
      if (mNode->getSleepStateColors()) {
        ImGui::TextWrapped(
            "Colors: awake red, candidate yellow, resting blue, static gray.");
      }
    }
  }

  void renderSimulationControls()
  {
    if (ImGui::CollapsingHeader("Simulation", ImGuiTreeNodeFlags_DefaultOpen)) {
      bool simulating = mViewer->isSimulating();
      if (ImGui::Checkbox("Run simulation", &simulating))
        mViewer->simulate(simulating);

      ImGui::SameLine();
      if (ImGui::Button("Reset"))
        mNode->resetWorld();

      if (ImGui::SliderFloat("Target RTF", &mTargetRtf, 0.05f, 4.0f, "%.2f"))
        mNode->setGuiTargetRtf(std::max(0.05, static_cast<double>(mTargetRtf)));

      if (ImGui::Checkbox("Sleep-state colors", &mSleepStateColors))
        mNode->setSleepStateColors(mSleepStateColors);

      if (ImGui::Button("Recenter camera"))
        applyCameraPose(*mViewer, mNode->getWorld());
    }
  }

  void renderSceneControls()
  {
    if (!ImGui::CollapsingHeader(
            "Generated Scene", ImGuiTreeNodeFlags_DefaultOpen))
      return;

    if (!mNode->getOptions().generatedObjects.has_value()) {
      ImGui::TextWrapped(
          "Loaded SDF scenes cannot be rebuilt from this panel.");
      return;
    }

    bool edited = false;
    auto noteEdit = [&](bool changed) {
      edited = edited || changed;
      mPendingSceneDirty = mPendingSceneDirty || changed;
    };

    mGeneratedObjects = std::clamp(mGeneratedObjects, 1, 10000);
    noteEdit(ImGui::SliderInt("Objects", &mGeneratedObjects, 3, 300));
    noteEdit(ImGui::InputInt("Object count", &mGeneratedObjects, 3, 30));
    mGeneratedObjects = std::clamp(mGeneratedObjects, 1, 10000);

    noteEdit(ImGui::SliderFloat(
        "Spacing", &mGeneratedSpacing, 0.6f, 3.0f, "%.2f m"));
    noteEdit(ImGui::Checkbox("Contact container", &mGenerateContainer));
    if (!mGenerateContainer) {
      noteEdit(ImGui::Checkbox("Capsule-only", &mGenerateCapsules));
    } else {
      mGenerateCapsules = false;
    }
    if (mGenerateContainer) {
      noteEdit(ImGui::InputInt("Container layers", &mContainerLayers, 1, 2));
      mContainerLayers = std::clamp(mContainerLayers, 1, 1000);
      noteEdit(ImGui::SliderFloat(
          "Container jitter", &mContainerJitter, 0.0f, 0.35f, "%.2f m"));
      noteEdit(ImGui::SliderFloat(
          "Initial speed", &mContainerInitialSpeed, 0.0f, 4.0f, "%.2f m/s"));
      noteEdit(ImGui::SliderFloat(
          "Initial spin",
          &mContainerInitialAngularSpeed,
          0.0f,
          8.0f,
          "%.2f rad/s"));
    }
    noteEdit(
        ImGui::SliderFloat("Drop height", &mDropHeight, 0.0f, 4.0f, "%.2f m"));

    static const char* kCollisionItems[]
        = {"default", "dart", "fcl", "bullet", "ode"};
    if (ImGui::Combo("Collision", &mCollisionIndex, kCollisionItems, 5)) {
      if (!collisionEngineAllowsPrimitiveShapes(mCollisionIndex))
        mPrimitiveShapes = false;
      noteEdit(true);
    }

    if (!collisionEngineAllowsPrimitiveShapes(mCollisionIndex))
      ImGui::BeginDisabled();
    noteEdit(ImGui::Checkbox("FCL primitive shapes", &mPrimitiveShapes));
    if (!collisionEngineAllowsPrimitiveShapes(mCollisionIndex)) {
      mPrimitiveShapes = false;
      ImGui::EndDisabled();
    }

    ImGui::Checkbox("Auto-apply edits", &mAutoApplySceneEdits);

    const bool canAutoApply = mAutoApplySceneEdits && mPendingSceneDirty
                              && !ImGui::IsAnyItemActive();
    if (canAutoApply) {
      applyPendingScene("Scene rebuilt from live edit.");
    }

    if (mPendingSceneDirty)
      ImGui::SameLine();
    if (ImGui::Button("Apply scene"))
      applyPendingScene("Scene rebuilt.");
    else if (edited && !mAutoApplySceneEdits)
      mStatusMessage = mPendingSceneDirty ? "Scene edits pending." : "";
  }

  void applyPendingScene(const char* successMessage)
  {
    std::string error;
    Options pending = makePendingOptions();
    if (replaceGeneratedScene(pending, &error)) {
      syncFromNode();
      mPendingSceneDirty = false;
      mStatusMessage = successMessage;
    } else {
      mPendingSceneDirty = false;
      mStatusMessage = "Scene rebuild failed: " + error;
    }
  }

  dart::gui::osg::ImGuiViewer* mViewer = nullptr;
  ContactBenchmarkGuiWorldNode* mNode = nullptr;
  ContactBenchmarkGuiEventHandler* mEventHandler = nullptr;
  int mGeneratedObjects = static_cast<int>(kDefaultGeneratedObjects);
  float mGeneratedSpacing = static_cast<float>(kDefaultGeneratedSpacing);
  bool mGenerateContainer = false;
  bool mGenerateCapsules = false;
  int mContainerLayers = static_cast<int>(kDefaultGeneratedContainerLayers);
  float mContainerJitter = static_cast<float>(kDefaultGeneratedContainerJitter);
  float mContainerInitialSpeed = 0.0f;
  float mContainerInitialAngularSpeed = 0.0f;
  float mDropHeight = static_cast<float>(kDefaultDropHeight);
  float mTargetRtf = 1.0f;
  int mCollisionIndex = 0;
  bool mPrimitiveShapes = false;
  bool mSleepStateColors = true;
  bool mAutoApplySceneEdits = true;
  bool mPendingSceneDirty = false;
  float mGuiScale = 1.0f;
  std::string mStatusMessage;
};

void runGui(const dart::simulation::WorldPtr& world, const Options& options)
{
  const auto replacedPlaneVisuals = replaceGuiPlaneVisualsWithFloorBoxes(world);
  if (replacedPlaneVisuals > 0) {
    std::cout
        << "Replaced GUI-only PlaneShape visuals with finite floor boxes: "
        << replacedPlaneVisuals << "\n";
  }

  ::osg::ref_ptr<ContactBenchmarkGuiWorldNode> node
      = new ContactBenchmarkGuiWorldNode(world, options);

  std::cout << "GUI status panel: RTF, simulation time, contacts, and sleep "
               "state. Press r to reset to t=0/drop state.\n";
  std::cout << "GUI target RTF: " << options.guiTargetRtf << "\n";
  std::cout << "GUI scale: " << options.guiScale << "\n";
  std::cout << "GUI starts " << (options.guiStart ? "running" : "paused")
            << ".\n";
  if (options.sleepStateColors) {
    std::cout << "Sleep-state GUI colors: awake=red, candidate=yellow, "
                 "resting=blue, static=gray.\n";
  }

  ::osg::ref_ptr<dart::gui::osg::ImGuiViewer> viewer
      = new dart::gui::osg::ImGuiViewer();
  ::osg::ref_ptr<ContactBenchmarkGuiEventHandler> eventHandler
      = new ContactBenchmarkGuiEventHandler(node.get());
  viewer->addWorldNode(node);
  viewer->getImGuiHandler()->setGuiScale(options.guiScale);
  viewer->getImGuiHandler()->addWidget(
      std::make_shared<ContactBenchmarkGuiWidget>(
          viewer.get(), node.get(), eventHandler.get(), options.guiScale));
  viewer->addEventHandler(eventHandler);
  viewer->simulate(options.guiStart);
  viewer->setUpViewInWindow(
      0,
      0,
      dart::gui::osg::scaleWindowExtent(1360, options.guiScale),
      dart::gui::osg::scaleWindowExtent(768, options.guiScale));
  applyCameraPose(*viewer, world);
  viewer->run();
}

int runGuiCapture(
    const dart::simulation::WorldPtr& world, const Options& options)
{
  if (!options.guiCapturePath.has_value())
    return 1;

  const std::filesystem::path capturePath(*options.guiCapturePath);
  if (!capturePath.parent_path().empty())
    std::filesystem::create_directories(capturePath.parent_path());

  const auto replacedPlaneVisuals = replaceGuiPlaneVisualsWithFloorBoxes(world);
  if (replacedPlaneVisuals > 0) {
    std::cout
        << "Replaced GUI-only PlaneShape visuals with finite floor boxes: "
        << replacedPlaneVisuals << "\n";
  }

  // Keep offscreen captures on static shape colors; the live GUI path owns
  // dynamic sleep-state recoloring.
  Options captureNodeOptions = options;
  captureNodeOptions.sleepStateColors = false;
  ::osg::ref_ptr<ContactBenchmarkGuiWorldNode> node
      = new ContactBenchmarkGuiWorldNode(world, captureNodeOptions);

  ::osg::ref_ptr<dart::gui::osg::ImGuiViewer> viewer
      = new dart::gui::osg::ImGuiViewer();
  ::osg::ref_ptr<ContactBenchmarkGuiEventHandler> eventHandler
      = new ContactBenchmarkGuiEventHandler(node.get());
  auto widget = std::make_shared<ContactBenchmarkGuiWidget>(
      viewer.get(), node.get(), eventHandler.get(), options.guiScale);
  viewer->setThreadingModel(::osgViewer::ViewerBase::SingleThreaded);
  viewer->addWorldNode(node);
  viewer->getImGuiHandler()->setGuiScale(options.guiScale);
  viewer->getImGuiHandler()->addWidget(widget);
  viewer->addEventHandler(eventHandler);
  viewer->simulate(false);
  viewer->setUpViewInWindow(
      0,
      0,
      dart::gui::osg::scaleWindowExtent(1360, options.guiScale),
      dart::gui::osg::scaleWindowExtent(768, options.guiScale));
  applyCameraPose(*viewer, world);
  viewer->realize();
  if (!viewer->isRealized()) {
    std::cerr << "GUI capture failed: viewer was not realized.\n";
    return 1;
  }

  applyCameraPose(*viewer, world);
  viewer->frame();
  if (options.guiCaptureExerciseWidget) {
    std::string error;
    if (!widget->exerciseSceneRebuildForTest(&error)) {
      std::cerr << "GUI capture widget exercise failed: " << error << "\n";
      return 1;
    }
    if (auto* currentNode = widget->getNode())
      node = currentNode;
    applyCameraPose(*viewer, node->getWorld());
    for (int i = 0; i < 3; ++i)
      viewer->frame();
  }

  viewer->simulate(false);
  const auto activeWorld = node->getWorld();
  if (!activeWorld) {
    std::cerr << "GUI capture failed: world was not available.\n";
    return 1;
  }

  const int startFrame = activeWorld->getSimFrames();
  const double worldTimeBefore = activeWorld->getTime();
  const auto startWallTime = std::chrono::steady_clock::now();
  std::size_t nextCheckpoint = options.checkpoint;
  auto printGuiCaptureCheckpoint = [&](std::size_t done) {
    if (options.quiet || options.checkpoint == 0)
      return;

    const auto currentWallTime = std::chrono::steady_clock::now();
    const std::chrono::duration<double> elapsed
        = currentWallTime - startWallTime;
    const double currentSimTime = activeWorld->getTime() - worldTimeBefore;
    const double currentRtf
        = elapsed.count() > 0.0 ? currentSimTime / elapsed.count() : 0.0;
    while (done >= nextCheckpoint
           && nextCheckpoint <= options.guiCaptureSteps) {
      printDiagnostics(
          nextCheckpoint,
          currentRtf,
          collectContactStats(activeWorld),
          collectSleepStats(activeWorld));
      nextCheckpoint += options.checkpoint;
    }
  };

  for (std::size_t step = 0; step < options.guiCaptureSteps; ++step) {
    activeWorld->step();
    printGuiCaptureCheckpoint(step + 1);
  }

  viewer->simulate(false);
  widget->hide();
  viewer->frame();
  viewer->captureScreen(*options.guiCapturePath);
  viewer->frame();

  const auto contacts = collectContactStats(activeWorld);
  const auto sleep = collectSleepStats(activeWorld);
  std::cout << "GUI capture wrote: " << *options.guiCapturePath << "\n";
  std::cout << "GUI capture frames: "
            << (activeWorld->getSimFrames() - startFrame) << " / "
            << options.guiCaptureSteps << "\n";
  std::cout << "GUI capture state: contacts " << contacts.contacts << ", pairs "
            << contacts.pairs << ", resting " << sleep.resting << "/"
            << sleep.mobile << ", candidates " << sleep.candidates
            << ", max velocity " << sleep.maxLinearSpeed << "\n";

  if (!std::filesystem::exists(capturePath)) {
    std::cerr << "GUI capture failed: screenshot file was not written.\n";
    return 1;
  }

  return 0;
}

int runHeadless(const dart::simulation::WorldPtr& world, const Options& options)
{
  if (options.warmup > 0) {
    std::cout << "\nWarming up " << options.warmup << " steps...\n";
    for (std::size_t step = 0; step < options.warmup; ++step)
      world->step();
  }

  std::cout << "\nSimulating " << options.steps << " steps...\n";

  ScopedProfileRecording profileRecording(options.profile);
  if (options.profile)
    dart::common::profile::resetProfile();

  const double timeStep = world->getTimeStep();
  const double worldTimeBefore = world->getTime();
  const int frameBefore = world->getSimFrames();
  const auto startTime = std::chrono::steady_clock::now();

  {
    DART_PROFILE_SCOPED_IF_N(options.profile, "contact_benchmark::runHeadless");
    for (std::size_t step = 0; step < options.steps; ++step) {
      {
        DART_PROFILE_SCOPED_IF_N(options.profile, "world->step");
        world->step();
      }

      const std::size_t done = step + 1;
      const auto currentTime = std::chrono::steady_clock::now();
      const std::chrono::duration<double> currentWallTime
          = currentTime - startTime;
      const double currentSimTime = world->getTime() - worldTimeBefore;
      const double currentRtf = currentWallTime.count() > 0
                                    ? currentSimTime / currentWallTime.count()
                                    : 0.0;

      const bool diagnosticStep
          = !options.quiet && options.checkpoint != 0
            && (done % options.checkpoint == 0 || done == options.steps);
      if (diagnosticStep) {
        printDiagnostics(
            done,
            currentRtf,
            collectContactStats(world),
            collectSleepStats(world));
      }
    }
  }

  const auto endTime = std::chrono::steady_clock::now();
  const std::chrono::duration<double> wallTime = endTime - startTime;
  const double worldTimeAfter = world->getTime();
  const int frameAfter = world->getSimFrames();

  const double expectedSimTime = static_cast<double>(options.steps) * timeStep;
  const double simTime = worldTimeAfter - worldTimeBefore;
  const long long expectedFrames = static_cast<long long>(options.steps);
  const long long frameDelta = static_cast<long long>(frameAfter) - frameBefore;
  const double simTimeError = std::abs(simTime - expectedSimTime);
  const double simTimeTolerance
      = std::max(1.0e-12, std::abs(expectedSimTime) * 1.0e-9);
  const bool timeAdvanced
      = frameDelta == expectedFrames && simTimeError <= simTimeTolerance;
  const double rtf = simTime / wallTime.count();
  const double avgStepTimeMs
      = (wallTime.count() * 1000.0) / static_cast<double>(options.steps);
  const auto contacts = collectContactStats(world);
  const auto sleep = collectSleepStats(world);
  const auto digest = collectFinalStateDigest(world, contacts, sleep);
  const auto maxContacts
      = world->getConstraintSolver()->getCollisionOption().maxNumContacts;
  const bool contactCapHit
      = maxContacts > 0 && contacts.contacts >= maxContacts;
  if (options.dumpFinalScenePath.has_value())
    writeSceneDump(world, *options.dumpFinalScenePath);

  std::cout << "\n--- Performance Results ---\n";
  std::cout << "Simulated Time:     " << simTime << " s\n";
  std::cout << "Expected Sim Time:  " << expectedSimTime << " s\n";
  std::cout << "World Time Before:  " << worldTimeBefore << " s\n";
  std::cout << "World Time After:   " << worldTimeAfter << " s\n";
  std::cout << "Frame Delta:        " << frameDelta << " / " << expectedFrames
            << "\n";
  std::cout << "Time Advanced:      " << (timeAdvanced ? "true" : "false")
            << "\n";
  std::cout << "Wall-Clock Time:    " << wallTime.count() << " s\n";
  std::cout << "Real-Time Factor:   " << rtf << "\n";
  std::cout << "Avg Step Time:      " << avgStepTimeMs << " ms/step\n";
  std::cout << "Final Contacts:     " << contacts.contacts << "\n";
  std::cout << "Final Contact Cap Hit: " << (contactCapHit ? "true" : "false")
            << "\n";
  std::cout << "Final Contact Pairs: " << contacts.pairs << "\n";
  std::cout << "Final Over Sleep Tol: " << contacts.contactsOverSleepTolerance
            << "\n";
  std::cout << "Final Zero Normals:  " << contacts.zeroNormalContacts << "\n";
  std::cout << "Final Max Penetration: " << contacts.maxPenetration << "\n";
  std::cout << "Final Resting:      " << sleep.resting << " / " << sleep.mobile
            << " mobile skeletons\n";
  std::cout << "Final Candidates:   " << sleep.candidates << " / "
            << sleep.mobile << " mobile skeletons\n";
  std::cout << "Final Islanded:     " << sleep.islanded << " / " << sleep.mobile
            << " mobile skeletons\n";
  std::cout << "Final Islands:      " << sleep.islands
            << " island(s), max mobile island size " << sleep.maxIslandMobile
            << "\n";
  std::cout << "Final Disturbed:    " << sleep.disturbed << " / "
            << sleep.mobile << " mobile skeletons\n";
  std::cout << "Final Below Sleep Speed: " << sleep.belowSleepSpeed << " / "
            << sleep.mobile << " mobile skeletons\n";
  std::cout << "Final Below Final Speed: " << sleep.belowFinalSpeed << " / "
            << sleep.mobile << " mobile skeletons\n";
  std::cout << "Final Below Wake Speed: " << sleep.belowWakeSpeed << " / "
            << sleep.mobile << " mobile skeletons\n";
  std::cout << "Final Dwell Ready:  " << sleep.dwellReady << " / "
            << sleep.mobile << " mobile skeletons\n";
  std::cout << "Final Dwell+Sleep Ready: " << sleep.dwellAndSleepReady << " / "
            << sleep.mobile << " mobile skeletons\n";
  std::cout << "Final Dwell+Final Ready: " << sleep.dwellAndFinalReady << " / "
            << sleep.mobile << " mobile skeletons\n";
  std::cout << "Final Dwell+Wake Ready: " << sleep.dwellAndWakeReady << " / "
            << sleep.mobile << " mobile skeletons\n";
  std::cout << "Final Max Smoothed Linear Speed: "
            << sleep.maxSmoothedLinearSpeed << "\n";
  std::cout << "Final Max Smoothed Angular Speed: "
            << sleep.maxSmoothedAngularSpeed << "\n";
  std::cout << "Final Max Rest Dwell Time: " << sleep.maxRestDwellTime << "\n";
  std::cout << "Final State Finite: " << (digest.finite ? "true" : "false")
            << "\n";
  std::cout << "Final State Hash:   0x" << std::hex << digest.hash << std::dec
            << "\n";
  std::cout << "Final State Sums:   position_l1 " << digest.positionAbsSum
            << " velocity_l1 " << digest.velocityAbsSum
            << " body_translation_l1 " << digest.bodyTranslationAbsSum
            << " max_abs_velocity " << digest.maxAbsVelocity << "\n";

  if (options.profile)
    dart::common::profile::printProfileSummary(std::cout);

  return digest.finite && timeAdvanced ? 0 : 2;
}

} // namespace

int main(int argc, char* argv[])
{
  if (isHelpRequest(argc, argv)) {
    printUsage(argv[0]);
    return 0;
  }

  Options options;
  try {
    options = parseOptions(argc, argv);
  } catch (const std::exception& e) {
    std::cerr << e.what() << "\n";
    printUsage(argv[0]);
    return 1;
  }

  dart::simulation::WorldPtr world;
  if (options.generatedObjects.has_value()) {
    std::cout << "Generating in-memory "
              << (options.generateContainer  ? "contact-container"
                  : options.generateCapsules ? "capsule-only"
                                             : "3-lane shape")
              << " world with " << *options.generatedObjects
              << " mobile objects\n";
    world = createGeneratedWorld(options);
  } else {
    std::filesystem::path p(*options.sdfPath);
    if (!p.is_absolute())
      p = std::filesystem::absolute(p);
    const std::string absoluteSdfPath = p.string();

    std::cout << "Loading SDF world file: " << absoluteSdfPath << "\n";

    dart::utils::SdfParser::Options parserOptions;
    parserOptions.mUsePlaneShapeForPlane = options.sdfPlaneShapes;
    if (auto detector = makeCollisionDetector(options))
      parserOptions.mCollisionDetector = detector;

    world = dart::utils::SdfParser::readWorld(absoluteSdfPath, parserOptions);
  }

  if (!world) {
    std::cerr << "Failed to create benchmark world\n";
    return 1;
  }

  try {
    applyOptions(world, options);
  } catch (const std::exception& e) {
    std::cerr << e.what() << "\n";
    return 1;
  }

  if (options.dropHeight > 0.0 && !options.generateContainer) {
    try {
      const auto raised = raiseMobileObjects(world, options.dropHeight);
      std::cout << "Raised mobile root bodies by " << options.dropHeight
                << " m: " << raised << "\n";
    } catch (const std::exception& e) {
      std::cerr << e.what() << "\n";
      return 1;
    }
  }

  wakeMobileObjects(world);
  printWorldSummary(world, options);

  if (options.guiCapturePath.has_value())
    return runGuiCapture(world, options);

  if (options.gui) {
    runGui(world, options);
    return 0;
  }

  return runHeadless(world, options);
}
