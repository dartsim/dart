// SPDX-License-Identifier: BSD-2-Clause
// Public-API driver shared by both arms, including DART 6.19 releases.
#include <dart/utils/sdf/SdfParser.hpp>
#include <dart/utils/urdf/DartLoader.hpp>

#include <dart/simulation/World.hpp>

#include <dart/constraint/ConstraintSolver.hpp>

#include <dart/collision/bullet/BulletCollisionDetector.hpp>
#include <dart/collision/dart/DARTCollisionDetector.hpp>
#include <dart/collision/fcl/FCLCollisionDetector.hpp>
#include <dart/collision/ode/OdeCollisionDetector.hpp>

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/Joint.hpp>
#include <dart/dynamics/PlaneShape.hpp>
#include <dart/dynamics/ShapeNode.hpp>
#include <dart/dynamics/Skeleton.hpp>

#include <dart/math/Constants.hpp>

#include <dart/common/Uri.hpp>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <stdexcept>
#include <string>

#include <cstdint>
#include <cstdio>
#include <cstring>

#if defined(__i386__) || defined(__x86_64__)
  #include <cpuid.h>
#endif

// Toggle this exact signature; a wildcard can also toggle nested functions.
// Consume every transform to include the lazy pose work done by gz-physics.
extern "C" void perf_alloc_begin() __attribute__((weak));
extern "C" void perf_alloc_end() __attribute__((weak));

__attribute__((noinline)) double stepAndRead(dart::simulation::World* world)
{
  if (perf_alloc_begin)
    perf_alloc_begin();
  world->step();
  double poses = 0.0;
  for (std::size_t s = 0; s < world->getNumSkeletons(); ++s) {
    const auto skeleton = world->getSkeleton(s);
    for (std::size_t b = 0; b < skeleton->getNumBodyNodes(); ++b)
      poses += skeleton->getBodyNode(b)->getWorldTransform().matrix().sum();
  }
  if (perf_alloc_end)
    perf_alloc_end();
  return poses;
}

namespace {

std::string cpuBrand()
{
#if defined(__i386__) || defined(__x86_64__)
  char brand[49] = {};
  if (__get_cpuid_max(0x80000000, nullptr) >= 0x80000004) {
    for (unsigned int i = 0; i < 3; ++i) {
      unsigned int a, b, c, d;
      __get_cpuid(0x80000002 + i, &a, &b, &c, &d);
      const unsigned int words[] = {a, b, c, d};
      std::memcpy(brand + i * sizeof(words), words, sizeof(words));
    }
    return brand;
  }
#endif
  return "unknown";
}

std::uint64_t mix(std::uint64_t hash, double value)
{
  std::uint64_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  return hash ^ (bits + 0x9e3779b97f4a7c15ULL + (hash << 6) + (hash >> 2));
}

std::size_t count(const std::string& value)
{
  std::size_t end;
  if (value.empty() || value.front() == '-')
    throw std::invalid_argument("expected a nonnegative integer: " + value);
  const auto result = std::stoull(value, &end);
  if (end != value.size())
    throw std::invalid_argument("invalid integer: " + value);
  return result;
}

} // namespace

int main(int argc, char** argv)
{
  try {
    std::string file, robot, detector = "ode", ground = "gzbox", data = "data";
    std::size_t warmup = 0, steps = 5, contacts = 10000, perPair = 0,
                threads = 1;
    bool noSleep = false;
    for (int i = 1; i < argc; ++i) {
      const std::string key = argv[i];
      if (key == "--cpu-only") {
        std::printf("Guest CPU: %s\n", cpuBrand().c_str());
        return 0;
      }
      if (key == "--disable-deactivation") {
        noSleep = true;
        continue;
      }
      if (key == "--help") {
        std::puts(
            "portable_step_bench [world.sdf | --robot atlas] "
            "[--warmup W] [--steps N] [--detector ode|dart|fcl|bullet] "
            "[--ground gzbox|plane|sdf] [--max-contacts N] "
            "[--max-contacts-per-pair N] [--world-threads N] "
            "[--disable-deactivation] [--data-dir data] [--cpu-only]");
        return 0;
      }
      if (key.rfind("--", 0) != 0 && file.empty()) {
        file = key;
        continue;
      }
      if (++i >= argc)
        throw std::invalid_argument("missing value for " + key);
      const std::string value = argv[i];
      if (key == "--robot")
        robot = value;
      else if (key == "--detector")
        detector = value;
      else if (key == "--ground")
        ground = value;
      else if (key == "--data-dir")
        data = value;
      else if (key == "--warmup")
        warmup = count(value);
      else if (key == "--steps")
        steps = count(value);
      else if (key == "--max-contacts")
        contacts = count(value);
      else if (key == "--max-contacts-per-pair")
        perPair = count(value);
      else if (key == "--world-threads")
        threads = count(value);
      else
        throw std::invalid_argument("unknown option: " + key);
    }
    if ((file.empty() == robot.empty()) || (!robot.empty() && robot != "atlas")
        || threads == 0 || contacts == 0)
      throw std::invalid_argument("select one SDF world or --robot atlas");
    if (ground != "gzbox" && ground != "plane" && ground != "sdf")
      throw std::invalid_argument("unknown ground: " + ground);

    dart::simulation::WorldPtr world;
    if (!robot.empty()) {
      world = dart::simulation::World::create();
      dart::utils::DartLoader loader;
      const auto floor = loader.parseSkeleton(dart::common::Uri::createFromPath(
          std::filesystem::absolute(data + "/sdf/atlas/ground.urdf").string()));
      const auto atlas = dart::utils::SdfParser::readSkeleton(
          dart::common::Uri::createFromPath(
              std::filesystem::absolute(
                  data + "/sdf/atlas/atlas_v3_no_head.sdf")
                  .string()));
      if (!floor || !atlas)
        throw std::runtime_error(
            "failed to load Atlas and ground from " + data);
      world->addSkeleton(floor);
      world->addSkeleton(atlas);
      atlas->setPosition(0, -0.5 * dart::math::constantsd::pi());
      for (std::size_t j = 0; j < atlas->getNumJoints(); ++j)
        atlas->getJoint(j)->setLimitEnforcement(true);
      world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));
      noSleep = true;
    } else {
      world
          = dart::utils::SdfParser::readWorld(dart::common::Uri::createFromPath(
              std::filesystem::absolute(file).string()));
      if (!world)
        throw std::runtime_error("failed to load " + file);
      // gz-physics ConstructPlane: a 2100 m box centered 1050 m below it.
      for (std::size_t s = 0; s < world->getNumSkeletons(); ++s) {
        const auto skeleton = world->getSkeleton(s);
        if (skeleton->isMobile() || ground == "sdf")
          continue;
        for (std::size_t b = 0; b < skeleton->getNumBodyNodes(); ++b) {
          const auto body = skeleton->getBodyNode(b);
          for (std::size_t n = 0; n < body->getNumShapeNodes(); ++n) {
            const auto shape = body->getShapeNode(n);
            if (!shape->getShape()->is<dart::dynamics::BoxShape>())
              continue;
            Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
            if (ground == "gzbox") {
              transform.translate(Eigen::Vector3d(0.0, 0.0, -1050.0));
              shape->setShape(std::make_shared<dart::dynamics::BoxShape>(
                  Eigen::Vector3d::Constant(2100.0)));
            } else {
              shape->setShape(std::make_shared<dart::dynamics::PlaneShape>(
                  Eigen::Vector3d::UnitZ(), 0.0));
            }
            shape->setRelativeTransform(transform);
          }
        }
      }
    }
    const auto solver = world->getConstraintSolver();
    if (detector == "ode")
      solver->setCollisionDetector(
          dart::collision::OdeCollisionDetector::create());
    else if (detector == "dart")
      solver->setCollisionDetector(
          dart::collision::DARTCollisionDetector::create());
    else if (detector == "fcl")
      solver->setCollisionDetector(
          dart::collision::FCLCollisionDetector::create());
    else if (detector == "bullet")
      solver->setCollisionDetector(
          dart::collision::BulletCollisionDetector::create());
    else
      throw std::invalid_argument("unknown detector: " + detector);
    solver->getCollisionOption().maxNumContacts = contacts;
#if PERF_HAS_PER_PAIR
    if (perPair)
      solver->getCollisionOption().maxNumContactsPerPair = perPair;
#else
    if (perPair) {
      std::puts("UNSUPPORTED: maxNumContactsPerPair");
      return 3;
    }
#endif
#if PERF_HAS_THREADS
    world->setNumSimulationThreads(threads);
#else
    if (threads != 1) {
      std::puts("UNSUPPORTED: setNumSimulationThreads");
      return 3;
    }
#endif
#if PERF_HAS_DEACTIVATION
    if (noSleep) {
      auto options = world->getDeactivationOptions();
      options.mEnabled = false;
      world->setDeactivationOptions(options);
    }
#else
    if (noSleep) {
      std::puts("UNSUPPORTED: DeactivationOptions");
      return 3;
    }
#endif

    // A volatile sink keeps the pose reads observable without allocations.
    volatile double poses = 0.0;
    for (std::size_t i = 0; i < warmup; ++i)
      poses = stepAndRead(world.get());
    const auto start = std::chrono::steady_clock::now();
    for (std::size_t i = 0; i < steps; ++i)
      poses = stepAndRead(world.get());
    const double ms = std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - start)
                          .count();
    (void)poses;

    std::uint64_t hash = 1469598103934665603ULL;
    bool finite = true;
    std::size_t mobile = 0, resting = 0;
    for (std::size_t s = 0; s < world->getNumSkeletons(); ++s) {
      const auto skeleton = world->getSkeleton(s);
      if (skeleton->isMobile()) {
        ++mobile;
        resting += skeleton->isResting();
        const Eigen::VectorXd q = skeleton->getPositions();
        const Eigen::VectorXd v = skeleton->getVelocities();
        finite = finite && q.allFinite() && v.allFinite();
        for (Eigen::Index j = 0; j < q.size(); ++j)
          hash = mix(hash, q[j]);
        for (Eigen::Index j = 0; j < v.size(); ++j)
          hash = mix(hash, v[j]);
      }
      for (std::size_t b = 0; b < skeleton->getNumBodyNodes(); ++b) {
        const auto& transform
            = skeleton->getBodyNode(b)->getWorldTransform().matrix();
        finite = finite && transform.allFinite();
        for (Eigen::Index j = 0; j < transform.size(); ++j)
          hash = mix(hash, transform.data()[j]);
      }
    }
    const auto finalContacts = world->getLastCollisionResult().getNumContacts();
    std::printf("Guest CPU: %s\n", cpuBrand().c_str());
    std::printf("Avg Step Time: %.6f ms/step\n", steps ? ms / steps : 0.0);
    std::printf(
        "Final State Hash: 0x%016llx\n", static_cast<unsigned long long>(hash));
    std::printf("Final State Finite: %s\n", finite ? "true" : "false");
    std::printf("Final Contacts: %zu\n", finalContacts);
    std::printf(
        "Final Contact Cap Hit: %s\n",
        finalContacts >= contacts ? "true" : "false");
    std::printf("Final Resting: %zu / %zu\n", resting, mobile);
    return finite ? 0 : 1;
  } catch (const std::exception& error) {
    std::fprintf(stderr, "%s\n", error.what());
    return 2;
  }
}
