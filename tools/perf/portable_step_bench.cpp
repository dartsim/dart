// SPDX-License-Identifier: BSD-2-Clause
// Public-API driver shared by both arms, including DART 6.19 releases.
// Older installs do not define HAVE_SDFORMAT; check for the header instead.
#if defined(__has_include) && __has_include(<dart/utils/sdf/SdfParser.hpp>)
  #include <dart/utils/sdf/SdfParser.hpp>
#endif
#include <dart/utils/urdf/DartLoader.hpp>

#include <dart/simulation/World.hpp>

#include <dart/constraint/BallJointConstraint.hpp>
#include <dart/constraint/ConstraintSolver.hpp>

#include <dart/collision/bullet/BulletCollisionDetector.hpp>
#include <dart/collision/dart/DARTCollisionDetector.hpp>
#include <dart/collision/fcl/FCLCollisionDetector.hpp>
#include <dart/collision/ode/OdeCollisionDetector.hpp>

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/Joint.hpp>
#include <dart/dynamics/PlaneShape.hpp>
#include <dart/dynamics/RevoluteJoint.hpp>
#include <dart/dynamics/ShapeNode.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/WeldJoint.hpp>

#include <dart/math/Constants.hpp>

#include <dart/common/Uri.hpp>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <cassert>
#include <cmath>
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

extern "C" __attribute__((noinline)) double stepWorldsAndRead(
    const std::vector<dart::simulation::WorldPtr>& worlds)
{
  if (perf_alloc_begin)
    perf_alloc_begin();
  double reads = 0.0;
  for (const auto& world : worlds) {
    world->step();
    for (std::size_t s = 0; s < world->getNumSkeletons(); ++s) {
      const auto skeleton = world->getSkeleton(s);
      for (std::size_t b = 0; b < skeleton->getNumBodyNodes(); ++b)
        reads += skeleton->getBodyNode(b)->getWorldTransform().matrix().sum();
    }
    const auto& result = world->getLastCollisionResult();
    reads += result.getNumContacts();
    for (const auto& contact : result.getContacts())
      reads += contact.point.sum() + contact.normal.sum() + contact.force.sum()
               + contact.penetrationDepth;
  }
  if (perf_alloc_end)
    perf_alloc_end();
  return reads;
}

namespace {

template <typename T, typename = void>
struct HasIsResting : std::false_type
{
};

template <typename T>
struct HasIsResting<
    T,
    std::void_t<decltype(std::declval<const T&>().isResting())>> : std::true_type
{
};

template <typename T>
bool isResting(const T& skeleton)
{
  if constexpr (HasIsResting<T>::value)
    return skeleton.isResting();
  else
    // Revisions without deactivation never put skeletons to sleep.
    return false;
}

dart::simulation::WorldPtr makeScene(const std::string& scene)
{
  using namespace dart::dynamics;
  const auto world = dart::simulation::World::create();
  world->setTimeStep(0.001);
  if (scene == "contact") {
    const auto floor = Skeleton::create("floor");
    floor->setMobile(false);
    auto* floorBody = floor->createJointAndBodyNodePair<WeldJoint>().second;
    const auto floorShape
        = std::make_shared<BoxShape>(Eigen::Vector3d(10.0, 10.0, 0.1));
    auto* floorNode
        = floorBody->createShapeNodeWith<CollisionAspect, DynamicsAspect>(
            floorShape);
    floorNode->setRelativeTranslation(Eigen::Vector3d(0.0, 0.0, -0.05));
    world->addSkeleton(floor);

    const auto box = Skeleton::create("box");
    auto pair = box->createJointAndBodyNodePair<FreeJoint>();
    const auto shape
        = std::make_shared<BoxShape>(Eigen::Vector3d::Constant(0.2));
    pair.second->createShapeNodeWith<CollisionAspect, DynamicsAspect>(shape);
    pair.second->setInertia(
        Inertia(1.0, Eigen::Vector3d::Zero(), shape->computeInertia(1.0)));
    pair.first->setPosition(5, 0.099);
    world->addSkeleton(box);
    return world;
  }
  if (scene != "closed-loop-friction")
    throw std::invalid_argument("unknown scene: " + scene);

  // Three moving bars and the fixed ground segment form a one-DOF four-bar.
  world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));
  const auto linkage = Skeleton::create("four_bar");
  const double length = 0.4;
  const auto shape
      = std::make_shared<BoxShape>(Eigen::Vector3d(length, 0.05, 0.05));
  BodyNode* parent = nullptr;
  for (std::size_t i = 0; i < 3; ++i) {
    RevoluteJoint::Properties properties;
    properties.mName = "joint" + std::to_string(i);
    properties.mAxis = Eigen::Vector3d::UnitZ();
    if (parent)
      properties.mT_ParentBodyToJoint.translation()
          = Eigen::Vector3d(length / 2.0, 0.0, 0.0);
    properties.mT_ChildBodyToJoint.translation()
        = Eigen::Vector3d(-length / 2.0, 0.0, 0.0);
    const auto pair = linkage->createJointAndBodyNodePair<RevoluteJoint>(
        parent, properties);
    pair.second->createShapeNodeWith<CollisionAspect, DynamicsAspect>(shape);
    pair.second->setInertia(
        Inertia(1.0, Eigen::Vector3d::Zero(), shape->computeInertia(1.0)));
    pair.first->setPosition(
        0, (i == 0 ? 0.5 : -0.5) * dart::math::constantsd::pi());
    pair.first->setCoulombFriction(0, 0.1);
    // Compatible initial motion exercises friction before the first solve.
    pair.first->setVelocity(0, i == 1 ? -0.1 : 0.1);
    parent = pair.second;
  }
  world->addSkeleton(linkage);
  const Eigen::Vector3d anchor
      = parent->getWorldTransform() * Eigen::Vector3d(length / 2.0, 0.0, 0.0);
  world->getConstraintSolver()->addConstraint(
      std::make_shared<dart::constraint::BallJointConstraint>(parent, anchor));
  assert(linkage->getNumDofs() == 3);
  assert(world->getConstraintSolver()->getNumConstraints() == 1);
  for (std::size_t i = 0; i < linkage->getNumJoints(); ++i)
    assert(linkage->getJoint(i)->getCoulombFriction(0) > 0.0);
  return world;
}

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
    std::string file, robot, scene, detector = "ode", ground = "gzbox",
                                    data = "data";
    std::size_t warmup = 0, steps = 5, contacts = 10000, perPair = 0,
                threads = 1, worldCount = 1;
    bool noSleep = false, manyWorlds = false;
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
            "portable_step_bench [world.sdf | --robot atlas | "
            "--scene contact|closed-loop-friction] "
            "[--warmup W] [--steps N] [--detector ode|dart|fcl|bullet] "
            "[--ground gzbox|plane|sdf] [--max-contacts N] "
            "[--max-contacts-per-pair N] [--world-threads N] "
            "[--worlds K] [--disable-deactivation] [--data-dir data] "
            "[--cpu-only]");
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
      else if (key == "--scene")
        scene = value;
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
      else if (key == "--worlds") {
        worldCount = count(value);
        manyWorlds = true;
      } else
        throw std::invalid_argument("unknown option: " + key);
    }
    if ((!file.empty() + !robot.empty() + !scene.empty() != 1)
        || (!robot.empty() && robot != "atlas") || threads == 0
        || contacts == 0)
      throw std::invalid_argument(
          "select one SDF world, --robot atlas or --scene");
    if (worldCount == 0)
      throw std::invalid_argument("--worlds must be positive");
    if (ground != "gzbox" && ground != "plane" && ground != "sdf")
      throw std::invalid_argument("unknown ground: " + ground);

    const auto makeWorld = [&]() {
      dart::simulation::WorldPtr world;
      if (!scene.empty()) {
        world = makeScene(scene);
#if defined(__has_include) && __has_include(<dart/utils/sdf/SdfParser.hpp>)
      } else if (!robot.empty()) {
        world = dart::simulation::World::create();
        dart::utils::DartLoader loader;
        const auto floor
            = loader.parseSkeleton(dart::common::Uri::createFromPath(
                std::filesystem::absolute(data + "/sdf/atlas/ground.urdf")
                    .string()));
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
        world = dart::utils::SdfParser::readWorld(
            dart::common::Uri::createFromPath(
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
#else
      } else {
        throw std::runtime_error("SDF loading is disabled in this DART install");
#endif
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
        return dart::simulation::WorldPtr{};
      }
#endif
#if PERF_HAS_THREADS
      world->setNumSimulationThreads(threads);
#else
      if (threads != 1) {
        std::puts("UNSUPPORTED: setNumSimulationThreads");
        return dart::simulation::WorldPtr{};
      }
#endif
#if PERF_HAS_DEACTIVATION
      if (noSleep) {
        auto options = world->getDeactivationOptions();
        options.mEnabled = false;
        world->setDeactivationOptions(options);
      }
#endif

      return world;
    };
    const auto world = makeWorld();
    if (!world)
      return 3;
    std::vector<dart::simulation::WorldPtr> worlds{world};
    for (std::size_t i = 1; i < worldCount; ++i) {
      const auto copy = makeWorld();
      if (!copy)
        return 3;
      worlds.push_back(copy);
    }

    // A volatile sink keeps the pose reads observable without allocations.
    volatile double poses = 0.0;
    const auto initialFrames = world->getSimFrames();
    struct Clock
    {
      int initialFrames;
      double timeBefore;
      int framesBefore;
    };
    std::vector<Clock> clocks;
    if (manyWorlds) {
      for (const auto& copy : worlds)
        clocks.push_back({copy->getSimFrames(), 0.0, 0});
      for (std::size_t i = 0; i < warmup; ++i)
        poses = stepWorldsAndRead(worlds);
      for (std::size_t i = 0; i < worlds.size(); ++i) {
        clocks[i].timeBefore = worlds[i]->getTime();
        clocks[i].framesBefore = worlds[i]->getSimFrames();
      }
    } else {
      for (std::size_t i = 0; i < warmup; ++i)
        poses = stepAndRead(world.get());
    }
    const double timeBefore = world->getTime();
    const auto framesBefore = world->getSimFrames();
    const double expectedTime = steps * world->getTimeStep();
    const auto start = std::chrono::steady_clock::now();
    if (manyWorlds) {
      for (std::size_t i = 0; i < steps; ++i)
        poses = stepWorldsAndRead(worlds);
    } else {
      for (std::size_t i = 0; i < steps; ++i)
        poses = stepAndRead(world.get());
    }
    const double ms = std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - start)
                          .count();
    (void)poses;

    const double finalTime = world->getTime();
    const auto finalFrames = world->getSimFrames();
    const double timeTolerance
        = std::max(1.0e-12, std::abs(expectedTime) * 1.0e-9);
    bool timeAdvanced
        = std::abs(finalTime - timeBefore - expectedTime) <= timeTolerance
          && static_cast<long long>(finalFrames) - framesBefore
                 == static_cast<long long>(steps)
          && static_cast<long long>(framesBefore) - initialFrames
                 == static_cast<long long>(warmup);
    std::uint64_t hash = 1469598103934665603ULL;
    bool finite = true;
    std::size_t mobile = 0, resting = 0, finalContacts = 0;
    bool contactCapHit = false;
    double loopGap = 0.0;
    for (std::size_t i = 0; i < worlds.size(); ++i) {
      const auto& world = worlds[i];
      if (manyWorlds) {
        const double elapsed = steps * world->getTimeStep();
        const double tolerance = std::max(1.0e-12, std::abs(elapsed) * 1.0e-9);
        timeAdvanced
            = timeAdvanced
              && std::abs(world->getTime() - clocks[i].timeBefore - elapsed)
                     <= tolerance
              && static_cast<long long>(world->getSimFrames())
                         - clocks[i].framesBefore
                     == static_cast<long long>(steps)
              && static_cast<long long>(clocks[i].framesBefore)
                         - clocks[i].initialFrames
                     == static_cast<long long>(warmup);
      }
      hash = mix(hash, world->getTime());
      hash = mix(hash, world->getSimFrames());
      if (scene == "closed-loop-friction") {
        const auto* tip = world->getSkeleton(0)->getBodyNode(2);
        loopGap = std::max(
            loopGap,
            (tip->getWorldTransform() * Eigen::Vector3d(0.2, 0.0, 0.0)
             - Eigen::Vector3d(0.4, 0.0, 0.0))
                .norm());
      }
      for (std::size_t s = 0; s < world->getNumSkeletons(); ++s) {
        const auto skeleton = world->getSkeleton(s);
        if (skeleton->isMobile()) {
          ++mobile;
          resting += isResting(*skeleton);
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
      const auto& collisionResult = world->getLastCollisionResult();
      const auto count = collisionResult.getNumContacts();
      finalContacts += count;
      contactCapHit = contactCapHit || count >= contacts;
      hash = mix(hash, count);
      for (const auto& contact : collisionResult.getContacts()) {
        for (const auto* vector :
             {&contact.point, &contact.normal, &contact.force}) {
          finite = finite && vector->allFinite();
          for (Eigen::Index j = 0; j < vector->size(); ++j)
            hash = mix(hash, (*vector)[j]);
        }
        finite = finite && std::isfinite(contact.penetrationDepth);
        hash = mix(hash, contact.penetrationDepth);
        for (const auto* frame :
             {contact.getShapeFrame1(), contact.getShapeFrame2()}) {
          finite = finite && frame != nullptr;
          // Shape node names repeat across skeletons; qualify them with the
          // skeleton's, which is unique in a World.
          const auto* shapeNode = frame ? frame->asShapeNode() : nullptr;
          const std::string name = shapeNode
                                       ? shapeNode->getSkeleton()->getName()
                                             + "/" + shapeNode->getName()
                                       : (frame ? frame->getName() : "");
          hash = mix(hash, name.size());
          for (unsigned char character : name)
            hash = mix(hash, character);
        }
      }
    }
    std::printf("Guest CPU: %s\n", cpuBrand().c_str());
    std::printf("Avg Step Time: %.6f ms/step\n", steps ? ms / steps : 0.0);
    std::printf(
        "Final State Hash: 0x%016llx\n", static_cast<unsigned long long>(hash));
    std::printf("Final State Finite: %s\n", finite ? "true" : "false");
    std::printf("Final Time: %.17g s\n", finalTime);
    std::printf("Final Frames: %d\n", finalFrames);
    std::printf("Time Advanced: %s\n", timeAdvanced ? "true" : "false");
    std::printf("Final Contacts: %zu\n", finalContacts);
    std::printf(
        "Final Contact Cap Hit: %s\n", contactCapHit ? "true" : "false");
    std::printf("Final Resting: %zu / %zu\n", resting, mobile);
    if (scene == "closed-loop-friction") {
      std::printf("Final Loop Closure Error: %.17g m\n", loopGap);
      std::printf(
          "Final Loop Closed: %s\n", loopGap < 1.0e-5 ? "true" : "false");
    }
    return finite && timeAdvanced && loopGap < 1.0e-5 ? 0 : 1;
  } catch (const std::exception& error) {
    std::fprintf(stderr, "%s\n", error.what());
    return 2;
  }
}
