/*
 * ReactPhysics3D profiling driver for collision pipeline baselines.
 *
 * Build example (from this repo):
 *   c++ -std=c++17 -O3 -DIS_RP3D_PROFILING_ENABLED \
 *     -I /path/to/reactphysics3d/include \
 *     docs/dev_tasks/experimental_collision/rp3d_profile_driver.cpp \
 *     -L /path/to/reactphysics3d/build_profile -lreactphysics3d \
 *     -Wl,-rpath,/path/to/reactphysics3d/build_profile \
 *     -o /tmp/rp3d_profile_driver
 */

#include <reactphysics3d/reactphysics3d.h>

#include <cstdlib>
#include <iostream>
#include <random>
#include <string>
#include <vector>

using namespace reactphysics3d;

namespace {

struct RunConfig {
  uint32 count = 10000;
  decimal range = decimal(50.0);
  int steps = 2;
  int warmup = 1;
  std::string out = "rp3d_profile.txt";
};

RunConfig ParseArgs(int argc, char** argv) {
  RunConfig config;
  if (argc > 1) {
    config.count = static_cast<uint32>(std::strtoul(argv[1], nullptr, 10));
  }
  if (argc > 2) {
    config.range = static_cast<decimal>(std::strtod(argv[2], nullptr));
  }
  if (argc > 3) {
    config.steps = std::atoi(argv[3]);
  }
  if (argc > 4) {
    config.warmup = std::atoi(argv[4]);
  }
  if (argc > 5) {
    config.out = argv[5];
  }
  return config;
}

}  // namespace

int main(int argc, char** argv) {
  RunConfig config = ParseArgs(argc, argv);

  PhysicsCommon physicsCommon;
  PhysicsWorld::WorldSettings settings;
  settings.worldName = "profile_" + std::to_string(config.count);
  settings.gravity = Vector3(decimal(0.0), decimal(0.0), decimal(0.0));
  settings.isSleepingEnabled = false;

  PhysicsWorld* world = physicsCommon.createPhysicsWorld(settings);
  SphereShape* shape = physicsCommon.createSphereShape(decimal(0.5));

  std::mt19937 rng(42);
  std::uniform_real_distribution<decimal> dist(-config.range, config.range);

  std::vector<RigidBody*> bodies;
  bodies.reserve(config.count);

  for (uint32 i = 0; i < config.count; ++i) {
    Vector3 position(dist(rng), dist(rng), dist(rng));
    Transform transform(position, Quaternion::identity());
    RigidBody* body = world->createRigidBody(transform);
    body->setType(BodyType::DYNAMIC);
    body->setLinearVelocity(Vector3(decimal(0.0), decimal(0.0), decimal(0.0)));
    body->setAngularVelocity(Vector3(decimal(0.0), decimal(0.0), decimal(0.0)));
    body->addCollider(shape, Transform::identity());
    bodies.push_back(body);
  }

#ifdef IS_RP3D_PROFILING_ENABLED
  Profiler* profiler = world->getProfiler();
  profiler->addFileDestination(config.out, Profiler::Format::Text);
#else
  std::cerr << "Profiling disabled; rebuild with IS_RP3D_PROFILING_ENABLED"
            << std::endl;
  return 1;
#endif

  const decimal timeStep = decimal(1.0 / 60.0);
  for (int i = 0; i < config.warmup; ++i) {
    world->update(timeStep);
  }

#ifdef IS_RP3D_PROFILING_ENABLED
  profiler->reset();
#endif

  for (int i = 0; i < config.steps; ++i) {
    world->update(timeStep);
  }

#ifdef IS_RP3D_PROFILING_ENABLED
  profiler->printReport();
#endif

  physicsCommon.destroyPhysicsWorld(world);
  physicsCommon.destroySphereShape(shape);

  std::cout << "Profile written: " << config.out << " (count=" << config.count
            << ", range=" << config.range << ", steps=" << config.steps
            << ", warmup=" << config.warmup << ")" << std::endl;
  return 0;
}
