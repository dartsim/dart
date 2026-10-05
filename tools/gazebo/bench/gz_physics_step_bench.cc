// Headless gz-physics stepping benchmark that drives the dartsim plugin the
// way gz-sim's Physics system does: load the SDF world with ConstructSdfWorld,
// apply the world's <max_contacts> with SetCollisionPairMaxContacts, then step
// with the world's dt and a fresh ForwardStep::Output (ChangedWorldPoses)
// every step.
//
// usage: gz_physics_step_bench <dartsim-plugin.so> <world.sdf> <steps>
//            [--detector NAME] [--max-contacts N] [--window K]
//            [--contacts-every K] [--sunk-z Z]
//
// Each window reports the step time, the real-time factor, and the number of
// changed poses per step (what gz-sim has to copy into its ECM). With
// --sunk-z, it also counts links of non-static models whose last published
// height is below Z (0.45 for 3k_shapes.sdf, whose bodies rest at 0.5).
// The run fails with exit status 4 if the first step does not publish every
// link of the non-static models, and with 3 if a published pose is not finite.
#include <gz/math/Pose3.hh>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/GetContacts.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/Model.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/World.hh>
#include <gz/physics/sdf/ConstructWorld.hh>
#include <gz/plugin/Loader.hh>
#include <sdf/Physics.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>

#include <algorithm>
#include <chrono>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {

using Features = gz::physics::FeatureList<
    gz::physics::sdf::ConstructSdfWorld,
    gz::physics::ForwardStep,
    gz::physics::CollisionPairMaxContacts,
    gz::physics::CollisionDetector,
    gz::physics::GetContactsFromLastStepFeature,
    gz::physics::GetModelFromWorld,
    gz::physics::GetLinkFromModel,
    gz::physics::ModelStaticState>;

std::uint64_t mix(std::uint64_t hash, double value)
{
  std::uint64_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  hash ^= bits + 0x9e3779b97f4a7c15ULL + (hash << 6) + (hash >> 2);
  return hash;
}

int usage(const char* program)
{
  std::fprintf(
      stderr,
      "usage: %s <dartsim-plugin.so> <world.sdf> <steps> "
      "[--detector ode|bullet|fcl|dart] "
      "[--max-contacts N] [--window K] [--contacts-every K] [--sunk-z Z]\n",
      program);
  return 2;
}

} // namespace

int main(int argc, char** argv)
{
  if (argc < 4)
    return usage(argv[0]);

  const std::string pluginLib = argv[1];
  const std::string worldFile = argv[2];
  const long steps = std::atol(argv[3]);
  std::string detector;
  long maxContacts = -1; // -1: the SDF world's <max_contacts>, like gz-sim
  long window = 1000;
  long contactsEvery = 0;
  std::optional<double> sunkZ;
  for (int i = 4; i < argc; i += 2) {
    const std::string key = argv[i];
    if (i + 1 >= argc)
      return usage(argv[0]);
    const char* value = argv[i + 1];
    if (key == "--detector") {
      detector = value;
      if (detector != "ode" && detector != "bullet" && detector != "fcl"
          && detector != "dart")
        return usage(argv[0]);
    } else if (key == "--max-contacts")
      maxContacts = std::atol(value);
    else if (key == "--window")
      window = std::max(1L, std::atol(value));
    else if (key == "--contacts-every")
      contactsEvery = std::atol(value);
    else if (key == "--sunk-z")
      sunkZ = std::atof(value);
    else
      return usage(argv[0]);
  }
  if (steps <= 0)
    return usage(argv[0]);

  using Clock = std::chrono::steady_clock;
  const auto loadStart = Clock::now();

  gz::plugin::Loader loader;
  std::string pluginName;
  for (const auto& name : loader.LoadLib(pluginLib)) {
    if (name.find("dartsim") != std::string::npos)
      pluginName = name;
  }
  if (pluginName.empty()) {
    std::fprintf(stderr, "no dartsim plugin in %s\n", pluginLib.c_str());
    return 1;
  }
  auto engine = gz::physics::RequestEngine3d<Features>::From(
      loader.Instantiate(pluginName));
  if (!engine) {
    std::fprintf(stderr, "the dartsim plugin lacks a requested feature\n");
    return 1;
  }

  sdf::Root root;
  const auto errors = root.Load(worldFile);
  for (const auto& error : errors)
    std::fprintf(stderr, "sdf: %s\n", error.Message().c_str());
  if (!errors.empty())
    return 1;
  const sdf::World* sdfWorld = root.WorldByIndex(0);
  if (!sdfWorld) {
    std::fprintf(stderr, "no world in %s\n", worldFile.c_str());
    return 1;
  }
  auto world = engine->ConstructWorld(*sdfWorld);
  if (!detector.empty()) {
    world->SetCollisionDetector(detector);
    if (world->GetCollisionDetector() != detector) {
      std::fprintf(
          stderr,
          "the dartsim plugin uses the %s detector, not %s\n",
          world->GetCollisionDetector().c_str(),
          detector.c_str());
      return 1;
    }
  }
  const sdf::Physics* physics = sdfWorld->PhysicsByIndex(0);
  if (maxContacts < 0)
    maxContacts = physics ? static_cast<long>(physics->MaxContacts()) : 20;
  world->SetCollisionPairMaxContacts(static_cast<std::size_t>(maxContacts));
  const double dt = physics ? physics->MaxStepSize() : 0.001;

  std::unordered_set<std::size_t> mobileLinks;
  for (std::size_t m = 0; m < world->GetModelCount(); ++m) {
    const auto model = world->GetModel(m);
    if (model->GetStatic())
      continue;
    for (std::size_t l = 0; l < model->GetLinkCount(); ++l)
      mobileLinks.insert(model->GetLink(l)->EntityID());
  }

  std::printf(
      "plugin=%s world=%s detector=%s max_contacts=%ld dt=%g mobile_links=%zu "
      "load_s=%.3f\n",
      pluginLib.c_str(),
      worldFile.c_str(),
      world->GetCollisionDetector().c_str(),
      maxContacts,
      dt,
      mobileLinks.size(),
      std::chrono::duration<double>(Clock::now() - loadStart).count());

  gz::physics::ForwardStep::Input input;
  gz::physics::ForwardStep::State state;
  input.Get<std::chrono::steady_clock::duration>()
      = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(dt));

  // Last published pose of every link, as gz-sim would hold it.
  std::unordered_map<std::size_t, gz::math::Pose3d> poses;
  double totalSeconds = 0.0;
  double windowSeconds = 0.0;
  long windowChanged = 0;
  bool finite = true;
  for (long i = 1; i <= steps; ++i) {
    gz::physics::ForwardStep::Output output; // fresh per step, like gz-sim
    const auto stepStart = Clock::now();
    world->Step(output, state, input);
    const double seconds
        = std::chrono::duration<double>(Clock::now() - stepStart).count();
    totalSeconds += seconds;
    windowSeconds += seconds;

    const auto& changed = output.Get<gz::physics::ChangedWorldPoses>().entries;
    windowChanged += static_cast<long>(changed.size());
    for (const auto& worldPose : changed) {
      poses[worldPose.body] = worldPose.pose;
      // Position and all four quaternion components.
      finite = finite && worldPose.pose.IsFinite();
    }
    // gz-physics has no previous pose to compare with on the first step, so it
    // publishes every link; a link missing then is never seen by gz-sim.
    if (i == 1) {
      const auto unpublished = std::count_if(
          mobileLinks.begin(), mobileLinks.end(), [&](std::size_t id) {
            return poses.count(id) == 0u;
          });
      if (unpublished > 0) {
        std::fprintf(
            stderr,
            "%ld of %zu mobile links were not published on the first step\n",
            static_cast<long>(unpublished),
            mobileLinks.size());
        return 4;
      }
    }

    std::string extra;
    if (contactsEvery > 0 && i % contactsEvery == 0) {
      extra += " contacts="
               + std::to_string(world->GetContactsFromLastStep().size());
    }
    if (i % window != 0 && i != steps) {
      if (!extra.empty()) {
        std::printf("step=%ld%s\n", i, extra.c_str());
        std::fflush(stdout);
      }
      continue;
    }

    const long n = (i % window == 0) ? window : (i % window);
    if (sunkZ) {
      long sunk = 0;
      double minMobileZ = std::numeric_limits<double>::infinity();
      for (const auto& [id, pose] : poses) {
        if (mobileLinks.count(id) == 0u)
          continue;
        minMobileZ = std::min(minMobileZ, pose.Pos().Z());
        if (pose.Pos().Z() < *sunkZ)
          ++sunk;
      }
      extra += " sunk=" + std::to_string(sunk)
               + " min_mobile_z=" + std::to_string(minMobileZ);
    }
    std::printf(
        "step=%ld window_ms_per_step=%.4f window_rtf=%.3f "
        "changed_poses_per_step=%.1f%s\n",
        i,
        1e3 * windowSeconds / static_cast<double>(n),
        dt * static_cast<double>(n) / windowSeconds,
        static_cast<double>(windowChanged) / static_cast<double>(n),
        extra.c_str());
    std::fflush(stdout);
    windowSeconds = 0.0;
    windowChanged = 0;
  }

  // Hash of the last published pose of every link, ordered by entity id.
  std::vector<std::size_t> ids;
  ids.reserve(poses.size());
  for (const auto& entry : poses)
    ids.push_back(entry.first);
  std::sort(ids.begin(), ids.end());
  std::uint64_t hash = 1469598103934665603ULL;
  double minZ = std::numeric_limits<double>::infinity();
  for (const auto id : ids) {
    const auto& pose = poses[id];
    for (const double value :
         {pose.Pos().X(),
          pose.Pos().Y(),
          pose.Pos().Z(),
          pose.Rot().W(),
          pose.Rot().X(),
          pose.Rot().Y(),
          pose.Rot().Z()}) {
      hash = mix(hash, value);
    }
    minZ = std::min(minZ, pose.Pos().Z());
  }
  std::printf(
      "SUMMARY steps=%ld total_step_s=%.3f avg_ms_per_step=%.4f rtf=%.3f "
      "links=%zu final_contacts=%zu min_link_z=%.6f finite=%d "
      "hash=0x%016llx\n",
      steps,
      totalSeconds,
      1e3 * totalSeconds / static_cast<double>(steps),
      dt * static_cast<double>(steps) / totalSeconds,
      ids.size(),
      world->GetContactsFromLastStep().size(),
      minZ,
      finite ? 1 : 0,
      static_cast<unsigned long long>(hash));
  return finite ? 0 : 3;
}
