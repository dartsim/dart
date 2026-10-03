// Raycast probe for the gz-physics 9 (Jetty) dartsim plugin.
//
// gz-physics 9 casts rays through DART's collision detectors itself: its ODE
// detector walks the ODE space of a DART OdeCollisionGroup subclass, and its
// Bullet detector the Bullet world of a DART BulletCollisionGroup subclass
// (DART's own raycast for single Bullet rays). A DART change to those groups
// or to the geometry DART builds for a shape (for example ODE cylinders as
// native dCylinders or as meshes) changes what Gazebo's rays hit, and the
// gz-physics suite only casts rays at spheres.
//
// This driver loads static upright and lying cylinders, a box and a sphere on
// a ground plane, steps once, casts single rays (GetRayIntersectionFromLastStep)
// and the same rays as one batch (GetBatchRayIntersectionFromLastStep), and
// compares every hit point, normal and fraction with the exact intersection
// and the batched results with the single ones. Bullet's convex raycasts are
// approximate (with DART 6.19.4 too: 0.13 m on the 2100 m ground box), so with
// --detector bullet only the batch is gated; compare its rows with those of
// another DART build instead.
//
// usage: gz_raycast_probe <dartsim-plugin.so> [--detector ode|bullet]
//            [--tolerance X]
//
// Exit status: 0 when every ray matches, 1 on any mismatch.

#include <gz/physics/ForwardStep.hh>
#include <gz/physics/GetBatchRayIntersection.hh>
#include <gz/physics/GetRayIntersection.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/World.hh>
#include <gz/physics/sdf/ConstructWorld.hh>
#include <gz/plugin/Loader.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <optional>
#include <string>
#include <vector>

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace {

namespace physics = gz::physics;

using Features = physics::FeatureList<
    physics::sdf::ConstructSdfWorld,
    physics::ForwardStep,
    physics::CollisionDetector,
    physics::GetRayIntersectionFromLastStepFeature,
    physics::GetBatchRayIntersectionFromLastStepFeature>;

// Both ray features name their result type RayIntersection; spell it out.
using RayIntersection = physics::GetRayIntersectionFromLastStepFeature::
    RayIntersectionT<physics::FeaturePolicy3d>;
using RayQuery = physics::GetBatchRayIntersectionFromLastStepFeature::RayT<
    physics::FeaturePolicy3d>;
using BatchedRayIntersectionData = physics::
    GetBatchRayIntersectionFromLastStepFeature::BatchedRayIntersectionDataT<
        physics::FeaturePolicy3d>;

constexpr double kRadius = 0.5;
constexpr double kLength = 1.0;

std::string staticModel(
    const std::string& name,
    const std::string& pose,
    const std::string& geometry)
{
  return "<model name=\"" + name + "\"><static>true</static><pose>" + pose
         + "</pose><link name=\"link\"><collision name=\"collision\">"
           "<geometry>"
         + geometry + "</geometry></collision></link></model>";
}

const std::string kWorld
    = "<?xml version=\"1.0\"?><sdf version=\"1.7\"><world name=\"rays\">"
      "<model name=\"ground\"><static>true</static><link name=\"link\">"
      "<collision name=\"collision\"><geometry><plane><normal>0 0 1</normal>"
      "<size>100 100</size></plane></geometry></collision></link></model>"
      + staticModel(
          "upright",
          "0 0 0.5 0 0 0",
          "<cylinder><radius>0.5</radius><length>1</length></cylinder>")
      + staticModel(
          "lying",
          "3 0 0.5 1.5707963267948966 0 0",
          "<cylinder><radius>0.5</radius><length>1</length></cylinder>")
      + staticModel("box", "6 0 0.5 0 0 0", "<box><size>1 1 1</size></box>")
      + staticModel(
          "sphere", "9 0 0.5 0 0 0", "<sphere><radius>0.5</radius></sphere>")
      + "</world></sdf>";

struct Ray
{
  std::string name;
  Eigen::Vector3d from;
  Eigen::Vector3d to;
  // The exact first hit, if any.
  std::optional<Eigen::Vector3d> point;
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
};

Ray hit(
    std::string name,
    const Eigen::Vector3d& from,
    const Eigen::Vector3d& to,
    const Eigen::Vector3d& point,
    const Eigen::Vector3d& normal)
{
  return {std::move(name), from, to, point, normal.normalized()};
}

std::vector<Ray> makeRays()
{
  std::vector<Ray> rays;
  // Upright cylinder at (0, 0, 0.5): radial rays at several azimuths (a
  // mesh cylinder is coarsest between its vertices), a chord, and the cap.
  const Eigen::Vector3d upright(0.0, 0.0, 0.5);
  for (const double degrees : {0.0, 10.0, 22.5, 45.0, 77.0}) {
    const double angle = degrees * M_PI / 180.0;
    const Eigen::Vector3d radial(std::cos(angle), std::sin(angle), 0.0);
    for (const double height : {0.0, 0.3}) {
      const Eigen::Vector3d center = upright + Eigen::Vector3d(0, 0, height);
      rays.push_back(hit(
          "upright_side_" + std::to_string(int(degrees * 10)) + "_z"
              + std::to_string(int(height * 10)),
          center + 2.0 * radial,
          center,
          center + kRadius * radial,
          radial));
    }
  }
  const double chordY = 0.3;
  const double chordX = std::sqrt(kRadius * kRadius - chordY * chordY);
  rays.push_back(hit(
      "upright_chord",
      Eigen::Vector3d(2.0, chordY, 0.5),
      Eigen::Vector3d(-2.0, chordY, 0.5),
      Eigen::Vector3d(chordX, chordY, 0.5),
      Eigen::Vector3d(chordX, chordY, 0.0)));
  rays.push_back(hit(
      "upright_cap",
      Eigen::Vector3d(0.2, 0.1, 3.0),
      Eigen::Vector3d(0.2, 0.1, 0.5),
      Eigen::Vector3d(0.2, 0.1, 1.0),
      Eigen::Vector3d::UnitZ()));

  // Lying cylinder at (3, 0, 0.5), axis along y: rays down onto its side at
  // several offsets from the axis, and one along the axis onto an end cap.
  for (const double offset : {0.0, 0.15, 0.3, 0.45}) {
    const double up = std::sqrt(kRadius * kRadius - offset * offset);
    rays.push_back(hit(
        "lying_side_" + std::to_string(int(offset * 100)),
        Eigen::Vector3d(3.0 + offset, 0.1, 3.0),
        Eigen::Vector3d(3.0 + offset, 0.1, 0.5),
        Eigen::Vector3d(3.0 + offset, 0.1, 0.5 + up),
        Eigen::Vector3d(offset, 0.0, up)));
  }
  rays.push_back(hit(
      "lying_cap",
      Eigen::Vector3d(3.1, -2.0, 0.6),
      Eigen::Vector3d(3.1, 0.0, 0.6),
      Eigen::Vector3d(3.1, -0.5 * kLength, 0.6),
      -Eigen::Vector3d::UnitY()));

  // Box at (6, 0, 0.5): top face and side face.
  rays.push_back(hit(
      "box_top",
      Eigen::Vector3d(6.2, 0.1, 3.0),
      Eigen::Vector3d(6.2, 0.1, 0.5),
      Eigen::Vector3d(6.2, 0.1, 1.0),
      Eigen::Vector3d::UnitZ()));
  rays.push_back(hit(
      "box_side",
      Eigen::Vector3d(4.5, 0.2, 0.3),
      Eigen::Vector3d(6.0, 0.2, 0.3),
      Eigen::Vector3d(5.5, 0.2, 0.3),
      -Eigen::Vector3d::UnitX()));

  // Sphere at (9, 0, 0.5): rays toward its center.
  const Eigen::Vector3d sphere(9.0, 0.0, 0.5);
  int index = 0;
  for (const Eigen::Vector3d& direction :
       {Eigen::Vector3d(1, 0, 0),
        Eigen::Vector3d(0, 1, 0),
        Eigen::Vector3d(0, 0, 1),
        Eigen::Vector3d(1, 1, 1),
        Eigen::Vector3d(-1, 0.3, 0.5)}) {
    const Eigen::Vector3d u = direction.normalized();
    rays.push_back(hit(
        "sphere_" + std::to_string(index++),
        sphere + 2.0 * u,
        sphere,
        sphere + kRadius * u,
        u));
  }

  // The ground (an SDF plane, built by gz-physics as a box), and a miss.
  rays.push_back(hit(
      "ground",
      Eigen::Vector3d(20.0, 0.0, 3.0),
      Eigen::Vector3d(20.0, 0.0, -1.0),
      Eigen::Vector3d(20.0, 0.0, 0.0),
      Eigen::Vector3d::UnitZ()));
  rays.push_back(
      {"miss",
       Eigen::Vector3d(20.0, 20.0, 3.0),
       Eigen::Vector3d(20.0, 20.0, 1.0),
       std::nullopt});
  return rays;
}

struct Result
{
  bool hit = false;
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
  double fraction = 0.0;
};

Result toResult(const RayIntersection& intersection)
{
  return {
      std::isfinite(intersection.fraction),
      intersection.point,
      intersection.normal,
      intersection.fraction};
}

// Largest error of `result` against the exact intersection of `ray`.
double error(const Ray& ray, const Result& result)
{
  if (!ray.point)
    return result.hit ? INFINITY : 0.0;
  if (!result.hit)
    return INFINITY;
  const double fraction
      = (*ray.point - ray.from).norm() / (ray.to - ray.from).norm();
  return std::max(
      {(result.point - *ray.point).cwiseAbs().maxCoeff(),
       (result.normal - ray.normal).cwiseAbs().maxCoeff(),
       std::abs(result.fraction - fraction)});
}

double difference(const Result& a, const Result& b)
{
  if (a.hit != b.hit)
    return INFINITY;
  if (!a.hit)
    return 0.0;
  return std::max(
      {(a.point - b.point).cwiseAbs().maxCoeff(),
       (a.normal - b.normal).cwiseAbs().maxCoeff(),
       std::abs(a.fraction - b.fraction)});
}

int usage(const char* program)
{
  std::fprintf(
      stderr,
      "usage: %s <dartsim-plugin.so> [--detector ode|bullet] "
      "[--tolerance X]\n",
      program);
  return 2;
}

} // namespace

int main(int argc, char** argv)
{
  if (argc < 2)
    return usage(argv[0]);
  const std::string pluginLib = argv[1];
  std::string detector = "ode";
  double tolerance = 1e-6;
  for (int i = 2; i < argc; i += 2) {
    const std::string key = argv[i];
    if (i + 1 >= argc)
      return usage(argv[0]);
    if (key == "--detector")
      detector = argv[i + 1];
    else if (key == "--tolerance")
      tolerance = std::atof(argv[i + 1]);
    else
      return usage(argv[0]);
  }

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
  const auto engine = physics::RequestEngine3d<Features>::From(
      loader.Instantiate(pluginName));
  if (!engine) {
    std::fprintf(stderr, "the dartsim plugin lacks a requested feature\n");
    return 1;
  }

  sdf::Root root;
  const auto errors = root.LoadSdfString(kWorld);
  if (!errors.empty() || !root.WorldByIndex(0)) {
    for (const auto& sdfError : errors)
      std::fprintf(stderr, "sdf: %s\n", sdfError.Message().c_str());
    return 1;
  }
  const auto world = engine->ConstructWorld(*root.WorldByIndex(0));
  world->SetCollisionDetector(detector);

  // Rays query the collision group of the last step.
  physics::ForwardStep::Output output;
  physics::ForwardStep::State state;
  physics::ForwardStep::Input input;
  input.Get<std::chrono::steady_clock::duration>() = std::chrono::milliseconds(1);
  world->Step(output, state, input);

  const auto rays = makeRays();
  std::vector<RayQuery> queries;
  for (const auto& ray : rays)
    queries.push_back({ray.from, ray.to});
  BatchedRayIntersectionData batchData;
  const bool batchSupported
      = world->GetBatchRayIntersectionFromLastStep(queries, batchData);
  const auto& batch = batchData.Get<std::vector<RayIntersection>>();

  const bool exact = detector == "ode";
  std::printf(
      "detector=%s tolerance=%g exact_geometry_gated=%d batch_supported=%d "
      "plugin=%s\n",
      world->GetCollisionDetector().c_str(),
      tolerance,
      exact ? 1 : 0,
      batchSupported ? 1 : 0,
      pluginLib.c_str());
  std::printf(
      "%-22s %-4s %-36s %-30s %-9s %-9s %-9s %s\n",
      "ray",
      "hit",
      "point",
      "normal",
      "fraction",
      "error",
      "batch",
      "verdict");

  std::size_t mismatches = 0;
  for (std::size_t i = 0; i < rays.size(); ++i) {
    const Result single = toResult(
        world->GetRayIntersectionFromLastStep(rays[i].from, rays[i].to)
            .Get<RayIntersection>());
    const Result batched = i < batch.size() ? toResult(batch[i]) : Result{};
    const double singleError = error(rays[i], single);
    const double batchDifference
        = i < batch.size() ? difference(single, batched) : INFINITY;
    const bool mismatch = (exact && !(singleError <= tolerance))
                          || !(batchDifference <= tolerance);
    mismatches += mismatch ? 1u : 0u;

    char point[64];
    char normal[64];
    std::snprintf(
        point,
        sizeof(point),
        "(%.6f, %.6f, %.6f)",
        single.point.x(),
        single.point.y(),
        single.point.z());
    std::snprintf(
        normal,
        sizeof(normal),
        "(%.4f, %.4f, %.4f)",
        single.normal.x(),
        single.normal.y(),
        single.normal.z());
    std::printf(
        "%-22s %-4d %-36s %-30s %-9.6f %-9.2e %-9.2e %s\n",
        rays[i].name.c_str(),
        single.hit ? 1 : 0,
        single.hit ? point : "-",
        single.hit ? normal : "-",
        single.fraction,
        singleError,
        batchDifference,
        mismatch ? "MISMATCH" : "match");
  }
  std::printf(
      "SUMMARY detector=%s rays=%zu mismatches=%zu\n",
      detector.c_str(),
      rays.size(),
      mismatches);
  return mismatches == 0 ? 0 : 1;
}
