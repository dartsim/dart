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

#include <dart/config.hpp>

#include <dart/collision/CollisionDetector.hpp>
#include <dart/collision/CollisionGroup.hpp>
#include <dart/collision/CollisionOption.hpp>
#include <dart/collision/CollisionResult.hpp>
#include <dart/collision/dart/DARTCollisionDetector.hpp>
#include <dart/collision/fcl/FCLCollisionDetector.hpp>
#include <dart/collision/native/NativeCollisionDetector.hpp>

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/ConvexMeshShape.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/Inertia.hpp>
#include <dart/dynamics/PlaneShape.hpp>
#include <dart/dynamics/ShapeNode.hpp>
#include <dart/dynamics/Skeleton.hpp>

#if HAVE_BULLET
  #include <dart/collision/bullet/BulletCollisionDetector.hpp>
#endif

#include <dart/math/detail/MasonryArchGeometry.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <exception>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <cctype>
#include <cmath>
#include <cstddef>

namespace {

constexpr double kPaperContractFriction = 0.8;
constexpr double kSourceDefaultDensity = 1000.0;
constexpr std::size_t kPaperArch25Contacts = 100u;
constexpr double kSourceGroundGap = 0.001;
constexpr std::size_t kMaxRepeatCount = 10000u;

using Pair = std::pair<std::string, std::string>;

enum class Backend
{
  Native,
  Fcl,
  Dart,
#if HAVE_BULLET
  Bullet,
#endif
};

struct ProbePolicy
{
  const char* label;
  dart::math::detail::MasonryArchBarrierGapPolicy gapPolicy;
  double endFaceExpansionMeters;
  double downwardShiftMeters;
};

struct ProbeScene
{
  std::vector<dart::dynamics::SkeletonPtr> stones;
  dart::dynamics::SkeletonPtr ground;
  std::vector<double> exactVolumes;
  std::vector<double> shapeAabbVolumes;
  std::vector<Eigen::Matrix3d> exactMoments;
  std::shared_ptr<dart::collision::CollisionDetector> detector;
  std::unique_ptr<dart::collision::CollisionGroup> group;
};

struct ProbeSample
{
  std::size_t contacts = 0u;
  std::map<Pair, std::size_t> graph;
  double minPenetration = std::numeric_limits<double>::quiet_NaN();
  double maxPenetration = std::numeric_limits<double>::quiet_NaN();
  double meanPenetration = std::numeric_limits<double>::quiet_NaN();
  std::size_t nonFiniteContacts = 0u;
  std::size_t adjacentStonePairs = 0u;
  std::size_t nonAdjacentStonePairs = 0u;
  std::size_t springerGroundPairs = 0u;
  std::size_t unexpectedGroundPairs = 0u;
};

const char* backendLabel(Backend backend)
{
  switch (backend) {
    case Backend::Native:
      return "native";
    case Backend::Fcl:
      return "fcl_convex";
    case Backend::Dart:
      return "dart_legacy";
#if HAVE_BULLET
    case Backend::Bullet:
      return "bullet_convex_hull";
#endif
  }
  return "unknown";
}

std::shared_ptr<dart::collision::CollisionDetector> makeDetector(
    Backend backend)
{
  switch (backend) {
    case Backend::Native:
      return dart::collision::NativeCollisionDetector::create();
    case Backend::Fcl: {
      auto detector = dart::collision::FCLCollisionDetector::create();
      // ConvexMeshShape is represented by fcl::Convex. Request FCL's own
      // convex contact points rather than DART's triangle-mesh post-process.
      detector->setPrimitiveShapeType(
          dart::collision::FCLCollisionDetector::PRIMITIVE);
      detector->setContactPointComputationMethod(
          dart::collision::FCLCollisionDetector::FCL);
      return detector;
    }
    case Backend::Dart:
      return dart::collision::DARTCollisionDetector::create();
#if HAVE_BULLET
    case Backend::Bullet:
      return dart::collision::BulletCollisionDetector::create();
#endif
  }
  return nullptr;
}

const std::vector<Backend>& backends()
{
  static const std::vector<Backend> values
      = { Backend::Native,
          Backend::Fcl,
          Backend::Dart,
#if HAVE_BULLET
          Backend::Bullet,
#endif
        };
  return values;
}

const std::vector<ProbePolicy>& policies()
{
  using Gap = dart::math::detail::MasonryArchBarrierGapPolicy;
  // The source-normalized arch starts 1 mm above the ground. The sensitivity
  // rows first remove only that barrier gap, then add 1/10/100 um of bounded
  // end-face closure and the same shallow ground penetration. None is an
  // author parameter or a target-count fit.
  static const std::vector<ProbePolicy> values = {
      {"source_offsets", Gap::IncludeSourceOffsets, 0.0, 0.0},
      {"nominal_touching", Gap::OmitSourceOffsets, 0.0, 0.0},
      {"ground_gap_removed", Gap::OmitSourceOffsets, 0.0, kSourceGroundGap},
      {"closure_1um", Gap::OmitSourceOffsets, 1e-6, kSourceGroundGap + 1e-6},
      {"closure_10um", Gap::OmitSourceOffsets, 1e-5, kSourceGroundGap + 1e-5},
      {"closure_100um", Gap::OmitSourceOffsets, 1e-4, kSourceGroundGap + 1e-4},
  };
  return values;
}

std::shared_ptr<dart::dynamics::ConvexMeshShape> makeConvexShape(
    const dart::math::detail::MasonryArchStoneWedgeGeometry& geometry)
{
  dart::dynamics::ConvexMeshShape::Vertices vertices;
  vertices.reserve(geometry.vertices.size());
  for (const auto& vertex : geometry.vertices)
    vertices.push_back(vertex - geometry.centroid);
  dart::dynamics::ConvexMeshShape::Triangles triangles;
  triangles.reserve(
      dart::math::detail::getMasonryArchStoneWedgeTriangles().size());
  for (const auto& triangle :
       dart::math::detail::getMasonryArchStoneWedgeTriangles()) {
    triangles.emplace_back(
        static_cast<Eigen::Index>(triangle[0]),
        static_cast<Eigen::Index>(triangle[1]),
        static_cast<Eigen::Index>(triangle[2]));
  }
  return std::make_shared<dart::dynamics::ConvexMeshShape>(vertices, triangles);
}

ProbeScene makeScene(
    std::size_t stoneCount, Backend backend, const ProbePolicy& policy)
{
  const auto geometry = dart::math::detail::generateMasonryArchStoneWedges(
      stoneCount, {}, policy.gapPolicy, policy.endFaceExpansionMeters);

  ProbeScene scene;
  scene.detector = makeDetector(backend);
  scene.group = scene.detector->createCollisionGroup();
  scene.stones.reserve(stoneCount);
  scene.exactVolumes.reserve(stoneCount);
  scene.shapeAabbVolumes.reserve(stoneCount);
  scene.exactMoments.reserve(stoneCount);

  for (std::size_t i = 0u; i < stoneCount; ++i) {
    auto skeleton = dart::dynamics::Skeleton::create(
        "masonry_arch_wedge_" + std::to_string(i));
    const auto pair
        = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
    auto* joint = pair.first;
    auto* body = pair.second;
    const auto shape = makeConvexShape(geometry[i]);
    auto* shapeNode = body->createShapeNodeWith<
        dart::dynamics::CollisionAspect,
        dart::dynamics::DynamicsAspect>(shape);
    shapeNode->setName(skeleton->getName() + "_shape");
    shapeNode->getDynamicsAspect()->setFrictionCoeff(kPaperContractFriction);

    const double mass = kSourceDefaultDensity * geometry[i].volume;
    dart::dynamics::Inertia inertia;
    inertia.setMass(mass);
    inertia.setMoment(mass * geometry[i].momentPerUnitMass);
    body->setInertia(inertia);

    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translation()
        = geometry[i].centroid
          - policy.downwardShiftMeters * Eigen::Vector3d::UnitZ();
    joint->setPositions(
        dart::dynamics::FreeJoint::convertToPositions(transform));

    // The source endpoints are pinned exactly as the paper describes. Every
    // dynamic-capability prerequisite is now present (centered mesh, exact
    // uniform-prism mass and inertia), but this program deliberately performs
    // collision queries only; it does not turn a contact-count observation
    // into a standing/impact or paper-parity claim.
    const bool isSpringer = i == 0u || i + 1u == stoneCount;
    skeleton->setMobile(!isSpringer);

    scene.group->addShapeFrame(shapeNode);
    scene.exactVolumes.push_back(geometry[i].volume);
    scene.shapeAabbVolumes.push_back(shape->getVolume());
    scene.exactMoments.push_back(mass * geometry[i].momentPerUnitMass);
    scene.stones.push_back(std::move(skeleton));
  }

  scene.ground = dart::dynamics::Skeleton::create("masonry_arch_ground");
  auto* groundBody
      = scene.ground->createJointAndBodyNodePair<dart::dynamics::FreeJoint>()
            .second;
  auto* groundShape = groundBody->createShapeNodeWith<
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(
      std::make_shared<dart::dynamics::PlaneShape>(
          Eigen::Vector3d::UnitZ(), 0.0));
  groundShape->setName("masonry_arch_ground_shape");
  groundShape->getDynamicsAspect()->setFrictionCoeff(kPaperContractFriction);
  scene.ground->setMobile(false);
  scene.group->addShapeFrame(groundShape);

  return scene;
}

Pair orderedPair(
    const dart::dynamics::ShapeFrame* first,
    const dart::dynamics::ShapeFrame* second)
{
  std::string firstName = first != nullptr ? first->getName() : "<null>";
  std::string secondName = second != nullptr ? second->getName() : "<null>";
  if (secondName < firstName)
    std::swap(firstName, secondName);
  return {firstName, secondName};
}

std::optional<std::size_t> stoneIndex(const std::string& name)
{
  constexpr const char* kPrefix = "masonry_arch_wedge_";
  constexpr const char* kSuffix = "_shape";
  const std::string prefix(kPrefix);
  const std::string suffix(kSuffix);
  if (name.rfind(prefix, 0u) != 0u
      || name.size() <= prefix.size() + suffix.size()
      || name.compare(name.size() - suffix.size(), suffix.size(), suffix)
             != 0) {
    return std::nullopt;
  }

  const std::string digits
      = name.substr(prefix.size(), name.size() - prefix.size() - suffix.size());
  if (digits.empty()
      || !std::all_of(digits.begin(), digits.end(), [](unsigned char c) {
           return std::isdigit(c) != 0;
         })) {
    return std::nullopt;
  }
  return static_cast<std::size_t>(std::stoull(digits));
}

bool isGround(const std::string& name)
{
  return name == "masonry_arch_ground_shape";
}

const char* pairKind(const Pair& pair, std::size_t stoneCount)
{
  const auto firstStone = stoneIndex(pair.first);
  const auto secondStone = stoneIndex(pair.second);
  if (firstStone && secondStone) {
    const std::size_t distance = *firstStone > *secondStone
                                     ? *firstStone - *secondStone
                                     : *secondStone - *firstStone;
    return distance == 1u ? "adjacent_stones" : "nonadjacent_stones";
  }

  const auto groundStone = firstStone ? firstStone : secondStone;
  if ((isGround(pair.first) || isGround(pair.second)) && groundStone) {
    return (*groundStone == 0u || *groundStone + 1u == stoneCount)
               ? "springer_ground"
               : "unexpected_interior_ground";
  }
  return "unexpected";
}

void classifyPairs(ProbeSample& sample, std::size_t stoneCount)
{
  for (const auto& [pair, contacts] : sample.graph) {
    static_cast<void>(contacts);
    const std::string kind(pairKind(pair, stoneCount));
    if (kind == "adjacent_stones")
      ++sample.adjacentStonePairs;
    else if (kind == "nonadjacent_stones" || kind == "unexpected")
      ++sample.nonAdjacentStonePairs;
    else if (kind == "springer_ground")
      ++sample.springerGroundPairs;
    else if (kind == "unexpected_interior_ground")
      ++sample.unexpectedGroundPairs;
  }
}

ProbeSample collide(const ProbeScene& scene, std::size_t stoneCount)
{
  dart::collision::CollisionOption option(true, 10000u);
  option.maxNumContactsPerPair = 8u;
  dart::collision::CollisionResult result;
  scene.group->collide(option, &result);

  ProbeSample sample;
  sample.contacts = result.getNumContacts();
  if (sample.contacts > 0u) {
    sample.minPenetration = std::numeric_limits<double>::infinity();
    sample.maxPenetration = -sample.minPenetration;
  }

  double penetrationSum = 0.0;
  for (const auto& contact : result.getContacts()) {
    ++sample.graph[orderedPair(
        contact.getShapeFrame1(), contact.getShapeFrame2())];
    if (!std::isfinite(contact.penetrationDepth) || !contact.point.allFinite()
        || !contact.normal.allFinite()) {
      ++sample.nonFiniteContacts;
      continue;
    }
    sample.minPenetration
        = std::min(sample.minPenetration, contact.penetrationDepth);
    sample.maxPenetration
        = std::max(sample.maxPenetration, contact.penetrationDepth);
    penetrationSum += contact.penetrationDepth;
  }
  if (sample.contacts > sample.nonFiniteContacts) {
    sample.meanPenetration
        = penetrationSum
          / static_cast<double>(sample.contacts - sample.nonFiniteContacts);
  }
  classifyPairs(sample, stoneCount);
  return sample;
}

bool samplesMatch(const ProbeSample& first, const ProbeSample& second)
{
  if (first.contacts != second.contacts || first.graph != second.graph
      || first.nonFiniteContacts != second.nonFiniteContacts) {
    return false;
  }

  if (first.contacts == 0u)
    return true;

  return std::abs(first.minPenetration - second.minPenetration) <= 1e-12
         && std::abs(first.maxPenetration - second.maxPenetration) <= 1e-12
         && std::abs(first.meanPenetration - second.meanPenetration) <= 1e-12;
}

const char* gapPolicyLabel(
    dart::math::detail::MasonryArchBarrierGapPolicy gapPolicy)
{
  return gapPolicy
                 == dart::math::detail::MasonryArchBarrierGapPolicy::
                     IncludeSourceOffsets
             ? "source_offsets"
             : "omitted_offsets";
}

bool runProbe(
    std::size_t stoneCount,
    Backend backend,
    const ProbePolicy& policy,
    std::size_t repeats)
{
  const char* gapPolicy = gapPolicyLabel(policy.gapPolicy);
  const char* backendName = backendLabel(backend);
  if (backend == Backend::Dart) {
    // DARTCollisionDetector explicitly rejects ConvexMeshShape, including
    // convex-mesh/convex-mesh and convex-mesh/plane pairs. Report that
    // capability result once per matrix row instead of constructing hundreds
    // of knowingly unsupported collision objects and flooding stderr.
    std::cout << "metadata,stone_count=" << stoneCount
              << ",backend=" << backendName << ",policy=" << policy.label
              << ",gap_policy=" << gapPolicy << ",backend_supported=false"
              << ",unsupported_reason=convex_mesh_shape_not_supported\n";
    std::cout << "verdict,stone_count=" << stoneCount
              << ",backend=" << backendName << ",policy=" << policy.label
              << ",gap_policy=" << gapPolicy
              << ",repeated_collision_stable=not_applicable"
              << ",numerical_100_contact_target_observed=not_applicable"
              << ",genuine_contact_graph=not_applicable"
              << ",dynamic_path_candidate=not_applicable"
              << ",paper_contract_proven=false\n";
    return true;
  }

  auto scene = makeScene(stoneCount, backend, policy);
  const double exactVolume = std::accumulate(
      scene.exactVolumes.begin(), scene.exactVolumes.end(), 0.0);
  const double shapeAabbVolume = std::accumulate(
      scene.shapeAabbVolumes.begin(), scene.shapeAabbVolumes.end(), 0.0);
  const std::size_t mobileSkeletons = static_cast<std::size_t>(std::count_if(
      scene.stones.begin(), scene.stones.end(), [](const auto& stone) {
        return stone->isMobile();
      }));
  const bool pinnedSpringersValid = !scene.stones.front()->isMobile()
                                    && !scene.stones.back()->isMobile()
                                    && mobileSkeletons + 2u == stoneCount;
  bool exactInertiaValid = scene.exactMoments.size() == scene.stones.size();
  for (std::size_t i = 0u; i < scene.stones.size(); ++i) {
    const auto* body = scene.stones[i]->getBodyNode(0);
    exactInertiaValid = exactInertiaValid && body != nullptr
                        && std::abs(
                               body->getInertia().getMass()
                               - kSourceDefaultDensity * scene.exactVolumes[i])
                               <= 1e-12
                        && body->getInertia().getMoment().isApprox(
                            scene.exactMoments[i], 1e-12);
  }

  std::cout << "metadata,stone_count=" << stoneCount
            << ",backend=" << backendName << ",policy=" << policy.label
            << ",gap_policy=" << gapPolicy
            << ",end_face_expansion_m=" << policy.endFaceExpansionMeters
            << ",downward_shift_m=" << policy.downwardShiftMeters
            << ",friction=" << kPaperContractFriction
            << ",pinned_springers=0:" << (stoneCount - 1u)
            << ",pinned_springers_valid="
            << (pinnedSpringersValid ? "true" : "false")
            << ",mobile_skeletons=" << mobileSkeletons
            << ",collision_only=true,dynamic_claim=false"
            << ",exact_polyhedral_inertia="
            << (exactInertiaValid ? "true" : "false")
            << ",source_density_kg_m3=" << kSourceDefaultDensity
            << ",exact_volume_m3=" << exactVolume
            << ",nominal_mass_from_exact_volume_kg="
            << exactVolume * kSourceDefaultDensity
            << ",convex_shape_aabb_volume_m3=" << shapeAabbVolume << '\n';

  bool stable = true;
  ProbeSample baseline;
  for (std::size_t repeat = 0u; repeat < repeats; ++repeat) {
    const ProbeSample sample = collide(scene, stoneCount);
    if (repeat == 0u)
      baseline = sample;
    else
      stable = stable && samplesMatch(baseline, sample);

    std::cout << "sample,stone_count=" << stoneCount
              << ",backend=" << backendName << ",policy=" << policy.label
              << ",gap_policy=" << gapPolicy << ",repeat=" << repeat
              << ",contacts=" << sample.contacts
              << ",unique_pairs=" << sample.graph.size()
              << ",adjacent_stone_pairs=" << sample.adjacentStonePairs
              << ",nonadjacent_stone_pairs=" << sample.nonAdjacentStonePairs
              << ",springer_ground_pairs=" << sample.springerGroundPairs
              << ",unexpected_ground_pairs=" << sample.unexpectedGroundPairs
              << ",min_penetration_m=" << sample.minPenetration
              << ",max_penetration_m=" << sample.maxPenetration
              << ",mean_penetration_m=" << sample.meanPenetration
              << ",nonfinite_contacts=" << sample.nonFiniteContacts << '\n';
  }

  for (const auto& [pair, contacts] : baseline.graph) {
    std::cout << "pair,stone_count=" << stoneCount << ",backend=" << backendName
              << ",policy=" << policy.label << ",gap_policy=" << gapPolicy
              << ",first=" << pair.first << ",second=" << pair.second
              << ",kind=" << pairKind(pair, stoneCount)
              << ",contacts=" << contacts << '\n';
  }

  const bool numericalTargetObserved
      = stoneCount == 25u && baseline.contacts == kPaperArch25Contacts;
  const bool genuineContactGraph = baseline.nonAdjacentStonePairs == 0u
                                   && baseline.unexpectedGroundPairs == 0u;
  const bool dynamicPathCandidate
      = numericalTargetObserved && genuineContactGraph && exactInertiaValid
        && pinnedSpringersValid && baseline.nonFiniteContacts == 0u;
  std::cout << "verdict,stone_count=" << stoneCount
            << ",backend=" << backendName << ",policy=" << policy.label
            << ",gap_policy=" << gapPolicy
            << ",repeated_collision_stable=" << (stable ? "true" : "false")
            << ",numerical_100_contact_target_observed="
            << (stoneCount != 25u
                    ? "not_applicable"
                    : (numericalTargetObserved ? "true" : "false"))
            << ",genuine_contact_graph="
            << (genuineContactGraph ? "true" : "false")
            << ",dynamic_path_candidate="
            << (stoneCount != 25u ? "not_applicable"
                                  : (dynamicPathCandidate ? "true" : "false"))
            << ",paper_contract_proven=false\n";
  return stable && baseline.nonFiniteContacts == 0u && pinnedSpringersValid
         && exactInertiaValid;
}

} // namespace

int main(int argc, char* argv[])
{
  std::size_t repeats = 10u;
  if (argc > 2) {
    std::cerr << "usage: " << argv[0] << " [repeat-count]\n";
    return 2;
  }
  if (argc == 2) {
    const std::string value(argv[1]);
    const bool allDigits
        = !value.empty()
          && std::all_of(value.begin(), value.end(), [](unsigned char ch) {
               return std::isdigit(ch) != 0;
             });
    if (!allDigits) {
      std::cerr << "repeat-count must be an integer in [1, " << kMaxRepeatCount
                << "]\n";
      return 2;
    }
    try {
      std::size_t consumed = 0u;
      const auto parsed = std::stoull(value, &consumed, 10);
      if (consumed != value.size() || parsed == 0u || parsed > kMaxRepeatCount
          || parsed > std::numeric_limits<std::size_t>::max()) {
        throw std::out_of_range("repeat-count");
      }
      repeats = static_cast<std::size_t>(parsed);
    } catch (const std::exception&) {
      std::cerr << "repeat-count must be an integer in [1, " << kMaxRepeatCount
                << "]\n";
      return 2;
    }
  }

  std::cout << std::setprecision(17);
  bool valid = true;
  for (const std::size_t stoneCount : {25u, 101u}) {
    for (const auto backend : backends()) {
      for (const auto& policy : policies())
        valid = runProbe(stoneCount, backend, policy, repeats) && valid;
    }
  }
  return valid ? 0 : 1;
}
