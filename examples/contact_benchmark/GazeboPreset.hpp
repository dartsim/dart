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

#ifndef DART_EXAMPLES_CONTACT_BENCHMARK_GAZEBO_PRESET_HPP_
#define DART_EXAMPLES_CONTACT_BENCHMARK_GAZEBO_PRESET_HPP_

#include <dart/simulation/World.hpp>

#include <dart/constraint/ConstraintSolver.hpp>

#include <dart/collision/CollisionFilter.hpp>
#include <dart/collision/CollisionGroup.hpp>
#include <dart/collision/CollisionObject.hpp>
#include <dart/collision/CollisionResult.hpp>

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/CapsuleShape.hpp>
#include <dart/dynamics/CylinderShape.hpp>
#include <dart/dynamics/EllipsoidShape.hpp>
#include <dart/dynamics/PlaneShape.hpp>
#include <dart/dynamics/ShapeNode.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/SphereShape.hpp>

#include <tinyxml2.h>

#include <algorithm>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <cmath>
#include <cstddef>

// The `--gz-preset` collision configuration: what the gz-physics dartsim plugin
// (gz-physics 7, 8 and 9) builds for an SDF world, so DART-only benchmark rows
// measure the contact path Gazebo actually runs.
namespace dart::examples::contact_benchmark {

/// Global contact cap set by gz-physics EntityManagementFeatures::
/// ConstructEmptyWorld.
constexpr std::size_t kGazeboMaxNumContacts = 10000;

/// Per-pair limit gz-sim passes to SetCollisionPairMaxContacts: the SDF
/// <physics><max_contacts> default.
constexpr std::size_t kGazeboDefaultCollisionPairMaxContacts = 20;

/// Side of the cube gz-physics SDFFeatures ConstructPlane builds for an SDF
/// <plane>.
constexpr double kGazeboPlaneBoxSize = 2100.0;

/// Pose tolerance of gz-physics SimulationFeatures::Write(ChangedWorldPoses).
constexpr double kGazeboChangedPoseTolerance = 1e-6;

/// A mobile body counts as sunk when its lowest collision point is this far
/// below the ground's top face.
constexpr double kSunkDepthTolerance = 0.05;

/// Mirrors gz-physics GzCollisionDetector::LimitCollisionPairMaxContacts:
/// keep the first `maxPairContacts` contacts of each object pair in detector
/// order, then let a deeper contact replace the last kept one.
inline void limitCollisionPairMaxContacts(
    collision::CollisionResult& result, std::size_t maxPairContacts)
{
  if (maxPairContacts == std::numeric_limits<std::size_t>::max())
    return;

  const std::vector<collision::Contact> allContacts = result.getContacts();
  result.clear();
  if (maxPairContacts == 0u)
    return;

  // Per ordered object pair: <contact count, index of the last kept contact>.
  // Both orders are counted together, like gz-physics.
  std::unordered_map<
      const collision::CollisionObject*,
      std::unordered_map<
          const collision::CollisionObject*,
          std::pair<std::size_t, std::size_t>>>
      contactMap;
  for (const auto& contact : allContacts) {
    auto& [count, lastIndex]
        = contactMap[contact.collisionObject1][contact.collisionObject2];
    ++count;
    auto& [otherCount, otherLastIndex]
        = contactMap[contact.collisionObject2][contact.collisionObject1];

    const std::size_t total = count + otherCount;
    if (total <= maxPairContacts) {
      if (total == maxPairContacts) {
        lastIndex = result.getNumContacts();
        otherLastIndex = lastIndex;
      }
      result.addContact(contact);
    } else {
      auto& kept = result.getContact(lastIndex);
      if (contact.penetrationDepth > kept.penetrationDepth)
        kept = contact;
    }
  }
}

/// Mirrors gz-physics GzCollisionDetector: the per-pair contact limit that
/// gz-physics' ODE and Bullet detectors apply after DART's collide().
class GazeboCollisionPairLimit
{
public:
  std::size_t getCollisionPairMaxContacts() const
  {
    return mMaxPairContacts;
  }

  void setCollisionPairMaxContacts(std::size_t maxPairContacts)
  {
    mMaxPairContacts = maxPairContacts;
  }

protected:
  explicit GazeboCollisionPairLimit(std::size_t maxPairContacts)
    : mMaxPairContacts(maxPairContacts)
  {
    // Do nothing
  }

  virtual ~GazeboCollisionPairLimit() = default;

private:
  std::size_t mMaxPairContacts;
};

/// Mirrors gz-physics GzOdeCollisionDetector / GzBulletCollisionDetector: the
/// DART detector followed by the per-pair limit gz-sim sets through
/// SetCollisionPairMaxContacts.
template <typename BaseDetector>
class GazeboPairLimitedDetector : public BaseDetector,
                                  public GazeboCollisionPairLimit
{
public:
  static std::shared_ptr<GazeboPairLimitedDetector> create(
      std::size_t maxPairContacts)
  {
    return std::shared_ptr<GazeboPairLimitedDetector>(
        new GazeboPairLimitedDetector(maxPairContacts));
  }

  std::shared_ptr<collision::CollisionDetector> cloneWithoutCollisionObjects()
      const override
  {
    return create(getCollisionPairMaxContacts());
  }

  bool collide(
      collision::CollisionGroup* group,
      const collision::CollisionOption& option,
      collision::CollisionResult* result) override
  {
    const bool collided = BaseDetector::collide(group, option, result);
    if (result)
      limitCollisionPairMaxContacts(*result, getCollisionPairMaxContacts());
    return collided;
  }

  bool collide(
      collision::CollisionGroup* group1,
      collision::CollisionGroup* group2,
      const collision::CollisionOption& option,
      collision::CollisionResult* result) override
  {
    const bool collided = BaseDetector::collide(group1, group2, option, result);
    if (result)
      limitCollisionPairMaxContacts(*result, getCollisionPairMaxContacts());
    return collided;
  }

private:
  explicit GazeboPairLimitedDetector(std::size_t maxPairContacts)
    : GazeboCollisionPairLimit(maxPairContacts)
  {
    // Do nothing
  }
};

/// Stands in for gz-physics BitmaskContactFilter, the BodyNodeCollisionFilter
/// subclass gz-physics installs in every world. Without SDF bitmasks that can
/// filter a pair (see findGazeboFilteringBitmask()) it makes the same
/// decisions as its base class; what matters for DART is that the world's
/// filter is a subclass rather than BodyNodeCollisionFilter itself.
class GazeboContactFilter final : public collision::BodyNodeCollisionFilter
{
public:
  bool ignoresCollision(
      const collision::CollisionObject* object1,
      const collision::CollisionObject* object2) const override
  {
    return collision::BodyNodeCollisionFilter::ignoresCollision(
        object1, object2);
  }
};

/// gz-physics BitmaskContactFilter ignores a pair of collisions when neither
/// one's SDF <category_bitmask> (default: its <collide_bitmask>) shares a bit
/// with the other's <collide_bitmask> (default 0xff). DART's SdfParser does not
/// read the masks, so GazeboContactFilter cannot apply them; it is sure to
/// match gz-physics only when every mask has all the bits of 0xff, which every
/// pair then shares, and is at most INT_MAX once sdformat stores it as an
/// unsigned int: gz-physics reads it with Get<int>, which yields 0 for larger
/// values such as 0xffffffff or -1. Returns the first mask element under
/// `node` that fails this, as "<name> <value>", or nothing.
inline std::optional<std::string> findGazeboFilteringBitmask(
    const tinyxml2::XMLNode& node)
{
  for (const auto* element = node.FirstChildElement(); element;
       element = element->NextSiblingElement()) {
    const std::string_view name = element->Name();
    unsigned mask = 0u;
    if (name != "collide_bitmask" && name != "category_bitmask") {
      if (auto found = findGazeboFilteringBitmask(*element))
        return found;
    } else if (
        element->QueryUnsignedText(&mask) != tinyxml2::XML_SUCCESS
        || (mask & 0xffu) != 0xffu
        || mask > static_cast<unsigned>(std::numeric_limits<int>::max())) {
      const char* value = element->GetText();
      return std::string(name) + " " + (value ? value : "");
    }
  }
  return std::nullopt;
}

/// DART's SdfParser reads only the <model> elements of a world and the <link>
/// and <joint> elements of a model, so a model that sdformat includes (an
/// <include> in a world or a model) or that gz-physics builds nested in a model
/// would be missing. Returns the first such element under `node`, as
/// "<include> <uri>" or "nested <model> <name>", or nothing.
inline std::optional<std::string> findSdfSkippedModel(
    const tinyxml2::XMLNode& node)
{
  const auto* parent = node.ToElement();
  const bool inModel = parent && std::string_view(parent->Name()) == "model";
  for (const auto* element = node.FirstChildElement(); element;
       element = element->NextSiblingElement()) {
    const std::string_view name = element->Name();
    if (name == "include") {
      const auto* uri = element->FirstChildElement("uri");
      const char* text = uri ? uri->GetText() : nullptr;
      return std::string("<include> ") + (text ? text : "");
    }
    if (inModel && name == "model") {
      const char* modelName = element->Attribute("name");
      return std::string("nested <model> ") + (modelName ? modelName : "");
    }
    if (auto found = findSdfSkippedModel(*element))
      return found;
  }
  return std::nullopt;
}

/// Rebuilds one collision PlaneShape as the box gz-physics builds for an SDF
/// <plane> (SDFFeatures ConstructPlane): a 2100 m cube rotated from +Z onto
/// the plane normal and shifted down by half its side, so its top face is the
/// plane. Raises `groundTop` to the plane's world height if it is horizontal.
/// Returns false, changing nothing, for other shapes.
inline bool rebuildPlaneLikeGazebo(
    dynamics::ShapeNode& shapeNode, std::optional<double>& groundTop)
{
  const auto plane = std::dynamic_pointer_cast<const dynamics::PlaneShape>(
      shapeNode.getShape());
  if (!plane)
    return false;

  const Eigen::Vector3d normal = plane->getNormal();
  const Eigen::Isometry3d planeFrame
      = shapeNode.getRelativeTransform()
        * Eigen::Translation3d(plane->getOffset() * normal);

  // Same rotation as gz-physics, including its asin() angle.
  Eigen::Isometry3d boxFrame = Eigen::Isometry3d::Identity();
  const Eigen::Vector3d axis = Eigen::Vector3d::UnitZ().cross(normal);
  const double angle = std::asin(axis.norm() / normal.norm());
  if (angle > 1e-12)
    boxFrame.rotate(Eigen::AngleAxisd(angle, axis.normalized()));
  boxFrame.translate(Eigen::Vector3d(0.0, 0.0, -0.5 * kGazeboPlaneBoxSize));

  shapeNode.setShape(std::make_shared<dynamics::BoxShape>(
      Eigen::Vector3d::Constant(kGazeboPlaneBoxSize)));
  shapeNode.setRelativeTransform(planeFrame * boxFrame);

  const Eigen::Isometry3d worldPlane
      = shapeNode.getParentFrame()->getWorldTransform() * planeFrame;
  if ((worldPlane.linear() * normal).z() > 1.0 - 1e-6) {
    const double top = worldPlane.translation().z();
    groundTop = groundTop ? std::max(*groundTop, top) : top;
  }
  return true;
}

/// Rebuilds every collision PlaneShape like gz-physics (see
/// rebuildPlaneLikeGazebo()) and then gives the world's constraint solver a
/// fresh clone of its collision detector. Returns the world height of the
/// highest horizontal plane, if any.
inline std::optional<double> rebuildPlanesLikeGazebo(simulation::World& world)
{
  std::optional<double> groundTop;
  bool rebuilt = false;
  for (std::size_t i = 0; i < world.getNumSkeletons(); ++i) {
    const auto skeleton = world.getSkeleton(i);
    for (std::size_t j = 0; j < skeleton->getNumBodyNodes(); ++j) {
      skeleton->getBodyNode(j)->eachShapeNodeWith<dynamics::CollisionAspect>(
          [&](dynamics::ShapeNode* shapeNode) {
            rebuilt |= rebuildPlaneLikeGazebo(*shapeNode, groundTop);
          });
    }
  }

  // DART's ODE detector cannot refresh the collision object of a shape node
  // whose shape was replaced (the next step crashes), so give the solver
  // fresh collision objects.
  if (rebuilt) {
    auto* solver = world.getConstraintSolver();
    solver->setCollisionDetector(
        solver->getCollisionDetector()->cloneWithoutCollisionObjects());
    // The last collision result still points at the old detector's collision
    // objects, which the swap destroyed.
    solver->clearLastCollisionResult();
  }
  return groundTop;
}

/// Lowest world height of a collision shape: exact for spheres, boxes,
/// cylinders, capsules and ellipsoids, the lowest local bounding-box corner
/// otherwise.
inline double computeLowestPointZ(const dynamics::ShapeNode& shapeNode)
{
  const Eigen::Isometry3d& transform = shapeNode.getWorldTransform();
  // World +Z expressed in the shape frame.
  const Eigen::Vector3d up = transform.linear().row(2).transpose();
  const double z = transform.translation().z();
  const double axial = std::abs(up.z());
  const double radial = std::sqrt(std::max(0.0, 1.0 - up.z() * up.z()));
  const dynamics::Shape* shape = shapeNode.getShape().get();

  if (const auto* sphere = dynamic_cast<const dynamics::SphereShape*>(shape))
    return z - sphere->getRadius();
  if (const auto* box = dynamic_cast<const dynamics::BoxShape*>(shape))
    return z - 0.5 * box->getSize().dot(up.cwiseAbs());
  if (const auto* cylinder
      = dynamic_cast<const dynamics::CylinderShape*>(shape)) {
    return z - 0.5 * cylinder->getHeight() * axial
           - cylinder->getRadius() * radial;
  }
  if (const auto* capsule = dynamic_cast<const dynamics::CapsuleShape*>(shape))
    return z - 0.5 * capsule->getHeight() * axial - capsule->getRadius();
  if (const auto* ellipsoid
      = dynamic_cast<const dynamics::EllipsoidShape*>(shape)) {
    return z - ellipsoid->getRadii().cwiseProduct(up).norm();
  }

  const auto& bounds = shape->getBoundingBox();
  double lowest = std::numeric_limits<double>::infinity();
  for (int corner = 0; corner < 8; ++corner) {
    const Eigen::Vector3d point(
        (corner & 1) ? bounds.getMax().x() : bounds.getMin().x(),
        (corner & 2) ? bounds.getMax().y() : bounds.getMin().y(),
        (corner & 4) ? bounds.getMax().z() : bounds.getMin().z());
    lowest = std::min(lowest, (transform * point).z());
  }
  return lowest;
}

inline bool isSunk(const dynamics::Skeleton& skeleton, double groundTop)
{
  bool sunk = false;
  for (std::size_t i = 0; i < skeleton.getNumBodyNodes() && !sunk; ++i) {
    skeleton.getBodyNode(i)->eachShapeNodeWith<dynamics::CollisionAspect>(
        [&](const dynamics::ShapeNode* shapeNode) {
          sunk = computeLowestPointZ(*shapeNode)
                 < groundTop - kSunkDepthTolerance;
          return !sunk;
        });
  }
  return sunk;
}

/// Counts mobile skeletons with a collision shape more than
/// kSunkDepthTolerance below `groundTop`.
inline std::size_t countSunkSkeletons(
    const simulation::World& world, double groundTop)
{
  std::size_t sunk = 0;
  for (std::size_t i = 0; i < world.getNumSkeletons(); ++i) {
    const auto skeleton = world.getSkeleton(i);
    if (!skeleton->isMobile())
      continue;

    if (isSunk(*skeleton, groundTop))
      ++sunk;
  }
  return sunk;
}

namespace detail {

using ShapePair
    = std::pair<const dynamics::ShapeFrame*, const dynamics::ShapeFrame*>;

struct ShapePairHash
{
  std::size_t operator()(const ShapePair& pair) const
  {
    const auto first = std::hash<const void*>()(pair.first);
    return first
           ^ (std::hash<const void*>()(pair.second) + 0x9e3779b97f4a7c15ULL
              + (first << 6) + (first >> 2));
  }
};

using ShapePairSet = std::unordered_set<ShapePair, ShapePairHash>;

inline ShapePair shapePairOf(const collision::Contact& contact)
{
  const auto* a = contact.collisionObject1->getShapeFrame();
  const auto* b = contact.collisionObject2->getShapeFrame();
  return std::less<const void*>()(a, b) ? ShapePair(a, b) : ShapePair(b, a);
}

inline bool isAwakeMobile(const collision::CollisionObject* object)
{
  const auto* body = object->getBodyNode();
  if (!body)
    return false;
  const auto skeleton = body->getSkeleton();
  return skeleton->isMobile() && !skeleton->isResting();
}

} // namespace detail

/// The contacts a world's next step can detect when no global contact cap
/// truncates them; see measureGazeboContactDemand().
struct GazeboContactDemand
{
  /// Contacts the world's detector reports, before gz-physics' per-pair
  /// limit. A global cap truncates this stream, so it is what the cap starves.
  std::size_t rawContacts = 0;
  /// The same contacts after gz-physics' per-pair limit: what the detector
  /// hands DART's constraint solver when the cap is not reached.
  std::size_t contacts = 0;
  /// Shape pairs in contact.
  std::size_t pairs = 0;
  /// Shape pairs in contact that have an awake mobile body.
  detail::ShapePairSet awakePairs;
};

/// Detects the world's contacts at its current state with a fresh clone of its
/// detector (so the world's detector state is untouched) and no global contact
/// cap. Measured before World::step(), it sees the positions that step's
/// collision detection uses.
inline GazeboContactDemand measureGazeboContactDemand(
    const simulation::World& world)
{
  const auto* solver = world.getConstraintSolver();
  const auto detector
      = solver->getCollisionDetector()->cloneWithoutCollisionObjects();
  // Detect before gz-physics' per-pair limit, then apply it here.
  std::optional<std::size_t> pairLimit;
  if (auto* limit = dynamic_cast<GazeboCollisionPairLimit*>(detector.get())) {
    pairLimit = limit->getCollisionPairMaxContacts();
    limit->setCollisionPairMaxContacts(std::numeric_limits<std::size_t>::max());
  }

  const auto group = detector->createCollisionGroup();
  for (std::size_t i = 0; i < world.getNumSkeletons(); ++i)
    group->addShapeFramesOf(world.getSkeleton(i).get());

  collision::CollisionOption option = solver->getCollisionOption();
  option.maxNumContacts = std::numeric_limits<std::size_t>::max();
  collision::CollisionResult result;
  group->collide(option, &result);

  GazeboContactDemand demand;
  demand.rawContacts = result.getNumContacts();
  if (pairLimit)
    limitCollisionPairMaxContacts(result, *pairLimit);
  demand.contacts = result.getNumContacts();

  detail::ShapePairSet pairs;
  for (const auto& contact : result.getContacts()) {
    const detail::ShapePair pair = detail::shapePairOf(contact);
    if (!pairs.insert(pair).second)
      continue;
    if (detail::isAwakeMobile(contact.collisionObject1)
        || detail::isAwakeMobile(contact.collisionObject2)) {
      demand.awakePairs.insert(pair);
    }
  }
  demand.pairs = pairs.size();
  return demand;
}

/// Counts the pairs of `demand` (measured before a step) with an awake mobile
/// body that got no contact in that step's `solved` result: the pairs the
/// global contact cap starved.
inline std::size_t countStarvedPairs(
    const GazeboContactDemand& demand, const collision::CollisionResult& solved)
{
  detail::ShapePairSet solvedPairs;
  for (const auto& contact : solved.getContacts())
    solvedPairs.insert(detail::shapePairOf(contact));

  std::size_t starved = 0;
  for (const auto& pair : demand.awakePairs)
    starved += solvedPairs.count(pair) == 0u ? 1u : 0u;
  return starved;
}

/// Mirrors gz-physics SimulationFeatures::Write(ChangedWorldPoses), the poses
/// gz-sim copies every step: a body's pose is published when a position or
/// quaternion component differs from the last published pose by more than
/// kGazeboChangedPoseTolerance, so slow drift is published every few steps.
class ChangedPoseTracker
{
public:
  /// Publishes the pose of every body that changed since its last published
  /// pose (every body on the first call) and returns how many changed.
  std::size_t update(const simulation::World& world)
  {
    std::size_t index = 0;
    std::size_t changed = 0;
    for (std::size_t i = 0; i < world.getNumSkeletons(); ++i) {
      const auto skeleton = world.getSkeleton(i);
      for (std::size_t j = 0; j < skeleton->getNumBodyNodes(); ++j, ++index) {
        const Eigen::Isometry3d& transform
            = skeleton->getBodyNode(j)->getWorldTransform();
        const BodyPose pose{
            transform.translation(),
            Eigen::Quaterniond(transform.linear()).coeffs()};
        if (index >= mPublished.size()) {
          mPublished.push_back(pose);
          ++changed;
        } else if (!isSamePose(pose, mPublished[index])) {
          mPublished[index] = pose;
          ++changed;
        }
      }
    }
    mPublished.resize(index);
    return changed;
  }

private:
  struct BodyPose
  {
    Eigen::Vector3d position;
    Eigen::Vector4d rotation;
  };

  static bool isSamePose(const BodyPose& a, const BodyPose& b)
  {
    return (a.position - b.position).cwiseAbs().maxCoeff()
               <= kGazeboChangedPoseTolerance
           && (a.rotation - b.rotation).cwiseAbs().maxCoeff()
                  <= kGazeboChangedPoseTolerance;
  }

  std::vector<BodyPose> mPublished;
};

} // namespace dart::examples::contact_benchmark

#endif // DART_EXAMPLES_CONTACT_BENCHMARK_GAZEBO_PRESET_HPP_
