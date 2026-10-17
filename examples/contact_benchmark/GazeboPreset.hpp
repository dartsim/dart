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

#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <sdf/Element.hh>
#include <sdf/ParserConfig.hh>
#include <sdf/Physics.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>
#include <sdf/parser.hh>

#include <algorithm>
#include <charconv>
#include <functional>
#include <initializer_list>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
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

/// Gravity inserted by sdformat's world XML schema when it is omitted.
constexpr double kGazeboDefaultGravity = -9.8;

/// Reads <max_contacts> from the first world's first physics profile, which
/// released gz-sim selects with PhysicsByIndex(0), even if a later profile is
/// marked default. Missing values use sdformat's default of 20.
inline std::size_t sdfCollisionPairMaxContacts(const sdf::Root& document)
{
  const auto* world = document.WorldByIndex(0);
  const auto* physics = world ? world->PhysicsByIndex(0) : nullptr;
  const auto element = physics ? physics->Element() : nullptr;
  const auto maxContacts
      = element ? element->FindElement("max_contacts") : nullptr;
  return maxContacts ? static_cast<std::size_t>(maxContacts->Get<int>())
                     : kGazeboDefaultCollisionPairMaxContacts;
}

/// Reads <max_step_size> from the first world's first physics profile, using
/// the same selection released gz-sim applies in its benchmark reference.
inline double sdfFirstPhysicsMaxStepSize(const sdf::Root& document)
{
  const auto* world = document.WorldByIndex(0);
  const auto* physics = world ? world->PhysicsByIndex(0) : nullptr;
  return physics ? physics->MaxStepSize() : 0.001;
}

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
/// filter a pair (see findUnsupportedGazeboPresetSdf()) it makes the same
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

namespace detail {

inline bool isOneOf(
    std::string_view value, std::initializer_list<std::string_view> choices)
{
  return std::find(choices.begin(), choices.end(), value) != choices.end();
}

// This is an allowlist of XML edges, not of names anywhere in the document.
// SdfParser::readWorld/readBodyNode/readShape/readJoint and released
// gz-physics dartsim SDFFeatures::ConstructSdf* are the two owners. In
// particular, capsule/ellipsoid are absent from DART's rigid shape reader,
// gz-physics falls back to fixed for revolute2.
inline bool supportsGazeboSdfChild(
    std::string_view parent, std::string_view name)
{
  if (parent.empty())
    return name == "sdf";
  if (parent == "sdf")
    return name == "world";
  if (parent == "world")
    return isOneOf(
        name,
        {"physics",
         "gravity",
         "model",
         "frame",
         "light",
         "scene",
         "gui",
         "plugin"});
  if (parent == "physics")
    return isOneOf(
        name,
        {"max_step_size",
         "real_time_factor",
         "real_time_update_rate",
         "max_contacts"});
  if (parent == "model")
    return isOneOf(
        name, {"static", "pose", "link", "joint", "frame", "self_collide"});
  if (parent == "frame")
    return name == "pose";
  if (parent == "link")
    return isOneOf(
        name,
        {"pose",
         "inertial",
         "collision",
         "visual",
         "sensor",
         "gravity",
         "self_collide",
         "kinematic"});
  if (parent == "inertial")
    return isOneOf(name, {"mass", "inertia", "pose"});
  if (parent == "inertia")
    return isOneOf(name, {"ixx", "iyy", "izz", "ixy", "ixz", "iyz"});
  if (parent == "collision")
    return isOneOf(name, {"pose", "geometry", "surface"});
  if (parent == "geometry")
    return isOneOf(name, {"box", "sphere", "cylinder", "plane"});
  if (parent == "visual_geometry")
    return name == "mesh";
  if (parent == "mesh")
    return isOneOf(name, {"uri", "scale"});
  if (parent == "box")
    return name == "size";
  if (parent == "sphere")
    return name == "radius";
  if (parent == "cylinder")
    return isOneOf(name, {"radius", "length"});
  if (parent == "plane")
    return isOneOf(name, {"normal", "size"});
  if (parent == "surface")
    return isOneOf(name, {"contact", "friction", "bounce"});
  if (parent == "friction")
    return name == "ode";
  if (parent == "ode")
    return isOneOf(name, {"mu", "mu2", "slip1", "slip2", "fdir1"});
  if (parent == "bounce")
    return name == "restitution_coefficient";
  if (parent == "contact")
    return isOneOf(name, {"collide_bitmask", "category_bitmask"});
  if (parent == "joint")
    return isOneOf(
        name,
        {"parent",
         "child",
         "pose",
         "axis",
         "axis2",
         "thread_pitch",
         "screw_thread_pitch"});
  if (isOneOf(parent, {"axis", "axis2"}))
    return isOneOf(
        name, {"xyz", "use_parent_model_frame", "dynamics", "limit"});
  if (parent == "dynamics")
    return isOneOf(
        name, {"damping", "friction", "spring_reference", "spring_stiffness"});
  if (parent == "limit")
    return isOneOf(name, {"lower", "upper", "effort", "velocity"});
  return false;
}

inline std::optional<std::string> findUnsupportedGazeboSdfElement(
    const sdf::Element& element,
    std::string_view parent,
    std::string_view elementPath);

inline std::optional<std::string> findUnsupportedIgnoredGazeboSdf(
    const sdf::Element& element, std::string_view elementPath)
{
  const std::string path(elementPath);
  // gz-sim also loads plugins attached to visuals and sensors as systems;
  // those subtrees are passive only in their absence.
  if (std::string_view(element.GetName()) == "plugin")
    return path + " can load a system that alters dynamics";
  for (auto child = element.GetFirstElement(); child;
       child = child->GetNextElement()) {
    if (auto unsupported = findUnsupportedIgnoredGazeboSdf(
            *child, path + "/" + child->GetName()))
      return unsupported;
  }
  if (std::string_view(element.GetName()) == "visual") {
    // SdfParser parses visuals while loading. Check the fields it reads;
    // their rendering properties otherwise do not constrain this subset.
    const auto geometry = element.FindElement("geometry");
    if (!geometry)
      return path + "/geometry is required by DART's visual loader";
    if (const auto pose = element.FindElement("pose")) {
      if (auto unsupported
          = findUnsupportedGazeboSdfElement(*pose, "collision", path + "/pose"))
        return unsupported;
    }
    for (auto shape = geometry->GetFirstElement(); shape;
         shape = shape->GetNextElement()) {
      const std::string_view shapeName = shape->GetName();
      if (isOneOf(shapeName, {"box", "sphere", "cylinder", "plane", "mesh"})) {
        if (auto unsupported = findUnsupportedGazeboSdfElement(
                *shape,
                shapeName == "mesh" ? "visual_geometry" : "geometry",
                path + "/geometry/" + shape->GetName()))
          return unsupported;
      }
    }
    const auto material = element.FindElement("material");
    const auto diffuse = material ? material->FindElement("diffuse") : nullptr;
    if (diffuse) {
      std::istringstream stream(
          diffuse->GetValue() ? diffuse->GetValue()->GetAsString() : "");
      double component = 0.0;
      std::size_t count = 0;
      while (stream >> component) {
        if (!std::isfinite(component))
          break;
        ++count;
      }
      if (!stream.eof() || (count != 3u && count != 4u))
        return path
               + "/material/diffuse must contain three or four finite numbers";
    }
  }
  return std::nullopt;
}

inline std::optional<std::string> findUnsupportedGazeboSdfElement(
    const sdf::Element& element,
    std::string_view parent,
    std::string_view elementPath)
{
  const std::string path(elementPath);
  const std::string_view name = element.GetName();
  if (!supportsGazeboSdfChild(parent, name))
    return path + " is outside the supported physics subset";

  // These subtrees create no physics entities or forces in the dartsim
  // builder. DART may load visuals, but they have no collision aspect.
  if (isOneOf(name, {"light", "scene", "gui", "visual", "sensor"}))
    return findUnsupportedIgnoredGazeboSdf(element, path);

  for (const auto& attribute : element.GetAttributes()) {
    const std::string& key = attribute->GetKey();
    const std::string value = attribute->GetAsString();
    bool supported = false;
    if (key == "name")
      supported = isOneOf(
                      name,
                      {"world",
                       "physics",
                       "model",
                       "link",
                       "collision",
                       "joint",
                       "frame",
                       "plugin"})
                  && !value.empty();
    else if (name == "sdf" && key == "version")
      supported = isOneOf(
          value,
          {"1.4", "1.5", "1.6", "1.7", "1.8", "1.9", "1.10", "1.11", "1.12"});
    else if (name == "pose") {
      supported
          = key == "relative_to" || (key == "frame" && value.empty())
            || (key == "degrees" && isOneOf(value, {"true", "false", "1", "0"}))
            || (key == "rotation_format"
                && isOneOf(value, {"euler_rpy", "quat_xyzw"}));
    } else if (name == "xyz" && key == "expressed_in")
      supported = true;
    else if (
        name == "model" && isOneOf(key, {"canonical_link", "placement_frame"}))
      supported = true;
    else if (name == "frame" && key == "attached_to")
      supported = true;
    else if (name == "inertial" && key == "auto")
      supported = isOneOf(value, {"false", "0"});
    else if (name == "physics") {
      supported
          = (key == "type"
             && isOneOf(value, {"ignored", "ode", "bullet", "simbody", "dart"}))
            || (key == "default"
                && isOneOf(value, {"true", "false", "1", "0"}));
    } else if (name == "joint" && key == "type")
      supported = isOneOf(
          value,
          {"fixed", "revolute", "prismatic", "universal", "ball", "screw"});
    else if (name == "plugin" && key == "filename")
      supported = true; // The filename/name pair is checked below.
    if (!supported)
      return path + "/@" + std::string(key) + "=\"" + std::string(value)
             + "\" is outside the supported physics subset";
  }

  if (name == "sdf" && !element.GetAttribute("version"))
    return path + "/@version is required";
  if (name == "plugin") {
    const auto filenameAttribute = element.GetAttribute("filename");
    const auto nameAttribute = element.GetAttribute("name");
    const std::string filename
        = filenameAttribute ? filenameAttribute->GetAsString() : "";
    const std::string pluginName
        = nameAttribute ? nameAttribute->GetAsString() : "";
    bool supported = false;
    // Accept only the empty standard systems in the benchmark worlds. Even
    // Physics configuration can select another engine, detector or solver.
    for (const auto system :
         {"physics", "user-commands", "scene-broadcaster"}) {
      const std::string_view className
          = std::string_view(system) == "physics"         ? "Physics"
            : std::string_view(system) == "user-commands" ? "UserCommands"
                                                          : "SceneBroadcaster";
      for (const bool legacy : {false, true}) {
        const std::string library
            = std::string(legacy ? "ignition-gazebo-" : "gz-sim-") + system
              + "-system";
        const std::string plugin
            = std::string(
                  legacy ? "ignition::gazebo::systems::" : "gz::sim::systems::")
              + std::string(className);
        supported = supported || (filename == library && pluginName == plugin);
      }
    }
    if (!supported)
      return path + " filename/name is not a supported standard system";
  }

  const std::string value
      = element.GetValue() ? element.GetValue()->GetAsString() : "";
  if (isOneOf(name, {"self_collide", "kinematic"})) {
    if (!isOneOf(value, {"false", "0"}))
      return path + " must be false";
  } else if (
      isOneOf(name, {"static", "use_parent_model_frame"})
      || (parent == "link" && name == "gravity")) {
    if (!isOneOf(value, {"true", "false", "1", "0"}))
      return path + " must be a boolean";
  }

  if (isOneOf(name, {"collide_bitmask", "category_bitmask"})) {
    // gz-physics Get<int> yields zero above INT_MAX. Retain all default
    // 0xff bits so its category/collide filter cannot drop any pair.
    unsigned mask = 0u;
    std::string_view token(value);
    const auto first = token.find_first_not_of(" \t\r\n");
    token = first == std::string_view::npos ? "" : token.substr(first);
    const auto last = token.find_last_not_of(" \t\r\n");
    token = last == std::string_view::npos ? "" : token.substr(0, last + 1);
    int base = 10;
    if (token.size() > 2 && token[0] == '0'
        && (token[1] == 'x' || token[1] == 'X')) {
      base = 16;
      token.remove_prefix(2);
    }
    const auto parsed = std::from_chars(
        token.data(), token.data() + token.size(), mask, base);
    if (parsed.ec != std::errc() || parsed.ptr != token.data() + token.size()
        || (mask & 0xffu) != 0xffu
        || mask > static_cast<unsigned>(std::numeric_limits<int>::max()))
      return path + " " + std::string(value) + " can filter collision pairs";
  }

  if (name == "pose" || (parent == "world" && name == "gravity")
      || name == "fdir1" || name == "xyz" || name == "normal" || name == "size"
      || name == "scale") {
    const bool quaternion
        = name == "pose" && element.GetAttribute("rotation_format")
          && element.GetAttribute("rotation_format")->GetAsString()
                 == "quat_xyzw";
    const std::size_t count = name == "pose" ? (quaternion ? 7u : 6u)
                              : name == "size" && parent == "plane" ? 2u
                                                                    : 3u;
    std::vector<double> values(count);
    std::istringstream stream{std::string(value)};
    for (auto& component : values) {
      if (!(stream >> component) || !std::isfinite(component))
        return path + " must contain " + std::to_string(count)
               + " finite numbers";
    }
    if (!(stream >> std::ws).eof())
      return path + " has extra components";
    if (quaternion
        && std::all_of(values.begin() + 3, values.end(), [](double number) {
             return number == 0.0;
           }))
      return path + " quaternion must be nonzero";
    if (quaternion) {
      Eigen::Vector4d robust(values[3], values[4], values[5], values[6]);
      robust /= robust.cwiseAbs().maxCoeff();
      robust.normalize();
      gz::math::Quaterniond native(values[6], values[3], values[4], values[5]);
      native.Normalize();
      const Eigen::Vector4d normalizedNative(
          native.X(), native.Y(), native.Z(), native.W());
      // DART protects its reader from overflow and tiny norms. Gazebo uses
      // native gz-math normalization, which can produce a different rotation.
      if (!normalizedNative.allFinite()
          || (normalizedNative - robust).norm() > 1e-12)
        return path + " quaternion normalization differs in Gazebo";
    }
    // Released gz-physics copies Moi() into the link frame without applying
    // the inertial pose rotation, while DART rotates the tensor into it.
    if (name == "pose" && parent == "inertial"
        && (values[3] != 0.0 || values[4] != 0.0 || values[5] != 0.0))
      return path + " rotation differs in Gazebo";
    if (name == "fdir1"
        && (values[0] != 0.0 || values[1] != 0.0 || values[2] != 0.0))
      return path + " must be the default 0 0 0";
    if (name == "size"
        && std::any_of(values.begin(), values.end(), [](double size) {
             return size <= 0.0;
           }))
      return path + " must be positive";
    if (name == "normal" || name == "xyz") {
      const double squaredNorm = values[0] * values[0] + values[1] * values[1]
                                 + values[2] * values[2];
      if (!std::isfinite(squaredNorm) || squaredNorm == 0.0)
        return path + " must be a finite nonzero vector";
      Eigen::Vector3d robust(values[0], values[1], values[2]);
      robust /= robust.cwiseAbs().maxCoeff();
      robust.normalize();
      gz::math::Vector3d native(values[0], values[1], values[2]);
      native.Normalize();
      const Eigen::Vector3d normalizedNative(
          native.X(), native.Y(), native.Z());
      if (!normalizedNative.allFinite()
          || (normalizedNative - robust).norm() > 1e-12)
        return path + " vector normalization differs in Gazebo";
    }
  }

  if (isOneOf(
          name,
          {"mass",
           "radius",
           "length",
           "thread_pitch",
           "screw_thread_pitch",
           "max_step_size",
           "real_time_factor",
           "real_time_update_rate",
           "ixx",
           "iyy",
           "izz",
           "ixy",
           "ixz",
           "iyz",
           "damping",
           "spring_reference",
           "spring_stiffness",
           "lower",
           "upper",
           "effort",
           "velocity",
           "mu",
           "mu2",
           "slip1",
           "slip2",
           "restitution_coefficient"})
      || (parent == "dynamics" && name == "friction")) {
    double number = 0.0;
    std::istringstream stream{std::string(value)};
    if (!(stream >> number) || !std::isfinite(number)
        || !(stream >> std::ws).eof())
      return path + " must contain one finite number";
    if (name == "thread_pitch" && number == 0.0)
      return path + " must be nonzero";
    if (isOneOf(name, {"mass", "radius", "length", "max_step_size"})
        && number <= 0.0)
      return path + " must be positive";
    if ((name == "lower" && number > 0.0) || (name == "upper" && number < 0.0))
      return path + " excludes zero; DART changes the initial joint position";
    if ((isOneOf(name, {"mu", "mu2"}) && number != 1.0)
        || (isOneOf(name, {"slip1", "slip2", "restitution_coefficient"})
            && number != 0.0))
      return path + " must retain the default contact material; DART's SDF parser ignores surface overrides";
  }
  if (name == "max_contacts") {
    int contacts = 0;
    std::istringstream stream{std::string(value)};
    if (!(stream >> contacts) || !(stream >> std::ws).eof() || contacts < 0)
      return path + " must be a nonnegative int";
  }

  if (parent == "joint" && isOneOf(name, {"axis", "axis2"})) {
    const auto joint = element.GetParent();
    const std::string type = joint->GetAttribute("type")
                                 ? joint->GetAttribute("type")->GetAsString()
                                 : "";
    if (!isOneOf(type, {"revolute", "prismatic", "universal", "screw"})
        || (name == "axis2" && type != "universal"))
      return path + " is unsupported for joint type " + std::string(type);
  }
  if (parent == "joint" && isOneOf(name, {"parent", "child"})) {
    const auto model = element.GetParent()->GetParent();
    bool found = name == "parent" && value == "world";
    for (auto link = model->FindElement("link"); link;
         link = link->GetNextElement("link")) {
      const auto linkName = link->GetAttribute("name");
      found = found || (linkName && value == linkName->GetAsString());
    }
    for (auto frame = model->FindElement("frame"); frame;
         frame = frame->GetNextElement("frame")) {
      const auto frameName = frame->GetAttribute("name");
      found = found || (frameName && value == frameName->GetAsString());
    }
    if (!found)
      return path + " must reference a link or frame in this model (or world as parent)";
  }

  // Require authored fields where the benchmark expects explicit values.
  // Canonical sdformat validation follows this subset check.
  const auto require = [&](const char* child) -> std::optional<std::string> {
    if (!element.FindElement(child))
      return path + "/" + std::string(child) + " is required";
    return std::nullopt;
  };
  if (isOneOf(name, {"world", "model", "link", "collision", "joint"})
      && !element.GetAttribute("name"))
    return path + "/@name is required";
  if (name == "joint") {
    if (!element.GetAttribute("type"))
      return path + "/@type is required";
    for (const auto child : {"parent", "child"}) {
      if (auto unsupported = require(child))
        return unsupported;
    }
    const std::string type = element.GetAttribute("type")->GetAsString();
    if (isOneOf(type, {"revolute", "prismatic", "universal", "screw"})) {
      if (auto unsupported = require("axis"))
        return unsupported;
    }
    if (type == "universal") {
      if (auto unsupported = require("axis2"))
        return unsupported;
    }
  }
  if (name == "inertia") {
    for (const auto child : {"ixx", "iyy", "izz", "ixy", "ixz", "iyz"}) {
      if (auto unsupported = require(child))
        return unsupported;
    }
  }
  if (name == "sdf" || name == "collision" || name == "box" || name == "sphere"
      || name == "cylinder" || name == "axis" || name == "axis2") {
    const char* child = name == "sdf"                           ? "world"
                        : name == "collision"                   ? "geometry"
                        : name == "box"                         ? "size"
                        : isOneOf(name, {"sphere", "cylinder"}) ? "radius"
                                                                : "xyz";
    if (auto unsupported = require(child))
      return unsupported;
    if (name == "cylinder") {
      if (auto unsupported = require("length"))
        return unsupported;
    }
  }
  if (name == "geometry" && !element.GetFirstElement())
    return path + " requires a supported primitive";

  std::unordered_map<std::string, std::unordered_set<std::string>> childNames;
  for (auto child = element.GetFirstElement(); child;
       child = child->GetNextElement()) {
    const std::string childPath = path + "/" + child->GetName();
    // Only one world/geometry/field is built. Repeated profiles and named
    // entities are the exceptions (the preset applies the first physics
    // profile's timestep and contact limit).
    if (!isOneOf(
            child->GetName(),
            {"physics",
             "model",
             "frame",
             "link",
             "joint",
             "collision",
             "visual",
             "sensor",
             "light",
             "plugin"})
        && element.FindElement(child->GetName()) != child)
      return childPath + " is repeated";
    if (name == "geometry" && child != element.GetFirstElement())
      return childPath + " is an extra geometry";
    if (const auto attribute = child->GetAttribute("name")) {
      const auto childName = attribute->GetAsString();
      if (!childNames[child->GetName()].insert(childName).second)
        return childPath + "/@name=\"" + childName + "\" is repeated";
    }
    if (auto unsupported
        = findUnsupportedGazeboSdfElement(*child, name, childPath))
      return unsupported;
  }
  if (name == "model") {
    // Both builders require a tree. DART otherwise follows an unbuilt
    // parent forever, while gz-physics rejects the closing joint.
    std::unordered_map<std::string, std::string> parents;
    for (auto joint = element.FindElement("joint"); joint;
         joint = joint->GetNextElement("joint")) {
      const std::string child
          = joint->FindElement("child")->GetValue()->GetAsString();
      const std::string parentName
          = joint->FindElement("parent")->GetValue()->GetAsString();
      if (!parents.emplace(child, parentName).second)
        return path + "/joint/child " + std::string(child)
               + " has multiple parent joints";
    }
    for (const auto& [child, parentName] : parents) {
      std::string ancestor = parentName;
      for (std::size_t depth = 0; depth <= parents.size(); ++depth) {
        if (ancestor == child)
          return path + "/joint/child " + std::string(child)
                 + " closes a joint chain";
        const auto next = parents.find(ancestor);
        if (next == parents.end())
          break;
        ancestor = next->second;
      }
    }
  }
  return std::nullopt;
}

} // namespace detail

/// Returns the first element path or attribute outside the physics subset
/// built equivalently by DART's SdfParser and released gz-physics dartsim.
/// Rendering and passive sensor subtrees are ignored; everything else must
/// have an explicitly supported parent, attributes and value.
inline std::optional<std::string> findUnsupportedGazeboPresetSdf(
    const std::string& xml, sdf::Root* loadedRoot = nullptr)
{
  // Inspect authored fields before sdformat inserts defaults or discards
  // unknown XML.
  auto document = std::make_shared<sdf::SDF>();
  auto element = std::make_shared<sdf::Element>();
  element->SetName("sdf");
  element->AddAttribute("version", "string", "", true);
  element->SetCopyChildren(true);
  document->SetRoot(element);
  sdf::ParserConfig config;
  sdf::Errors errors;
  if (!sdf::readStringWithoutConversion(xml, config, document, errors)
      || !errors.empty())
    return errors.empty() ? "missing /sdf" : "SDF: " + errors.front().Message();
  if (auto unsupported
      = detail::findUnsupportedGazeboSdfElement(*document->Root(), "", "/sdf"))
    return unsupported;
  sdf::Root root;
  errors = (loadedRoot ? loadedRoot : &root)->LoadSdfString(xml, config);
  if (!errors.empty())
    return "SDF: " + errors.front().Message();
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
