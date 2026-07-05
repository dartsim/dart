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

#include "dart/io/urdf_writer.hpp"

#include "dart/math/geometry.hpp"

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/cylinder_shape.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/joint.hpp>
#include <dart/dynamics/mesh_shape.hpp>
#include <dart/dynamics/mimic_dof_properties.hpp>
#include <dart/dynamics/planar_joint.hpp>
#include <dart/dynamics/prismatic_joint.hpp>
#include <dart/dynamics/revolute_joint.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/soft_body_node.hpp>
#include <dart/dynamics/sphere_shape.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <tinyxml2.h>

#include <algorithm>
#include <charconv>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>

#include <cmath>

namespace dart {
namespace io {

namespace {

using WriteError = common::Error;
using StringResult = common::Result<std::string, WriteError>;
using WriteResult = common::Result<void, WriteError>;
using LimitResult = common::Result<std::optional<double>, WriteError>;
using ScalarResult = common::Result<double, WriteError>;

struct LimitAttributes
{
  double mLower{0.0};
  double mUpper{0.0};
  double mVelocity{0.0};
  double mEffort{0.0};
};

struct ContinuousLimitAttributes
{
  double mVelocity{0.0};
  double mEffort{0.0};
};

using JointLimitResult
    = common::Result<std::optional<LimitAttributes>, WriteError>;
using ContinuousJointLimitResult
    = common::Result<std::optional<ContinuousLimitAttributes>, WriteError>;

constexpr double kTolerance = 1e-12;

WriteResult ok()
{
  return WriteResult::ok();
}

WriteResult fail(std::string message)
{
  return WriteResult::err(WriteError(std::move(message)));
}

std::string formatDouble(double value)
{
  char buffer[64];
  auto* end = buffer + sizeof(buffer);
  auto result = std::to_chars(
      buffer,
      end,
      value,
      std::chars_format::general,
      std::numeric_limits<double>::max_digits10);
  if (result.ec == std::errc{}) {
    return std::string(buffer, result.ptr);
  }

  return std::to_string(value);
}

void setDoubleAttribute(
    tinyxml2::XMLElement& element, const char* name, double value)
{
  element.SetAttribute(name, formatDouble(value).c_str());
}

std::string formatVector3(const Eigen::Vector3d& vector)
{
  return formatDouble(vector.x()) + " " + formatDouble(vector.y()) + " "
         + formatDouble(vector.z());
}

bool isFinite(double value)
{
  return std::isfinite(value);
}

bool isFinite(const Eigen::Vector3d& vector)
{
  return vector.allFinite();
}

bool isFinite(const Eigen::Matrix3d& matrix)
{
  return matrix.allFinite();
}

bool isFinite(const Eigen::Isometry3d& transform)
{
  return transform.matrix().allFinite();
}

bool isIdentity(const Eigen::Isometry3d& transform)
{
  return transform.matrix().isApprox(Eigen::Matrix4d::Identity(), kTolerance);
}

bool isSameScalar(double lhs, double rhs)
{
  if (std::isnan(lhs) || std::isnan(rhs)) {
    return false;
  }

  if (std::isinf(lhs) || std::isinf(rhs)) {
    return std::isinf(lhs) && std::isinf(rhs)
           && std::signbit(lhs) == std::signbit(rhs);
  }

  return std::abs(lhs - rhs) <= kTolerance;
}

bool isDefaultLowerLimit(double value)
{
  return std::isinf(value) && value < 0.0;
}

bool isDefaultUpperLimit(double value)
{
  return std::isinf(value) && value > 0.0;
}

bool isDefaultLimitPair(double lower, double upper)
{
  return isDefaultLowerLimit(lower) && isDefaultUpperLimit(upper);
}

std::string formatOriginRpy(const Eigen::Isometry3d& transform)
{
  const Eigen::Vector3d zyx = math::matrixToEulerZYX(transform.linear());
  return formatDouble(zyx.z()) + " " + formatDouble(zyx.y()) + " "
         + formatDouble(zyx.x());
}

tinyxml2::XMLElement* appendElement(
    tinyxml2::XMLDocument& doc, tinyxml2::XMLNode& parent, const char* name)
{
  auto* element = doc.NewElement(name);
  parent.InsertEndChild(element);
  return element;
}

void appendOrigin(
    tinyxml2::XMLDocument& doc,
    tinyxml2::XMLNode& parent,
    const Eigen::Isometry3d& transform)
{
  auto* origin = appendElement(doc, parent, "origin");
  origin->SetAttribute("xyz", formatVector3(transform.translation()).c_str());
  origin->SetAttribute("rpy", formatOriginRpy(transform).c_str());
}

std::string context(const dynamics::Joint& joint)
{
  return "URDF joint [" + joint.getName() + "]";
}

std::string context(const dynamics::BodyNode& body)
{
  return "URDF link [" + body.getName() + "]";
}

template <typename Getter>
ScalarResult uniformLimitValue(
    const dynamics::Joint& joint, Getter getter, std::string_view label)
{
  const double first = getter(0);
  if (std::isnan(first)) {
    return ScalarResult::err(WriteError(
        "Cannot write " + context(joint) + " with NaN " + std::string(label)
        + " limits."));
  }

  for (std::size_t i = 1; i < joint.getNumDofs(); ++i) {
    const double value = getter(i);
    if (std::isnan(value)) {
      return ScalarResult::err(WriteError(
          "Cannot write " + context(joint) + " with NaN " + std::string(label)
          + " limits."));
    }

    if (!isSameScalar(value, first)) {
      return ScalarResult::err(
          WriteError(
              "Cannot write " + context(joint) + " with non-uniform "
              + std::string(label)
              + " limits because URDF <limit> stores one scalar value for "
                "multi-DoF joints."));
    }
  }

  return ScalarResult::ok(first);
}

template <typename Getter>
ScalarResult uniformFiniteDofMetadata(
    const dynamics::Joint& joint, Getter getter, std::string_view label)
{
  const double first = getter(0);
  if (!isFinite(first)) {
    return ScalarResult::err(WriteError(
        "Cannot write " + context(joint) + " with non-finite "
        + std::string(label) + "."));
  }

  for (std::size_t i = 1; i < joint.getNumDofs(); ++i) {
    const double value = getter(i);
    if (!isFinite(value)) {
      return ScalarResult::err(WriteError(
          "Cannot write " + context(joint) + " with non-finite "
          + std::string(label) + "."));
    }

    if (!isSameScalar(value, first)) {
      return ScalarResult::err(WriteError(
          "Cannot write " + context(joint) + " with non-uniform "
          + std::string(label)
          + " because URDF stores one scalar value for multi-DoF joints."));
    }
  }

  return ScalarResult::ok(first);
}

LimitResult symmetricAbsoluteLimit(
    const dynamics::Joint& joint,
    double lower,
    double upper,
    std::string_view dartName,
    std::string_view urdfName)
{
  if (std::isnan(lower) || std::isnan(upper)) {
    return LimitResult::err(WriteError(
        "Cannot write " + context(joint) + " with NaN " + std::string(dartName)
        + " limits."));
  }

  const bool lowerIsDefault = std::isinf(lower) && lower < 0.0;
  const bool upperIsDefault = std::isinf(upper) && upper > 0.0;
  if (lowerIsDefault && upperIsDefault) {
    return LimitResult::ok(std::nullopt);
  }

  if (!std::isfinite(lower) || !std::isfinite(upper)) {
    return LimitResult::err(WriteError(
        "Cannot write " + context(joint) + " with asymmetric "
        + std::string(dartName) + " limits because URDF joint "
        + std::string(urdfName) + " stores a maximum absolute value."));
  }

  if (lower > upper) {
    return LimitResult::err(WriteError(
        "Cannot write " + context(joint) + " with invalid "
        + std::string(dartName) + " limits."));
  }

  if (upper < 0.0 || lower > 0.0 || std::abs(lower + upper) > kTolerance) {
    return LimitResult::err(WriteError(
        "Cannot write " + context(joint) + " with asymmetric "
        + std::string(dartName) + " limits because URDF joint "
        + std::string(urdfName) + " stores a maximum absolute value."));
  }

  return LimitResult::ok(std::max(std::abs(lower), std::abs(upper)));
}

JointLimitResult multiDofJointLimitAttributes(const dynamics::Joint& joint)
{
  const auto lower = uniformLimitValue(
      joint,
      [&](std::size_t i) { return joint.getPositionLowerLimit(i); },
      "position lower");
  if (lower.isErr()) {
    return JointLimitResult::err(lower.error());
  }

  const auto upper = uniformLimitValue(
      joint,
      [&](std::size_t i) { return joint.getPositionUpperLimit(i); },
      "position upper");
  if (upper.isErr()) {
    return JointLimitResult::err(upper.error());
  }

  const auto velocityLower = uniformLimitValue(
      joint,
      [&](std::size_t i) { return joint.getVelocityLowerLimit(i); },
      "velocity lower");
  if (velocityLower.isErr()) {
    return JointLimitResult::err(velocityLower.error());
  }

  const auto velocityUpper = uniformLimitValue(
      joint,
      [&](std::size_t i) { return joint.getVelocityUpperLimit(i); },
      "velocity upper");
  if (velocityUpper.isErr()) {
    return JointLimitResult::err(velocityUpper.error());
  }

  const auto forceLower = uniformLimitValue(
      joint,
      [&](std::size_t i) { return joint.getForceLowerLimit(i); },
      "force/effort lower");
  if (forceLower.isErr()) {
    return JointLimitResult::err(forceLower.error());
  }

  const auto forceUpper = uniformLimitValue(
      joint,
      [&](std::size_t i) { return joint.getForceUpperLimit(i); },
      "force/effort upper");
  if (forceUpper.isErr()) {
    return JointLimitResult::err(forceUpper.error());
  }

  const bool defaultPosition = isDefaultLimitPair(lower.value(), upper.value());
  const bool defaultVelocity
      = isDefaultLimitPair(velocityLower.value(), velocityUpper.value());
  const bool defaultForce
      = isDefaultLimitPair(forceLower.value(), forceUpper.value());

  if (defaultPosition && defaultVelocity && defaultForce) {
    return JointLimitResult::ok(std::nullopt);
  }

  if (!isFinite(lower.value()) || !isFinite(upper.value())) {
    return JointLimitResult::err(
        WriteError(
            "Cannot write " + context(joint)
            + " with partially unbounded multi-DoF position limits because "
              "URDF "
              "<limit> requires finite lower and upper values when authored."));
  }

  if (lower.value() > upper.value()) {
    return JointLimitResult::err(WriteError(
        "Cannot write " + context(joint)
        + " with invalid multi-DoF position limits."));
  }

  const auto velocityLimit = symmetricAbsoluteLimit(
      joint,
      velocityLower.value(),
      velocityUpper.value(),
      "velocity",
      "velocity");
  if (velocityLimit.isErr()) {
    return JointLimitResult::err(velocityLimit.error());
  }
  if (!velocityLimit.value()) {
    return JointLimitResult::err(WriteError(
        "Cannot write " + context(joint)
        + " without a finite symmetric URDF velocity limit."));
  }

  const auto effortLimit = symmetricAbsoluteLimit(
      joint, forceLower.value(), forceUpper.value(), "force/effort", "effort");
  if (effortLimit.isErr()) {
    return JointLimitResult::err(effortLimit.error());
  }
  if (!effortLimit.value()) {
    return JointLimitResult::err(WriteError(
        "Cannot write " + context(joint)
        + " without a finite symmetric URDF effort limit."));
  }

  return JointLimitResult::ok(
      LimitAttributes{
          lower.value(),
          upper.value(),
          *velocityLimit.value(),
          *effortLimit.value()});
}

ContinuousJointLimitResult continuousJointLimitAttributes(
    const dynamics::Joint& joint)
{
  const auto velocityLimit = symmetricAbsoluteLimit(
      joint,
      joint.getVelocityLowerLimit(0),
      joint.getVelocityUpperLimit(0),
      "velocity",
      "velocity");
  if (velocityLimit.isErr()) {
    return ContinuousJointLimitResult::err(velocityLimit.error());
  }

  const auto effortLimit = symmetricAbsoluteLimit(
      joint,
      joint.getForceLowerLimit(0),
      joint.getForceUpperLimit(0),
      "force/effort",
      "effort");
  if (effortLimit.isErr()) {
    return ContinuousJointLimitResult::err(effortLimit.error());
  }

  if (!velocityLimit.value() && !effortLimit.value()) {
    return ContinuousJointLimitResult::ok(std::nullopt);
  }
  if (!velocityLimit.value()) {
    return ContinuousJointLimitResult::err(WriteError(
        "Cannot write " + context(joint)
        + " without a finite symmetric URDF velocity limit."));
  }
  if (!effortLimit.value()) {
    return ContinuousJointLimitResult::err(WriteError(
        "Cannot write " + context(joint)
        + " without a finite symmetric URDF effort limit."));
  }

  return ContinuousJointLimitResult::ok(
      ContinuousLimitAttributes{*velocityLimit.value(), *effortLimit.value()});
}

void appendLimit(
    tinyxml2::XMLDocument& doc,
    tinyxml2::XMLElement& jointElement,
    const LimitAttributes& limit)
{
  auto* limitElement = appendElement(doc, jointElement, "limit");
  setDoubleAttribute(*limitElement, "lower", limit.mLower);
  setDoubleAttribute(*limitElement, "upper", limit.mUpper);
  setDoubleAttribute(*limitElement, "velocity", limit.mVelocity);
  setDoubleAttribute(*limitElement, "effort", limit.mEffort);
}

void appendContinuousLimit(
    tinyxml2::XMLDocument& doc,
    tinyxml2::XMLElement& jointElement,
    const ContinuousLimitAttributes& limit)
{
  auto* limitElement = appendElement(doc, jointElement, "limit");
  setDoubleAttribute(*limitElement, "velocity", limit.mVelocity);
  setDoubleAttribute(*limitElement, "effort", limit.mEffort);
}

WriteResult validateBody(const dynamics::BodyNode& body)
{
  const auto& inertia = body.getInertia();
  if (!isFinite(inertia.getMass()) || inertia.getMass() <= 0.0) {
    return fail(
        "Cannot write " + context(body) + " with non-finite or non-positive "
        "mass.");
  }

  if (!isFinite(inertia.getLocalCOM())) {
    return fail(
        "Cannot write " + context(body) + " with non-finite local center of "
        "mass.");
  }

  if (!isFinite(inertia.getMoment())) {
    return fail(
        "Cannot write " + context(body) + " with non-finite inertia matrix.");
  }

  return ok();
}

WriteResult writeInertial(
    tinyxml2::XMLDocument& doc,
    tinyxml2::XMLElement& linkElement,
    const dynamics::BodyNode& body)
{
  if (auto result = validateBody(body); result.isErr()) {
    return result;
  }

  auto* inertial = appendElement(doc, linkElement, "inertial");

  Eigen::Isometry3d origin = Eigen::Isometry3d::Identity();
  origin.translation() = body.getLocalCOM();
  appendOrigin(doc, *inertial, origin);

  auto* mass = appendElement(doc, *inertial, "mass");
  setDoubleAttribute(*mass, "value", body.getMass());

  const Eigen::Matrix3d moment = body.getInertia().getMoment();
  auto* inertia = appendElement(doc, *inertial, "inertia");
  setDoubleAttribute(*inertia, "ixx", moment(0, 0));
  setDoubleAttribute(*inertia, "iyy", moment(1, 1));
  setDoubleAttribute(*inertia, "izz", moment(2, 2));
  setDoubleAttribute(*inertia, "ixy", moment(0, 1));
  setDoubleAttribute(*inertia, "ixz", moment(0, 2));
  setDoubleAttribute(*inertia, "iyz", moment(1, 2));

  return ok();
}

WriteResult writeGeometry(
    tinyxml2::XMLDocument& doc,
    tinyxml2::XMLElement& parent,
    const dynamics::Shape& shape)
{
  auto* geometry = appendElement(doc, parent, "geometry");

  if (const auto* box = dynamic_cast<const dynamics::BoxShape*>(&shape)) {
    if (!isFinite(box->getSize())) {
      return fail("Cannot write URDF box geometry with non-finite size.");
    }

    auto* element = appendElement(doc, *geometry, "box");
    element->SetAttribute("size", formatVector3(box->getSize()).c_str());
    return ok();
  }

  if (const auto* sphere = dynamic_cast<const dynamics::SphereShape*>(&shape)) {
    if (!isFinite(sphere->getRadius()) || sphere->getRadius() < 0.0) {
      return fail("Cannot write URDF sphere geometry with invalid radius.");
    }

    auto* element = appendElement(doc, *geometry, "sphere");
    setDoubleAttribute(*element, "radius", sphere->getRadius());
    return ok();
  }

  if (const auto* cylinder
      = dynamic_cast<const dynamics::CylinderShape*>(&shape)) {
    if (!isFinite(cylinder->getRadius()) || !isFinite(cylinder->getHeight())
        || cylinder->getRadius() < 0.0 || cylinder->getHeight() < 0.0) {
      return fail(
          "Cannot write URDF cylinder geometry with invalid dimensions.");
    }

    auto* element = appendElement(doc, *geometry, "cylinder");
    setDoubleAttribute(*element, "radius", cylinder->getRadius());
    setDoubleAttribute(*element, "length", cylinder->getHeight());
    return ok();
  }

  if (const auto* mesh = dynamic_cast<const dynamics::MeshShape*>(&shape)) {
    const auto& uri = mesh->getMeshUri2();
    const std::string uriText = uri.toString();
    if (uriText.empty()) {
      return fail("Cannot write URDF mesh geometry without a mesh URI.");
    }

    const bool hasAbsolutePath = uri.mPath && uri.mPath->starts_with('/');
    if (!uri.mScheme && !hasAbsolutePath) {
      return fail(
          "Cannot write targetless relative URDF mesh URI [" + uriText + "].");
    }

    const bool isFileUri = uri.mScheme && *uri.mScheme == "file";
    const bool hasAuthority = uri.mAuthority && !uri.mAuthority->empty();
    if (isFileUri && (!hasAbsolutePath || hasAuthority)) {
      return fail(
          "Cannot write relative or host-qualified URDF file mesh URI ["
          + uriText + "] without a destination file policy.");
    }

    if (!isFinite(mesh->getScale())) {
      return fail("Cannot write URDF mesh geometry with non-finite scale.");
    }

    auto* element = appendElement(doc, *geometry, "mesh");
    element->SetAttribute("filename", uriText.c_str());
    element->SetAttribute("scale", formatVector3(mesh->getScale()).c_str());
    return ok();
  }

  return fail(
      "Cannot write DART shape [" + std::string(shape.getType())
      + "] as URDF geometry.");
}

WriteResult writeMaterial(
    tinyxml2::XMLDocument& doc,
    tinyxml2::XMLElement& visual,
    const dynamics::ShapeNode& shapeNode)
{
  const auto* visualAspect = shapeNode.getVisualAspect();
  if (!visualAspect) {
    return ok();
  }

  const double reflectance = visualAspect->getReflectance();
  if (reflectance >= 0.0 || !std::isfinite(reflectance)) {
    return fail(
        "Cannot write URDF visual [" + shapeNode.getName()
        + "] with a DART visual reflectance factor because URDF material has "
          "no reflectance field.");
  }

  const Eigen::Vector4d rgba = visualAspect->getRGBA();
  if (!rgba.allFinite()) {
    return fail(
        "Cannot write URDF visual [" + shapeNode.getName()
        + "] with non-finite material color.");
  }

  if (visualAspect->usesDefaultColor()
      && rgba.isApprox(dynamics::VisualAspect::getDefaultRGBA(), kTolerance)) {
    return ok();
  }

  auto* material = appendElement(doc, visual, "material");
  material->SetAttribute("name", (shapeNode.getName() + "_material").c_str());
  auto* color = appendElement(doc, *material, "color");
  color->SetAttribute(
      "rgba",
      (formatDouble(rgba.x()) + " " + formatDouble(rgba.y()) + " "
       + formatDouble(rgba.z()) + " " + formatDouble(rgba.w()))
          .c_str());
  return ok();
}

WriteResult validateCollisionMetadata(const dynamics::ShapeNode& shapeNode)
{
  const auto body = shapeNode.getBodyNodePtr();
  if (body && !body->isCollidable()) {
    return fail(
        "Cannot write URDF collision [" + shapeNode.getName()
        + "] on non-collidable DART BodyNode [" + body->getName()
        + "] because URDF <collision> has no link-level collidable-disable "
          "field.");
  }

  const auto* collisionAspect = shapeNode.getCollisionAspect();
  if (collisionAspect && !collisionAspect->isCollidable()) {
    return fail(
        "Cannot write URDF collision [" + shapeNode.getName()
        + "] with a disabled DART collision aspect because URDF <collision> "
          "has no collidable-disable field.");
  }

  const auto* dynamicsAspect = shapeNode.getDynamicsAspect();
  if (!dynamicsAspect) {
    return ok();
  }

  constexpr double kDefaultFriction = 1.0;
  constexpr double kDefaultRestitution = 0.0;
  constexpr double kDefaultSlip = -1.0;

  const Eigen::Vector3d& firstFrictionDirection
      = dynamicsAspect->getFirstFrictionDirection();
  const bool hasDefaultDynamics
      = isSameScalar(
            dynamicsAspect->getPrimaryFrictionCoeff(), kDefaultFriction)
        && isSameScalar(
            dynamicsAspect->getSecondaryFrictionCoeff(), kDefaultFriction)
        && isSameScalar(
            dynamicsAspect->getRestitutionCoeff(), kDefaultRestitution)
        && isSameScalar(
            dynamicsAspect->getPrimarySlipCompliance(), kDefaultSlip)
        && isSameScalar(
            dynamicsAspect->getSecondarySlipCompliance(), kDefaultSlip)
        && isFinite(firstFrictionDirection)
        && firstFrictionDirection.isZero(kTolerance)
        && dynamicsAspect->getFirstFrictionDirectionFrame() == nullptr;
  if (!hasDefaultDynamics) {
    return fail(
        "Cannot write URDF collision [" + shapeNode.getName()
        + "] with DART collision dynamics metadata because URDF <collision> "
          "has no surface or contact-dynamics fields.");
  }

  return ok();
}

template <class AspectT>
WriteResult writeShapeNodes(
    tinyxml2::XMLDocument& doc,
    tinyxml2::XMLElement& linkElement,
    const dynamics::BodyNode& body,
    const char* elementName)
{
  const std::size_t shapeCount = body.getNumShapeNodesWith<AspectT>();
  for (std::size_t i = 0; i < shapeCount; ++i) {
    const auto* shapeNode = body.getShapeNodeWith<AspectT>(i);
    if (!shapeNode) {
      continue;
    }

    if (!isFinite(shapeNode->getRelativeTransform())) {
      return fail(
          "Cannot write URDF shape [" + shapeNode->getName()
          + "] with non-finite local pose.");
    }

    if constexpr (std::is_same_v<AspectT, dynamics::CollisionAspect>) {
      if (auto result = validateCollisionMetadata(*shapeNode); result.isErr()) {
        return result;
      }
    }

    auto* element = appendElement(doc, linkElement, elementName);
    if (!shapeNode->getName().empty()) {
      element->SetAttribute("name", shapeNode->getName().c_str());
    }
    appendOrigin(doc, *element, shapeNode->getRelativeTransform());

    if (auto result = writeGeometry(doc, *element, *shapeNode->getShape());
        result.isErr()) {
      return result;
    }

    if constexpr (std::is_same_v<AspectT, dynamics::VisualAspect>) {
      if (auto result = writeMaterial(doc, *element, *shapeNode);
          result.isErr()) {
        return result;
      }
    }
  }

  return ok();
}

WriteResult writeLink(
    tinyxml2::XMLDocument& doc,
    tinyxml2::XMLElement& robot,
    const dynamics::BodyNode& body,
    const UrdfWriter::WriteOptions& options)
{
  if (dynamic_cast<const dynamics::SoftBodyNode*>(&body)) {
    return fail(
        "Cannot write DART SoftBodyNode [" + body.getName()
        + "] as URDF link because URDF has no point-mass, spring, damping, "
          "or soft mesh topology semantics.");
  }

  auto* link = appendElement(doc, robot, "link");
  link->SetAttribute("name", body.getName().c_str());

  if (auto result = writeInertial(doc, *link, body); result.isErr()) {
    return result;
  }

  if (options.includeVisuals) {
    if (auto result
        = writeShapeNodes<dynamics::VisualAspect>(doc, *link, body, "visual");
        result.isErr()) {
      return result;
    }
  }

  if (options.includeCollisions) {
    if (auto result = writeShapeNodes<dynamics::CollisionAspect>(
            doc, *link, body, "collision");
        result.isErr()) {
      return result;
    }
  }

  return ok();
}

WriteResult writeJointLimit(
    tinyxml2::XMLDocument& doc,
    tinyxml2::XMLElement& jointElement,
    const dynamics::Joint& joint,
    bool continuous)
{
  if (continuous) {
    const auto limit = continuousJointLimitAttributes(joint);
    if (limit.isErr()) {
      return WriteResult::err(limit.error());
    }

    if (limit.value()) {
      appendContinuousLimit(doc, jointElement, *limit.value());
    }

    return ok();
  }

  if (joint.getNumDofs() == 0) {
    return ok();
  }

  if (joint.getNumDofs() > 1) {
    const auto limit = multiDofJointLimitAttributes(joint);
    if (limit.isErr()) {
      return WriteResult::err(limit.error());
    }

    if (limit.value()) {
      appendLimit(doc, jointElement, *limit.value());
    }

    return ok();
  }

  const double lower = joint.getPositionLowerLimit(0);
  const double upper = joint.getPositionUpperLimit(0);
  if (!isFinite(lower) || !isFinite(upper)) {
    return fail(
        "Cannot write " + context(joint)
        + " with unbounded URDF position limits.");
  }
  if (lower > upper) {
    return fail("Cannot write " + context(joint) + " with invalid limits.");
  }

  const auto velocityLimit = symmetricAbsoluteLimit(
      joint,
      joint.getVelocityLowerLimit(0),
      joint.getVelocityUpperLimit(0),
      "velocity",
      "velocity");
  if (velocityLimit.isErr()) {
    return WriteResult::err(velocityLimit.error());
  }
  if (!velocityLimit.value()) {
    return fail(
        "Cannot write " + context(joint)
        + " without a finite symmetric URDF velocity limit.");
  }

  const auto effortLimit = symmetricAbsoluteLimit(
      joint,
      joint.getForceLowerLimit(0),
      joint.getForceUpperLimit(0),
      "force/effort",
      "effort");
  if (effortLimit.isErr()) {
    return WriteResult::err(effortLimit.error());
  }
  if (!effortLimit.value()) {
    return fail(
        "Cannot write " + context(joint)
        + " without a finite symmetric URDF effort limit.");
  }

  appendLimit(
      doc,
      jointElement,
      LimitAttributes{
          lower, upper, *velocityLimit.value(), *effortLimit.value()});
  return ok();
}

WriteResult writeJointDynamics(
    tinyxml2::XMLDocument& doc,
    tinyxml2::XMLElement& jointElement,
    const dynamics::Joint& joint)
{
  if (joint.getNumDofs() == 0) {
    return ok();
  }

  double damping = joint.getDampingCoefficient(0);
  double friction = joint.getCoulombFriction(0);

  if (joint.getNumDofs() == 1) {
    if (!isFinite(damping) || !isFinite(friction)) {
      return fail(
          "Cannot write " + context(joint)
          + " with non-finite dynamics metadata.");
    }
  } else {
    const auto uniformDamping = uniformFiniteDofMetadata(
        joint,
        [&](std::size_t i) { return joint.getDampingCoefficient(i); },
        "damping metadata");
    if (uniformDamping.isErr()) {
      return WriteResult::err(uniformDamping.error());
    }

    const auto uniformFriction = uniformFiniteDofMetadata(
        joint,
        [&](std::size_t i) { return joint.getCoulombFriction(i); },
        "friction metadata");
    if (uniformFriction.isErr()) {
      return WriteResult::err(uniformFriction.error());
    }

    damping = uniformDamping.value();
    friction = uniformFriction.value();
  }

  if (damping == 0.0 && friction == 0.0) {
    return ok();
  }

  auto* dynamics = appendElement(doc, jointElement, "dynamics");
  setDoubleAttribute(*dynamics, "damping", damping);
  setDoubleAttribute(*dynamics, "friction", friction);
  return ok();
}

WriteResult writeJointMimic(
    tinyxml2::XMLDocument& doc,
    tinyxml2::XMLElement& jointElement,
    const dynamics::Joint& joint)
{
  if (!joint.hasActuatorType(dynamics::Joint::MIMIC)) {
    return ok();
  }

  if (joint.getNumDofs() != 1
      || joint.getActuatorType(0) != dynamics::Joint::MIMIC) {
    return fail(
        "Cannot write " + context(joint)
        + " with mimic metadata because URDF <mimic> can only represent one "
          "dependent DoF.");
  }

  const auto mimicProps = joint.getMimicDofProperties();
  if (mimicProps.empty() || mimicProps[0].mReferenceJoint == nullptr) {
    return fail(
        "Cannot write " + context(joint)
        + " with mimic actuator because no reference joint is configured.");
  }

  const auto& mimic = mimicProps[0];
  if (!isFinite(mimic.mMultiplier) || !isFinite(mimic.mOffset)) {
    return fail(
        "Cannot write " + context(joint)
        + " with non-finite mimic multiplier or offset.");
  }

  if (mimic.mReferenceDofIndex != 0
      || mimic.mReferenceJoint->getNumDofs() != 1) {
    return fail(
        "Cannot write " + context(joint) + " with mimic reference joint ["
        + mimic.mReferenceJoint->getName()
        + "] because URDF <mimic> has no reference DoF selector.");
  }

  const auto jointSkeleton = joint.getSkeleton();
  const auto referenceSkeleton = mimic.mReferenceJoint->getSkeleton();
  if (!jointSkeleton || !referenceSkeleton
      || jointSkeleton.get() != referenceSkeleton.get()) {
    return fail(
        "Cannot write " + context(joint) + " with mimic reference joint ["
        + mimic.mReferenceJoint->getName()
        + "] because the reference joint is not in the same Skeleton.");
  }

  if (!mimic.mReferenceJoint->getParentBodyNode()) {
    return fail(
        "Cannot write " + context(joint)
        + " with mimic reference joint [" + mimic.mReferenceJoint->getName()
        + "] because URDF root links do not serialize parent-joint "
          "metadata.");
  }

  if (mimic.mConstraintType == dynamics::MimicConstraintType::Coupler) {
    if (std::abs(mimic.mOffset) > kTolerance) {
      return fail(
          "Cannot write " + context(joint)
          + " with a coupler mimic offset because URDF SimpleTransmission "
            "does not preserve mimic offsets.");
    }

    if (mimic.mMultiplier == 0.0) {
      return fail(
          "Cannot write " + context(joint)
          + " with a zero coupler mimic multiplier because URDF "
            "SimpleTransmission mechanical reductions must be nonzero.");
    }

    if (mimic.mReferenceJoint->getActuatorType() == dynamics::Joint::MIMIC) {
      return fail(
          "Cannot write " + context(joint) + " with mimic reference joint ["
          + mimic.mReferenceJoint->getName()
          + "] because URDF SimpleTransmission needs an independent reference "
            "joint.");
    }

    return ok();
  }

  auto* mimicElement = appendElement(doc, jointElement, "mimic");
  mimicElement->SetAttribute("joint", mimic.mReferenceJoint->getName().c_str());
  setDoubleAttribute(*mimicElement, "multiplier", mimic.mMultiplier);
  setDoubleAttribute(*mimicElement, "offset", mimic.mOffset);
  return ok();
}

WriteResult writeJoint(
    tinyxml2::XMLDocument& doc,
    tinyxml2::XMLElement& robot,
    const dynamics::Joint& joint)
{
  const auto* parent = joint.getParentBodyNode();
  const auto* child = joint.getChildBodyNode();
  if (!parent || !child) {
    return ok();
  }

  if (!isIdentity(joint.getTransformFromChildBodyNode())) {
    return fail(
        "Cannot write " + context(joint)
        + " because URDF requires the child link frame to coincide with the "
          "joint frame.");
  }

  if (!isFinite(joint.getTransformFromParentBodyNode())) {
    return fail(
        "Cannot write " + context(joint)
        + " with non-finite parent-to-joint transform.");
  }

  auto* jointElement = appendElement(doc, robot, "joint");
  jointElement->SetAttribute("name", joint.getName().c_str());

  bool continuous = false;
  if (const auto* revolute
      = dynamic_cast<const dynamics::RevoluteJoint*>(&joint)) {
    const bool unbounded = std::isinf(joint.getPositionLowerLimit(0))
                           && joint.getPositionLowerLimit(0) < 0.0
                           && std::isinf(joint.getPositionUpperLimit(0))
                           && joint.getPositionUpperLimit(0) > 0.0;
    continuous = unbounded || joint.isCyclic(0);
    jointElement->SetAttribute("type", continuous ? "continuous" : "revolute");

    auto* axis = appendElement(doc, *jointElement, "axis");
    if (!isFinite(revolute->getAxis())) {
      return fail("Cannot write " + context(joint) + " with non-finite axis.");
    }
    axis->SetAttribute("xyz", formatVector3(revolute->getAxis()).c_str());
  } else if (
      const auto* prismatic
      = dynamic_cast<const dynamics::PrismaticJoint*>(&joint)) {
    jointElement->SetAttribute("type", "prismatic");

    auto* axis = appendElement(doc, *jointElement, "axis");
    if (!isFinite(prismatic->getAxis())) {
      return fail("Cannot write " + context(joint) + " with non-finite axis.");
    }
    axis->SetAttribute("xyz", formatVector3(prismatic->getAxis()).c_str());
  } else if (
      const auto* planar = dynamic_cast<const dynamics::PlanarJoint*>(&joint)) {
    if (planar->getPlaneType() == dynamics::PlanarJoint::PlaneType::ARBITRARY) {
      return fail(
          "Cannot write " + context(joint)
          + " with arbitrary planar axes because URDF planar joints store only "
            "the plane normal.");
    }

    const Eigen::Vector3d axis = planar->getRotationalAxis();
    if (!isFinite(axis) || axis.norm() <= kTolerance) {
      return fail(
          "Cannot write " + context(joint) + " with invalid planar axis.");
    }

    jointElement->SetAttribute("type", "planar");
    auto* axisElement = appendElement(doc, *jointElement, "axis");
    axisElement->SetAttribute("xyz", formatVector3(axis.normalized()).c_str());
  } else if (dynamic_cast<const dynamics::FreeJoint*>(&joint)) {
    jointElement->SetAttribute("type", "floating");
  } else if (dynamic_cast<const dynamics::WeldJoint*>(&joint)) {
    jointElement->SetAttribute("type", "fixed");
  } else {
    return fail(
        "Cannot write DART joint [" + joint.getName() + "] of type ["
        + std::string(joint.getType()) + "] as URDF.");
  }

  appendOrigin(doc, *jointElement, joint.getTransformFromParentBodyNode());

  auto* parentElement = appendElement(doc, *jointElement, "parent");
  parentElement->SetAttribute("link", parent->getName().c_str());

  auto* childElement = appendElement(doc, *jointElement, "child");
  childElement->SetAttribute("link", child->getName().c_str());

  if (auto result = writeJointLimit(doc, *jointElement, joint, continuous);
      result.isErr()) {
    return result;
  }

  if (auto result = writeJointDynamics(doc, *jointElement, joint);
      result.isErr()) {
    return result;
  }

  if (auto result = writeJointMimic(doc, *jointElement, joint);
      result.isErr()) {
    return result;
  }

  return ok();
}

void writeSimpleTransmission(
    tinyxml2::XMLDocument& doc,
    tinyxml2::XMLElement& robot,
    const std::string& name,
    const std::string& actuatorName,
    const std::string& jointName,
    double mechanicalReduction)
{
  auto* transmission = appendElement(doc, robot, "transmission");
  transmission->SetAttribute("name", name.c_str());

  auto* type = appendElement(doc, *transmission, "type");
  type->SetText("transmission_interface/SimpleTransmission");

  auto* joint = appendElement(doc, *transmission, "joint");
  joint->SetAttribute("name", jointName.c_str());

  auto* actuator = appendElement(doc, *transmission, "actuator");
  actuator->SetAttribute("name", actuatorName.c_str());

  auto* reduction = appendElement(doc, *actuator, "mechanicalReduction");
  reduction->SetText(formatDouble(mechanicalReduction).c_str());
}

WriteResult writeCouplerTransmissions(
    tinyxml2::XMLDocument& doc,
    tinyxml2::XMLElement& robot,
    const dynamics::Skeleton& skeleton)
{
  for (std::size_t i = 0; i < skeleton.getNumJoints(); ++i) {
    const auto* joint = skeleton.getJoint(i);
    if (!joint || !joint->hasActuatorType(dynamics::Joint::MIMIC)
        || joint->getNumDofs() != 1
        || joint->getActuatorType(0) != dynamics::Joint::MIMIC) {
      continue;
    }

    const auto mimicProps = joint->getMimicDofProperties();
    if (mimicProps.empty()
        || mimicProps[0].mConstraintType
               != dynamics::MimicConstraintType::Coupler) {
      continue;
    }

    const auto& mimic = mimicProps[0];
    const auto* reference = mimic.mReferenceJoint;
    if (!reference) {
      return fail(
          "Cannot write " + context(*joint)
          + " with coupler transmission because no reference joint is "
            "configured.");
    }

    const std::string actuatorName = "dart_coupler_" + reference->getName()
                                     + "_" + joint->getName() + "_actuator";
    writeSimpleTransmission(
        doc,
        robot,
        "dart_coupler_" + reference->getName() + "_reference_for_"
            + joint->getName(),
        actuatorName,
        reference->getName(),
        mimic.mMultiplier);
    writeSimpleTransmission(
        doc,
        robot,
        "dart_coupler_" + joint->getName() + "_follower",
        actuatorName,
        joint->getName(),
        1.0);
  }

  return ok();
}

WriteResult validateRootJoint(const dynamics::BodyNode& root)
{
  const auto* joint = root.getParentJoint();
  if (!joint) {
    return fail("Cannot write URDF root link without a root joint.");
  }

  if (!dynamic_cast<const dynamics::FreeJoint*>(joint)
      && !dynamic_cast<const dynamics::WeldJoint*>(joint)) {
    return fail(
        "Cannot write URDF root joint [" + joint->getName()
        + "] because URDF root links do not carry parent-joint type metadata.");
  }

  if (!isIdentity(joint->getTransformFromParentBodyNode())
      || !isIdentity(joint->getTransformFromChildBodyNode())) {
    return fail(
        "Cannot write URDF root joint [" + joint->getName()
        + "] with non-identity placement because URDF has no root-link pose.");
  }

  if (joint->hasActuatorType(dynamics::Joint::MIMIC)) {
    return fail(
        "Cannot write URDF root joint [" + joint->getName()
        + "] with mimic metadata because URDF root links do not serialize "
          "parent-joint metadata.");
  }

  return ok();
}

} // namespace

namespace UrdfWriter {

common::Result<std::string, common::Error> tryWriteSkeletonToString(
    const dynamics::Skeleton& skeleton)
{
  return tryWriteSkeletonToString(skeleton, WriteOptions{});
}

common::Result<std::string, common::Error> tryWriteSkeletonToString(
    const dynamics::Skeleton& skeleton, const WriteOptions& options)
{
  if (skeleton.getNumBodyNodes() == 0) {
    return StringResult::err(
        WriteError("Cannot write URDF for an empty Skeleton."));
  }

  if (skeleton.getNumTrees() != 1) {
    return StringResult::err(WriteError(
        "Cannot write URDF for Skeleton [" + skeleton.getName()
        + "] with multiple root trees."));
  }

  const auto* root = skeleton.getRootBodyNode();
  if (auto result = validateRootJoint(*root); result.isErr()) {
    return StringResult::err(result.error());
  }

  tinyxml2::XMLDocument doc;
  doc.InsertEndChild(doc.NewDeclaration());
  auto* robot = doc.NewElement("robot");
  robot->SetAttribute("name", skeleton.getName().c_str());
  doc.InsertEndChild(robot);

  for (std::size_t i = 0; i < skeleton.getNumBodyNodes(); ++i) {
    const auto* body = skeleton.getBodyNode(i);
    if (!body) {
      continue;
    }

    if (auto result = writeLink(doc, *robot, *body, options); result.isErr()) {
      return StringResult::err(result.error());
    }
  }

  for (std::size_t i = 0; i < skeleton.getNumJoints(); ++i) {
    const auto* joint = skeleton.getJoint(i);
    if (!joint || !joint->getParentBodyNode()) {
      continue;
    }

    if (auto result = writeJoint(doc, *robot, *joint); result.isErr()) {
      return StringResult::err(result.error());
    }
  }

  if (auto result = writeCouplerTransmissions(doc, *robot, skeleton);
      result.isErr()) {
    return StringResult::err(result.error());
  }

  tinyxml2::XMLPrinter printer;
  doc.Print(&printer);
  return StringResult::ok(printer.CStr());
}

} // namespace UrdfWriter

} // namespace io
} // namespace dart
