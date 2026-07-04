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

#include "dart/utils/sdf/sdf_writer.hpp"

#include "dart/math/geometry.hpp"

#include <dart/dynamics/ball_joint.hpp>
#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/capsule_shape.hpp>
#include <dart/dynamics/cone_shape.hpp>
#include <dart/dynamics/convex_mesh_shape.hpp>
#include <dart/dynamics/cylinder_shape.hpp>
#include <dart/dynamics/ellipsoid_shape.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/heightmap_shape.hpp>
#include <dart/dynamics/joint.hpp>
#include <dart/dynamics/mesh_shape.hpp>
#include <dart/dynamics/plane_shape.hpp>
#include <dart/dynamics/prismatic_joint.hpp>
#include <dart/dynamics/revolute_joint.hpp>
#include <dart/dynamics/screw_joint.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/sphere_shape.hpp>
#include <dart/dynamics/universal_joint.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <gz/math/Inertial.hh>
#include <sdf/Box.hh>
#include <sdf/Capsule.hh>
#include <sdf/Collision.hh>
#include <sdf/Cone.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Ellipsoid.hh>
#include <sdf/Geometry.hh>
#include <sdf/Joint.hh>
#include <sdf/JointAxis.hh>
#include <sdf/Link.hh>
#include <sdf/Material.hh>
#include <sdf/Mesh.hh>
#include <sdf/Model.hh>
#include <sdf/Pbr.hh>
#include <sdf/Root.hh>
#include <sdf/Sphere.hh>
#include <sdf/Surface.hh>
#include <sdf/Visual.hh>
#include <sdf/World.hh>

#include <algorithm>
#include <charconv>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

#include <cmath>

namespace dart {
namespace utils {
namespace SdfParser {

namespace {

using WriteError = common::Error;
using WriteResult = common::Result<void, WriteError>;
using StringResult = common::Result<std::string, WriteError>;
using GeometryResult = common::Result<sdf::Geometry, WriteError>;

gz::math::Vector3d toGzVector3(const Eigen::Vector3d& vector)
{
  return gz::math::Vector3d(vector.x(), vector.y(), vector.z());
}

gz::math::Pose3d toGzPose(const Eigen::Isometry3d& transform)
{
  const Eigen::Vector3d zyx = math::matrixToEulerZYX(transform.linear());
  return gz::math::Pose3d(
      transform.translation().x(),
      transform.translation().y(),
      transform.translation().z(),
      zyx.z(),
      zyx.y(),
      zyx.x());
}

gz::math::Color toGzColor(const Eigen::Vector4d& color)
{
  return gz::math::Color(color.x(), color.y(), color.z(), color.w());
}

bool isFinite(const Eigen::Isometry3d& transform)
{
  return transform.matrix().allFinite();
}

bool isFinite(const Eigen::Matrix3d& matrix)
{
  return matrix.allFinite();
}

bool isFinite(const Eigen::Vector3d& vector)
{
  return vector.allFinite();
}

bool isFinite(const Eigen::Vector4d& vector)
{
  return vector.allFinite();
}

bool isZero(const Eigen::Vector3d& vector)
{
  return vector.isZero(0.0);
}

std::optional<std::pair<int, int>> parseSdfMajorMinor(std::string_view version)
{
  const auto dot = version.find('.');
  if (dot == std::string_view::npos || dot == 0 || dot + 1 >= version.size()) {
    return std::nullopt;
  }

  int major = 0;
  const auto majorResult
      = std::from_chars(version.data(), version.data() + dot, major);
  if (majorResult.ec != std::errc{}
      || majorResult.ptr != version.data() + dot) {
    return std::nullopt;
  }

  int minor = 0;
  const auto minorBegin = version.data() + dot + 1;
  const auto minorEnd = version.data() + version.size();
  const auto minorResult = std::from_chars(minorBegin, minorEnd, minor);
  if (minorResult.ec != std::errc{} || minorResult.ptr != minorEnd) {
    return std::nullopt;
  }

  return std::pair{major, minor};
}

bool isSdfVersionAtLeast(std::string_view version, int major, int minor)
{
  const auto parsed = parseSdfMajorMinor(version);
  if (!parsed) {
    return false;
  }

  return parsed->first > major
         || (parsed->first == major && parsed->second >= minor);
}

bool hasScheme(const common::Uri& uri, std::string_view scheme)
{
  return uri.mScheme && *uri.mScheme == scheme;
}

bool hasNonEmptyAuthority(const common::Uri& uri)
{
  return uri.mAuthority && !uri.mAuthority->empty();
}

bool hasAbsolutePath(const common::Uri& uri)
{
  return uri.mPath && uri.mPath->starts_with('/');
}

WriteResult fail(std::string message)
{
  return WriteResult::err(WriteError(std::move(message)));
}

WriteResult ok()
{
  return WriteResult::ok();
}

bool isImplicitRootJoint(const dynamics::Joint& joint)
{
  return dynamic_cast<const dynamics::FreeJoint*>(&joint)
         || dynamic_cast<const dynamics::WeldJoint*>(&joint);
}

bool isExplicitWorldRootJoint(const dynamics::Joint& joint)
{
  return dynamic_cast<const dynamics::RevoluteJoint*>(&joint)
         || dynamic_cast<const dynamics::PrismaticJoint*>(&joint)
         || dynamic_cast<const dynamics::ScrewJoint*>(&joint)
         || dynamic_cast<const dynamics::UniversalJoint*>(&joint)
         || dynamic_cast<const dynamics::BallJoint*>(&joint);
}

// These helpers only patch sdformat's serialized Element tree for fields that
// typed DOM does not expose or ToElement() omits/emits under legacy names.
sdf::ElementPtr findChildElement(
    const sdf::ElementPtr& parent, std::string_view name)
{
  if (!parent || name.empty()) {
    return nullptr;
  }

  const std::string nameString(name);
  sdf::ElementPtr childElement = parent->GetFirstElement();
  if (!childElement) {
    return nullptr;
  }
  if (childElement->GetName() == nameString) {
    return childElement;
  }

  return childElement->GetNextElement(nameString);
}

sdf::ElementPtr findNextSiblingElement(
    const sdf::ElementPtr& element, std::string_view name)
{
  if (!element || name.empty()) {
    return nullptr;
  }

  return element->GetNextElement(std::string(name));
}

WriteResult setBoolElement(
    const sdf::ElementPtr& parent,
    std::string_view elementName,
    bool value,
    std::string_view context)
{
  sdf::ElementPtr element = findChildElement(parent, elementName);
  if (!element) {
    element = std::make_shared<sdf::Element>();
    element->SetName(std::string(elementName));
    element->AddValue("bool", value ? "true" : "false", false);
    parent->InsertElement(element);
  }

  if (!element->Set(value)) {
    return fail(
        "sdformat failed to serialize " + std::string(context) + " ["
        + std::string(elementName) + "].");
  }

  element->SetExplicitlySetInFile(true);
  return ok();
}

WriteResult setDoubleElement(
    const sdf::ElementPtr& parent,
    std::string_view elementName,
    double value,
    std::string_view context)
{
  sdf::ElementPtr element = findChildElement(parent, elementName);
  if (!element) {
    element = std::make_shared<sdf::Element>();
    element->SetName(std::string(elementName));
    element->AddValue("double", "0", false);
    parent->InsertElement(element);
  }

  if (!element->Set(value)) {
    return fail(
        "sdformat failed to serialize " + std::string(context) + " ["
        + std::string(elementName) + "].");
  }

  element->SetExplicitlySetInFile(true);
  return ok();
}

WriteResult findOrCreateElement(
    const sdf::ElementPtr& parent,
    std::string_view elementName,
    sdf::ElementPtr& element)
{
  element = findChildElement(parent, elementName);
  if (element) {
    return ok();
  }

  element = std::make_shared<sdf::Element>();
  element->SetName(std::string(elementName));
  parent->InsertElement(element);
  element->SetExplicitlySetInFile(true);
  return ok();
}

sdf::ElementPtr findModelElement(const sdf::ElementPtr& rootElement)
{
  if (const sdf::ElementPtr modelElement
      = findChildElement(rootElement, "model")) {
    return modelElement;
  }

  const sdf::ElementPtr worldElement = findChildElement(rootElement, "world");
  if (!worldElement) {
    return nullptr;
  }

  return findChildElement(worldElement, "model");
}

WriteResult preserveFalseLinkGravity(
    const sdf::ElementPtr& rootElement, const sdf::Model& model)
{
  const sdf::ElementPtr modelElement = findModelElement(rootElement);
  if (!modelElement) {
    return fail("sdformat failed to serialize the SDF model.");
  }

  sdf::ElementPtr linkElement = findChildElement(modelElement, "link");
  for (uint64_t i = 0; i < model.LinkCount(); ++i) {
    const sdf::Link* link = model.LinkByIndex(i);
    if (!link || !linkElement) {
      return fail("sdformat failed to serialize an SDF link.");
    }

    if (link->EnableGravity()) {
      linkElement = findNextSiblingElement(linkElement, "link");
      continue;
    }

    // sdformat's Link DOM stores this value but omits the <gravity> element
    // when serializing a freshly built link, so preserve false through the
    // sdformat Element tree before converting to text.
    if (auto result = setBoolElement(
            linkElement, "gravity", false, "SDF link [" + link->Name() + "]");
        result.isErr()) {
      return result;
    }

    linkElement = findNextSiblingElement(linkElement, "link");
  }

  return ok();
}

WriteResult setModernScrewThreadPitchElement(
    const sdf::ElementPtr& jointElement, double pitch, std::string_view context)
{
  sdf::ElementPtr pitchElement
      = findChildElement(jointElement, "screw_thread_pitch");
  if (!pitchElement) {
    pitchElement = findChildElement(jointElement, "thread_pitch");
    if (pitchElement) {
      pitchElement->SetName("screw_thread_pitch");
    }
  }

  while (sdf::ElementPtr legacyElement
         = findChildElement(jointElement, "thread_pitch")) {
    jointElement->RemoveChild(legacyElement);
  }

  return setDoubleElement(jointElement, "screw_thread_pitch", pitch, context);
}

WriteResult preserveModernScrewThreadPitch(
    const sdf::ElementPtr& rootElement,
    const sdf::Model& model,
    std::string_view version)
{
  if (!isSdfVersionAtLeast(version, 1, 10)) {
    return ok();
  }

  const sdf::ElementPtr modelElement = findModelElement(rootElement);
  if (!modelElement) {
    return fail("sdformat failed to serialize the SDF model.");
  }

  sdf::ElementPtr jointElement = findChildElement(modelElement, "joint");
  for (uint64_t i = 0; i < model.JointCount(); ++i) {
    const sdf::Joint* joint = model.JointByIndex(i);
    if (!joint || !jointElement) {
      return fail("sdformat failed to serialize an SDF joint.");
    }

    if (joint->Type() != sdf::JointType::SCREW) {
      jointElement = findNextSiblingElement(jointElement, "joint");
      continue;
    }

    // sdformat's Joint DOM stores the modern screw pitch value, but
    // ToElement() still materializes the deprecated sibling. Keep the value in
    // the sdformat Element tree and write the schema-preferred SDF 1.10+ name.
    if (auto result = setModernScrewThreadPitchElement(
            jointElement,
            joint->ScrewThreadPitch(),
            "SDF screw joint [" + joint->Name() + "]");
        result.isErr()) {
      return result;
    }

    jointElement = findNextSiblingElement(jointElement, "joint");
  }

  return ok();
}

WriteResult preserveCollisionRestitution(
    const sdf::ElementPtr& rootElement,
    const dynamics::Skeleton& skeleton,
    const WriteOptions& options)
{
  if (!options.includeCollisions) {
    return ok();
  }

  const sdf::ElementPtr modelElement = findModelElement(rootElement);
  if (!modelElement) {
    return fail("sdformat failed to serialize the SDF model.");
  }

  sdf::ElementPtr linkElement = findChildElement(modelElement, "link");
  for (std::size_t i = 0; i < skeleton.getNumBodyNodes(); ++i) {
    const auto* bodyNode = skeleton.getBodyNode(i);
    if (!bodyNode || !linkElement) {
      return fail("sdformat failed to serialize an SDF link.");
    }

    WriteResult result = ok();
    sdf::ElementPtr collisionElement
        = findChildElement(linkElement, "collision");
    bodyNode->eachShapeNodeWith<dynamics::CollisionAspect>(
        [&](const dynamics::ShapeNode* shapeNode) {
          if (result.isErr()) {
            return;
          }
          if (!collisionElement) {
            result = fail("sdformat failed to serialize an SDF collision.");
            return;
          }

          const auto* dynamicsAspect = shapeNode->getDynamicsAspect();
          if (dynamicsAspect) {
            const double restitution = dynamicsAspect->getRestitutionCoeff();
            if (restitution != 0.0) {
              sdf::ElementPtr surfaceElement;
              result = findOrCreateElement(
                  collisionElement, "surface", surfaceElement);
              if (result.isErr()) {
                return;
              }

              sdf::ElementPtr bounceElement;
              result = findOrCreateElement(
                  surfaceElement, "bounce", bounceElement);
              if (result.isErr()) {
                return;
              }

              const std::string context
                  = "SDF collision [" + shapeNode->getName() + "] bounce";
              result = setDoubleElement(
                  bounceElement,
                  "restitution_coefficient",
                  restitution,
                  context);
              if (result.isErr()) {
                return;
              }

              // DART stores only a restitution coefficient, not SDF's
              // velocity threshold. A zero threshold preserves the writer's
              // coefficient behavior for this supported subset.
              result
                  = setDoubleElement(bounceElement, "threshold", 0.0, context);
              if (result.isErr()) {
                return;
              }
            }
          }

          collisionElement
              = findNextSiblingElement(collisionElement, "collision");
        });
    if (result.isErr()) {
      return result;
    }

    linkElement = findNextSiblingElement(linkElement, "link");
  }

  return ok();
}

sdf::JointType jointType(const dynamics::Joint& joint)
{
  if (dynamic_cast<const dynamics::WeldJoint*>(&joint)) {
    return sdf::JointType::FIXED;
  }
  if (const auto* revolute
      = dynamic_cast<const dynamics::RevoluteJoint*>(&joint)) {
    return revolute->isCyclic(0) ? sdf::JointType::CONTINUOUS
                                 : sdf::JointType::REVOLUTE;
  }
  if (dynamic_cast<const dynamics::PrismaticJoint*>(&joint)) {
    return sdf::JointType::PRISMATIC;
  }
  if (dynamic_cast<const dynamics::ScrewJoint*>(&joint)) {
    return sdf::JointType::SCREW;
  }
  if (dynamic_cast<const dynamics::UniversalJoint*>(&joint)) {
    return sdf::JointType::UNIVERSAL;
  }
  if (dynamic_cast<const dynamics::BallJoint*>(&joint)) {
    return sdf::JointType::BALL;
  }
  return sdf::JointType::INVALID;
}

std::optional<Eigen::Vector3d> singleDofJointAxis(const dynamics::Joint& joint)
{
  if (const auto* revolute
      = dynamic_cast<const dynamics::RevoluteJoint*>(&joint)) {
    return revolute->getAxis();
  }
  if (const auto* prismatic
      = dynamic_cast<const dynamics::PrismaticJoint*>(&joint)) {
    return prismatic->getAxis();
  }
  if (const auto* screw = dynamic_cast<const dynamics::ScrewJoint*>(&joint)) {
    return screw->getAxis();
  }
  return std::nullopt;
}

WriteResult applyMimic(
    sdf::JointAxis& sdfAxis,
    const dynamics::Joint& joint,
    std::size_t dofIndex,
    const WriteOptions& options)
{
  const auto mimicProps = joint.getMimicDofProperties();
  if (dofIndex >= mimicProps.size()) {
    return ok();
  }

  const auto& mimic = mimicProps[dofIndex];
  if (mimic.mReferenceJoint == nullptr) {
    return ok();
  }

  if (!isSdfVersionAtLeast(options.version, 1, 11)) {
    return fail(
        "Cannot write SDF mimic metadata for joint [" + joint.getName()
        + "] with SDF version [" + options.version
        + "]; mimic requires SDF 1.11 or newer.");
  }
  if (mimic.mConstraintType != dynamics::MimicConstraintType::Motor) {
    return fail(
        "Cannot write SDF mimic metadata for joint [" + joint.getName()
        + "] with a coupler constraint because SDF mimic does not preserve "
          "DART's coupler enforcement mode.");
  }
  if (!std::isfinite(mimic.mMultiplier) || !std::isfinite(mimic.mOffset)) {
    return fail(
        "Cannot write SDF mimic metadata for joint [" + joint.getName()
        + "] with non-finite multiplier or offset.");
  }
  if (mimic.mReferenceDofIndex >= mimic.mReferenceJoint->getNumDofs()) {
    return fail(
        "Cannot write SDF mimic metadata for joint [" + joint.getName()
        + "] with a reference DoF outside joint ["
        + mimic.mReferenceJoint->getName() + "].");
  }
  if (mimic.mReferenceDofIndex > 1) {
    return fail(
        "Cannot write SDF mimic metadata for joint [" + joint.getName()
        + "] because SDF mimic can only reference axis or axis2.");
  }

  const auto jointSkeleton = joint.getSkeleton();
  const auto referenceSkeleton = mimic.mReferenceJoint->getSkeleton();
  if (!jointSkeleton || !referenceSkeleton
      || jointSkeleton.get() != referenceSkeleton.get()) {
    return fail(
        "Cannot write SDF mimic metadata for joint [" + joint.getName()
        + "] because the reference joint [" + mimic.mReferenceJoint->getName()
        + "] is not in the same Skeleton.");
  }

  sdfAxis.SetMimic(
      sdf::MimicConstraint(
          mimic.mReferenceJoint->getName(),
          mimic.mReferenceDofIndex == 1 ? "axis2" : "axis",
          mimic.mMultiplier,
          mimic.mOffset,
          0.0));

  return ok();
}

GeometryResult makeGeometry(const dynamics::Shape& shape)
{
  sdf::Geometry geometry;

  if (const auto* box = dynamic_cast<const dynamics::BoxShape*>(&shape)) {
    const Eigen::Vector3d& size = box->getSize();
    if (!isFinite(size)) {
      return GeometryResult::err(
          WriteError("Cannot write SDF box geometry with non-finite size."));
    }
    sdf::Box sdfBox;
    sdfBox.SetSize(toGzVector3(size));
    geometry.SetType(sdf::GeometryType::BOX);
    geometry.SetBoxShape(sdfBox);
  } else if (
      const auto* sphere = dynamic_cast<const dynamics::SphereShape*>(&shape)) {
    const double radius = sphere->getRadius();
    if (!std::isfinite(radius)) {
      return GeometryResult::err(WriteError(
          "Cannot write SDF sphere geometry with non-finite radius."));
    }
    sdf::Sphere sdfSphere;
    sdfSphere.SetRadius(radius);
    geometry.SetType(sdf::GeometryType::SPHERE);
    geometry.SetSphereShape(sdfSphere);
  } else if (
      const auto* cylinder
      = dynamic_cast<const dynamics::CylinderShape*>(&shape)) {
    const double radius = cylinder->getRadius();
    const double height = cylinder->getHeight();
    if (!std::isfinite(radius) || !std::isfinite(height)) {
      return GeometryResult::err(WriteError(
          "Cannot write SDF cylinder geometry with non-finite dimensions."));
    }
    sdf::Cylinder sdfCylinder;
    sdfCylinder.SetRadius(radius);
    sdfCylinder.SetLength(height);
    geometry.SetType(sdf::GeometryType::CYLINDER);
    geometry.SetCylinderShape(sdfCylinder);
  } else if (
      const auto* capsule
      = dynamic_cast<const dynamics::CapsuleShape*>(&shape)) {
    const double radius = capsule->getRadius();
    const double height = capsule->getHeight();
    if (!std::isfinite(radius) || !std::isfinite(height)) {
      return GeometryResult::err(WriteError(
          "Cannot write SDF capsule geometry with non-finite dimensions."));
    }
    sdf::Capsule sdfCapsule;
    sdfCapsule.SetRadius(radius);
    sdfCapsule.SetLength(height);
    geometry.SetType(sdf::GeometryType::CAPSULE);
    geometry.SetCapsuleShape(sdfCapsule);
  } else if (
      const auto* cone = dynamic_cast<const dynamics::ConeShape*>(&shape)) {
    const double radius = cone->getRadius();
    const double height = cone->getHeight();
    if (!std::isfinite(radius) || !std::isfinite(height)) {
      return GeometryResult::err(WriteError(
          "Cannot write SDF cone geometry with non-finite dimensions."));
    }
    sdf::Cone sdfCone;
    sdfCone.SetRadius(radius);
    sdfCone.SetLength(height);
    geometry.SetType(sdf::GeometryType::CONE);
    geometry.SetConeShape(sdfCone);
  } else if (
      const auto* ellipsoid
      = dynamic_cast<const dynamics::EllipsoidShape*>(&shape)) {
    const Eigen::Vector3d& diameters = ellipsoid->getDiameters();
    if (!isFinite(diameters)) {
      return GeometryResult::err(WriteError(
          "Cannot write SDF ellipsoid geometry with non-finite diameters."));
    }
    sdf::Ellipsoid sdfEllipsoid;
    sdfEllipsoid.SetRadii(toGzVector3(diameters * 0.5));
    geometry.SetType(sdf::GeometryType::ELLIPSOID);
    geometry.SetEllipsoidShape(sdfEllipsoid);
  } else if (dynamic_cast<const dynamics::PlaneShape*>(&shape)) {
    return GeometryResult::err(WriteError(
        "Cannot write DART PlaneShape as SDF plane geometry because DART "
        "planes do not carry the finite SDF plane size required for "
        "read/write/read."));
  } else if (
      dynamic_cast<const dynamics::HeightmapShapef*>(&shape)
      || dynamic_cast<const dynamics::HeightmapShaped*>(&shape)) {
    return GeometryResult::err(WriteError(
        "Cannot write DART HeightmapShape as SDF heightmap geometry because "
        "SDF heightmaps require a source heightmap URI, and the targetless "
        "string writer has no destination URI or resource policy for generated "
        "heightmap resources."));
  } else if (
      const auto* convexMesh
      = dynamic_cast<const dynamics::ConvexMeshShape*>(&shape)) {
    if (!convexMesh->hasValidMesh()) {
      return GeometryResult::err(WriteError(
          "Cannot write SDF convex mesh geometry without a valid mesh."));
    }
    return GeometryResult::err(WriteError(
        "Cannot write DART ConvexMeshShape as SDF mesh geometry because the "
        "writer has no target SDF URI for generated mesh resources."));
  } else if (
      const auto* mesh = dynamic_cast<const dynamics::MeshShape*>(&shape)) {
    const common::Uri& meshUri = mesh->getMeshUri2();
    const auto uri = meshUri.toString();
    if (uri.empty()
        || (hasScheme(meshUri, "file")
            && meshUri.getFilesystemPath().empty())) {
      return GeometryResult::err(
          WriteError("Cannot write SDF mesh geometry without a mesh URI."));
    }
    if (!meshUri.mScheme) {
      return GeometryResult::err(WriteError(
          "Cannot write SDF mesh geometry with a relative mesh URI because "
          "the writer has no target SDF URI for resolving resources."));
    }
    if (hasScheme(meshUri, "file")
        && (hasNonEmptyAuthority(meshUri) || !hasAbsolutePath(meshUri))) {
      return GeometryResult::err(WriteError(
          "Cannot write SDF mesh geometry with a relative or host-qualified "
          "file URI; use an absolute file URI or a non-file resource URI."));
    }
    const Eigen::Vector3d& scale = mesh->getScale();
    if (!isFinite(scale)) {
      return GeometryResult::err(
          WriteError("Cannot write SDF mesh geometry with non-finite scale."));
    }
    sdf::Mesh sdfMesh;
    sdfMesh.SetUri(uri);
    sdfMesh.SetScale(toGzVector3(scale));
    geometry.SetType(sdf::GeometryType::MESH);
    geometry.SetMeshShape(sdfMesh);
  } else {
    return GeometryResult::err(WriteError(
        "Unsupported shape type for SDF writing: "
        + std::string(shape.getType())));
  }

  return GeometryResult::ok(geometry);
}

WriteResult applyMaterial(
    sdf::Visual& visual, const dynamics::ShapeNode& shapeNode)
{
  const auto* visualAspect = shapeNode.getVisualAspect();
  if (!visualAspect) {
    return ok();
  }

  sdf::Material material;
  bool hasMaterial = false;

  const double reflectance = visualAspect->getReflectance();
  if (reflectance >= 0.0 || !std::isfinite(reflectance)) {
    return fail(
        "Cannot write SDF visual [" + shapeNode.getName()
        + "] with a DART visual reflectance factor because the SDF material "
          "mapping has no equivalent reflectance field.");
  }

  if (!visualAspect->usesDefaultColor()) {
    const Eigen::Vector4d color = visualAspect->getRGBA();
    if (!isFinite(color)) {
      return fail(
          "Cannot write SDF visual [" + shapeNode.getName()
          + "] with a non-finite material color.");
    }

    material.SetDiffuse(toGzColor(color));
    hasMaterial = true;
  }

  const double metallic = visualAspect->getMetallic();
  const double roughness = visualAspect->getRoughness();
  const bool hasMetallic = metallic >= 0.0 || !std::isfinite(metallic);
  const bool hasRoughness = roughness >= 0.0 || !std::isfinite(roughness);
  if (hasMetallic || hasRoughness) {
    if ((hasMetallic && (!std::isfinite(metallic) || metallic > 1.0))
        || (hasRoughness && (!std::isfinite(roughness) || roughness > 1.0))) {
      return fail(
          "Cannot write SDF visual [" + shapeNode.getName()
          + "] with a non-finite or out-of-range PBR factor.");
    }

    sdf::PbrWorkflow workflow;
    workflow.SetType(sdf::PbrWorkflowType::METAL);
    if (hasMetallic) {
      workflow.SetMetalness(metallic);
    }
    if (hasRoughness) {
      workflow.SetRoughness(roughness);
    }

    sdf::Pbr pbr;
    pbr.SetWorkflow(sdf::PbrWorkflowType::METAL, workflow);
    material.SetPbrMaterial(pbr);
    hasMaterial = true;
  }

  if (hasMaterial) {
    visual.SetMaterial(material);
  }

  return ok();
}

WriteResult validateVisualMeshPolicy(const dynamics::ShapeNode& shapeNode)
{
  const auto shape = shapeNode.getShape();
  const auto* mesh = dynamic_cast<const dynamics::MeshShape*>(shape.get());
  if (!mesh) {
    return ok();
  }

  if (mesh->getColorMode() != dynamics::MeshShape::MATERIAL_COLOR) {
    return fail(
        "Cannot write SDF visual [" + shapeNode.getName()
        + "] with a non-default DART mesh color mode because the SDF mesh "
          "DOM has no equivalent render-policy field.");
  }

  if (mesh->getAlphaMode() != dynamics::MeshShape::BLEND) {
    return fail(
        "Cannot write SDF visual [" + shapeNode.getName()
        + "] with a non-default DART mesh alpha mode because the SDF mesh "
          "DOM has no equivalent render-policy field.");
  }

  return ok();
}

WriteResult applyCollisionSurface(
    sdf::Collision& collision, const dynamics::ShapeNode& shapeNode)
{
  const auto* collisionAspect = shapeNode.getCollisionAspect();
  const auto* bodyNode = shapeNode.getBodyNodePtr().get();
  const bool writeContact
      = (bodyNode != nullptr && !bodyNode->isCollidable())
        || (collisionAspect != nullptr && !collisionAspect->isCollidable());

  const auto* dynamicsAspect = shapeNode.getDynamicsAspect();
  if (!dynamicsAspect) {
    if (writeContact) {
      sdf::Contact contact;
      contact.SetCollideBitmask(0u);

      sdf::Surface surface;
      surface.SetContact(contact);
      collision.SetSurface(surface);
    }

    return ok();
  }

  constexpr double kDefaultFriction = 1.0;

  const double primaryFriction = dynamicsAspect->getPrimaryFrictionCoeff();
  const double secondaryFriction = dynamicsAspect->getSecondaryFrictionCoeff();
  const double primarySlip = dynamicsAspect->getPrimarySlipCompliance();
  const double secondarySlip = dynamicsAspect->getSecondarySlipCompliance();
  const double restitution = dynamicsAspect->getRestitutionCoeff();
  const Eigen::Vector3d& firstFrictionDirection
      = dynamicsAspect->getFirstFrictionDirection();

  if (!std::isfinite(primaryFriction) || primaryFriction < 0.0
      || !std::isfinite(secondaryFriction) || secondaryFriction < 0.0) {
    return fail(
        "Cannot write SDF collision [" + shapeNode.getName()
        + "] with non-finite or negative friction.");
  }
  if (!std::isfinite(primarySlip) || !std::isfinite(secondarySlip)) {
    return fail(
        "Cannot write SDF collision [" + shapeNode.getName()
        + "] with non-finite slip compliance.");
  }
  if (!std::isfinite(restitution) || restitution < 0.0 || restitution > 1.0) {
    return fail(
        "Cannot write SDF collision [" + shapeNode.getName()
        + "] with non-finite or out-of-range restitution.");
  }
  if (!isFinite(firstFrictionDirection)) {
    return fail(
        "Cannot write SDF collision [" + shapeNode.getName()
        + "] with a non-finite friction direction.");
  }

  const bool writePrimaryFriction = primaryFriction != kDefaultFriction;
  const bool writeSecondaryFriction = secondaryFriction != kDefaultFriction;
  const bool writePrimarySlip = primarySlip >= 0.0 && primarySlip != 0.0;
  const bool writeSecondarySlip = secondarySlip >= 0.0 && secondarySlip != 0.0;
  const bool writeFrictionDirection = !isZero(firstFrictionDirection);
  const bool writeBounce = restitution != 0.0;
  const bool writeFriction = writePrimaryFriction || writeSecondaryFriction
                             || writePrimarySlip || writeSecondarySlip
                             || writeFrictionDirection;
  if (!writeContact && !writeFriction && !writeBounce) {
    return ok();
  }

  if (writeFrictionDirection) {
    const auto* directionFrame
        = dynamicsAspect->getFirstFrictionDirectionFrame();
    if (directionFrame != nullptr && directionFrame != &shapeNode) {
      return fail(
          "Cannot write SDF collision [" + shapeNode.getName()
          + "] with a friction direction expressed in a non-collision frame.");
    }
  }

  sdf::Surface surface;
  if (writeContact) {
    sdf::Contact contact;
    contact.SetCollideBitmask(0u);
    surface.SetContact(contact);
  }
  if (writeFriction) {
    sdf::ODE ode;
    if (writePrimaryFriction) {
      ode.SetMu(primaryFriction);
    }
    if (writeSecondaryFriction) {
      ode.SetMu2(secondaryFriction);
    }
    if (writePrimarySlip) {
      ode.SetSlip1(primarySlip);
    }
    if (writeSecondarySlip) {
      ode.SetSlip2(secondarySlip);
    }
    if (writeFrictionDirection) {
      ode.SetFdir1(toGzVector3(firstFrictionDirection));
    }

    sdf::Friction friction;
    friction.SetODE(ode);
    surface.SetFriction(friction);
  }
  collision.SetSurface(surface);

  return ok();
}

WriteResult addShapeNode(
    sdf::Link& link, const dynamics::ShapeNode& shapeNode, bool visual)
{
  const auto shape = shapeNode.getShape();
  if (!shape) {
    return fail(
        "Cannot write SDF " + std::string(visual ? "visual" : "collision")
        + " [" + shapeNode.getName() + "] without a Shape.");
  }

  const Eigen::Isometry3d& pose = shapeNode.getRelativeTransform();
  if (!isFinite(pose)) {
    return fail(
        "Cannot write SDF " + std::string(visual ? "visual" : "collision")
        + " [" + shapeNode.getName() + "] with a non-finite pose.");
  }

  auto geometryResult = makeGeometry(*shape);
  if (geometryResult.isErr()) {
    return WriteResult::err(geometryResult.error());
  }

  if (visual) {
    if (auto result = validateVisualMeshPolicy(shapeNode); result.isErr()) {
      return result;
    }

    sdf::Visual sdfVisual;
    sdfVisual.SetName(shapeNode.getName());
    sdfVisual.SetRawPose(toGzPose(pose));
    sdfVisual.SetGeom(geometryResult.value());
    if (const auto* visualAspect = shapeNode.getVisualAspect()) {
      sdfVisual.SetCastShadows(visualAspect->getShadowed());
      if (visualAspect->isHidden()) {
        sdfVisual.SetVisibilityFlags(0u);
      }

      const double alpha = visualAspect->getAlpha();
      if (alpha != 1.0) {
        if (!std::isfinite(alpha) || alpha < 0.0 || alpha > 1.0) {
          return fail(
              "Cannot write SDF visual [" + shapeNode.getName()
              + "] with a non-finite or out-of-range visual transparency.");
        }

        sdfVisual.SetTransparency(static_cast<float>(1.0 - alpha));
      }
    }
    if (auto result = applyMaterial(sdfVisual, shapeNode); result.isErr()) {
      return result;
    }
    if (!link.AddVisual(sdfVisual)) {
      return fail(
          "Cannot write duplicate SDF visual [" + shapeNode.getName() + "].");
    }
  } else {
    sdf::Collision sdfCollision;
    sdfCollision.SetName(shapeNode.getName());
    sdfCollision.SetRawPose(toGzPose(pose));
    sdfCollision.SetGeom(geometryResult.value());
    if (auto result = applyCollisionSurface(sdfCollision, shapeNode);
        result.isErr()) {
      return result;
    }
    if (!link.AddCollision(sdfCollision)) {
      return fail(
          "Cannot write duplicate SDF collision [" + shapeNode.getName()
          + "].");
    }
  }

  return ok();
}

WriteResult buildLink(
    sdf::Link& link,
    const dynamics::BodyNode& bodyNode,
    const Eigen::Isometry3d& modelPose,
    const WriteOptions& options)
{
  if (!isFinite(modelPose)) {
    return fail(
        "Cannot write SDF link [" + bodyNode.getName()
        + "] with a non-finite pose.");
  }

  link.SetName(bodyNode.getName());
  link.SetRawPose(toGzPose(modelPose));
  link.SetEnableGravity(bodyNode.getGravityMode());

  const auto& inertia = bodyNode.getInertia();
  const Eigen::Matrix3d moment = inertia.getMoment();
  const Eigen::Vector3d& com = inertia.getLocalCOM();
  if (!std::isfinite(inertia.getMass()) || !isFinite(moment)
      || !isFinite(com)) {
    return fail(
        "Cannot write SDF link [" + bodyNode.getName()
        + "] with non-finite inertial data.");
  }

  gz::math::MassMatrix3d massMatrix;
  if (!massMatrix.SetMass(inertia.getMass())
      || !massMatrix.SetInertiaMatrix(
          moment(0, 0),
          moment(1, 1),
          moment(2, 2),
          moment(0, 1),
          moment(0, 2),
          moment(1, 2))) {
    return fail(
        "Cannot write SDF link [" + bodyNode.getName()
        + "] with invalid inertial data.");
  }

  gz::math::Inertiald gzInertial;
  if (!gzInertial.SetMassMatrix(massMatrix)
      || !gzInertial.SetPose(
          gz::math::Pose3d(com.x(), com.y(), com.z(), 0.0, 0.0, 0.0))
      || !link.SetInertial(gzInertial)) {
    return fail(
        "Cannot write SDF link [" + bodyNode.getName()
        + "] with invalid inertial data.");
  }

  if (options.includeVisuals) {
    WriteResult result = ok();
    bodyNode.eachShapeNodeWith<dynamics::VisualAspect>(
        [&](const dynamics::ShapeNode* shapeNode) {
          if (result.isErr()) {
            return;
          }
          result = addShapeNode(link, *shapeNode, true);
        });
    if (result.isErr()) {
      return result;
    }
  }

  if (options.includeCollisions) {
    WriteResult result = ok();
    bodyNode.eachShapeNodeWith<dynamics::CollisionAspect>(
        [&](const dynamics::ShapeNode* shapeNode) {
          if (result.isErr()) {
            return;
          }
          result = addShapeNode(link, *shapeNode, false);
        });
    if (result.isErr()) {
      return result;
    }
  }

  return ok();
}

WriteResult computeLinkModelPose(
    const dynamics::BodyNode& bodyNode,
    std::map<const dynamics::BodyNode*, Eigen::Isometry3d>& cache,
    Eigen::Isometry3d& pose)
{
  if (const auto it = cache.find(&bodyNode); it != cache.end()) {
    pose = it->second;
    return ok();
  }

  const dynamics::Joint* joint = bodyNode.getParentJoint();
  if (!joint) {
    return fail(
        "Cannot write SDF link [" + bodyNode.getName()
        + "] without a parent joint.");
  }

  Eigen::Isometry3d parentPose = Eigen::Isometry3d::Identity();
  if (const dynamics::BodyNode* parent = joint->getParentBodyNode()) {
    if (auto result = computeLinkModelPose(*parent, cache, parentPose);
        result.isErr()) {
      return result;
    }
  } else if (
      !isImplicitRootJoint(*joint) && !isExplicitWorldRootJoint(*joint)) {
    return fail(
        "Unsupported SDF root joint type [" + std::string(joint->getType())
        + "] for link [" + bodyNode.getName() + "].");
  }

  pose = parentPose * joint->getTransformFromParentBodyNode()
         * joint->getTransformFromChildBodyNode().inverse();
  cache.emplace(&bodyNode, pose);
  return ok();
}

WriteResult configureAxis(
    sdf::JointAxis& sdfAxis,
    const dynamics::Joint& joint,
    const Eigen::Vector3d& axis,
    std::size_t dofIndex,
    const WriteOptions& options)
{
  if (dofIndex >= joint.getNumDofs()) {
    return fail(
        "Cannot write SDF joint [" + joint.getName()
        + "] axis for missing DoF index " + std::to_string(dofIndex) + ".");
  }
  if (!isFinite(axis)) {
    return fail(
        "Cannot write SDF joint [" + joint.getName()
        + "] with a non-finite axis.");
  }

  const auto errors = sdfAxis.SetXyz(toGzVector3(axis));
  if (!errors.empty()) {
    return fail(
        "Cannot write SDF joint [" + joint.getName()
        + "] with an invalid axis.");
  }

  const double lower = joint.getPositionLowerLimit(dofIndex);
  const double upper = joint.getPositionUpperLimit(dofIndex);
  if (std::isfinite(lower)) {
    sdfAxis.SetLower(lower);
  }
  if (std::isfinite(upper)) {
    sdfAxis.SetUpper(upper);
  }

  const double damping = joint.getDampingCoefficient(dofIndex);
  const double friction = joint.getCoulombFriction(dofIndex);
  const double springReference = joint.getRestPosition(dofIndex);
  const double springStiffness = joint.getSpringStiffness(dofIndex);
  if (!std::isfinite(damping) || !std::isfinite(friction)
      || !std::isfinite(springReference) || !std::isfinite(springStiffness)) {
    return fail(
        "Cannot write SDF joint [" + joint.getName()
        + "] with non-finite dynamics.");
  }

  sdfAxis.SetDamping(damping);
  sdfAxis.SetFriction(friction);
  sdfAxis.SetSpringReference(springReference);
  sdfAxis.SetSpringStiffness(springStiffness);

  if (auto result = applyMimic(sdfAxis, joint, dofIndex, options);
      result.isErr()) {
    return result;
  }

  return ok();
}

WriteResult validateBallJointState(const dynamics::BallJoint& joint)
{
  for (std::size_t i = 0; i < joint.getNumDofs(); ++i) {
    const double lower = joint.getPositionLowerLimit(i);
    const double upper = joint.getPositionUpperLimit(i);
    const double damping = joint.getDampingCoefficient(i);
    const double friction = joint.getCoulombFriction(i);
    const double springReference = joint.getRestPosition(i);
    const double springStiffness = joint.getSpringStiffness(i);
    const auto mimicProps = joint.getMimicDofProperties();
    if (!std::isfinite(damping) || !std::isfinite(friction)
        || !std::isfinite(springReference) || !std::isfinite(springStiffness)) {
      return fail(
          "Cannot write SDF ball joint [" + joint.getName()
          + "] with non-finite dynamics.");
    }
    if (i < mimicProps.size() && mimicProps[i].mReferenceJoint != nullptr) {
      return fail(
          "Cannot write SDF ball joint [" + joint.getName()
          + "] with mimic metadata because DART's SDF ball joint round-trip "
            "does not represent axis metadata.");
    }
    if (std::isfinite(lower) || std::isfinite(upper)) {
      return fail(
          "Cannot write SDF ball joint [" + joint.getName()
          + "] with position limits because DART's SDF ball joint "
            "round-trip does not represent them.");
    }
    if (damping != 0.0 || friction != 0.0 || springReference != 0.0
        || springStiffness != 0.0) {
      return fail(
          "Cannot write SDF ball joint [" + joint.getName()
          + "] with dynamics because DART's SDF ball joint round-trip "
            "does not represent them.");
    }
  }

  return ok();
}

WriteResult buildJoint(
    sdf::Joint& sdfJoint,
    const dynamics::Joint& joint,
    const WriteOptions& options)
{
  const dynamics::BodyNode* parent = joint.getParentBodyNode();
  const dynamics::BodyNode* child = joint.getChildBodyNode();
  if (!parent) {
    if (isImplicitRootJoint(joint)) {
      return ok();
    }
    if (!isExplicitWorldRootJoint(joint)) {
      return fail(
          "Unsupported SDF root joint type [" + std::string(joint.getType())
          + "] for explicit parent-world joint [" + joint.getName() + "].");
    }
  }
  if (!child) {
    return fail(
        "Cannot write SDF joint [" + joint.getName()
        + "] without a child link.");
  }

  const sdf::JointType type = jointType(joint);
  if (type == sdf::JointType::INVALID) {
    return fail(
        "Unsupported joint type for SDF writing: "
        + std::string(joint.getType()) + " [" + joint.getName() + "].");
  }

  const Eigen::Isometry3d& childToJoint = joint.getTransformFromChildBodyNode();
  if (!isFinite(childToJoint)) {
    return fail(
        "Cannot write SDF joint [" + joint.getName()
        + "] with a non-finite pose.");
  }

  sdfJoint.SetName(joint.getName());
  sdfJoint.SetType(type);
  sdfJoint.SetParentName(parent ? parent->getName() : "world");
  sdfJoint.SetChildName(child->getName());
  sdfJoint.SetRawPose(toGzPose(childToJoint));

  if (const auto* screw = dynamic_cast<const dynamics::ScrewJoint*>(&joint)) {
    const double pitch = screw->getPitch();
    if (!std::isfinite(pitch)) {
      return fail(
          "Cannot write SDF screw joint [" + joint.getName()
          + "] with a non-finite pitch.");
    }
    sdfJoint.SetScrewThreadPitch(pitch);
  }
  if (const auto* universal
      = dynamic_cast<const dynamics::UniversalJoint*>(&joint)) {
    sdf::JointAxis axis1;
    if (auto result
        = configureAxis(axis1, joint, universal->getAxis1(), 0, options);
        result.isErr()) {
      return result;
    }
    sdfJoint.SetAxis(0, axis1);

    sdf::JointAxis axis2;
    if (auto result
        = configureAxis(axis2, joint, universal->getAxis2(), 1, options);
        result.isErr()) {
      return result;
    }
    sdfJoint.SetAxis(1, axis2);
  } else if (const auto axis = singleDofJointAxis(joint)) {
    sdf::JointAxis sdfAxis;
    if (auto result = configureAxis(sdfAxis, joint, *axis, 0, options);
        result.isErr()) {
      return result;
    }
    sdfJoint.SetAxis(0, sdfAxis);
  } else if (
      const auto* ball = dynamic_cast<const dynamics::BallJoint*>(&joint)) {
    if (auto result = validateBallJointState(*ball); result.isErr()) {
      return result;
    }
  }

  return ok();
}

Eigen::Vector3d defaultDartGravity()
{
  return Eigen::Vector3d(0.0, 0.0, -9.81);
}

bool requiresWorldGravity(const dynamics::Skeleton& skeleton)
{
  return !skeleton.getGravity().isApprox(defaultDartGravity(), 1e-12);
}

} // namespace

common::Result<std::string, common::Error> tryWriteSkeletonToString(
    const dynamics::Skeleton& skeleton, const WriteOptions& options)
{
  if (options.version.empty()) {
    return StringResult::err(WriteError("SDF version must not be empty."));
  }
  const Eigen::Vector3d& gravity = skeleton.getGravity();
  if (!isFinite(gravity)) {
    return StringResult::err(
        WriteError("Cannot write SDF with non-finite skeleton gravity."));
  }

  std::map<const dynamics::BodyNode*, Eigen::Isometry3d> linkModelPoses;
  for (std::size_t i = 0; i < skeleton.getNumBodyNodes(); ++i) {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    if (auto result
        = computeLinkModelPose(*skeleton.getBodyNode(i), linkModelPoses, pose);
        result.isErr()) {
      return StringResult::err(result.error());
    }
  }

  sdf::Model model;
  model.SetName(skeleton.getName());
  model.SetStatic(!skeleton.isMobile());
  model.SetSelfCollide(skeleton.getSelfCollisionCheck());

  for (std::size_t i = 0; i < skeleton.getNumBodyNodes(); ++i) {
    const auto* bodyNode = skeleton.getBodyNode(i);
    sdf::Link link;
    if (auto result
        = buildLink(link, *bodyNode, linkModelPoses.at(bodyNode), options);
        result.isErr()) {
      return StringResult::err(result.error());
    }
    if (!model.AddLink(link)) {
      return StringResult::err(WriteError(
          "Cannot write duplicate SDF link [" + bodyNode->getName() + "]."));
    }
  }

  for (std::size_t i = 0; i < skeleton.getNumJoints(); ++i) {
    const auto* joint = skeleton.getJoint(i);
    if (joint->getParentBodyNode() == nullptr && isImplicitRootJoint(*joint)) {
      continue;
    }

    sdf::Joint sdfJoint;
    if (auto result = buildJoint(sdfJoint, *joint, options); result.isErr()) {
      return StringResult::err(result.error());
    }
    if (!model.AddJoint(sdfJoint)) {
      return StringResult::err(WriteError(
          "Cannot write duplicate SDF joint [" + joint->getName() + "]."));
    }
  }

  sdf::Root root;
  root.SetVersion(options.version);
  if (requiresWorldGravity(skeleton)) {
    sdf::World world;
    world.SetName(
        skeleton.getName().empty() ? "default" : skeleton.getName() + "_world");
    world.SetGravity(toGzVector3(gravity));
    if (!world.AddModel(model)) {
      return StringResult::err(WriteError(
          "Failed to add SDF model [" + skeleton.getName()
          + "] to world for gravity serialization."));
    }

    const sdf::Errors errors = root.AddWorld(world);
    if (!errors.empty()) {
      return StringResult::err(
          WriteError("Failed to add SDF world for gravity serialization."));
    }
  } else {
    root.SetModel(model);
  }

  const sdf::ElementPtr element = root.ToElement();
  if (!element) {
    return StringResult::err(WriteError("sdformat failed to serialize SDF."));
  }
  if (auto result = preserveFalseLinkGravity(element, model); result.isErr()) {
    return StringResult::err(result.error());
  }
  if (auto result
      = preserveModernScrewThreadPitch(element, model, options.version);
      result.isErr()) {
    return StringResult::err(result.error());
  }
  if (auto result = preserveCollisionRestitution(element, skeleton, options);
      result.isErr()) {
    return StringResult::err(result.error());
  }

  return StringResult::ok(element->ToString(""));
}

} // namespace SdfParser
} // namespace utils
} // namespace dart
