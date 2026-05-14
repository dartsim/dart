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

#include <dart/gui/experimental/scene.hpp>

#include <dart/simulation/world.hpp>

#include <dart/collision/collision_result.hpp>
#include <dart/collision/contact.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/capsule_shape.hpp>
#include <dart/dynamics/cone_shape.hpp>
#include <dart/dynamics/cylinder_shape.hpp>
#include <dart/dynamics/ellipsoid_shape.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/mesh_shape.hpp>
#include <dart/dynamics/multi_sphere_convex_hull_shape.hpp>
#include <dart/dynamics/plane_shape.hpp>
#include <dart/dynamics/pyramid_shape.hpp>
#include <dart/dynamics/shape.hpp>
#include <dart/dynamics/shape_frame.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/simple_frame.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/sphere_shape.hpp>

#include <dart/math/tri_mesh.hpp>

#include <algorithm>
#include <array>
#include <fstream>
#include <limits>
#include <string>
#include <utility>

#include <cmath>

namespace dart::gui::experimental {

RenderableId makeRenderableId(const dynamics::ShapeFrame& shapeFrame)
{
  return reinterpret_cast<RenderableId>(&shapeFrame);
}

RenderableId makeRenderableId(const dynamics::ShapeNode& shapeNode)
{
  return makeRenderableId(static_cast<const dynamics::ShapeFrame&>(shapeNode));
}

namespace {

void setLocalBounds(
    GeometryDescriptor& descriptor,
    const Eigen::Vector3d& min,
    const Eigen::Vector3d& max)
{
  descriptor.localBoundsMin = min;
  descriptor.localBoundsMax = max;
  descriptor.hasLocalBounds = true;
}

void setSymmetricLocalBounds(
    GeometryDescriptor& descriptor, const Eigen::Vector3d& halfExtents)
{
  setLocalBounds(descriptor, -halfExtents, halfExtents);
}

void setPlaneProxyBounds(
    GeometryDescriptor& descriptor,
    const Eigen::Vector3d& normal,
    double offset)
{
  static constexpr double halfExtent = 1.0;
  static constexpr double halfThickness = 0.02;

  Eigen::Vector3d unitNormal = normal;
  if (unitNormal.squaredNorm() < 1e-12) {
    unitNormal = Eigen::Vector3d::UnitZ();
  } else {
    unitNormal.normalize();
  }

  const Eigen::Vector3d seed = std::abs(unitNormal.z()) < 0.9
                                   ? Eigen::Vector3d::UnitZ()
                                   : Eigen::Vector3d::UnitX();
  const Eigen::Vector3d axisU = seed.cross(unitNormal).normalized();
  const Eigen::Vector3d axisV = unitNormal.cross(axisU).normalized();
  const Eigen::Vector3d center = unitNormal * offset;

  Eigen::Vector3d min = center;
  Eigen::Vector3d max = center;
  const std::array<Eigen::Vector3d, 6> points
      = {center - axisU * halfExtent - axisV * halfExtent,
         center + axisU * halfExtent - axisV * halfExtent,
         center + axisU * halfExtent + axisV * halfExtent,
         center - axisU * halfExtent + axisV * halfExtent,
         center - unitNormal * halfThickness,
         center + unitNormal * halfThickness};
  for (const Eigen::Vector3d& point : points) {
    min = min.cwiseMin(point);
    max = max.cwiseMax(point);
  }

  setLocalBounds(descriptor, min, max);
}

std::optional<double> intersectLocalBounds(
    const Eigen::Vector3d& origin,
    const Eigen::Vector3d& direction,
    const Eigen::Vector3d& boundsMin,
    const Eigen::Vector3d& boundsMax)
{
  static constexpr double epsilon = 1e-12;

  double tMin = 0.0;
  double tMax = std::numeric_limits<double>::infinity();

  for (int axis = 0; axis < 3; ++axis) {
    if (std::abs(direction[axis]) < epsilon) {
      if (origin[axis] < boundsMin[axis] || origin[axis] > boundsMax[axis]) {
        return std::nullopt;
      }
      continue;
    }

    double t0 = (boundsMin[axis] - origin[axis]) / direction[axis];
    double t1 = (boundsMax[axis] - origin[axis]) / direction[axis];
    if (t0 > t1) {
      std::swap(t0, t1);
    }
    tMin = std::max(tMin, t0);
    tMax = std::min(tMax, t1);
    if (tMax < tMin) {
      return std::nullopt;
    }
  }

  return tMin;
}

Eigen::Vector4d rgba(double red, double green, double blue, double alpha = 1.0)
{
  return {red, green, blue, alpha};
}

void appendLine(
    std::vector<DebugLineDescriptor>& lines,
    const Eigen::Vector3d& from,
    const Eigen::Vector3d& to,
    const Eigen::Vector4d& color,
    std::string label = {})
{
  DebugLineDescriptor line;
  line.from = from;
  line.to = to;
  line.rgba = color;
  line.label = std::move(label);
  lines.push_back(std::move(line));
}

void appendFrameAxes(
    std::vector<DebugLineDescriptor>& lines,
    const Eigen::Isometry3d& transform,
    double axisLength,
    const std::string& labelPrefix)
{
  if (axisLength <= 0.0 || !std::isfinite(axisLength)) {
    return;
  }

  const Eigen::Vector3d origin = transform.translation();
  const Eigen::Matrix3d rotation = transform.linear();
  appendLine(
      lines,
      origin,
      origin + rotation.col(0).normalized() * axisLength,
      rgba(0.9, 0.28, 0.28),
      labelPrefix.empty() ? "x" : labelPrefix + ".x");
  appendLine(
      lines,
      origin,
      origin + rotation.col(1).normalized() * axisLength,
      rgba(0.31, 0.75, 0.43),
      labelPrefix.empty() ? "y" : labelPrefix + ".y");
  appendLine(
      lines,
      origin,
      origin + rotation.col(2).normalized() * axisLength,
      rgba(0.28, 0.47, 0.92),
      labelPrefix.empty() ? "z" : labelPrefix + ".z");
}

void appendCenterOfMassMarker(
    std::vector<DebugLineDescriptor>& lines,
    const Eigen::Vector3d& center,
    double radius,
    const std::string& labelPrefix)
{
  if (radius <= 0.0 || !std::isfinite(radius)) {
    return;
  }

  const Eigen::Vector4d color = rgba(0.22, 0.82, 0.86);
  appendLine(
      lines,
      center - Eigen::Vector3d::UnitX() * radius,
      center + Eigen::Vector3d::UnitX() * radius,
      color,
      labelPrefix + ".com.x");
  appendLine(
      lines,
      center - Eigen::Vector3d::UnitY() * radius,
      center + Eigen::Vector3d::UnitY() * radius,
      color,
      labelPrefix + ".com.y");
  appendLine(
      lines,
      center - Eigen::Vector3d::UnitZ() * radius,
      center + Eigen::Vector3d::UnitZ() * radius,
      color,
      labelPrefix + ".com.z");
}

} // namespace

std::optional<GeometryDescriptor> describeShape(const dynamics::Shape& shape)
{
  GeometryDescriptor descriptor;
  descriptor.shapeType = std::string(shape.getType());

  if (const auto* box = dynamic_cast<const dynamics::BoxShape*>(&shape)) {
    descriptor.kind = ShapeKind::Box;
    descriptor.size = box->getSize();
    setSymmetricLocalBounds(descriptor, descriptor.size * 0.5);
    return descriptor;
  }

  if (const auto* sphere = dynamic_cast<const dynamics::SphereShape*>(&shape)) {
    descriptor.kind = ShapeKind::Sphere;
    descriptor.radius = sphere->getRadius();
    descriptor.size = Eigen::Vector3d::Constant(sphere->getRadius() * 2.0);
    setSymmetricLocalBounds(
        descriptor, Eigen::Vector3d::Constant(sphere->getRadius()));
    return descriptor;
  }

  if (const auto* ellipsoid
      = dynamic_cast<const dynamics::EllipsoidShape*>(&shape)) {
    descriptor.kind = ShapeKind::Ellipsoid;
    descriptor.size = ellipsoid->getDiameters();
    setSymmetricLocalBounds(descriptor, descriptor.size * 0.5);
    return descriptor;
  }

  if (const auto* cylinder
      = dynamic_cast<const dynamics::CylinderShape*>(&shape)) {
    descriptor.kind = ShapeKind::Cylinder;
    descriptor.radius = cylinder->getRadius();
    descriptor.height = cylinder->getHeight();
    descriptor.size = Eigen::Vector3d(
        cylinder->getRadius() * 2.0,
        cylinder->getRadius() * 2.0,
        cylinder->getHeight());
    setSymmetricLocalBounds(descriptor, descriptor.size * 0.5);
    return descriptor;
  }

  if (const auto* capsule
      = dynamic_cast<const dynamics::CapsuleShape*>(&shape)) {
    descriptor.kind = ShapeKind::Capsule;
    descriptor.radius = capsule->getRadius();
    descriptor.height = capsule->getHeight();
    descriptor.size = Eigen::Vector3d(
        capsule->getRadius() * 2.0,
        capsule->getRadius() * 2.0,
        capsule->getHeight() + capsule->getRadius() * 2.0);
    setSymmetricLocalBounds(descriptor, descriptor.size * 0.5);
    return descriptor;
  }

  if (const auto* cone = dynamic_cast<const dynamics::ConeShape*>(&shape)) {
    descriptor.kind = ShapeKind::Cone;
    descriptor.radius = cone->getRadius();
    descriptor.height = cone->getHeight();
    descriptor.size = Eigen::Vector3d(
        cone->getRadius() * 2.0, cone->getRadius() * 2.0, cone->getHeight());
    setSymmetricLocalBounds(descriptor, descriptor.size * 0.5);
    return descriptor;
  }

  if (const auto* pyramid
      = dynamic_cast<const dynamics::PyramidShape*>(&shape)) {
    descriptor.kind = ShapeKind::Pyramid;
    descriptor.height = pyramid->getHeight();
    descriptor.size = Eigen::Vector3d(
        pyramid->getBaseWidth(), pyramid->getBaseDepth(), pyramid->getHeight());
    setSymmetricLocalBounds(descriptor, descriptor.size * 0.5);
    return descriptor;
  }

  if (const auto* multiSphere
      = dynamic_cast<const dynamics::MultiSphereConvexHullShape*>(&shape)) {
    descriptor.kind = ShapeKind::MultiSphere;
    const auto& spheres = multiSphere->getSpheres();
    descriptor.sphereCenters.reserve(spheres.size());
    descriptor.sphereRadii.reserve(spheres.size());
    Eigen::Vector3d min = Eigen::Vector3d::Zero();
    Eigen::Vector3d max = Eigen::Vector3d::Zero();
    bool hasBounds = false;
    for (const auto& [radius, center] : spheres) {
      if (radius <= 0.0) {
        continue;
      }
      descriptor.sphereRadii.push_back(radius);
      descriptor.sphereCenters.push_back(center);
      descriptor.radius = std::max(descriptor.radius, radius);
      const Eigen::Vector3d extent = Eigen::Vector3d::Constant(radius);
      if (!hasBounds) {
        min = center - extent;
        max = center + extent;
        hasBounds = true;
      } else {
        min = min.cwiseMin(center - extent);
        max = max.cwiseMax(center + extent);
      }
    }
    if (hasBounds) {
      descriptor.size = max - min;
      setLocalBounds(descriptor, min, max);
    }
    return descriptor;
  }

  if (const auto* mesh = dynamic_cast<const dynamics::MeshShape*>(&shape)) {
    descriptor.kind = ShapeKind::Mesh;
    descriptor.scale = mesh->getScale();
    descriptor.meshUri = mesh->getMeshUri();
    const auto triMesh = mesh->getTriMesh();
    if (triMesh != nullptr && !triMesh->getVertices().empty()) {
      Eigen::Vector3d min
          = descriptor.scale.cwiseProduct(triMesh->getVertices().front());
      Eigen::Vector3d max = min;
      for (const Eigen::Vector3d& vertex : triMesh->getVertices()) {
        const Eigen::Vector3d scaled = descriptor.scale.cwiseProduct(vertex);
        min = min.cwiseMin(scaled);
        max = max.cwiseMax(scaled);
      }
      setLocalBounds(descriptor, min, max);
    }
    return descriptor;
  }

  if (const auto* plane = dynamic_cast<const dynamics::PlaneShape*>(&shape)) {
    descriptor.kind = ShapeKind::Plane;
    descriptor.normal = plane->getNormal();
    descriptor.offset = plane->getOffset();
    setPlaneProxyBounds(descriptor, descriptor.normal, descriptor.offset);
    return descriptor;
  }

  return std::nullopt;
}

std::optional<RenderableDescriptor> makeRenderableDescriptor(
    const dynamics::ShapeFrame& shapeFrame,
    const dynamics::VisualAspect& visualAspect,
    const std::string& skeletonName,
    const std::string& bodyName,
    const std::string& shapeNodeName,
    const dynamics::WeakConstSimpleFramePtr& simpleFrame = {})
{
  const auto shape = shapeFrame.getShape();
  if (!shape) {
    return std::nullopt;
  }

  auto geometry = describeShape(*shape);
  if (!geometry) {
    return std::nullopt;
  }

  RenderableDescriptor descriptor;
  descriptor.id = makeRenderableId(shapeFrame);
  descriptor.shapeFrame = &shapeFrame;
  descriptor.shapeNode = shapeFrame.asShapeNode();
  descriptor.shape = shape.get();
  if (descriptor.shapeNode != nullptr) {
    descriptor.skeleton = descriptor.shapeNode->getSkeleton();
  }
  descriptor.simpleFrame = simpleFrame;
  descriptor.skeletonName = skeletonName;
  descriptor.bodyName = bodyName;
  descriptor.shapeFrameName = shapeFrame.getName();
  descriptor.shapeNodeName = shapeNodeName;
  descriptor.geometry = std::move(*geometry);
  descriptor.material.rgba = visualAspect.getRGBA();
  descriptor.material.visible = !visualAspect.isHidden();
  descriptor.material.castsShadows = visualAspect.getShadowed();
  descriptor.material.receivesShadows = visualAspect.getShadowed();
  descriptor.worldTransform = shapeFrame.getWorldTransform();
  descriptor.shapeFrameVersion = shapeFrame.getVersion();
  descriptor.shapeNodeVersion = descriptor.shapeNode != nullptr
                                    ? descriptor.shapeNode->getVersion()
                                    : 0;
  descriptor.shapeVersion = shape->getVersion();
  return descriptor;
}

std::vector<RenderableDescriptor> extractRenderables(
    const simulation::World& world)
{
  std::vector<RenderableDescriptor> renderables;

  for (std::size_t skeletonIndex = 0; skeletonIndex < world.getNumSkeletons();
       ++skeletonIndex) {
    const auto skeleton = world.getSkeleton(skeletonIndex);
    if (!skeleton) {
      continue;
    }

    skeleton->eachBodyNode([&](const dynamics::BodyNode* bodyNode) {
      bodyNode->eachShapeNodeWith<dynamics::VisualAspect>(
          [&](const dynamics::ShapeNode* shapeNode) {
            const auto* visualAspect = shapeNode->getVisualAspect();
            auto descriptor = makeRenderableDescriptor(
                *shapeNode,
                *visualAspect,
                skeleton->getName(),
                bodyNode->getName(),
                shapeNode->getName());
            if (descriptor) {
              renderables.push_back(std::move(*descriptor));
            }
          });
    });
  }

  for (std::size_t i = 0; i < world.getNumSimpleFrames(); ++i) {
    const auto simpleFrame = world.getSimpleFrame(i);
    if (simpleFrame == nullptr || !simpleFrame->hasVisualAspect()) {
      continue;
    }

    auto descriptor = makeRenderableDescriptor(
        *simpleFrame, *simpleFrame->getVisualAspect(), {}, {}, {}, simpleFrame);
    if (descriptor) {
      renderables.push_back(std::move(*descriptor));
    }
  }

  return renderables;
}

std::optional<double> intersectRenderable(
    const RenderableDescriptor& renderable, const PickRay& ray)
{
  if (!renderable.material.visible || !renderable.geometry.hasLocalBounds) {
    return std::nullopt;
  }

  const double directionNorm = ray.direction.norm();
  if (!std::isfinite(directionNorm) || directionNorm < 1e-12) {
    return std::nullopt;
  }

  const Eigen::Vector3d direction = ray.direction / directionNorm;
  const Eigen::Isometry3d localFromWorld = renderable.worldTransform.inverse();
  const Eigen::Vector3d localOrigin = localFromWorld * ray.origin;
  const Eigen::Vector3d localDirection
      = (localFromWorld.linear() * direction).normalized();

  return intersectLocalBounds(
      localOrigin,
      localDirection,
      renderable.geometry.localBoundsMin,
      renderable.geometry.localBoundsMax);
}

std::optional<PickHit> pickNearestRenderable(
    const std::vector<RenderableDescriptor>& renderables,
    const PickRay& ray,
    double maxDistance)
{
  std::optional<PickHit> nearest;
  for (std::size_t i = 0; i < renderables.size(); ++i) {
    const std::optional<double> distance
        = intersectRenderable(renderables[i], ray);
    if (!distance || *distance > maxDistance) {
      continue;
    }
    if (!nearest || *distance < nearest->distance) {
      PickHit hit;
      hit.id = renderables[i].id;
      hit.renderableIndex = i;
      hit.distance = *distance;
      hit.point = ray.origin + ray.direction.normalized() * *distance;
      nearest = hit;
    }
  }

  return nearest;
}

std::optional<Eigen::Vector3d> intersectPlane(
    const PickRay& ray,
    const Eigen::Vector3d& planePoint,
    const Eigen::Vector3d& planeNormal)
{
  if (!ray.origin.allFinite() || !ray.direction.allFinite()
      || !planePoint.allFinite() || !planeNormal.allFinite()) {
    return std::nullopt;
  }

  const double directionNorm = ray.direction.norm();
  const double normalNorm = planeNormal.norm();
  if (directionNorm < 1e-12 || normalNorm < 1e-12) {
    return std::nullopt;
  }

  const Eigen::Vector3d direction = ray.direction / directionNorm;
  const Eigen::Vector3d normal = planeNormal / normalNorm;
  const double denom = direction.dot(normal);
  if (std::abs(denom) < 1e-12) {
    return std::nullopt;
  }

  const double distance = (planePoint - ray.origin).dot(normal) / denom;
  if (distance < 0.0) {
    return std::nullopt;
  }

  return ray.origin + direction * distance;
}

std::optional<Eigen::Vector3d> computePlaneDragTranslation(
    const PickRay& previousRay,
    const PickRay& currentRay,
    const Eigen::Vector3d& planePoint,
    const Eigen::Vector3d& planeNormal)
{
  const auto previousPoint
      = intersectPlane(previousRay, planePoint, planeNormal);
  const auto currentPoint = intersectPlane(currentRay, planePoint, planeNormal);
  if (!previousPoint || !currentPoint) {
    return std::nullopt;
  }
  return *currentPoint - *previousPoint;
}

bool translateFreeJointRenderable(
    const RenderableDescriptor& renderable,
    const Eigen::Vector3d& worldTranslation)
{
  if (!worldTranslation.allFinite()) {
    return false;
  }

  const auto skeleton = renderable.skeleton.lock();
  if (!skeleton || renderable.bodyName.empty()
      || renderable.shapeNodeName.empty()) {
    return false;
  }

  const auto* bodyNode = skeleton->getBodyNode(renderable.bodyName);
  if (bodyNode == nullptr) {
    return false;
  }
  const dynamics::ShapeNode* shapeNode = nullptr;
  bodyNode->eachShapeNode([&](const dynamics::ShapeNode* candidate) {
    if (candidate->getName() == renderable.shapeNodeName) {
      shapeNode = candidate;
      return false;
    }
    return true;
  });
  if (shapeNode == nullptr || shapeNode != renderable.shapeNode
      || shapeNode->getVersion() != renderable.shapeNodeVersion) {
    return false;
  }

  auto* mutableBodyNode = const_cast<dynamics::BodyNode*>(bodyNode);
  if (dynamic_cast<dynamics::FreeJoint*>(mutableBodyNode->getParentJoint())
      == nullptr) {
    return false;
  }

  Eigen::Isometry3d transform = mutableBodyNode->getWorldTransform();
  transform.translation() += worldTranslation;
  dynamics::FreeJoint::setTransformOf(mutableBodyNode, transform);
  return true;
}

bool translateSimpleFrameRenderable(
    const RenderableDescriptor& renderable,
    const Eigen::Vector3d& worldTranslation)
{
  if (!worldTranslation.allFinite()) {
    return false;
  }

  const auto simpleFrame = renderable.simpleFrame.lock();
  if (simpleFrame == nullptr || simpleFrame.get() != renderable.shapeFrame
      || simpleFrame->getVersion() != renderable.shapeFrameVersion) {
    return false;
  }

  auto* mutableSimpleFrame
      = const_cast<dynamics::SimpleFrame*>(simpleFrame.get());
  Eigen::Isometry3d transform = mutableSimpleFrame->getWorldTransform();
  transform.translation() += worldTranslation;
  mutableSimpleFrame->setTransform(transform, dynamics::Frame::World());
  return true;
}

bool translateFrameRenderable(
    const RenderableDescriptor& renderable,
    const Eigen::Vector3d& worldTranslation)
{
  return translateFreeJointRenderable(renderable, worldTranslation)
         || translateSimpleFrameRenderable(renderable, worldTranslation);
}

void normalizeRunOptions(RunOptions& options)
{
  options.width = std::max(1, options.width);
  options.height = std::max(1, options.height);
  if (options.headless && options.maxFrames < 0) {
    options.maxFrames = 1;
  }
  if (!options.screenshotPath.empty() && options.maxFrames < 0) {
    options.maxFrames = 1;
  }
}

bool shouldRequestScreenshot(
    const RunOptions& options, int renderedFrames, bool screenshotRequested)
{
  if (options.screenshotPath.empty() || screenshotRequested) {
    return false;
  }
  if (options.maxFrames < 0) {
    return true;
  }
  return renderedFrames + 1 >= options.maxFrames;
}

bool shouldStopAfterFrame(const RunOptions& options, int renderedFrames)
{
  return options.maxFrames >= 0 && renderedFrames >= options.maxFrames;
}

void togglePaused(ViewerLifecycleState& state)
{
  state.paused = !state.paused;
}

void requestSingleStep(ViewerLifecycleState& state, bool pause)
{
  if (pause) {
    state.paused = true;
  }
  state.stepOnce = true;
}

bool shouldAdvanceSimulation(const ViewerLifecycleState& state)
{
  return !state.paused || state.stepOnce;
}

void markSimulationAdvanced(ViewerLifecycleState& state)
{
  state.stepOnce = false;
}

bool shouldRequestScreenshot(
    const RunOptions& options, const ViewerLifecycleState& state)
{
  return shouldRequestScreenshot(
      options, state.renderedFrames, state.screenshotRequested);
}

void markScreenshotRequested(ViewerLifecycleState& state)
{
  state.screenshotRequested = true;
}

void markFrameRendered(ViewerLifecycleState& state)
{
  if (state.renderedFrames < std::numeric_limits<int>::max()) {
    ++state.renderedFrames;
  }
  state.skippedFrames = 0;
}

void markFrameSkipped(ViewerLifecycleState& state)
{
  if (state.skippedFrames < std::numeric_limits<int>::max()) {
    ++state.skippedFrames;
  }
}

bool shouldStopAfterFrame(
    const RunOptions& options, const ViewerLifecycleState& state)
{
  return shouldStopAfterFrame(options, state.renderedFrames);
}

bool writeRgbaPpm(
    const std::string& path,
    std::uint32_t width,
    std::uint32_t height,
    const std::vector<std::uint8_t>& rgbaPixels,
    bool originBottomLeft,
    std::string* errorMessage)
{
  const auto fail = [errorMessage](const std::string& message) {
    if (errorMessage != nullptr) {
      *errorMessage = message;
    }
    return false;
  };

  if (width == 0 || height == 0) {
    return fail("PPM dimensions must be nonzero");
  }

  constexpr std::size_t channels = 4;
  const auto maxSize = std::numeric_limits<std::size_t>::max();
  const auto imageWidth = static_cast<std::size_t>(width);
  const auto imageHeight = static_cast<std::size_t>(height);
  if (imageWidth > maxSize / imageHeight
      || imageWidth * imageHeight > maxSize / channels) {
    return fail("PPM image dimensions overflow the addressable buffer size");
  }

  const std::size_t expectedSize = imageWidth * imageHeight * channels;
  if (rgbaPixels.size() != expectedSize) {
    return fail("PPM RGBA buffer size does not match width * height * 4 bytes");
  }

  std::ofstream out(path, std::ios::binary);
  if (!out) {
    return fail("Failed to open PPM output path: " + path);
  }

  out << "P6\n" << width << " " << height << "\n255\n";
  for (std::uint32_t y = 0; y < height; ++y) {
    const std::uint32_t sourceY = originBottomLeft ? height - 1 - y : y;
    const std::size_t row = static_cast<std::size_t>(sourceY) * imageWidth * 4;
    for (std::uint32_t x = 0; x < width; ++x) {
      const auto* pixel = &rgbaPixels[row + static_cast<std::size_t>(x) * 4];
      out.write(reinterpret_cast<const char*>(pixel), 3);
    }
  }

  if (!out) {
    return fail("Failed while writing PPM output path: " + path);
  }

  return true;
}

OrbitCameraBasis makeOrbitCameraBasis(const OrbitCamera& camera)
{
  OrbitCameraBasis basis;
  basis.eye = cameraEye(camera);
  basis.forward = camera.target - basis.eye;
  if (basis.forward.squaredNorm() < 1e-12) {
    basis.forward = -Eigen::Vector3d::UnitX();
  } else {
    basis.forward.normalize();
  }

  const Eigen::Vector3d worldUp = Eigen::Vector3d::UnitZ();
  basis.right = basis.forward.cross(worldUp);
  if (basis.right.squaredNorm() < 1e-12) {
    basis.right = Eigen::Vector3d::UnitX();
  } else {
    basis.right.normalize();
  }
  basis.up = basis.right.cross(basis.forward).normalized();
  return basis;
}

Eigen::Vector3d cameraEye(const OrbitCamera& camera)
{
  return camera.target
         + Eigen::Vector3d(
             camera.distance * std::cos(camera.pitch) * std::cos(camera.yaw),
             camera.distance * std::cos(camera.pitch) * std::sin(camera.yaw),
             camera.distance * std::sin(camera.pitch));
}

void updateOrbitCamera(OrbitCamera& camera, const OrbitCameraUpdate& update)
{
  if (update.orbit) {
    camera.yaw -= update.deltaX * update.orbitScale;
    camera.pitch += update.deltaY * update.orbitScale;
  }

  const double minPitch = std::min(update.minPitch, update.maxPitch);
  const double maxPitch = std::max(update.minPitch, update.maxPitch);
  camera.pitch = std::clamp(camera.pitch, minPitch, maxPitch);

  if (update.pan) {
    const OrbitCameraBasis basis = makeOrbitCameraBasis(camera);
    const double panScale = camera.distance * update.panScale;
    camera.target -= basis.right * update.deltaX * panScale;
    camera.target += basis.up * update.deltaY * panScale;
  }

  if (update.scrollDelta != 0.0) {
    camera.distance *= std::exp(-update.scrollDelta * update.scrollScale);
  }

  const double minDistance = std::max(0.0, update.minDistance);
  const double maxDistance = std::max(minDistance, update.maxDistance);
  camera.distance = std::clamp(camera.distance, minDistance, maxDistance);
}

PickRay makePerspectivePickRay(
    const OrbitCamera& camera,
    double cursorX,
    double cursorY,
    int width,
    int height,
    double verticalFovRadians)
{
  if (!std::isfinite(verticalFovRadians) || verticalFovRadians <= 0.0) {
    verticalFovRadians = 0.7853981633974483;
  }

  const int safeWidth = std::max(1, width);
  const int safeHeight = std::max(1, height);
  const OrbitCameraBasis basis = makeOrbitCameraBasis(camera);
  const double aspect
      = static_cast<double>(safeWidth) / static_cast<double>(safeHeight);
  const double ndcX = (2.0 * cursorX / static_cast<double>(safeWidth)) - 1.0;
  const double ndcY = 1.0 - (2.0 * cursorY / static_cast<double>(safeHeight));
  const double halfHeight = std::tan(verticalFovRadians * 0.5);

  PickRay ray;
  ray.origin = basis.eye;
  ray.direction = (basis.forward + basis.right * ndcX * aspect * halfHeight
                   + basis.up * ndcY * halfHeight)
                      .normalized();
  return ray;
}

std::vector<DebugLineDescriptor> makeGridDebugLines(
    const DebugDrawOptions& options)
{
  std::vector<DebugLineDescriptor> lines;
  if (!options.drawGrid || options.gridHalfExtent <= 0.0
      || options.gridSpacing <= 0.0 || !std::isfinite(options.gridHalfExtent)
      || !std::isfinite(options.gridSpacing)) {
    return lines;
  }

  const auto steps = static_cast<int>(
      std::ceil(options.gridHalfExtent / options.gridSpacing));
  lines.reserve(static_cast<std::size_t>(steps * 4 + 4));
  const Eigen::Vector4d gridColor = rgba(0.46, 0.49, 0.5, 0.59);
  for (int i = -steps; i <= steps; ++i) {
    const double coordinate = static_cast<double>(i) * options.gridSpacing;
    if (std::abs(coordinate) > options.gridHalfExtent + 1e-12) {
      continue;
    }

    appendLine(
        lines,
        {-options.gridHalfExtent, coordinate, options.gridZ},
        {options.gridHalfExtent, coordinate, options.gridZ},
        gridColor,
        "grid");
    appendLine(
        lines,
        {coordinate, -options.gridHalfExtent, options.gridZ},
        {coordinate, options.gridHalfExtent, options.gridZ},
        gridColor,
        "grid");
  }

  return lines;
}

std::vector<DebugLineDescriptor> makeFrameDebugLines(
    const Eigen::Isometry3d& transform,
    double axisLength,
    const std::string& labelPrefix)
{
  std::vector<DebugLineDescriptor> lines;
  lines.reserve(3);
  appendFrameAxes(lines, transform, axisLength, labelPrefix);
  return lines;
}

std::vector<DebugLineDescriptor> makeSelectionDebugLines(
    const RenderableDescriptor& renderable,
    const Eigen::Vector4d& rgba,
    const std::string& labelPrefix)
{
  std::vector<DebugLineDescriptor> lines;
  if (!renderable.material.visible || !renderable.geometry.hasLocalBounds) {
    return lines;
  }

  const Eigen::Vector3d boundsMin = renderable.geometry.localBoundsMin;
  const Eigen::Vector3d boundsMax = renderable.geometry.localBoundsMax;
  if (!boundsMin.allFinite() || !boundsMax.allFinite()
      || !renderable.worldTransform.matrix().allFinite()) {
    return lines;
  }

  const Eigen::Vector3d min = boundsMin.cwiseMin(boundsMax);
  const Eigen::Vector3d max = boundsMin.cwiseMax(boundsMax);
  if ((max - min).cwiseAbs().maxCoeff() < 1e-12) {
    return lines;
  }

  std::array<Eigen::Vector3d, 8> corners
      = {Eigen::Vector3d(min.x(), min.y(), min.z()),
         Eigen::Vector3d(max.x(), min.y(), min.z()),
         Eigen::Vector3d(max.x(), max.y(), min.z()),
         Eigen::Vector3d(min.x(), max.y(), min.z()),
         Eigen::Vector3d(min.x(), min.y(), max.z()),
         Eigen::Vector3d(max.x(), min.y(), max.z()),
         Eigen::Vector3d(max.x(), max.y(), max.z()),
         Eigen::Vector3d(min.x(), max.y(), max.z())};
  for (Eigen::Vector3d& corner : corners) {
    corner = renderable.worldTransform * corner;
  }

  const std::string label
      = labelPrefix.empty() ? "selection.bounds" : labelPrefix + ".bounds";
  const std::array<std::pair<std::size_t, std::size_t>, 12> edges
      = {std::pair<std::size_t, std::size_t>{0, 1},
         {1, 2},
         {2, 3},
         {3, 0},
         {4, 5},
         {5, 6},
         {6, 7},
         {7, 4},
         {0, 4},
         {1, 5},
         {2, 6},
         {3, 7}};

  lines.reserve(edges.size());
  for (const auto& [from, to] : edges) {
    appendLine(lines, corners[from], corners[to], rgba, label);
  }

  return lines;
}

std::vector<DebugLineDescriptor> extractContactDebugLines(
    const collision::CollisionResult& result, const DebugDrawOptions& options)
{
  std::vector<DebugLineDescriptor> lines;
  if (!options.drawContacts) {
    return lines;
  }

  const auto contacts = result.getContacts();
  lines.reserve(contacts.size() * 4u);
  const Eigen::Vector4d pointColor = rgba(1.0, 0.92, 0.38);
  const Eigen::Vector4d normalColor = rgba(1.0, 0.75, 0.25);
  const Eigen::Vector4d forceColor = rgba(0.93, 0.31, 0.67);
  for (const auto& contact : contacts) {
    const Eigen::Vector3d point = contact.point;
    appendLine(
        lines,
        point - Eigen::Vector3d::UnitX() * options.contactMarkerHalfExtent,
        point + Eigen::Vector3d::UnitX() * options.contactMarkerHalfExtent,
        pointColor,
        "contact.point");
    appendLine(
        lines,
        point - Eigen::Vector3d::UnitY() * options.contactMarkerHalfExtent,
        point + Eigen::Vector3d::UnitY() * options.contactMarkerHalfExtent,
        pointColor,
        "contact.point");

    if (options.drawContactNormals
        && contact.normal.squaredNorm()
               > collision::Contact::getNormalEpsilonSquared()) {
      appendLine(
          lines,
          point,
          point + contact.normal.normalized() * options.contactNormalLength,
          normalColor,
          "contact.normal");
    }

    const double forceNorm = contact.force.norm();
    if (options.drawContactForces && forceNorm > 1e-9) {
      const double forceLength = std::clamp(
          forceNorm * options.contactForceScale,
          options.contactForceMinLength,
          options.contactForceMaxLength);
      appendLine(
          lines,
          point,
          point + contact.force.normalized() * forceLength,
          forceColor,
          "contact.force");
    }
  }

  return lines;
}

std::vector<DebugLineDescriptor> extractDebugLines(
    const simulation::World& world, const DebugDrawOptions& options)
{
  std::vector<DebugLineDescriptor> lines = makeGridDebugLines(options);

  if (options.drawWorldFrame) {
    auto worldFrameLines = makeFrameDebugLines(
        Eigen::Isometry3d::Identity(), options.worldFrameAxisLength, "world");
    lines.insert(lines.end(), worldFrameLines.begin(), worldFrameLines.end());
  }

  if (options.drawBodyFrames) {
    for (std::size_t skeletonIndex = 0; skeletonIndex < world.getNumSkeletons();
         ++skeletonIndex) {
      const auto skeleton = world.getSkeleton(skeletonIndex);
      if (!skeleton) {
        continue;
      }

      skeleton->eachBodyNode([&](const dynamics::BodyNode* bodyNode) {
        const std::string label
            = skeleton->getName() + "/" + bodyNode->getName();
        auto bodyFrameLines = makeFrameDebugLines(
            bodyNode->getWorldTransform(), options.bodyFrameAxisLength, label);
        lines.insert(lines.end(), bodyFrameLines.begin(), bodyFrameLines.end());
      });
    }
  }

  if (options.drawCentersOfMass) {
    for (std::size_t skeletonIndex = 0; skeletonIndex < world.getNumSkeletons();
         ++skeletonIndex) {
      const auto skeleton = world.getSkeleton(skeletonIndex);
      if (!skeleton || skeleton->getMass() <= 0.0
          || !std::isfinite(skeleton->getMass())) {
        continue;
      }

      appendCenterOfMassMarker(
          lines,
          skeleton->getCOM(),
          options.centerOfMassMarkerRadius,
          skeleton->getName());
    }
  }

  auto contactLines
      = extractContactDebugLines(world.getLastCollisionResult(), options);
  lines.insert(lines.end(), contactLines.begin(), contactLines.end());

  return lines;
}

} // namespace dart::gui::experimental
