/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include "dart/collision/dart/DARTCollisionDetector.hpp"

#include "dart/collision/CollisionObject.hpp"
#include "dart/collision/dart/DartCollisionEngine.hpp"
#include "dart/collision/dart/DARTCollisionGroup.hpp"
#include "dart/collision/dart/DARTCollisionObject.hpp"
#include "dart/common/Logging.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/CylinderShape.hpp"
#include "dart/dynamics/EllipsoidShape.hpp"
#include "dart/dynamics/PlaneShape.hpp"
#include "dart/dynamics/ShapeFrame.hpp"
#include "dart/dynamics/SphereShape.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace dart {
namespace collision {

namespace {

constexpr double kRaycastEps = 1.0e-12;

struct RaycastCandidate
{
  Eigen::Vector3d point{Eigen::Vector3d::Zero()};
  Eigen::Vector3d normal{Eigen::Vector3d::Zero()};
  double fraction{0.0};
};

bool raycastIntersectsAabb(
    const Eigen::Vector3d& from,
    const Eigen::Vector3d& dir,
    const Eigen::Vector3d& aabbMin,
    const Eigen::Vector3d& aabbMax)
{
  double tmin = 0.0;
  double tmax = 1.0;

  for (int i = 0; i < 3; ++i) {
    if (std::abs(dir[i]) < kRaycastEps) {
      if (from[i] < aabbMin[i] || from[i] > aabbMax[i])
        return false;
      continue;
    }

    const double invDir = 1.0 / dir[i];
    double t1 = (aabbMin[i] - from[i]) * invDir;
    double t2 = (aabbMax[i] - from[i]) * invDir;
    if (t1 > t2)
      std::swap(t1, t2);

    tmin = std::max(tmin, t1);
    tmax = std::min(tmax, t2);
    if (tmin > tmax)
      return false;
  }

  return tmax >= 0.0 && tmin <= 1.0;
}

bool raycastSphere(
    const CoreObject& core,
    const Eigen::Vector3d& from,
    const Eigen::Vector3d& dir,
    RaycastCandidate* candidate)
{
  if (!candidate)
    return false;

  const Eigen::Vector3d center = core.worldTransform.translation();
  const double radius = core.shape.radius;
  const Eigen::Vector3d m = from - center;

  const double a = dir.dot(dir);
  if (a < kRaycastEps)
    return false;

  const double b = 2.0 * m.dot(dir);
  const double c = m.dot(m) - radius * radius;
  const double discriminant = b * b - 4.0 * a * c;

  if (discriminant < 0.0)
    return false;

  const double sqrtDiscriminant = std::sqrt(discriminant);
  const double inv2a = 0.5 / a;
  double t0 = (-b - sqrtDiscriminant) * inv2a;
  double t1 = (-b + sqrtDiscriminant) * inv2a;

  if (t0 > t1)
    std::swap(t0, t1);

  double t = t0;
  if (t < 0.0 || t > 1.0) {
    t = t1;
    if (t < 0.0 || t > 1.0)
      return false;
  }

  candidate->fraction = t;
  candidate->point = from + t * dir;
  candidate->normal = (candidate->point - center).normalized();

  return true;
}

bool raycastPlane(
    const CoreObject& core,
    const Eigen::Vector3d& from,
    const Eigen::Vector3d& dir,
    RaycastCandidate* candidate)
{
  if (!candidate)
    return false;

  const Eigen::Isometry3d inv = core.worldTransform.inverse();
  const Eigen::Vector3d fromLocal = inv * from;
  const Eigen::Vector3d dirLocal = inv.linear() * dir;
  const Eigen::Vector3d& normalLocal = core.shape.planeNormal;

  const double denom = normalLocal.dot(dirLocal);
  if (std::abs(denom) < kRaycastEps)
    return false;

  const double t
      = (core.shape.planeOffset - normalLocal.dot(fromLocal)) / denom;
  if (t < 0.0 || t > 1.0)
    return false;

  candidate->fraction = t;
  candidate->point = from + t * dir;
  candidate->normal
      = (core.worldTransform.linear() * normalLocal).normalized();

  return true;
}

bool raycastBox(
    const CoreObject& core,
    const Eigen::Vector3d& from,
    const Eigen::Vector3d& dir,
    RaycastCandidate* candidate)
{
  if (!candidate)
    return false;

  const Eigen::Isometry3d inv = core.worldTransform.inverse();
  const Eigen::Vector3d fromLocal = inv * from;
  const Eigen::Vector3d dirLocal = inv.linear() * dir;
  const Eigen::Vector3d halfSize = 0.5 * core.shape.size;

  double tmin = 0.0;
  double tmax = 1.0;
  int axisEntry = -1;
  int axisExit = -1;
  double signEntry = 0.0;
  double signExit = 0.0;

  for (int i = 0; i < 3; ++i) {
    if (std::abs(dirLocal[i]) < kRaycastEps) {
      if (fromLocal[i] < -halfSize[i] || fromLocal[i] > halfSize[i])
        return false;
      continue;
    }

    double t1 = (-halfSize[i] - fromLocal[i]) / dirLocal[i];
    double t2 = (halfSize[i] - fromLocal[i]) / dirLocal[i];
    double entrySign = (t1 > t2) ? 1.0 : -1.0;

    if (t1 > t2)
      std::swap(t1, t2);

    if (t1 > tmin) {
      tmin = t1;
      axisEntry = i;
      signEntry = entrySign;
    }

    if (t2 < tmax) {
      tmax = t2;
      axisExit = i;
      signExit = -entrySign;
    }

    if (tmin > tmax)
      return false;
  }

  if (tmax < 0.0)
    return false;

  double tHit = tmin;
  int axis = axisEntry;
  double sign = signEntry;
  if (tHit < 0.0) {
    tHit = tmax;
    axis = axisExit;
    sign = signExit;
  }

  if (tHit < 0.0 || tHit > 1.0)
    return false;

  Eigen::Vector3d normalLocal = Eigen::Vector3d::Zero();
  if (axis >= 0)
    normalLocal[axis] = sign;

  candidate->fraction = tHit;
  candidate->point = from + tHit * dir;
  candidate->normal
      = (core.worldTransform.linear() * normalLocal).normalized();

  return true;
}

bool raycastCylinder(
    const CoreObject& core,
    const Eigen::Vector3d& from,
    const Eigen::Vector3d& dir,
    RaycastCandidate* candidate)
{
  if (!candidate)
    return false;

  const Eigen::Isometry3d inv = core.worldTransform.inverse();
  const Eigen::Vector3d fromLocal = inv * from;
  const Eigen::Vector3d dirLocal = inv.linear() * dir;
  const double radius = core.shape.radius;
  const double halfHeight = 0.5 * core.shape.height;

  double bestT = std::numeric_limits<double>::infinity();
  Eigen::Vector3d bestNormal = Eigen::Vector3d::Zero();
  bool hit = false;

  const double a = dirLocal.x() * dirLocal.x() + dirLocal.y() * dirLocal.y();
  if (a > kRaycastEps) {
    const double b
        = 2.0 * (fromLocal.x() * dirLocal.x() + fromLocal.y() * dirLocal.y());
    const double c = fromLocal.x() * fromLocal.x()
                     + fromLocal.y() * fromLocal.y() - radius * radius;
    const double discriminant = b * b - 4.0 * a * c;
    if (discriminant >= 0.0) {
      const double sqrtDiscriminant = std::sqrt(discriminant);
      const double inv2a = 0.5 / a;
      double t0 = (-b - sqrtDiscriminant) * inv2a;
      double t1 = (-b + sqrtDiscriminant) * inv2a;

      if (t0 > t1)
        std::swap(t0, t1);

      auto checkSideHit = [&](double t) {
        if (t < 0.0 || t > 1.0)
          return;

        const double z = fromLocal.z() + t * dirLocal.z();
        if (z < -halfHeight || z > halfHeight)
          return;

        if (t < bestT) {
          const Eigen::Vector3d pointLocal = fromLocal + t * dirLocal;
          Eigen::Vector3d normalLocal(pointLocal.x(), pointLocal.y(), 0.0);
          if (normalLocal.norm() > kRaycastEps)
            normalLocal.normalize();
          bestNormal = core.worldTransform.linear() * normalLocal;
          bestT = t;
          hit = true;
        }
      };

      checkSideHit(t0);
      checkSideHit(t1);
    }
  }

  if (std::abs(dirLocal.z()) > kRaycastEps) {
    const double invDz = 1.0 / dirLocal.z();
    const double tTop = (halfHeight - fromLocal.z()) * invDz;
    const double tBottom = (-halfHeight - fromLocal.z()) * invDz;

    auto checkCapHit = [&](double t, double capZ) {
      if (t < 0.0 || t > 1.0)
        return;

      const double x = fromLocal.x() + t * dirLocal.x();
      const double y = fromLocal.y() + t * dirLocal.y();
      if (x * x + y * y > radius * radius)
        return;

      if (t < bestT) {
        Eigen::Vector3d normalLocal(0.0, 0.0, capZ > 0.0 ? 1.0 : -1.0);
        bestNormal = core.worldTransform.linear() * normalLocal;
        bestT = t;
        hit = true;
      }
    };

    checkCapHit(tTop, halfHeight);
    checkCapHit(tBottom, -halfHeight);
  }

  if (!hit)
    return false;

  candidate->fraction = bestT;
  candidate->point = from + bestT * dir;
  candidate->normal = bestNormal.normalized();

  return true;
}

bool raycastCoreObject(
    const CoreObject& core,
    const Eigen::Vector3d& from,
    const Eigen::Vector3d& dir,
    RaycastCandidate* candidate)
{
  switch (core.shape.type) {
    case CoreShapeType::kSphere:
      return raycastSphere(core, from, dir, candidate);
    case CoreShapeType::kBox:
      return raycastBox(core, from, dir, candidate);
    case CoreShapeType::kCylinder:
      return raycastCylinder(core, from, dir, candidate);
    case CoreShapeType::kPlane:
      return raycastPlane(core, from, dir, candidate);
    default:
      return false;
  }
}

} // namespace

//==============================================================================
DARTCollisionDetector::Registrar<DARTCollisionDetector>
    DARTCollisionDetector::mRegistrar{
        DARTCollisionDetector::getStaticType(),
        []() -> std::shared_ptr<dart::collision::DARTCollisionDetector> {
          return dart::collision::DARTCollisionDetector::create();
        }};

//==============================================================================
std::shared_ptr<DARTCollisionDetector> DARTCollisionDetector::create()
{
  return std::shared_ptr<DARTCollisionDetector>(new DARTCollisionDetector());
}

//==============================================================================
std::shared_ptr<CollisionDetector>
DARTCollisionDetector::cloneWithoutCollisionObjects() const
{
  return DARTCollisionDetector::create();
}

//==============================================================================
const std::string& DARTCollisionDetector::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& DARTCollisionDetector::getStaticType()
{
  static const std::string type = "dart";
  return type;
}

//==============================================================================
std::unique_ptr<CollisionGroup> DARTCollisionDetector::createCollisionGroup()
{
  return std::make_unique<DARTCollisionGroup>(shared_from_this());
}

//==============================================================================
static bool checkGroupValidity(DARTCollisionDetector* cd, CollisionGroup* group)
{
  if (cd != group->getCollisionDetector().get()) {
    DART_ERROR(
        "Attempting to check collision for a collision group that is created "
        "from a different collision detector instance.");

    return false;
  }

  return true;
}

//==============================================================================
bool DARTCollisionDetector::collide(
    CollisionGroup* group,
    const CollisionOption& option,
    CollisionResult* result)
{
  if (result)
    result->clear();

  if (0u == option.maxNumContacts) [[unlikely]] {
    DART_WARN(
        "CollisionOption::maxNumContacts is 0; skipping collision detection. "
        "Use maxNumContacts >= 1 for binary checks.");
    return false;
  }

  if (!checkGroupValidity(this, group))
    return false;

  auto casted = static_cast<DARTCollisionGroup*>(group);
  casted->updateEngineData();
  const auto& objects = casted->mCollisionObjects;

  return mEngine->collide(objects, option, result);
}

//==============================================================================
bool DARTCollisionDetector::collide(
    CollisionGroup* group1,
    CollisionGroup* group2,
    const CollisionOption& option,
    CollisionResult* result)
{
  if (result)
    result->clear();

  if (0u == option.maxNumContacts) {
    DART_WARN(
        "CollisionOption::maxNumContacts is 0; skipping collision detection. "
        "Use maxNumContacts >= 1 for binary checks.");
    return false;
  }

  if (!checkGroupValidity(this, group1))
    return false;

  if (!checkGroupValidity(this, group2))
    return false;

  auto casted1 = static_cast<DARTCollisionGroup*>(group1);
  auto casted2 = static_cast<DARTCollisionGroup*>(group2);

  casted1->updateEngineData();
  casted2->updateEngineData();

  const auto& objects1 = casted1->mCollisionObjects;
  const auto& objects2 = casted2->mCollisionObjects;

  return mEngine->collide(objects1, objects2, option, result);
}

//==============================================================================
double DARTCollisionDetector::distance(
    CollisionGroup* /*group*/,
    const DistanceOption& /*option*/,
    DistanceResult* /*result*/)
{
  DART_WARN(
      "This collision detector does not support (signed) distance queries. "
      "Returning 0.0.");

  return 0.0;
}

//==============================================================================
double DARTCollisionDetector::distance(
    CollisionGroup* /*group1*/,
    CollisionGroup* /*group2*/,
    const DistanceOption& /*option*/,
    DistanceResult* /*result*/)
{
  DART_WARN(
      "This collision detector does not support (signed) distance queries. "
      "Returning 0.0.");

  return 0.0;
}

//==============================================================================
bool DARTCollisionDetector::raycast(
    CollisionGroup* group,
    const Eigen::Vector3d& from,
    const Eigen::Vector3d& to,
    const RaycastOption& option,
    RaycastResult* result)
{
  if (result)
    result->clear();

  if (!checkGroupValidity(this, group))
    return false;

  auto casted = static_cast<DARTCollisionGroup*>(group);
  casted->updateEngineData();

  const Eigen::Vector3d dir = to - from;
  if (dir.squaredNorm() < kRaycastEps)
    return false;

  bool hasHit = false;
  RayHit bestHit;
  double bestFraction = std::numeric_limits<double>::infinity();
  std::vector<RayHit> hits;

  for (auto* object : casted->mCollisionObjects) {
    if (!object || !option.passesFilter(object))
      continue;

    const auto* dartObject = static_cast<DARTCollisionObject*>(object);
    const auto& core = dartObject->getCoreObject();
    if (core.shape.type == CoreShapeType::kNone
        || core.shape.type == CoreShapeType::kUnsupported) {
      continue;
    }

    if (!raycastIntersectsAabb(
            from, dir, core.worldAabbMin, core.worldAabbMax)) {
      continue;
    }

    RaycastCandidate candidate;
    if (!raycastCoreObject(core, from, dir, &candidate))
      continue;

    RayHit hit;
    hit.mCollisionObject = object;
    hit.mPoint = candidate.point;
    hit.mNormal = candidate.normal;
    hit.mFraction = candidate.fraction;

    if (!option.mEnableAllHits) {
      if (!hasHit || hit.mFraction < bestFraction) {
        bestHit = hit;
        bestFraction = hit.mFraction;
        hasHit = true;
      }
    } else {
      hits.push_back(hit);
      hasHit = true;
    }
  }

  if (!result)
    return hasHit;

  if (!hasHit)
    return false;

  if (!option.mEnableAllHits) {
    result->mRayHits.push_back(bestHit);
  } else {
    if (option.mSortByClosest) {
      std::sort(
          hits.begin(),
          hits.end(),
          [](const RayHit& a, const RayHit& b) {
            return a.mFraction < b.mFraction;
          });
    }
    result->mRayHits = std::move(hits);
  }

  return result->hasHit();
}

//==============================================================================
DARTCollisionDetector::DARTCollisionDetector() : CollisionDetector()
{
  mCollisionObjectManager.reset(new ManagerForSharableCollisionObjects(this));
  mEngine = std::make_unique<DartCollisionEngine>();
}

//==============================================================================
void warnUnsupportedShapeType(const dynamics::ShapeFrame* shapeFrame)
{
  if (!shapeFrame)
    return;

  const auto& shape = shapeFrame->getShape();
  const auto& shapeType = shape->getType();

  if (shapeType == dynamics::SphereShape::getStaticType())
    return;

  if (shapeType == dynamics::BoxShape::getStaticType())
    return;

  if (shapeType == dynamics::CylinderShape::getStaticType())
    return;

  if (shapeType == dynamics::PlaneShape::getStaticType())
    return;

  if (shapeType == dynamics::EllipsoidShape::getStaticType()) {
    const auto& ellipsoid
        = std::static_pointer_cast<const dynamics::EllipsoidShape>(shape);

    if (ellipsoid->isSphere())
      return;
  }

  DART_ERROR(
      "[DARTCollisionDetector] Attempting to create shape type [{}] that is "
      "not supported by DARTCollisionDetector. Currently, only SphereShape, "
      "BoxShape, CylinderShape, PlaneShape, and EllipsoidShape (only when all "
      "the radii are equal) are supported. This shape will always get "
      "penetrated by other objects.",
      shapeType);
}

//==============================================================================
std::unique_ptr<CollisionObject> DARTCollisionDetector::createCollisionObject(
    const dynamics::ShapeFrame* shapeFrame)
{
  warnUnsupportedShapeType(shapeFrame);

  return std::unique_ptr<DARTCollisionObject>(
      new DARTCollisionObject(this, shapeFrame));
}

//==============================================================================
void DARTCollisionDetector::refreshCollisionObject(CollisionObject* /*object*/)
{
  // Do nothing
}
} // namespace collision
} // namespace dart
