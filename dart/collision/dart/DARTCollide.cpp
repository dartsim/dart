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

#include "dart/collision/dart/DARTCollide.hpp"

#include "dart/collision/CollisionGroup.hpp"
#include "dart/collision/dart/DARTCollisionDetector.hpp"
#include "dart/collision/dart/DARTCollisionObject.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/CylinderShape.hpp"
#include "dart/dynamics/PlaneShape.hpp"
#include "dart/dynamics/Shape.hpp"
#include "dart/dynamics/SimpleFrame.hpp"
#include "dart/dynamics/SphereShape.hpp"

#include <limits>
#include <memory>

namespace dart {
namespace collision {

namespace {

//==============================================================================
int collideShapeFrames(
    const dynamics::ShapeFrame* frame1,
    const dynamics::ShapeFrame* frame2,
    CollisionObject* object1,
    CollisionObject* object2,
    CollisionResult& result)
{
  if (!frame1 || !frame2)
    return 0;

  auto detector = DARTCollisionDetector::create();
  auto group1 = detector->createCollisionGroup(frame1);
  auto group2 = detector->createCollisionGroup(frame2);

  CollisionResult adapterResult;
  group1->collide(
      group2.get(),
      CollisionOption(true, std::numeric_limits<std::size_t>::max()),
      &adapterResult);

  for (const auto& adapterContact : adapterResult.getContacts()) {
    Contact contact = adapterContact;
    contact.collisionObject1 = object1;
    contact.collisionObject2 = object2;
    contact.userData = nullptr;
    result.addContact(contact);
  }

  return static_cast<int>(adapterResult.getNumContacts());
}

//==============================================================================
std::shared_ptr<dynamics::SimpleFrame> makeShapeFrame(
    const dynamics::ConstShapePtr& shape, const Eigen::Isometry3d& transform)
{
  if (!shape)
    return nullptr;

  auto frame = dynamics::SimpleFrame::createShared(dynamics::Frame::World());
  frame->setShape(std::const_pointer_cast<dynamics::Shape>(shape));
  frame->setRelativeTransform(transform);
  return frame;
}

//==============================================================================
int collideShapes(
    const dynamics::ConstShapePtr& shape1,
    const Eigen::Isometry3d& transform1,
    const dynamics::ConstShapePtr& shape2,
    const Eigen::Isometry3d& transform2,
    CollisionObject* object1,
    CollisionObject* object2,
    CollisionResult& result)
{
  auto frame1 = makeShapeFrame(shape1, transform1);
  auto frame2 = makeShapeFrame(shape2, transform2);
  return collideShapeFrames(
      frame1.get(), frame2.get(), object1, object2, result);
}

} // namespace

//==============================================================================
int collide(CollisionObject* o1, CollisionObject* o2, CollisionResult& result)
{
  if (!o1 || !o2)
    return 0;

  return collideShapeFrames(
      o1->getShapeFrame(), o2->getShapeFrame(), o1, o2, result);
}

//==============================================================================
int collide(
    DARTCollisionObject* o1, DARTCollisionObject* o2, CollisionResult& result)
{
  return collide(
      static_cast<CollisionObject*>(o1),
      static_cast<CollisionObject*>(o2),
      result);
}

//==============================================================================
int collidePlaneShape(
    DARTCollisionObject* planeObject,
    DARTCollisionObject* otherObject,
    const Eigen::Isometry3d& planeTransform,
    const Eigen::Isometry3d& otherTransform,
    bool planeFirst,
    CollisionResult& result)
{
  if (!planeObject || !otherObject)
    return 0;

  const auto planeShape = planeObject->getShape();
  const auto otherShape = otherObject->getShape();
  if (!planeShape || !otherShape)
    return 0;

  if (planeFirst) {
    return collideShapes(
        planeShape,
        planeTransform,
        otherShape,
        otherTransform,
        planeObject,
        otherObject,
        result);
  }

  return collideShapes(
      otherShape,
      otherTransform,
      planeShape,
      planeTransform,
      otherObject,
      planeObject,
      result);
}

//==============================================================================
int collidePlaneShape(
    DARTCollisionObject* planeObject,
    DARTCollisionObject* otherObject,
    bool planeFirst,
    CollisionResult& result)
{
  if (planeFirst)
    return collide(planeObject, otherObject, result);

  return collide(otherObject, planeObject, result);
}

//==============================================================================
int collideBoxBox(
    CollisionObject* o1,
    CollisionObject* o2,
    const Eigen::Vector3d& size0,
    const Eigen::Isometry3d& T0,
    const Eigen::Vector3d& size1,
    const Eigen::Isometry3d& T1,
    CollisionResult& result)
{
  return collideShapes(
      std::make_shared<dynamics::BoxShape>(size0),
      T0,
      std::make_shared<dynamics::BoxShape>(size1),
      T1,
      o1,
      o2,
      result);
}

//==============================================================================
int collideBoxSphere(
    CollisionObject* o1,
    CollisionObject* o2,
    const Eigen::Vector3d& size0,
    const Eigen::Isometry3d& T0,
    const double& r1,
    const Eigen::Isometry3d& T1,
    CollisionResult& result)
{
  return collideShapes(
      std::make_shared<dynamics::BoxShape>(size0),
      T0,
      std::make_shared<dynamics::SphereShape>(r1),
      T1,
      o1,
      o2,
      result);
}

//==============================================================================
int collideSphereBox(
    CollisionObject* o1,
    CollisionObject* o2,
    const double& r0,
    const Eigen::Isometry3d& T0,
    const Eigen::Vector3d& size1,
    const Eigen::Isometry3d& T1,
    CollisionResult& result)
{
  return collideShapes(
      std::make_shared<dynamics::SphereShape>(r0),
      T0,
      std::make_shared<dynamics::BoxShape>(size1),
      T1,
      o1,
      o2,
      result);
}

//==============================================================================
int collideSphereSphere(
    CollisionObject* o1,
    CollisionObject* o2,
    const double& r0,
    const Eigen::Isometry3d& c0,
    const double& r1,
    const Eigen::Isometry3d& c1,
    CollisionResult& result)
{
  return collideShapes(
      std::make_shared<dynamics::SphereShape>(r0),
      c0,
      std::make_shared<dynamics::SphereShape>(r1),
      c1,
      o1,
      o2,
      result);
}

//==============================================================================
int collideCylinderSphere(
    CollisionObject* o1,
    CollisionObject* o2,
    const double& cyl_rad,
    const double& half_height,
    const Eigen::Isometry3d& T0,
    const double& sphere_rad,
    const Eigen::Isometry3d& T1,
    CollisionResult& result)
{
  return collideShapes(
      std::make_shared<dynamics::CylinderShape>(cyl_rad, 2.0 * half_height),
      T0,
      std::make_shared<dynamics::SphereShape>(sphere_rad),
      T1,
      o1,
      o2,
      result);
}

//==============================================================================
int collideCylinderPlane(
    CollisionObject* o1,
    CollisionObject* o2,
    const double& cyl_rad,
    const double& half_height,
    const Eigen::Isometry3d& T0,
    const Eigen::Vector3d& plane_normal,
    const Eigen::Isometry3d& T1,
    CollisionResult& result)
{
  return collideShapes(
      std::make_shared<dynamics::CylinderShape>(cyl_rad, 2.0 * half_height),
      T0,
      std::make_shared<dynamics::PlaneShape>(plane_normal, 0.0),
      T1,
      o1,
      o2,
      result);
}

} // namespace collision
} // namespace dart
