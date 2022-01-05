/*
 * Copyright (c) 2011-2021, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include "dart/constraint/ContactSurface.hpp"

#include <utility>

#include "dart/collision/CollisionObject.hpp"
#include "dart/collision/Contact.hpp"
#include "dart/constraint/ContactConstraint.hpp"
#include "dart/constraint/ContactSurface.hpp"

namespace dart {
namespace constraint {

//==============================================================================
ContactSurfaceHandler::ContactSurfaceHandler(ContactSurfaceHandlerPtr parent)
  : mParent(std::move(parent))
{
  // Do nothing
}

//==============================================================================
void ContactSurfaceHandler::setParent(ContactSurfaceHandlerPtr parent)
{
  if (parent.get() != this)
    this->mParent = std::move(parent);
}

//==============================================================================
ContactSurfaceParams ContactSurfaceHandler::createParams(
    const collision::Contact& contact,
    size_t numContactsOnCollisionObject) const
{
  if (mParent != nullptr)
    return mParent->createParams(contact, numContactsOnCollisionObject);
  return {};
}

//==============================================================================
ContactConstraintPtr ContactSurfaceHandler::createConstraint(
    collision::Contact& contact,
    size_t numContactsOnCollisionObject,
    const double timeStep) const
{
  auto params = createParams(contact, numContactsOnCollisionObject);
  return std::make_shared<ContactConstraint>(contact, timeStep, params);
}

//==============================================================================
ContactSurfaceParams DefaultContactSurfaceHandler::createParams(
    const collision::Contact& contact,
    size_t numContactsOnCollisionObject) const
{
  ContactSurfaceParams params = ContactSurfaceHandler::createParams(
      contact, numContactsOnCollisionObject);

  const auto* shapeNodeA = const_cast<dynamics::ShapeFrame*>(
                               contact.collisionObject1->getShapeFrame())
                               ->asShapeNode();
  const auto* shapeNodeB = const_cast<dynamics::ShapeFrame*>(
                               contact.collisionObject2->getShapeFrame())
                               ->asShapeNode();

  const double restitutionCoeffA = computeRestitutionCoefficient(shapeNodeA);
  const double restitutionCoeffB = computeRestitutionCoefficient(shapeNodeB);

  params.mRestitutionCoeff = restitutionCoeffA * restitutionCoeffB;

  const double frictionCoeffA = computePrimaryFrictionCoefficient(shapeNodeA);
  const double frictionCoeffB = computePrimaryFrictionCoefficient(shapeNodeB);
  const double secondaryFrictionCoeffA
      = computeSecondaryFrictionCoefficient(shapeNodeA);
  const double secondaryFrictionCoeffB
      = computeSecondaryFrictionCoefficient(shapeNodeB);

  params.mPrimaryFrictionCoeff = (std::min)(frictionCoeffA, frictionCoeffB);
  params.mSecondaryFrictionCoeff
      = (std::min)(secondaryFrictionCoeffA, secondaryFrictionCoeffB);

  if (params.mPrimaryFrictionCoeff > DART_FRICTION_COEFF_THRESHOLD
      || params.mSecondaryFrictionCoeff > DART_FRICTION_COEFF_THRESHOLD)
  {
    // The slip compliance acts like a damper at each contact point so the total
    // damping for each collision is multiplied by the number of contact points
    // (numContacts). To eliminate this dependence on numContacts, the inverse
    // damping is multiplied by numContacts.
    const double slipComplianceA = computePrimarySlipCompliance(shapeNodeA);
    const double slipComplianceB = computePrimarySlipCompliance(shapeNodeB);

    // Combine slip compliances through addition
    params.mPrimarySlipCompliance = slipComplianceA + slipComplianceB;

    const double secondarySlipComplianceA
        = computeSecondarySlipCompliance(shapeNodeA);
    const double secondarySlipComplianceB
        = computeSecondarySlipCompliance(shapeNodeB);

    // Combine slip compliances through addition
    params.mSecondarySlipCompliance
        = secondarySlipComplianceA + secondarySlipComplianceB;

    // Check shapeNodes for valid friction direction unit vectors
    auto frictionDirA = computeWorldFirstFrictionDir(shapeNodeA);
    auto frictionDirB = computeWorldFirstFrictionDir(shapeNodeB);

    // resulting friction direction unit vector
    bool nonzeroDirA
        = frictionDirA.squaredNorm() >= DART_CONTACT_CONSTRAINT_EPSILON_SQUARED;
    bool nonzeroDirB
        = frictionDirB.squaredNorm() >= DART_CONTACT_CONSTRAINT_EPSILON_SQUARED;

    // only consider custom friction direction if one has nonzero length
    if (nonzeroDirA || nonzeroDirB)
    {
      // if A and B are both set, choose one with smaller friction coefficient
      // since it's friction properties will dominate
      if (nonzeroDirA && nonzeroDirB)
      {
        if (frictionCoeffA <= frictionCoeffB)
        {
          params.mFirstFrictionalDirection = frictionDirA.normalized();
        }
        else
        {
          params.mFirstFrictionalDirection = frictionDirB.normalized();
        }
      }
      else if (nonzeroDirA)
      {
        params.mFirstFrictionalDirection = frictionDirA.normalized();
      }
      else
      {
        params.mFirstFrictionalDirection = frictionDirB.normalized();
      }
    }
    else
    {
      params.mFirstFrictionalDirection = DART_DEFAULT_FRICTION_DIR;
    }
  }

  params.mContactSurfaceMotionVelocity = Eigen::Vector3d::Zero();

  return params;
}

//==============================================================================
ContactConstraintPtr DefaultContactSurfaceHandler::createConstraint(
    collision::Contact& contact,
    const size_t numContactsOnCollisionObject,
    const double timeStep) const
{
  auto constraint = ContactSurfaceHandler::createConstraint(
      contact, numContactsOnCollisionObject, timeStep);

  constraint->setPrimarySlipCompliance(
      constraint->getPrimarySlipCompliance()
      * static_cast<double>(numContactsOnCollisionObject));
  constraint->setSecondarySlipCompliance(
      constraint->getSecondarySlipCompliance()
      * static_cast<double>(numContactsOnCollisionObject));

  return constraint;
}

//==============================================================================
double DefaultContactSurfaceHandler::computeFrictionCoefficient(
    const dynamics::ShapeNode* shapeNode)
{
  assert(shapeNode);

  auto dynamicAspect = shapeNode->getDynamicsAspect();

  if (dynamicAspect == nullptr)
  {
    dtwarn << "[ContactConstraint] Attempt to extract "
           << "friction coefficient "
           << "from a ShapeNode that doesn't have DynamicAspect. The default "
           << "value (" << DART_DEFAULT_FRICTION_COEFF << ") will be used "
           << "instead.\n";
    return DART_DEFAULT_FRICTION_COEFF;
  }

  return dynamicAspect->getFrictionCoeff();
}

//==============================================================================
double DefaultContactSurfaceHandler::computePrimaryFrictionCoefficient(
    const dynamics::ShapeNode* shapeNode)
{
  assert(shapeNode);

  auto dynamicAspect = shapeNode->getDynamicsAspect();

  if (dynamicAspect == nullptr)
  {
    dtwarn << "[ContactConstraint] Attempt to extract "
           << "primary friction coefficient "
           << "from a ShapeNode that doesn't have DynamicAspect. The default "
           << "value (" << DART_DEFAULT_FRICTION_COEFF << ") will be used "
           << "instead.\n";
    return DART_DEFAULT_FRICTION_COEFF;
  }

  return dynamicAspect->getPrimaryFrictionCoeff();
}

//==============================================================================
double DefaultContactSurfaceHandler::computeSecondaryFrictionCoefficient(
    const dynamics::ShapeNode* shapeNode)
{
  assert(shapeNode);

  auto dynamicAspect = shapeNode->getDynamicsAspect();

  if (dynamicAspect == nullptr)
  {
    dtwarn << "[ContactConstraint] Attempt to extract "
           << "secondary friction coefficient "
           << "from a ShapeNode that doesn't have DynamicAspect. The default "
           << "value (" << DART_DEFAULT_FRICTION_COEFF << ") will be used "
           << "instead.\n";
    return DART_DEFAULT_FRICTION_COEFF;
  }

  return dynamicAspect->getSecondaryFrictionCoeff();
}

//==============================================================================
double DefaultContactSurfaceHandler::computePrimarySlipCompliance(
    const dynamics::ShapeNode* shapeNode)
{
  assert(shapeNode);

  auto dynamicAspect = shapeNode->getDynamicsAspect();

  if (dynamicAspect == nullptr)
  {
    dtwarn << "[ContactConstraint] Attempt to extract slip compliance "
           << "from a ShapeNode that doesn't have DynamicAspect. The default "
           << "value (" << DART_DEFAULT_SLIP_COMPLIANCE << ") will be used "
           << "instead.\n";
    return DART_DEFAULT_SLIP_COMPLIANCE;
  }

  double slipCompliance = dynamicAspect->getPrimarySlipCompliance();
  if (slipCompliance < 0)
  {
    return DART_DEFAULT_SLIP_COMPLIANCE;
  }
  return slipCompliance;
}

//==============================================================================
double DefaultContactSurfaceHandler::computeSecondarySlipCompliance(
    const dynamics::ShapeNode* shapeNode)
{
  assert(shapeNode);

  auto dynamicAspect = shapeNode->getDynamicsAspect();

  if (dynamicAspect == nullptr)
  {
    dtwarn << "[ContactConstraint] Attempt to extract "
           << "secondary slip compliance "
           << "from a ShapeNode that doesn't have DynamicAspect. The default "
           << "value (" << DART_DEFAULT_SLIP_COMPLIANCE << ") will be used "
           << "instead.\n";
    return DART_DEFAULT_SLIP_COMPLIANCE;
  }

  double slipCompliance = dynamicAspect->getSecondarySlipCompliance();
  if (slipCompliance < 0)
  {
    return DART_DEFAULT_SLIP_COMPLIANCE;
  }
  return slipCompliance;
}

//==============================================================================
Eigen::Vector3d DefaultContactSurfaceHandler::computeWorldFirstFrictionDir(
    const dynamics::ShapeNode* shapeNode)
{
  assert(shapeNode);

  auto dynamicAspect = shapeNode->getDynamicsAspect();

  if (dynamicAspect == nullptr)
  {
    dtwarn << "[ContactConstraint] Attempt to extract friction direction "
           << "from a ShapeNode that doesn't have DynamicAspect. The default "
           << "value (" << DART_DEFAULT_FRICTION_DIR << ") will be used "
           << "instead.\n";
    return DART_DEFAULT_FRICTION_DIR;
  }

  auto frame = dynamicAspect->getFirstFrictionDirectionFrame();
  Eigen::Vector3d frictionDir = dynamicAspect->getFirstFrictionDirection();

  // rotate using custom frame if it is specified
  if (frame)
  {
    return frame->getWorldTransform().linear() * frictionDir;
  }
  // otherwise rotate using shapeNode
  return shapeNode->getWorldTransform().linear() * frictionDir;
}

//==============================================================================
double DefaultContactSurfaceHandler::computeRestitutionCoefficient(
    const dynamics::ShapeNode* shapeNode)
{
  assert(shapeNode);

  auto dynamicAspect = shapeNode->getDynamicsAspect();

  if (dynamicAspect == nullptr)
  {
    dtwarn << "[ContactConstraint] Attempt to extract restitution coefficient "
           << "from a ShapeNode that doesn't have DynamicAspect. The default "
           << "value (" << DART_DEFAULT_RESTITUTION_COEFF << ") will be used "
           << "instead.\n";
    return DART_DEFAULT_RESTITUTION_COEFF;
  }

  return dynamicAspect->getRestitutionCoeff();
}

} // namespace constraint
} // namespace dart
