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

#include "dart/constraint/ContactSurface.hpp"

#include "dart/collision/CollisionObject.hpp"
#include "dart/collision/Contact.hpp"
#include "dart/common/Macros.hpp"
#include "dart/constraint/ContactConstraint.hpp"
#include "dart/constraint/ContactSurface.hpp"

#include <memory_resource>
#include <utility>

#include <cmath>

namespace dart {
namespace constraint {

namespace {

//==============================================================================
dynamics::ShapeNode* getContactShapeNode(
    const collision::CollisionObject* collisionObject)
{
  if (collisionObject == nullptr)
    return nullptr;

  return const_cast<dynamics::ShapeNode*>(collisionObject->getShapeNode());
}

// Contact constraints are rebuilt every simulation step. A process-lifetime
// pool keeps the public shared_ptr-returning API intact while avoiding repeated
// malloc/free churn once the pool reaches the scene's high-water mark.
std::pmr::memory_resource* getContactConstraintPool()
{
  static auto* pool = new std::pmr::synchronized_pool_resource;
  return pool;
}

//==============================================================================
double sanitizeFrictionCoefficient(
    const dynamics::ShapeNode* shapeNode,
    double coeff,
    const char* coefficientName)
{
  if (!std::isfinite(coeff) || coeff < 0.0) {
    dtwarn << "[ContactConstraint] Invalid " << coefficientName << " (" << coeff
           << ") from ShapeNode [" << shapeNode->getName()
           << "]. Friction must be non-negative and finite. Using default "
           << "value (" << DART_DEFAULT_FRICTION_COEFF << ").\n";
    return DART_DEFAULT_FRICTION_COEFF;
  }

  return coeff;
}

//==============================================================================
double sanitizeSlipCompliance(
    const dynamics::ShapeNode* shapeNode,
    double slipCompliance,
    const char* complianceName)
{
  // Negative values (including the -1.0 sentinel) mean "use default".
  // This is by design - see ShapeFrame.hpp documentation.
  if (slipCompliance < 0.0)
    return DART_DEFAULT_SLIP_COMPLIANCE;

  if (!std::isfinite(slipCompliance)) {
    dtwarn << "[ContactConstraint] Invalid " << complianceName << " ("
           << slipCompliance << ") from ShapeNode [" << shapeNode->getName()
           << "]. Slip compliance must be finite. Using "
           << "default value (" << DART_DEFAULT_SLIP_COMPLIANCE << ").\n";
    return DART_DEFAULT_SLIP_COMPLIANCE;
  }

  return slipCompliance;
}

//==============================================================================
double sanitizeRestitutionCoefficient(
    const dynamics::ShapeNode* shapeNode, double coeff)
{
  if (!std::isfinite(coeff) || coeff < 0.0 || coeff > 1.0) {
    dtwarn << "[ContactConstraint] Invalid restitution coefficient (" << coeff
           << ") from ShapeNode [" << shapeNode->getName()
           << "]. Restitution must be in range [0, 1] and finite. Using "
           << "default value (" << DART_DEFAULT_RESTITUTION_COEFF << ").\n";
    return DART_DEFAULT_RESTITUTION_COEFF;
  }

  return coeff;
}

//==============================================================================
Eigen::Vector3d computeWorldFirstFrictionDirFromAspect(
    const dynamics::ShapeNode* shapeNode,
    const dynamics::DynamicsAspect* dynamicAspect)
{
  auto frame = dynamicAspect->getFirstFrictionDirectionFrame();
  Eigen::Vector3d frictionDir = dynamicAspect->getFirstFrictionDirection();

  if (frictionDir.squaredNorm() < DART_CONTACT_CONSTRAINT_EPSILON_SQUARED)
    return Eigen::Vector3d::Zero();

  // rotate using custom frame if it is specified
  if (frame)
    return frame->getWorldTransform().linear() * frictionDir;

  // otherwise rotate using shapeNode
  return shapeNode->getWorldTransform().linear() * frictionDir;
}

//==============================================================================
bool hasDefaultContactSurfaceProperties(
    const dynamics::DynamicsAspect* dynamicAspect)
{
  return dynamicAspect->getRestitutionCoeff() == DART_DEFAULT_RESTITUTION_COEFF
         && dynamicAspect->getPrimaryFrictionCoeff()
                == DART_DEFAULT_FRICTION_COEFF
         && dynamicAspect->getSecondaryFrictionCoeff()
                == DART_DEFAULT_FRICTION_COEFF
         && dynamicAspect->getPrimarySlipCompliance() == -1.0
         && dynamicAspect->getSecondarySlipCompliance() == -1.0
         && dynamicAspect->getFirstFrictionDirectionFrame() == nullptr
         && dynamicAspect->getFirstFrictionDirection().squaredNorm()
                < DART_CONTACT_CONSTRAINT_EPSILON_SQUARED;
}

} // namespace

//==============================================================================
ContactSurfaceHandler::ContactSurfaceHandler(ContactSurfaceHandlerPtr parent)
  : mParent(std::move(parent))
{
  // Do nothing
}

//==============================================================================
ContactSurfaceHandlerPtr ContactSurfaceHandler::getParent()
{
  return this->mParent;
}

//==============================================================================
void ContactSurfaceHandler::setParent(ContactSurfaceHandlerPtr parent)
{
  if (parent.get() != this)
    this->mParent = std::move(parent);
  else
    dtwarn << "Cannot assign self as parent handler.";
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
  std::pmr::polymorphic_allocator<ContactConstraint> allocator(
      getContactConstraintPool());
  return std::allocate_shared<ContactConstraint>(
      allocator, contact, timeStep, params);
}

//==============================================================================
ContactSurfaceParams DefaultContactSurfaceHandler::createParams(
    const collision::Contact& contact,
    size_t numContactsOnCollisionObject) const
{
  ContactSurfaceParams params = ContactSurfaceHandler::createParams(
      contact, numContactsOnCollisionObject);

  const auto* shapeNodeA = getContactShapeNode(contact.collisionObject1);
  const auto* shapeNodeB = getContactShapeNode(contact.collisionObject2);
  if (shapeNodeA == nullptr || shapeNodeB == nullptr) {
    dtwarn << "[ContactConstraint] Ignoring contact surface parameters for "
           << "a contact with a null collision object or missing ShapeNode.\n";
    return params;
  }

  const auto* dynamicAspectA = shapeNodeA->getDynamicsAspect();
  const auto* dynamicAspectB = shapeNodeB->getDynamicsAspect();
  if (mParent == nullptr && dynamicAspectA != nullptr
      && dynamicAspectB != nullptr
      && hasDefaultContactSurfaceProperties(dynamicAspectA)
      && hasDefaultContactSurfaceProperties(dynamicAspectB)) {
    return params;
  }

  double restitutionCoeffA;
  double restitutionCoeffB;
  double frictionCoeffA;
  double frictionCoeffB;
  double secondaryFrictionCoeffA;
  double secondaryFrictionCoeffB;
  double slipComplianceA = DART_DEFAULT_SLIP_COMPLIANCE;
  double slipComplianceB = DART_DEFAULT_SLIP_COMPLIANCE;
  double secondarySlipComplianceA = DART_DEFAULT_SLIP_COMPLIANCE;
  double secondarySlipComplianceB = DART_DEFAULT_SLIP_COMPLIANCE;
  Eigen::Vector3d frictionDirA = Eigen::Vector3d::Zero();
  Eigen::Vector3d frictionDirB = Eigen::Vector3d::Zero();

  if (dynamicAspectA == nullptr || dynamicAspectB == nullptr) {
    restitutionCoeffA = computeRestitutionCoefficient(shapeNodeA);
    restitutionCoeffB = computeRestitutionCoefficient(shapeNodeB);

    frictionCoeffA = computePrimaryFrictionCoefficient(shapeNodeA);
    frictionCoeffB = computePrimaryFrictionCoefficient(shapeNodeB);
    secondaryFrictionCoeffA = computeSecondaryFrictionCoefficient(shapeNodeA);
    secondaryFrictionCoeffB = computeSecondaryFrictionCoefficient(shapeNodeB);
  } else {
    restitutionCoeffA = sanitizeRestitutionCoefficient(
        shapeNodeA, dynamicAspectA->getRestitutionCoeff());
    restitutionCoeffB = sanitizeRestitutionCoefficient(
        shapeNodeB, dynamicAspectB->getRestitutionCoeff());

    frictionCoeffA = sanitizeFrictionCoefficient(
        shapeNodeA,
        dynamicAspectA->getPrimaryFrictionCoeff(),
        "primary friction coefficient");
    frictionCoeffB = sanitizeFrictionCoefficient(
        shapeNodeB,
        dynamicAspectB->getPrimaryFrictionCoeff(),
        "primary friction coefficient");
    secondaryFrictionCoeffA = sanitizeFrictionCoefficient(
        shapeNodeA,
        dynamicAspectA->getSecondaryFrictionCoeff(),
        "secondary friction coefficient");
    secondaryFrictionCoeffB = sanitizeFrictionCoefficient(
        shapeNodeB,
        dynamicAspectB->getSecondaryFrictionCoeff(),
        "secondary friction coefficient");
  }

  params.mRestitutionCoeff = restitutionCoeffA * restitutionCoeffB;

  params.mPrimaryFrictionCoeff = (std::min)(frictionCoeffA, frictionCoeffB);
  params.mSecondaryFrictionCoeff
      = (std::min)(secondaryFrictionCoeffA, secondaryFrictionCoeffB);

  if (params.mPrimaryFrictionCoeff > DART_FRICTION_COEFF_THRESHOLD
      || params.mSecondaryFrictionCoeff > DART_FRICTION_COEFF_THRESHOLD) {
    // The slip compliance acts like a damper at each contact point so the total
    // damping for each collision is multiplied by the number of contact points
    // (numContacts). To eliminate this dependence on numContacts, the inverse
    // damping is multiplied by numContacts.
    if (dynamicAspectA == nullptr || dynamicAspectB == nullptr) {
      slipComplianceA = computePrimarySlipCompliance(shapeNodeA);
      slipComplianceB = computePrimarySlipCompliance(shapeNodeB);

      secondarySlipComplianceA = computeSecondarySlipCompliance(shapeNodeA);
      secondarySlipComplianceB = computeSecondarySlipCompliance(shapeNodeB);

      frictionDirA = computeWorldFirstFrictionDir(shapeNodeA);
      frictionDirB = computeWorldFirstFrictionDir(shapeNodeB);
    } else {
      slipComplianceA = sanitizeSlipCompliance(
          shapeNodeA,
          dynamicAspectA->getPrimarySlipCompliance(),
          "primary slip compliance");
      slipComplianceB = sanitizeSlipCompliance(
          shapeNodeB,
          dynamicAspectB->getPrimarySlipCompliance(),
          "primary slip compliance");

      secondarySlipComplianceA = sanitizeSlipCompliance(
          shapeNodeA,
          dynamicAspectA->getSecondarySlipCompliance(),
          "secondary slip compliance");
      secondarySlipComplianceB = sanitizeSlipCompliance(
          shapeNodeB,
          dynamicAspectB->getSecondarySlipCompliance(),
          "secondary slip compliance");

      frictionDirA
          = computeWorldFirstFrictionDirFromAspect(shapeNodeA, dynamicAspectA);
      frictionDirB
          = computeWorldFirstFrictionDirFromAspect(shapeNodeB, dynamicAspectB);
    }

    // Combine slip compliances through addition
    params.mPrimarySlipCompliance = slipComplianceA + slipComplianceB;

    // Combine slip compliances through addition
    params.mSecondarySlipCompliance
        = secondarySlipComplianceA + secondarySlipComplianceB;

    // resulting friction direction unit vector
    bool nonzeroDirA
        = frictionDirA.squaredNorm() >= DART_CONTACT_CONSTRAINT_EPSILON_SQUARED;
    bool nonzeroDirB
        = frictionDirB.squaredNorm() >= DART_CONTACT_CONSTRAINT_EPSILON_SQUARED;

    // only consider custom friction direction if one has nonzero length
    if (nonzeroDirA || nonzeroDirB) {
      // if A and B are both set, choose one with smaller friction coefficient
      // since it's friction properties will dominate
      if (nonzeroDirA && nonzeroDirB) {
        if (frictionCoeffA <= frictionCoeffB) {
          params.mFirstFrictionalDirection = frictionDirA.normalized();
        } else {
          params.mFirstFrictionalDirection = frictionDirB.normalized();
        }
      } else if (nonzeroDirA) {
        params.mFirstFrictionalDirection = frictionDirA.normalized();
      } else {
        params.mFirstFrictionalDirection = frictionDirB.normalized();
      }
    } else {
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
  auto params = createParams(contact, numContactsOnCollisionObject);
  const auto contactCount = static_cast<double>(numContactsOnCollisionObject);
  params.mPrimarySlipCompliance *= contactCount;
  params.mSecondarySlipCompliance *= contactCount;

  return createConstraint(contact, timeStep, params);
}

//==============================================================================
ContactConstraintPtr DefaultContactSurfaceHandler::createConstraint(
    collision::Contact& contact,
    const double timeStep,
    const ContactSurfaceParams& params) const
{
  std::pmr::polymorphic_allocator<ContactConstraint> allocator(
      getContactConstraintPool());
  return std::allocate_shared<ContactConstraint>(
      allocator, contact, timeStep, params);
}

//==============================================================================
double DefaultContactSurfaceHandler::computeFrictionCoefficient(
    const dynamics::ShapeNode* shapeNode)
{
  DART_ASSERT(shapeNode);

  auto dynamicAspect = shapeNode->getDynamicsAspect();

  if (dynamicAspect == nullptr) {
    dtwarn << "[ContactConstraint] Attempt to extract "
           << "friction coefficient "
           << "from a ShapeNode that doesn't have DynamicAspect. The default "
           << "value (" << DART_DEFAULT_FRICTION_COEFF << ") will be used "
           << "instead.\n";
    return DART_DEFAULT_FRICTION_COEFF;
  }

  const double coeff = dynamicAspect->getFrictionCoeff();
  if (!std::isfinite(coeff) || coeff < 0.0) {
    dtwarn << "[ContactConstraint] Invalid friction coefficient (" << coeff
           << ") from ShapeNode [" << shapeNode->getName()
           << "]. Friction must be non-negative and finite. Using default "
           << "value (" << DART_DEFAULT_FRICTION_COEFF << ").\n";
    return DART_DEFAULT_FRICTION_COEFF;
  }

  return coeff;
}

//==============================================================================
double DefaultContactSurfaceHandler::computePrimaryFrictionCoefficient(
    const dynamics::ShapeNode* shapeNode)
{
  DART_ASSERT(shapeNode);

  auto dynamicAspect = shapeNode->getDynamicsAspect();

  if (dynamicAspect == nullptr) {
    dtwarn << "[ContactConstraint] Attempt to extract "
           << "primary friction coefficient "
           << "from a ShapeNode that doesn't have DynamicAspect. The default "
           << "value (" << DART_DEFAULT_FRICTION_COEFF << ") will be used "
           << "instead.\n";
    return DART_DEFAULT_FRICTION_COEFF;
  }

  const double coeff = dynamicAspect->getPrimaryFrictionCoeff();
  if (!std::isfinite(coeff) || coeff < 0.0) {
    dtwarn << "[ContactConstraint] Invalid primary friction coefficient ("
           << coeff << ") from ShapeNode [" << shapeNode->getName()
           << "]. Friction must be non-negative and finite. Using default "
           << "value (" << DART_DEFAULT_FRICTION_COEFF << ").\n";
    return DART_DEFAULT_FRICTION_COEFF;
  }

  return coeff;
}

//==============================================================================
double DefaultContactSurfaceHandler::computeSecondaryFrictionCoefficient(
    const dynamics::ShapeNode* shapeNode)
{
  DART_ASSERT(shapeNode);

  auto dynamicAspect = shapeNode->getDynamicsAspect();

  if (dynamicAspect == nullptr) {
    dtwarn << "[ContactConstraint] Attempt to extract "
           << "secondary friction coefficient "
           << "from a ShapeNode that doesn't have DynamicAspect. The default "
           << "value (" << DART_DEFAULT_FRICTION_COEFF << ") will be used "
           << "instead.\n";
    return DART_DEFAULT_FRICTION_COEFF;
  }

  const double coeff = dynamicAspect->getSecondaryFrictionCoeff();
  if (!std::isfinite(coeff) || coeff < 0.0) {
    dtwarn << "[ContactConstraint] Invalid secondary friction coefficient ("
           << coeff << ") from ShapeNode [" << shapeNode->getName()
           << "]. Friction must be non-negative and finite. Using default "
           << "value (" << DART_DEFAULT_FRICTION_COEFF << ").\n";
    return DART_DEFAULT_FRICTION_COEFF;
  }

  return coeff;
}

//==============================================================================
double DefaultContactSurfaceHandler::computePrimarySlipCompliance(
    const dynamics::ShapeNode* shapeNode)
{
  DART_ASSERT(shapeNode);

  auto dynamicAspect = shapeNode->getDynamicsAspect();

  if (dynamicAspect == nullptr) {
    dtwarn << "[ContactConstraint] Attempt to extract slip compliance "
           << "from a ShapeNode that doesn't have DynamicAspect. The default "
           << "value (" << DART_DEFAULT_SLIP_COMPLIANCE << ") will be used "
           << "instead.\n";
    return DART_DEFAULT_SLIP_COMPLIANCE;
  }

  const double slipCompliance = dynamicAspect->getPrimarySlipCompliance();

  // Negative values (including the -1.0 sentinel) mean "use default".
  // This is by design - see ShapeFrame.hpp documentation.
  if (slipCompliance < 0.0) {
    return DART_DEFAULT_SLIP_COMPLIANCE;
  }

  if (!std::isfinite(slipCompliance)) {
    dtwarn << "[ContactConstraint] Invalid primary slip compliance ("
           << slipCompliance << ") from ShapeNode [" << shapeNode->getName()
           << "]. Slip compliance must be finite. Using "
           << "default value (" << DART_DEFAULT_SLIP_COMPLIANCE << ").\n";
    return DART_DEFAULT_SLIP_COMPLIANCE;
  }

  return slipCompliance;
}

//==============================================================================
double DefaultContactSurfaceHandler::computeSecondarySlipCompliance(
    const dynamics::ShapeNode* shapeNode)
{
  DART_ASSERT(shapeNode);

  auto dynamicAspect = shapeNode->getDynamicsAspect();

  if (dynamicAspect == nullptr) {
    dtwarn << "[ContactConstraint] Attempt to extract "
           << "secondary slip compliance "
           << "from a ShapeNode that doesn't have DynamicAspect. The default "
           << "value (" << DART_DEFAULT_SLIP_COMPLIANCE << ") will be used "
           << "instead.\n";
    return DART_DEFAULT_SLIP_COMPLIANCE;
  }

  const double slipCompliance = dynamicAspect->getSecondarySlipCompliance();

  // Negative values (including the -1.0 sentinel) mean "use default".
  // This is by design - see ShapeFrame.hpp documentation.
  if (slipCompliance < 0.0) {
    return DART_DEFAULT_SLIP_COMPLIANCE;
  }

  if (!std::isfinite(slipCompliance)) {
    dtwarn << "[ContactConstraint] Invalid secondary slip compliance ("
           << slipCompliance << ") from ShapeNode [" << shapeNode->getName()
           << "]. Slip compliance must be finite. Using "
           << "default value (" << DART_DEFAULT_SLIP_COMPLIANCE << ").\n";
    return DART_DEFAULT_SLIP_COMPLIANCE;
  }

  return slipCompliance;
}

//==============================================================================
Eigen::Vector3d DefaultContactSurfaceHandler::computeWorldFirstFrictionDir(
    const dynamics::ShapeNode* shapeNode)
{
  DART_ASSERT(shapeNode);

  auto dynamicAspect = shapeNode->getDynamicsAspect();

  if (dynamicAspect == nullptr) {
    dtwarn << "[ContactConstraint] Attempt to extract friction direction "
           << "from a ShapeNode that doesn't have DynamicAspect. The default "
           << "value (" << DART_DEFAULT_FRICTION_DIR << ") will be used "
           << "instead.\n";
    return DART_DEFAULT_FRICTION_DIR;
  }

  auto frame = dynamicAspect->getFirstFrictionDirectionFrame();
  Eigen::Vector3d frictionDir = dynamicAspect->getFirstFrictionDirection();

  if (frictionDir.squaredNorm() < DART_CONTACT_CONSTRAINT_EPSILON_SQUARED)
    return Eigen::Vector3d::Zero();

  // rotate using custom frame if it is specified
  if (frame) {
    return frame->getWorldTransform().linear() * frictionDir;
  }
  // otherwise rotate using shapeNode
  return shapeNode->getWorldTransform().linear() * frictionDir;
}

//==============================================================================
double DefaultContactSurfaceHandler::computeRestitutionCoefficient(
    const dynamics::ShapeNode* shapeNode)
{
  DART_ASSERT(shapeNode);

  auto dynamicAspect = shapeNode->getDynamicsAspect();

  if (dynamicAspect == nullptr) {
    dtwarn << "[ContactConstraint] Attempt to extract restitution coefficient "
           << "from a ShapeNode that doesn't have DynamicAspect. The default "
           << "value (" << DART_DEFAULT_RESTITUTION_COEFF << ") will be used "
           << "instead.\n";
    return DART_DEFAULT_RESTITUTION_COEFF;
  }

  const double coeff = dynamicAspect->getRestitutionCoeff();
  if (!std::isfinite(coeff) || coeff < 0.0 || coeff > 1.0) {
    dtwarn << "[ContactConstraint] Invalid restitution coefficient (" << coeff
           << ") from ShapeNode [" << shapeNode->getName()
           << "]. Restitution must be in range [0, 1] and finite. Using "
           << "default value (" << DART_DEFAULT_RESTITUTION_COEFF << ").\n";
    return DART_DEFAULT_RESTITUTION_COEFF;
  }

  return coeff;
}

} // namespace constraint
} // namespace dart
