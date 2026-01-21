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

#include "dart/utils/skel_parser.hpp"

#include "dart/collision/collision_object.hpp"
#include "dart/collision/dart/dart_collision_detector.hpp"
#include "dart/collision/fcl/fcl_collision_detector.hpp"
#include "dart/common/logging.hpp"
#include "dart/common/macros.hpp"
#include "dart/config.hpp"
#include "dart/constraint/constraint_solver.hpp"
#include "dart/dynamics/ball_joint.hpp"
#include "dart/dynamics/body_node.hpp"
#include "dart/dynamics/box_shape.hpp"
#include "dart/dynamics/capsule_shape.hpp"
#include "dart/dynamics/cone_shape.hpp"
#include "dart/dynamics/cylinder_shape.hpp"
#include "dart/dynamics/ellipsoid_shape.hpp"
#include "dart/dynamics/euler_joint.hpp"
#include "dart/dynamics/free_joint.hpp"
#include "dart/dynamics/joint.hpp"
#include "dart/dynamics/marker.hpp"
#include "dart/dynamics/mesh_shape.hpp"
#include "dart/dynamics/multi_sphere_convex_hull_shape.hpp"
#include "dart/dynamics/planar_joint.hpp"
#include "dart/dynamics/plane_shape.hpp"
#include "dart/dynamics/prismatic_joint.hpp"
#include "dart/dynamics/pyramid_shape.hpp"
#include "dart/dynamics/revolute_joint.hpp"
#include "dart/dynamics/screw_joint.hpp"
#include "dart/dynamics/shape_node.hpp"
#include "dart/dynamics/skeleton.hpp"
#include "dart/dynamics/soft_body_node.hpp"
#include "dart/dynamics/soft_mesh_shape.hpp"
#include "dart/dynamics/sphere_shape.hpp"
#include "dart/dynamics/translational_joint.hpp"
#include "dart/dynamics/translational_joint2_d.hpp"
#include "dart/dynamics/universal_joint.hpp"
#include "dart/dynamics/weld_joint.hpp"
#include "dart/math/geometry.hpp"
#include "dart/utils/composite_resource_retriever.hpp"
#include "dart/utils/dart_resource_retriever.hpp"
#include "dart/utils/mesh_loader.hpp"
#include "dart/utils/xml_helpers.hpp"

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <algorithm>
#include <optional>
#include <string_view>
#include <unordered_map>
#include <vector>

#include <cmath>
#include <cstddef>

namespace dart {
namespace utils {

namespace {

enum NextResult
{
  VALID,
  CONTINUE,
  BREAK,
  CREATE_FREEJOINT_ROOT
};

using BodyPropPtr = std::shared_ptr<dynamics::BodyNode::Properties>;
using JointPropPtr = std::shared_ptr<dynamics::Joint::Properties>;

struct SkelBodyNode
{
  BodyPropPtr properties;
  Eigen::Isometry3d initTransform;
  std::vector<dynamics::Marker::BasicProperties> markers;
  std::string type;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct SkelJoint
{
  JointPropPtr properties;
  Eigen::VectorXd position;
  Eigen::VectorXd velocity;
  Eigen::VectorXd acceleration;
  Eigen::VectorXd force;
  std::string parentName;
  std::string childName;
  std::string type;
};

// first: BodyNode name | second: BodyNode information
using BodyMap = common::aligned_map<std::string, SkelBodyNode>;

// first: Child BodyNode name | second: Joint information
using JointMap = std::map<std::string, SkelJoint>;

// first: Order that Joint appears in file | second: Child BodyNode name
using IndexToJoint = std::map<std::size_t, std::string>;

// first: Child BodyNode name | second: Order that Joint appears in file
using JointToIndex = std::map<std::string, std::size_t>;

bool isSoftMeshShape(const dynamics::Shape* shape)
{
  return shape && shape->getType() == dynamics::SoftMeshShape::getStaticType();
}

std::vector<dynamics::ShapeNode*> collectRigidShapeNodes(
    dynamics::BodyNode* bodyNode)
{
  std::vector<dynamics::ShapeNode*> shapes;
  const auto count = bodyNode->getNumShapeNodes();
  shapes.reserve(count);

  for (auto i = 0u; i < count; ++i) {
    auto* shapeNode = bodyNode->getShapeNode(i);
    if (shapeNode == nullptr || isSoftMeshShape(shapeNode->getShape().get()))
      continue;
    shapes.push_back(shapeNode);
  }

  return shapes;
}

bool setInertiaFromShapeNodes(dynamics::BodyNode* bodyNode)
{
  const double bodyMass = bodyNode->getMass();
  if (bodyMass <= 0.0)
    return false;

  const auto shapeNodes = collectRigidShapeNodes(bodyNode);
  if (shapeNodes.empty())
    return false;

  std::unordered_map<const dynamics::ShapeNode*, double> weights;
  double totalWeight = 0.0;

  for (const auto* shapeNode : shapeNodes) {
    double weight = shapeNode->getShape()->getVolume();
    if (!std::isfinite(weight) || weight <= 0.0)
      weight = 1.0;
    weights[shapeNode] = weight;
    totalWeight += weight;
  }

  if (totalWeight <= 0.0)
    totalWeight = static_cast<double>(weights.size());

  const auto composite = bodyNode->computeInertiaFromShapeNodes(
      [&](const dynamics::ShapeNode* shapeNode) -> std::optional<double> {
        const auto it = weights.find(shapeNode);
        if (it == weights.end())
          return std::nullopt;
        return bodyMass * (it->second / totalWeight);
      });

  if (!composite.has_value())
    return false;

  Eigen::Matrix3d moment = composite->getMoment();
  const Eigen::Vector3d desiredCom = bodyNode->getInertia().getLocalCOM();
  const Eigen::Vector3d delta = composite->getLocalCOM() - desiredCom;
  if (!delta.isZero(1e-12))
    moment = math::parallelAxisTheorem(moment, delta, bodyMass);

  auto inertia = bodyNode->getInertia();
  inertia.setMoment(moment);
  bodyNode->setInertia(inertia);
  return true;
}

simulation::WorldPtr readWorld(
    tinyxml2::XMLElement* _worldElement,
    const common::Uri& _baseUri,
    const common::ResourceRetrieverPtr& _retriever);

dart::dynamics::SkeletonPtr readSkeleton(
    tinyxml2::XMLElement* _skeletonElement,
    const common::Uri& _baseUri,
    const common::ResourceRetrieverPtr& _retriever);

SkelBodyNode readBodyNode(
    tinyxml2::XMLElement* _bodyElement,
    const Eigen::Isometry3d& _skeletonFrame,
    const common::Uri& _baseUri,
    const common::ResourceRetrieverPtr& _retriever);

SkelBodyNode readSoftBodyNode(
    tinyxml2::XMLElement* _softBodyNodeElement,
    const Eigen::Isometry3d& _skeletonFrame,
    const common::Uri& _baseUri,
    const common::ResourceRetrieverPtr& _retriever);

dynamics::ShapePtr readShape(
    tinyxml2::XMLElement* shapeElement,
    std::string_view bodyName,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever);

/// Read marker
dynamics::Marker::BasicProperties readMarker(
    tinyxml2::XMLElement* _markerElement);

void readJoint(
    tinyxml2::XMLElement* _jointElement,
    const BodyMap& _bodyNodes,
    JointMap& _joints,
    IndexToJoint& _order,
    JointToIndex& _lookup);

JointPropPtr readRevoluteJoint(
    tinyxml2::XMLElement* _jointElement,
    SkelJoint& _joint,
    std::string_view _name);

JointPropPtr readPrismaticJoint(
    tinyxml2::XMLElement* _jointElement,
    SkelJoint& _joint,
    std::string_view _name);

JointPropPtr readScrewJoint(
    tinyxml2::XMLElement* _jointElement,
    SkelJoint& _joint,
    std::string_view _name);

JointPropPtr readUniversalJoint(
    tinyxml2::XMLElement* _universalJointElement,
    SkelJoint& _joint,
    std::string_view _name);

JointPropPtr readBallJoint(
    tinyxml2::XMLElement* _jointElement,
    SkelJoint& _joint,
    std::string_view _name);

JointPropPtr readEulerJoint(
    tinyxml2::XMLElement* _jointElement,
    SkelJoint& _joint,
    std::string_view _name);

JointPropPtr readTranslationalJoint(
    tinyxml2::XMLElement* _jointElement,
    SkelJoint& _joint,
    std::string_view _name);

JointPropPtr readTranslationalJoint2D(
    tinyxml2::XMLElement* _jointElement,
    SkelJoint& _joint,
    std::string_view _name);

JointPropPtr readPlanarJoint(
    tinyxml2::XMLElement* _jointElement,
    SkelJoint& _joint,
    std::string_view _name);

JointPropPtr readFreeJoint(
    tinyxml2::XMLElement* _jointElement,
    SkelJoint& _joint,
    std::string_view _name);

JointPropPtr readWeldJoint(
    tinyxml2::XMLElement* _jointElement,
    SkelJoint& _joint,
    std::string_view _name);

common::ResourceRetrieverPtr getRetriever(
    const common::ResourceRetrieverPtr& _retriever);

dynamics::ShapeNode* readShapeNode(
    dynamics::BodyNode* bodyNode,
    tinyxml2::XMLElement* shapeNodeEle,
    std::string_view shapeNodeName,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever);

void readVisualizationShapeNode(
    dynamics::BodyNode* bodyNode,
    tinyxml2::XMLElement* vizShapeNodeEle,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever);

void readCollisionShapeNode(
    dynamics::BodyNode* bodyNode,
    tinyxml2::XMLElement* collShapeNodeEle,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever);

void readAspects(
    const dynamics::SkeletonPtr& skeleton,
    tinyxml2::XMLElement* skeletonElement,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever);

tinyxml2::XMLElement* checkFormatAndGetWorldElement(
    tinyxml2::XMLDocument& _document);

simulation::WorldPtr readWorld(
    tinyxml2::XMLElement* _worldElement,
    const common::Uri& _baseUri,
    const common::ResourceRetrieverPtr& _retriever);

NextResult getNextJointAndNodePair(
    JointMap::iterator& it,
    BodyMap::const_iterator& child,
    dynamics::BodyNode*& parent,
    const dynamics::SkeletonPtr skeleton,
    JointMap& joints,
    const BodyMap& bodyNodes);

template <typename BodyType>
std::pair<dynamics::Joint*, dynamics::BodyNode*> createJointAndNodePair(
    dynamics::SkeletonPtr skeleton,
    dynamics::BodyNode* parent,
    const SkelJoint& joint,
    const typename BodyType::Properties& body);

bool createJointAndNodePair(
    dynamics::SkeletonPtr skeleton,
    dynamics::BodyNode* parent,
    const SkelJoint& joint,
    const SkelBodyNode& body);

dynamics::SkeletonPtr readSkeleton(
    tinyxml2::XMLElement* _skeletonElement,
    const common::Uri& _baseUri,
    const common::ResourceRetrieverPtr& _retriever);

SkelBodyNode readBodyNode(
    tinyxml2::XMLElement* _bodyNodeElement,
    const Eigen::Isometry3d& _skeletonFrame,
    const common::Uri& _baseUri,
    const common::ResourceRetrieverPtr& _retriever);

SkelBodyNode readSoftBodyNode(
    tinyxml2::XMLElement* _softBodyNodeElement,
    const Eigen::Isometry3d& _skeletonFrame,
    const common::Uri& _baseUri,
    const common::ResourceRetrieverPtr& _retriever);

dynamics::ShapePtr readShape(
    tinyxml2::XMLElement* vizEle,
    std::string_view bodyName,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever);

dynamics::Marker::BasicProperties readMarker(
    tinyxml2::XMLElement* _markerElement);

void readJoint(
    tinyxml2::XMLElement* _jointElement,
    const BodyMap& _bodyNodes,
    JointMap& _joints,
    IndexToJoint& _order,
    JointToIndex& _lookup);

void getDofAttributeIfItExists(
    std::string_view _attribute,
    double* _value,
    std::string_view _element_type,
    const tinyxml2::XMLElement* _xmlElement,
    std::string_view _jointName,
    std::size_t _index);

void setDofLimitAttributes(
    tinyxml2::XMLElement* _dofElement,
    std::string_view _element_type,
    std::string_view _jointName,
    std::size_t _index,
    double* lower,
    double* upper,
    double* initial);

template <typename PropertyType>
void readAllDegreesOfFreedom(
    tinyxml2::XMLElement* _jointElement,
    PropertyType& _properties,
    SkelJoint& _joint,
    std::string_view _jointName,
    std::size_t _numDofs);

template <typename PropertyType>
void readDegreeOfFreedom(
    tinyxml2::XMLElement* _dofElement,
    PropertyType& properties,
    SkelJoint& joint,
    std::string_view jointName,
    std::size_t numDofs);

template <typename PropertyType>
void readJointDynamicsAndLimit(
    tinyxml2::XMLElement* _jointElement,
    PropertyType& _properties,
    SkelJoint& _joint,
    std::string_view _name,
    std::size_t _numAxis);

JointPropPtr readWeldJoint(
    tinyxml2::XMLElement* _jointElement, SkelJoint& _joint, std::string_view);

JointPropPtr readRevoluteJoint(
    tinyxml2::XMLElement* _jointElement,
    SkelJoint& _joint,
    std::string_view _name);

JointPropPtr readPrismaticJoint(
    tinyxml2::XMLElement* _jointElement,
    SkelJoint& _joint,
    std::string_view _name);

JointPropPtr readScrewJoint(
    tinyxml2::XMLElement* _jointElement,
    SkelJoint& _joint,
    std::string_view _name);

JointPropPtr readUniversalJoint(
    tinyxml2::XMLElement* _jointElement,
    SkelJoint& _joint,
    std::string_view _name);

JointPropPtr readBallJoint(
    tinyxml2::XMLElement* _jointElement,
    SkelJoint& _joint,
    std::string_view _name);

JointPropPtr readEulerJoint(
    tinyxml2::XMLElement* _jointElement,
    SkelJoint& _joint,
    std::string_view _name);

JointPropPtr readTranslationalJoint(
    tinyxml2::XMLElement* _jointElement,
    SkelJoint& _joint,
    std::string_view _name);

JointPropPtr readPlanarJoint(
    tinyxml2::XMLElement* _jointElement,
    SkelJoint& _joint,
    std::string_view _name);

JointPropPtr readFreeJoint(
    tinyxml2::XMLElement* _jointElement,
    SkelJoint& _joint,
    std::string_view _name);

common::ResourceRetrieverPtr getRetriever(
    const common::ResourceRetrieverPtr& _retriever);

} // anonymous namespace

//==============================================================================
simulation::WorldPtr SkelParser::readWorld(
    const common::Uri& _uri, const common::ResourceRetrieverPtr& _retriever)
{
  const common::ResourceRetrieverPtr retriever = getRetriever(_retriever);

  //--------------------------------------------------------------------------
  // Load xml and create Document
  tinyxml2::XMLDocument _dartFile;
  try {
    openXMLFile(_dartFile, _uri, retriever);
  } catch (std::exception const& e) {
    DART_ERROR(
        "[readWorld] LoadFile [{}] Failed: {}", _uri.toString(), e.what());
    return nullptr;
  }

  tinyxml2::XMLElement* worldElement = checkFormatAndGetWorldElement(_dartFile);
  if (!worldElement) {
    DART_ERROR(
        "[readWorld] File named [{}] could not be parsed!", _uri.toString());
    return nullptr;
  }

  return ::dart::utils::readWorld(worldElement, _uri, retriever);
}

//==============================================================================
simulation::WorldPtr SkelParser::readWorldXML(
    std::string_view xmlString,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever)
{
  const common::ResourceRetrieverPtr resolvedRetriever
      = getRetriever(retriever);

  tinyxml2::XMLDocument dartXml;
  const std::string xmlStringCopy(xmlString);
  if (dartXml.Parse(xmlStringCopy.c_str()) != tinyxml2::XML_SUCCESS) {
    dartXml.PrintError();
    return nullptr;
  }

  tinyxml2::XMLElement* worldElement = checkFormatAndGetWorldElement(dartXml);
  if (!worldElement) {
    DART_ERROR("[readWorldXML] XML String could not be parsed!");
    return nullptr;
  }

  return ::dart::utils::readWorld(worldElement, baseUri, resolvedRetriever);
}

//==============================================================================
dynamics::SkeletonPtr SkelParser::readSkeleton(
    const common::Uri& uri, const common::ResourceRetrieverPtr& nullOrRetriever)
{
  const common::ResourceRetrieverPtr retriever = getRetriever(nullOrRetriever);

  //--------------------------------------------------------------------------
  // Load xml and create Document
  tinyxml2::XMLDocument dartFile;
  try {
    openXMLFile(dartFile, uri, retriever);
  } catch (std::exception const& e) {
    DART_ERROR("LoadFile [{}] Fails: {}", uri.toString(), e.what());
    return nullptr;
  }

  //--------------------------------------------------------------------------
  // Load DART
  tinyxml2::XMLElement* skelElement = nullptr;
  skelElement = dartFile.FirstChildElement("skel");
  if (skelElement == nullptr) {
    DART_ERROR(
        "Skel file[{}] does not contain <skel> as the element.",
        uri.toString());
    return nullptr;
  }

  //--------------------------------------------------------------------------
  // Load World
  tinyxml2::XMLElement* skeletonElement = nullptr;
  skeletonElement = skelElement->FirstChildElement("skeleton");
  if (skeletonElement == nullptr) {
    DART_ERROR(
        "Skel file[{}] does not contain <skeleton> element under <skel> "
        "element.",
        uri.toString());
    return nullptr;
  }

  dynamics::SkeletonPtr newSkeleton
      = ::dart::utils::readSkeleton(skeletonElement, uri, retriever);

  return newSkeleton;
}

namespace {

//==============================================================================
dynamics::ShapeNode* readShapeNode(
    dynamics::BodyNode* bodyNode,
    tinyxml2::XMLElement* shapeNodeEle,
    std::string_view shapeNodeName,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever)
{
  DART_ASSERT(bodyNode);

  auto shape = readShape(shapeNodeEle, bodyNode->getName(), baseUri, retriever);
  auto shapeNode = bodyNode->createShapeNode(shape, std::string(shapeNodeName));

  // Transformation
  if (hasElement(shapeNodeEle, "transformation")) {
    Eigen::Isometry3d W = getValueIsometry3d(shapeNodeEle, "transformation");
    shapeNode->setRelativeTransform(W);
  }

  return shapeNode;
}

//==============================================================================
void readVisualizationShapeNode(
    dynamics::BodyNode* bodyNode,
    tinyxml2::XMLElement* vizShapeNodeEle,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever)
{
  dynamics::ShapeNode* newShapeNode = readShapeNode(
      bodyNode,
      vizShapeNodeEle,
      bodyNode->getName() + " - visual shape",
      baseUri,
      retriever);

  auto visualAspect = newShapeNode->getVisualAspect(true);

  // color
  if (hasElement(vizShapeNodeEle, "color")) {
    Eigen::VectorXd color = getValueVectorXd(vizShapeNodeEle, "color");

    if (color.size() == 3) {
      visualAspect->setColor(static_cast<Eigen::Vector3d>(color));
    } else if (color.size() == 4) {
      visualAspect->setColor(static_cast<Eigen::Vector4d>(color));
    } else {
      DART_WARN(
          "[readVisualizationShapeNode] Invalid format for <color> element; "
          "{}d vector is given. It should be either 3d vector or 4d vector "
          "(the 4th element is for alpha). Ignoring the given color value.",
          color.size());
    }
  }
}

//==============================================================================
void readCollisionShapeNode(
    dynamics::BodyNode* bodyNode,
    tinyxml2::XMLElement* collShapeNodeEle,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever)
{
  dynamics::ShapeNode* newShapeNode = readShapeNode(
      bodyNode,
      collShapeNodeEle,
      bodyNode->getName() + " - collision shape",
      baseUri,
      retriever);

  auto collisionAspect = newShapeNode->getCollisionAspect(true);
  newShapeNode->createDynamicsAspect();

  // collidable
  if (hasElement(collShapeNodeEle, "collidable")) {
    const bool collidable = getValueDouble(collShapeNodeEle, "collidable");
    collisionAspect->setCollidable(collidable);
  }
}

//==============================================================================
void readAspects(
    const dynamics::SkeletonPtr& skeleton,
    tinyxml2::XMLElement* skeletonElement,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever)
{
  ElementEnumerator xmlBodies(skeletonElement, "body");
  while (xmlBodies.next()) {
    auto bodyElement = xmlBodies.get();
    auto bodyNodeName = getAttributeString(bodyElement, "name");
    auto bodyNode = skeleton->getBodyNode(bodyNodeName);

    // visualization_shape
    ElementEnumerator vizShapes(bodyElement, "visualization_shape");
    while (vizShapes.next())
      readVisualizationShapeNode(bodyNode, vizShapes.get(), baseUri, retriever);

    // collision_shape
    ElementEnumerator collShapes(bodyElement, "collision_shape");
    while (collShapes.next())
      readCollisionShapeNode(bodyNode, collShapes.get(), baseUri, retriever);

    // Update inertia if unspecified
    if (hasElement(bodyElement, "inertia")) {
      tinyxml2::XMLElement* inertiaElement = getElement(bodyElement, "inertia");

      if (!hasElement(inertiaElement, "moment_of_inertia"))
        setInertiaFromShapeNodes(bodyNode);
    }
  }
}

//==============================================================================
tinyxml2::XMLElement* checkFormatAndGetWorldElement(
    tinyxml2::XMLDocument& _document)
{
  //--------------------------------------------------------------------------
  // Check xml tag
  tinyxml2::XMLElement* skelElement = nullptr;
  skelElement = _document.FirstChildElement("skel");
  if (skelElement == nullptr) {
    DART_ERROR("XML Document does not contain <skel> as the root element.");
    return nullptr;
  }

  //--------------------------------------------------------------------------
  // Load World
  tinyxml2::XMLElement* worldElement = nullptr;
  worldElement = skelElement->FirstChildElement("world");
  if (worldElement == nullptr) {
    DART_ERROR(
        "XML Document does not contain a <world> element under the <skel> "
        "element.");
    return nullptr;
  }

  return worldElement;
}

//==============================================================================
static std::shared_ptr<collision::CollisionDetector>
createFclMeshCollisionDetector()
{
  auto cd = collision::CollisionDetector::getFactory()->create("fcl");
  auto fcl = std::static_pointer_cast<collision::FCLCollisionDetector>(cd);
  fcl->setPrimitiveShapeType(collision::FCLCollisionDetector::MESH);
  fcl->setContactPointComputationMethod(collision::FCLCollisionDetector::DART);

  return fcl;
}

//==============================================================================
static std::shared_ptr<collision::CollisionDetector>
createFclPrimitiveCollisionDetector()
{
  auto cd = collision::CollisionDetector::getFactory()->create("fcl");
  auto fcl = std::static_pointer_cast<collision::FCLCollisionDetector>(cd);
  fcl->setPrimitiveShapeType(collision::FCLCollisionDetector::PRIMITIVE);
  fcl->setContactPointComputationMethod(collision::FCLCollisionDetector::DART);

  return fcl;
}

//==============================================================================
simulation::WorldPtr readWorld(
    tinyxml2::XMLElement* _worldElement,
    const common::Uri& _baseUri,
    const common::ResourceRetrieverPtr& _retriever)
{
  DART_ASSERT(_worldElement != nullptr);

  // Create a world
  simulation::WorldPtr newWorld = simulation::World::create();

  //--------------------------------------------------------------------------
  // Load physics
  tinyxml2::XMLElement* physicsElement
      = _worldElement->FirstChildElement("physics");
  if (physicsElement != nullptr) {
    // Time step
    tinyxml2::XMLElement* timeStepElement = nullptr;
    timeStepElement = physicsElement->FirstChildElement("time_step");
    if (timeStepElement != nullptr) {
      std::string strTimeStep = timeStepElement->GetText();
      double timeStep = toDouble(strTimeStep);
      newWorld->setTimeStep(timeStep);
    }

    // Gravity
    tinyxml2::XMLElement* gravityElement = nullptr;
    gravityElement = physicsElement->FirstChildElement("gravity");
    if (gravityElement != nullptr) {
      std::string strGravity = gravityElement->GetText();
      Eigen::Vector3d gravity = toVector3d(strGravity);
      newWorld->setGravity(gravity);
    }

    // Collision detector
    std::shared_ptr<collision::CollisionDetector> collision_detector;

    if (hasElement(physicsElement, "collision_detector")) {
      const auto cdType = getValueString(physicsElement, "collision_detector");

      if (cdType == "fcl_mesh") {
        collision_detector = createFclMeshCollisionDetector();
      } else if (cdType == "fcl") {
        collision_detector
            = collision::CollisionDetector::getFactory()->create("fcl");
        auto cd = std::static_pointer_cast<collision::FCLCollisionDetector>(
            collision_detector);
        cd->setPrimitiveShapeType(collision::FCLCollisionDetector::PRIMITIVE);
        cd->setContactPointComputationMethod(
            collision::FCLCollisionDetector::DART);
      } else {
        collision_detector
            = collision::CollisionDetector::getFactory()->create(cdType);
      }

      DART_WARN_IF(
          !collision_detector,
          "Unknown collision detector[{}]. Default collision "
          "detector[fcl_mesh] will be loaded.",
          cdType);
    }

    if (!collision_detector)
      collision_detector = createFclPrimitiveCollisionDetector();

    newWorld->setCollisionDetector(collision_detector);
  }

  //--------------------------------------------------------------------------
  // Load soft skeletons
  ElementEnumerator SkeletonElements(_worldElement, "skeleton");
  while (SkeletonElements.next()) {
    dynamics::SkeletonPtr newSkeleton = ::dart::utils::readSkeleton(
        SkeletonElements.get(), _baseUri, _retriever);

    newWorld->addSkeleton(newSkeleton);
  }

  return newWorld;
}

//==============================================================================
NextResult getNextJointAndNodePair(
    JointMap::iterator& it,
    BodyMap::const_iterator& child,
    dynamics::BodyNode*& parent,
    const dynamics::SkeletonPtr skeleton,
    JointMap& joints,
    const BodyMap& bodyNodes)
{
  NextResult result = VALID;
  const SkelJoint& joint = it->second;
  parent = skeleton->getBodyNode(joint.parentName);
  if (nullptr == parent && !joint.parentName.empty()) {
    // Find the properties of the parent Joint of the current Joint, because it
    // does not seem to be created yet.
    JointMap::iterator check_parent_joint = joints.find(joint.parentName);
    if (check_parent_joint == joints.end()) {
      BodyMap::const_iterator check_parent_node
          = bodyNodes.find(joint.parentName);
      if (check_parent_node == bodyNodes.end()) {
        DART_ERROR(
            "[getNextJointAndNodePair] Could not find BodyNode named [{}] "
            "requested as parent of the Joint named [{}]. We will now quit "
            "parsing.",
            joint.parentName,
            joint.properties->mName);
        return BREAK;
      }

      // If the current Joint has a parent BodyNode but does not have a parent
      // Joint, then we need to create a FreeJoint for the parent BodyNode.
      result = CREATE_FREEJOINT_ROOT;
    } else {
      it = check_parent_joint;
      return CONTINUE; // Create the parent before creating the current Joint
    }
  }

  // Find the child node of this Joint, so we can create them together
  child = bodyNodes.find(joint.childName);
  if (child == bodyNodes.end()) {
    DART_ERROR(
        "[getNextJointAndNodePair] Could not find BodyNode named [{}] "
        "requested as child of Joint [{}]. This should not be possible! We "
        "will now quit parsing. Please report this bug!",
        joint.childName,
        joint.properties->mName);
    return BREAK;
  }

  return result;
}

//==============================================================================
template <typename BodyType>
std::pair<dynamics::Joint*, dynamics::BodyNode*> createJointAndNodePair(
    dynamics::SkeletonPtr skeleton,
    dynamics::BodyNode* parent,
    const SkelJoint& joint,
    const typename BodyType::Properties& body)
{
  if (std::string("weld") == joint.type)
    return skeleton->createJointAndBodyNodePair<dynamics::WeldJoint, BodyType>(
        parent,
        static_cast<const dynamics::WeldJoint::Properties&>(*joint.properties),
        body);

  else if (std::string("prismatic") == joint.type)
    return skeleton
        ->createJointAndBodyNodePair<dynamics::PrismaticJoint, BodyType>(
            parent,
            static_cast<const dynamics::PrismaticJoint::Properties&>(
                *joint.properties),
            body);

  else if (std::string("revolute") == joint.type)
    return skeleton
        ->createJointAndBodyNodePair<dynamics::RevoluteJoint, BodyType>(
            parent,
            static_cast<const dynamics::RevoluteJoint::Properties&>(
                *joint.properties),
            body);

  else if (std::string("universal") == joint.type)
    return skeleton
        ->createJointAndBodyNodePair<dynamics::UniversalJoint, BodyType>(
            parent,
            static_cast<const dynamics::UniversalJoint::Properties&>(
                *joint.properties),
            body);

  else if (std::string("ball") == joint.type)
    return skeleton->createJointAndBodyNodePair<dynamics::BallJoint, BodyType>(
        parent,
        static_cast<const dynamics::BallJoint::Properties&>(*joint.properties),
        body);

  else if (std::string("euler") == joint.type)
    return skeleton->createJointAndBodyNodePair<dynamics::EulerJoint, BodyType>(
        parent,
        static_cast<const dynamics::EulerJoint::Properties&>(*joint.properties),
        body);

  else if (std::string("translational") == joint.type)
    return skeleton
        ->createJointAndBodyNodePair<dynamics::TranslationalJoint, BodyType>(
            parent,
            static_cast<const dynamics::TranslationalJoint::Properties&>(
                *joint.properties),
            body);

  else if (std::string("planar") == joint.type)
    return skeleton
        ->createJointAndBodyNodePair<dynamics::PlanarJoint, BodyType>(
            parent,
            static_cast<const dynamics::PlanarJoint::Properties&>(
                *joint.properties),
            body);

  else if (std::string("free") == joint.type)
    return skeleton->createJointAndBodyNodePair<dynamics::FreeJoint, BodyType>(
        parent,
        static_cast<const dynamics::FreeJoint::Properties&>(*joint.properties),
        body);

  else {
    DART_ERROR(
        "[createJointAndNodePair] Unsupported Joint type ({}) for Joint named "
        "[{}]! It will be discarded.",
        joint.type,
        joint.properties->mName);
    return std::pair<dynamics::Joint*, dynamics::BodyNode*>(nullptr, nullptr);
  }
}

//==============================================================================
bool createJointAndNodePair(
    dynamics::SkeletonPtr skeleton,
    dynamics::BodyNode* parent,
    const SkelJoint& joint,
    const SkelBodyNode& body)
{
  std::pair<dynamics::Joint*, dynamics::BodyNode*> pair;
  if (body.type.empty())
    pair = createJointAndNodePair<dynamics::BodyNode>(
        skeleton,
        parent,
        joint,
        static_cast<const dynamics::BodyNode::Properties&>(*body.properties));
  else if (std::string("soft") == body.type)
    pair = createJointAndNodePair<dynamics::SoftBodyNode>(
        skeleton,
        parent,
        joint,
        static_cast<const dynamics::SoftBodyNode::Properties&>(
            *body.properties));
  else {
    DART_ERROR(
        "[createJointAndNodePair] Invalid type ({}) for BodyNode named [{}]",
        body.type,
        body.properties->mName);
    return false;
  }

  if (pair.first == nullptr || pair.second == nullptr)
    return false;

  dynamics::Joint* newJoint = pair.first;
  newJoint->setPositions(joint.position);
  newJoint->setVelocities(joint.velocity);
  newJoint->setAccelerations(joint.acceleration);
  newJoint->setForces(joint.force);

  dynamics::BodyNode* bn = pair.second;
  for (std::size_t i = 0; i < body.markers.size(); ++i)
    bn->createNode<dynamics::Marker>(body.markers[i]);

  return true;
}

//==============================================================================
dynamics::SkeletonPtr readSkeleton(
    tinyxml2::XMLElement* _skeletonElement,
    const common::Uri& _baseUri,
    const common::ResourceRetrieverPtr& _retriever)
{
  DART_ASSERT(_skeletonElement != nullptr);

  dynamics::SkeletonPtr newSkeleton = dynamics::Skeleton::create();
  Eigen::Isometry3d skeletonFrame = Eigen::Isometry3d::Identity();

  //--------------------------------------------------------------------------
  // Name attribute
  std::string name = getAttributeString(_skeletonElement, "name");
  newSkeleton->setName(name);

  //--------------------------------------------------------------------------
  // transformation
  if (hasElement(_skeletonElement, "transformation")) {
    Eigen::Isometry3d W
        = getValueIsometry3d(_skeletonElement, "transformation");
    skeletonFrame = W;
  }

  //--------------------------------------------------------------------------
  // immobile attribute
  tinyxml2::XMLElement* mobileElement = nullptr;
  mobileElement = _skeletonElement->FirstChildElement("mobile");
  if (mobileElement != nullptr) {
    newSkeleton->setMobile(toBool(mobileElement->GetText()));
  }

  //--------------------------------------------------------------------------
  // Bodies
  BodyMap bodyNodes;
  ElementEnumerator xmlBodies(_skeletonElement, "body");
  while (xmlBodies.next()) {
    SkelBodyNode newBodyNode = readSoftBodyNode(
        xmlBodies.get(), skeletonFrame, _baseUri, _retriever);

    BodyMap::const_iterator it = bodyNodes.find(newBodyNode.properties->mName);
    if (it != bodyNodes.end()) {
      DART_ERROR(
          "[readSkeleton] Skeleton named [{}] has multiple BodyNodes with the "
          "name [{}], but BodyNode names must be unique! We will discard all "
          "BodyNodes with a repeated name.",
          name,
          newBodyNode.properties->mName);
      continue;
    }

    bodyNodes[newBodyNode.properties->mName] = newBodyNode;
  }

  //--------------------------------------------------------------------------
  // Joints
  JointMap joints;
  IndexToJoint order;
  JointToIndex lookup;
  ElementEnumerator xmlJoints(_skeletonElement, "joint");
  while (xmlJoints.next())
    readJoint(xmlJoints.get(), bodyNodes, joints, order, lookup);

  //--------------------------------------------------------------------------
  // Assemble skeleton
  JointMap::iterator it = joints.find(order.begin()->second);
  BodyMap::const_iterator child;
  dynamics::BodyNode* parent;
  while (it != joints.end()) {
    NextResult result = getNextJointAndNodePair(
        it, child, parent, newSkeleton, joints, bodyNodes);

    if (BREAK == result)
      break;
    else if (CONTINUE == result)
      continue;
    else if (CREATE_FREEJOINT_ROOT == result) {
      // If a root FreeJoint is needed for the parent of the current joint, then
      // create it
      BodyMap::const_iterator rootNode = bodyNodes.find(it->second.parentName);
      SkelJoint rootJoint;
      rootJoint.properties = dynamics::FreeJoint::Properties::createShared(
          dynamics::Joint::Properties("root", rootNode->second.initTransform));
      rootJoint.type = "free";

      if (!createJointAndNodePair(
              newSkeleton, nullptr, rootJoint, rootNode->second))
        break;

      continue;
    }

    if (!createJointAndNodePair(newSkeleton, parent, it->second, child->second))
      break;

    JointToIndex::iterator index = lookup.find(it->first);
    order.erase(index->second);
    lookup.erase(index);
    joints.erase(it);

    IndexToJoint::iterator nextJoint = order.begin();
    if (nextJoint == order.end())
      break;

    it = joints.find(nextJoint->second);
  }

  // Read aspects here since aspects cannot be added if the BodyNodes haven't
  // created yet.
  readAspects(newSkeleton, _skeletonElement, _baseUri, _retriever);

  newSkeleton->resetPositions();
  newSkeleton->resetVelocities();

  return newSkeleton;
}

//==============================================================================
SkelBodyNode readBodyNode(
    tinyxml2::XMLElement* _bodyNodeElement,
    const Eigen::Isometry3d& _skeletonFrame,
    const common::Uri& /*_baseUri*/,
    const common::ResourceRetrieverPtr& /*_retriever*/)
{
  DART_ASSERT(_bodyNodeElement != nullptr);

  BodyPropPtr newBodyNode(new dynamics::BodyNode::Properties);
  Eigen::Isometry3d initTransform = Eigen::Isometry3d::Identity();

  // Name attribute
  newBodyNode->mName = getAttributeString(_bodyNodeElement, "name");

  //--------------------------------------------------------------------------
  // gravity
  if (hasElement(_bodyNodeElement, "gravity")) {
    newBodyNode->mGravityMode = getValueBool(_bodyNodeElement, "gravity");
  }

  //--------------------------------------------------------------------------
  // self_collide
  //    if (hasElement(_bodyElement, "self_collide"))
  //    {
  //        bool gravityMode = getValueBool(_bodyElement, "self_collide");
  //    }

  //--------------------------------------------------------------------------
  // transformation
  if (hasElement(_bodyNodeElement, "transformation")) {
    Eigen::Isometry3d W
        = getValueIsometry3d(_bodyNodeElement, "transformation");
    initTransform = _skeletonFrame * W;
  } else {
    initTransform = _skeletonFrame;
  }

  //--------------------------------------------------------------------------
  // inertia
  if (hasElement(_bodyNodeElement, "inertia")) {
    tinyxml2::XMLElement* inertiaElement
        = getElement(_bodyNodeElement, "inertia");

    // mass
    double mass = getValueDouble(inertiaElement, "mass");
    newBodyNode->mInertia.setMass(mass);

    // moment of inertia
    if (hasElement(inertiaElement, "moment_of_inertia")) {
      tinyxml2::XMLElement* moiElement
          = getElement(inertiaElement, "moment_of_inertia");

      double ixx = getValueDouble(moiElement, "ixx");
      double iyy = getValueDouble(moiElement, "iyy");
      double izz = getValueDouble(moiElement, "izz");

      double ixy = getValueDouble(moiElement, "ixy");
      double ixz = getValueDouble(moiElement, "ixz");
      double iyz = getValueDouble(moiElement, "iyz");

      newBodyNode->mInertia.setMoment(ixx, iyy, izz, ixy, ixz, iyz);
    }

    // offset
    if (hasElement(inertiaElement, "offset")) {
      Eigen::Vector3d offset = getValueVector3d(inertiaElement, "offset");
      newBodyNode->mInertia.setLocalCOM(offset);
    }
  }

  SkelBodyNode skelBodyNode;
  skelBodyNode.properties = newBodyNode;
  skelBodyNode.initTransform = initTransform;

  //--------------------------------------------------------------------------
  // marker
  ElementEnumerator markers(_bodyNodeElement, "marker");
  while (markers.next()) {
    skelBodyNode.markers.push_back(readMarker(markers.get()));
  }

  return skelBodyNode;
}

//==============================================================================
SkelBodyNode readSoftBodyNode(
    tinyxml2::XMLElement* _softBodyNodeElement,
    const Eigen::Isometry3d& _skeletonFrame,
    const common::Uri& _baseUri,
    const common::ResourceRetrieverPtr& _retriever)
{
  //---------------------------------- Note ------------------------------------
  // SoftBodyNode is created if _softBodyNodeElement has <soft_shape>.
  // Otherwise, BodyNode is created.

  //----------------------------------------------------------------------------
  DART_ASSERT(_softBodyNodeElement != nullptr);

  SkelBodyNode standardBodyNode = readBodyNode(
      _softBodyNodeElement, _skeletonFrame, _baseUri, _retriever);

  // If _softBodyNodeElement has no <soft_shape>, return rigid body node
  if (!hasElement(_softBodyNodeElement, "soft_shape"))
    return standardBodyNode;

  //----------------------------------------------------------------------------
  // Soft properties
  dynamics::SoftBodyNode::UniqueProperties newSoftBodyNode;

  if (hasElement(_softBodyNodeElement, "soft_shape")) {
    tinyxml2::XMLElement* softShapeEle
        = getElement(_softBodyNodeElement, "soft_shape");

    // mass
    double totalMass = getValueDouble(softShapeEle, "total_mass");

    // transformation
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    if (hasElement(softShapeEle, "transformation"))
      T = getValueIsometry3d(softShapeEle, "transformation");

    // geometry
    tinyxml2::XMLElement* geometryEle = getElement(softShapeEle, "geometry");
    if (hasElement(geometryEle, "sphere")) {
      tinyxml2::XMLElement* sphereEle = getElement(geometryEle, "sphere");
      const auto radius = getValueDouble(sphereEle, "radius");
      const auto nSlices = getValueUInt(sphereEle, "num_slices");
      const auto nStacks = getValueUInt(sphereEle, "num_stacks");
      newSoftBodyNode = dynamics::SoftBodyNodeHelper::makeSphereProperties(
          radius, nSlices, nStacks, totalMass);
    } else if (hasElement(geometryEle, "box")) {
      tinyxml2::XMLElement* boxEle = getElement(geometryEle, "box");
      Eigen::Vector3d size = getValueVector3d(boxEle, "size");
      Eigen::Vector3i frags = getValueVector3i(boxEle, "frags");
      newSoftBodyNode = dynamics::SoftBodyNodeHelper::makeBoxProperties(
          size, T, frags, totalMass);
    } else if (hasElement(geometryEle, "ellipsoid")) {
      tinyxml2::XMLElement* ellipsoidEle = getElement(geometryEle, "ellipsoid");
      Eigen::Vector3d size = getValueVector3d(ellipsoidEle, "size");
      const auto nSlices = getValueUInt(ellipsoidEle, "num_slices");
      const auto nStacks = getValueUInt(ellipsoidEle, "num_stacks");
      newSoftBodyNode = dynamics::SoftBodyNodeHelper::makeEllipsoidProperties(
          size, nSlices, nStacks, totalMass);
    } else if (hasElement(geometryEle, "cylinder")) {
      tinyxml2::XMLElement* ellipsoidEle = getElement(geometryEle, "cylinder");
      double radius = getValueDouble(ellipsoidEle, "radius");
      double height = getValueDouble(ellipsoidEle, "height");
      double nSlices = getValueDouble(ellipsoidEle, "num_slices");
      double nStacks = getValueDouble(ellipsoidEle, "num_stacks");
      double nRings = getValueDouble(ellipsoidEle, "num_rings");
      newSoftBodyNode = dynamics::SoftBodyNodeHelper::makeCylinderProperties(
          radius, height, nSlices, nStacks, nRings, totalMass);
    } else {
      DART_ERROR(
          "[readSoftBodyNode] Unknown soft shape in SoftBodyNode named [{}]",
          standardBodyNode.properties->mName);
    }

    // kv
    if (hasElement(softShapeEle, "kv")) {
      newSoftBodyNode.mKv = getValueDouble(softShapeEle, "kv");
    }

    // ke
    if (hasElement(softShapeEle, "ke")) {
      newSoftBodyNode.mKe = getValueDouble(softShapeEle, "ke");
    }

    // damp
    if (hasElement(softShapeEle, "damp")) {
      newSoftBodyNode.mDampCoeff = getValueDouble(softShapeEle, "damp");
    }
  }

  SkelBodyNode softBodyNode;
  softBodyNode.properties = dynamics::SoftBodyNode::Properties::createShared(
      *standardBodyNode.properties, newSoftBodyNode);

  softBodyNode.initTransform = standardBodyNode.initTransform;
  softBodyNode.type = "soft";

  return softBodyNode;
}

//==============================================================================
dynamics::ShapePtr readShape(
    tinyxml2::XMLElement* vizEle,
    std::string_view bodyName,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever)
{
  dynamics::ShapePtr newShape;

  // Geometry
  DART_ASSERT(hasElement(vizEle, "geometry"));
  tinyxml2::XMLElement* geometryEle = getElement(vizEle, "geometry");

  if (hasElement(geometryEle, "sphere")) {
    tinyxml2::XMLElement* sphereEle = getElement(geometryEle, "sphere");
    const auto radius = getValueDouble(sphereEle, "radius");
    newShape = dynamics::ShapePtr(new dynamics::SphereShape(radius));
  } else if (hasElement(geometryEle, "box")) {
    tinyxml2::XMLElement* boxEle = getElement(geometryEle, "box");
    Eigen::Vector3d size = getValueVector3d(boxEle, "size");
    newShape = dynamics::ShapePtr(new dynamics::BoxShape(size));
  } else if (hasElement(geometryEle, "ellipsoid")) {
    tinyxml2::XMLElement* ellipsoidEle = getElement(geometryEle, "ellipsoid");
    Eigen::Vector3d size = getValueVector3d(ellipsoidEle, "size");
    newShape = dynamics::ShapePtr(new dynamics::EllipsoidShape(size));
  } else if (hasElement(geometryEle, "cylinder")) {
    tinyxml2::XMLElement* cylinderEle = getElement(geometryEle, "cylinder");
    double radius = getValueDouble(cylinderEle, "radius");
    double height = getValueDouble(cylinderEle, "height");
    newShape = dynamics::ShapePtr(new dynamics::CylinderShape(radius, height));
  } else if (hasElement(geometryEle, "capsule")) {
    tinyxml2::XMLElement* capsuleEle = getElement(geometryEle, "capsule");
    double radius = getValueDouble(capsuleEle, "radius");
    double height = getValueDouble(capsuleEle, "height");
    newShape = dynamics::ShapePtr(new dynamics::CapsuleShape(radius, height));
  } else if (hasElement(geometryEle, "cone")) {
    tinyxml2::XMLElement* coneEle = getElement(geometryEle, "cone");
    double radius = getValueDouble(coneEle, "radius");
    double height = getValueDouble(coneEle, "height");
    newShape = dynamics::ShapePtr(new dynamics::ConeShape(radius, height));
  } else if (hasElement(geometryEle, "pyramid")) {
    tinyxml2::XMLElement* coneEle = getElement(geometryEle, "pyramid");
    double base_width = getValueDouble(coneEle, "base_width");
    double base_depth = getValueDouble(coneEle, "base_depth");
    double height = getValueDouble(coneEle, "height");
    newShape = dynamics::ShapePtr(
        new dynamics::PyramidShape(base_width, base_depth, height));
  } else if (hasElement(geometryEle, "plane")) {
    tinyxml2::XMLElement* planeEle = getElement(geometryEle, "plane");
    Eigen::Vector3d normal = getValueVector3d(planeEle, "normal");
    if (hasElement(planeEle, "offset")) {
      double offset = getValueDouble(planeEle, "offset");
      newShape = dynamics::ShapePtr(new dynamics::PlaneShape(normal, offset));
    } else if (hasElement(planeEle, "point")) {
      DART_ERROR(
          "[readShape] <point> element of <plane> was removed. Please use the "
          "<offset> element instead.");
      return nullptr;
    } else {
      DART_WARN(
          "[readShape] <offset> element is not specified for plane shape. DART "
          "will use 0.0.");
      newShape = dynamics::ShapePtr(new dynamics::PlaneShape(normal, 0.0));
    }
  } else if (hasElement(geometryEle, "multi_sphere")) {
    tinyxml2::XMLElement* multiSphereEle
        = getElement(geometryEle, "multi_sphere");

    ElementEnumerator xmlSpheres(multiSphereEle, "sphere");
    dynamics::MultiSphereConvexHullShape::Spheres spheres;
    while (xmlSpheres.next()) {
      const double radius = getValueDouble(xmlSpheres.get(), "radius");
      const Eigen::Vector3d position
          = getValueVector3d(xmlSpheres.get(), "position");

      spheres.emplace_back(radius, position);
    }

    newShape
        = dynamics::ShapePtr(new dynamics::MultiSphereConvexHullShape(spheres));
  } else if (hasElement(geometryEle, "mesh")) {
    tinyxml2::XMLElement* meshEle = getElement(geometryEle, "mesh");
    std::string filename = getValueString(meshEle, "file_name");
    Eigen::Vector3d scale = getValueVector3d(meshEle, "scale");

    const common::Uri meshUri = common::Uri::createFromRelativeUri(
        baseUri, std::string_view{filename});

    auto loader = std::make_unique<utils::MeshLoaderd>();
    auto triMeshUnique = loader->load(meshUri.toString(), retriever);

    if (triMeshUnique) {
      std::shared_ptr<math::TriMesh<double>> triMesh(std::move(triMeshUnique));
      newShape = std::make_shared<dynamics::MeshShape>(
          scale, std::move(triMesh), meshUri, retriever);
    } else {
      DART_ERROR("Fail to load model[{}].", filename);
    }
  } else {
    DART_ERROR(
        "[readShape] Unknown visualization shape in BodyNode named [{}]",
        bodyName);
    DART_ASSERT(0);
    return nullptr;
  }

  return newShape;
}

//==============================================================================
dynamics::Marker::BasicProperties readMarker(
    tinyxml2::XMLElement* _markerElement)
{
  // Name attribute
  std::string name = getAttributeString(_markerElement, "name");

  // offset
  Eigen::Vector3d offset = Eigen::Vector3d::Zero();
  if (hasElement(_markerElement, "offset"))
    offset = getValueVector3d(_markerElement, "offset");

  dynamics::Marker::BasicProperties newMarker;
  newMarker.mName = name;
  newMarker.mRelativeTf.translation() = offset;

  return newMarker;
}

//==============================================================================
void readJoint(
    tinyxml2::XMLElement* _jointElement,
    const BodyMap& _bodyNodes,
    JointMap& _joints,
    IndexToJoint& _order,
    JointToIndex& _lookup)
{
  DART_ASSERT(_jointElement != nullptr);

  //--------------------------------------------------------------------------
  // Name attribute
  std::string name = getAttributeString(_jointElement, "name");

  SkelJoint joint;

  //--------------------------------------------------------------------------
  // Type attribute
  joint.type = getAttributeString(_jointElement, "type");
  DART_ASSERT(!joint.type.empty());
  if (joint.type == std::string("weld"))
    joint.properties = readWeldJoint(_jointElement, joint, name);
  else if (joint.type == std::string("prismatic"))
    joint.properties = readPrismaticJoint(_jointElement, joint, name);
  else if (joint.type == std::string("revolute"))
    joint.properties = readRevoluteJoint(_jointElement, joint, name);
  else if (joint.type == std::string("screw"))
    joint.properties = readScrewJoint(_jointElement, joint, name);
  else if (joint.type == std::string("universal"))
    joint.properties = readUniversalJoint(_jointElement, joint, name);
  else if (joint.type == std::string("ball"))
    joint.properties = readBallJoint(_jointElement, joint, name);
  else if (joint.type == std::string("euler"))
    joint.properties = readEulerJoint(_jointElement, joint, name);
  else if (joint.type == std::string("translational"))
    joint.properties = readTranslationalJoint(_jointElement, joint, name);
  else if (joint.type == std::string("translational2d"))
    joint.properties = readTranslationalJoint2D(_jointElement, joint, name);
  else if (joint.type == std::string("planar"))
    joint.properties = readPlanarJoint(_jointElement, joint, name);
  else if (joint.type == std::string("free"))
    joint.properties = readFreeJoint(_jointElement, joint, name);
  else {
    DART_ERROR(
        "[readJoint] Unsupported joint type [{}] requested by Joint named "
        "[{}]. This Joint will be discarded.",
        joint.type,
        name);
    return;
  }
  DART_ASSERT(joint.properties != nullptr);

  joint.properties->mName = name;

  //--------------------------------------------------------------------------
  // Actuator attribute
  if (hasAttribute(_jointElement, "actuator")) {
    const std::string actuator = getAttributeString(_jointElement, "actuator");

    if (actuator == "force")
      joint.properties->mActuatorType = dynamics::Joint::FORCE;
    else if (actuator == "passive")
      joint.properties->mActuatorType = dynamics::Joint::PASSIVE;
    else if (actuator == "servo")
      joint.properties->mActuatorType = dynamics::Joint::SERVO;
    else if (actuator == "acceleration")
      joint.properties->mActuatorType = dynamics::Joint::ACCELERATION;
    else if (actuator == "velocity")
      joint.properties->mActuatorType = dynamics::Joint::VELOCITY;
    else if (actuator == "locked")
      joint.properties->mActuatorType = dynamics::Joint::LOCKED;
    else
      DART_ERROR(
          "Joint named [{}] contains invalid actuator attribute [{}].",
          name,
          actuator);
  } else {
    joint.properties->mActuatorType = dynamics::Joint::DefaultActuatorType;
  }

  //--------------------------------------------------------------------------
  // parent
  BodyMap::const_iterator parent = _bodyNodes.end();
  if (hasElement(_jointElement, "parent")) {
    joint.parentName = getValueString(_jointElement, "parent");
    parent = _bodyNodes.find(joint.parentName);
  } else {
    DART_ERROR(
        "[readJoint] Joint named [{}] is missing a parent BodyNode!", name);
    DART_ASSERT(0);
  }

  // Use an empty string (rather than "world") to indicate that the joint has no
  // parent
  if (joint.parentName == std::string("world") && !_bodyNodes.contains("world"))
    joint.parentName.clear();

  if (parent == _bodyNodes.end() && !joint.parentName.empty()) {
    DART_ERROR(
        "[readJoint] Could not find a BodyNode named [{}] requested as the "
        "parent of Joint named [{}]!",
        joint.parentName,
        name);
    return;
  }

  //--------------------------------------------------------------------------
  // child
  BodyMap::const_iterator child = _bodyNodes.end();
  if (hasElement(_jointElement, "child")) {
    joint.childName = getValueString(_jointElement, "child");
    child = _bodyNodes.find(joint.childName);
  } else {
    DART_ERROR(
        "[readJoint] Joint named [{}] is missing a child BodyNode!", name);
    DART_ASSERT(0);
  }

  if (child == _bodyNodes.end()) {
    DART_ERROR(
        "[readJoint] Could not find a BodyNode named [{}] requested as the "
        "child of Joint named [{}]!",
        joint.childName,
        name);
    return;
  }

  //--------------------------------------------------------------------------
  // transformation
  Eigen::Isometry3d parentWorld = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d childToJoint = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d childWorld = child->second.initTransform;

  if (parent != _bodyNodes.end())
    parentWorld = parent->second.initTransform;

  if (hasElement(_jointElement, "transformation"))
    childToJoint = getValueIsometry3d(_jointElement, "transformation");

  Eigen::Isometry3d parentToJoint
      = parentWorld.inverse() * childWorld * childToJoint;

  joint.properties->mT_ParentBodyToJoint = parentToJoint;
  DART_ERROR_IF(
      !math::verifyTransform(joint.properties->mT_ParentBodyToJoint),
      "[readJoint] Invalid parent to Joint transform for Joint named "
      "[{}]:\\n{}",
      name,
      joint.properties->mT_ParentBodyToJoint.matrix());

  joint.properties->mT_ChildBodyToJoint = childToJoint;
  DART_ERROR_IF(
      !math::verifyTransform(joint.properties->mT_ChildBodyToJoint),
      "[readJoint] Invalid child to Joint transform for Joint named [{}]:\\n{}",
      name,
      joint.properties->mT_ChildBodyToJoint.matrix());

  JointMap::iterator it = _joints.find(joint.childName);
  if (it != _joints.end()) {
    DART_ERROR(
        "[readJoint] BodyNode named [{}] has been assigned two parent Joints: "
        "[{}] and [{}]. A BodyNode must have exactly one parent Joint. [{}] "
        "will be discarded!",
        joint.childName,
        it->second.properties->mName,
        name,
        name);
    return;
  }

  _joints[joint.childName] = joint;

  // Keep track of when each joint appeared in the file
  std::size_t nextIndex;
  IndexToJoint::reverse_iterator lastIndex = _order.rbegin();
  if (lastIndex == _order.rend())
    nextIndex = 0;
  else
    nextIndex = lastIndex->first + 1;

  _order[nextIndex] = joint.childName;
  _lookup[joint.childName] = nextIndex;
}

//==============================================================================
void getDofAttributeIfItExists(
    std::string_view _attribute,
    double* _value,
    std::string_view _element_type,
    const tinyxml2::XMLElement* _xmlElement,
    std::string_view _jointName,
    std::size_t _index)
{
  const std::string attributeName(_attribute);
  DART_ERROR_IF(
      _xmlElement->QueryDoubleAttribute(attributeName.c_str(), _value)
          == tinyxml2::XML_WRONG_ATTRIBUTE_TYPE,
      "[getDofAttributeIfItExists] Invalid type for [{}] attribute of [{}] "
      "element in the [{}] dof of Joint [{}].",
      _attribute,
      _element_type,
      _index,
      _jointName);
}

//==============================================================================
void setDofLimitAttributes(
    tinyxml2::XMLElement* _dofElement,
    std::string_view _element_type,
    std::string_view _jointName,
    std::size_t _index,
    double* lower,
    double* upper,
    double* initial)
{
  const tinyxml2::XMLElement* xmlElement
      = getElement(_dofElement, _element_type);

  getDofAttributeIfItExists(
      "lower", lower, _element_type, xmlElement, _jointName, _index);
  getDofAttributeIfItExists(
      "upper", upper, _element_type, xmlElement, _jointName, _index);
  getDofAttributeIfItExists(
      "initial", initial, _element_type, xmlElement, _jointName, _index);
}

//==============================================================================
// This structure exists to allow a common interface for setting values in
// GenericJoint::Properties
struct DofProxy
{
  std::size_t index;
  bool valid;

  double* lowerPosition;
  double* upperPosition;
  double* initalPosition;

  double* lowerVelocity;
  double* upperVelocity;
  double* initialVelocity;

  double* lowerAcceleration;
  double* upperAcceleration;
  double* initialAcceleration;

  double* lowerForce;
  double* upperForce;
  double* initialForce;

  double* springStiffness;
  double* restPosition;
  double* dampingCoefficient;
  double* friction;

  bool* preserveName;
  std::string* name;

  template <typename PropertyType>
  DofProxy(
      PropertyType& properties,
      SkelJoint& joint,
      std::size_t _index,
      std::string_view jointName)
    : index(_index),
      valid(true),

      lowerPosition(&properties.mPositionLowerLimits.data()[index]),
      upperPosition(&properties.mPositionUpperLimits.data()[index]),
      initalPosition(&properties.mInitialPositions.data()[index]),

      lowerVelocity(&properties.mVelocityLowerLimits.data()[index]),
      upperVelocity(&properties.mVelocityUpperLimits.data()[index]),
      initialVelocity(&properties.mInitialVelocities.data()[index]),

      lowerAcceleration(&properties.mAccelerationLowerLimits.data()[index]),
      upperAcceleration(&properties.mAccelerationUpperLimits.data()[index]),
      initialAcceleration(&joint.acceleration.data()[index]),

      lowerForce(&properties.mForceLowerLimits.data()[index]),
      upperForce(&properties.mForceUpperLimits.data()[index]),
      initialForce(&joint.force.data()[index]),

      springStiffness(&properties.mSpringStiffnesses.data()[index]),
      restPosition(&properties.mRestPositions.data()[index]),
      dampingCoefficient(&properties.mDampingCoefficients.data()[index]),
      friction(&properties.mFrictions.data()[index]),

      preserveName(&properties.mPreserveDofNames[index]),
      name(&properties.mDofNames[index])
  {
    if ((int)index >= properties.mPositionLowerLimits.size()) {
      DART_ERROR(
          "[SkelParser] Joint named [{}] has a dof element ({}) which is out "
          "of bounds (max {})",
          jointName,
          index,
          properties.mPositionLowerLimits.size() - 1);
      valid = false;
    }
  }
};

//==============================================================================
template <typename PropertyType>
void readAllDegreesOfFreedom(
    tinyxml2::XMLElement* _jointElement,
    PropertyType& _properties,
    SkelJoint& _joint,
    std::string_view _jointName,
    std::size_t _numDofs)
{
  if (_joint.position.size() < (int)_numDofs) {
    _joint.position.resize(_numDofs);
    _joint.position.setZero();
  }

  if (_joint.velocity.size() < (int)_numDofs) {
    _joint.velocity.resize(_numDofs);
    _joint.velocity.setZero();
  }

  if (_joint.acceleration.size() < (int)_numDofs) {
    _joint.acceleration.resize((int)_numDofs);
    _joint.acceleration.setZero();
  }

  if (_joint.force.size() < (int)_numDofs) {
    _joint.force.resize(_numDofs);
    _joint.force.setZero();
  }

  ElementEnumerator DofElements(_jointElement, "dof");
  while (DofElements.next())
    readDegreeOfFreedom(
        DofElements.get(), _properties, _joint, _jointName, _numDofs);
}

//==============================================================================
template <typename PropertyType>
void readDegreeOfFreedom(
    tinyxml2::XMLElement* _dofElement,
    PropertyType& properties,
    SkelJoint& joint,
    std::string_view jointName,
    std::size_t numDofs)
{
  int localIndex = -1;
  int xml_err = _dofElement->QueryIntAttribute("local_index", &localIndex);

  // If the localIndex is out of bounds, quit
  if (localIndex >= (int)numDofs) {
    DART_ERROR(
        "[readDegreeOfFreedom] Joint named '{}' contains dof element with "
        "invalid number attribute [{}]. It must be less than {}.",
        jointName,
        localIndex,
        numDofs);
    return;
  }

  // If no localIndex was found, report an error and quit
  if (localIndex == -1 && numDofs > 1) {
    if (tinyxml2::XML_NO_ATTRIBUTE == xml_err) {
      DART_ERROR(
          "[readDegreeOfFreedom] Joint named [{}] has [{}] DOFs, but the xml "
          "contains a dof element without its local_index specified. For "
          "Joints with multiple DOFs, all dof elements must specify their "
          "local_index attribute.",
          jointName,
          numDofs);
    } else if (tinyxml2::XML_WRONG_ATTRIBUTE_TYPE == xml_err) {
      DART_ERROR(
          "[readDegreeOfFreedom] Joint named [{}] has a dof element with a "
          "wrongly formatted local_index attribute.",
          jointName);
    }

    return;
  }
  // Unless the joint is a single-dof joint
  else if (localIndex == -1 && numDofs == 1)
    localIndex = 0;

  DofProxy proxy(properties, joint, localIndex, jointName);

  const char* name = _dofElement->Attribute("name");
  if (name) {
    *proxy.name = std::string(name);
    *proxy.preserveName = true;
  }

  if (hasElement(_dofElement, "position")) {
    setDofLimitAttributes(
        _dofElement,
        "position",
        jointName,
        localIndex,
        proxy.lowerPosition,
        proxy.upperPosition,
        proxy.initalPosition);
  }

  if (hasElement(_dofElement, "velocity")) {
    setDofLimitAttributes(
        _dofElement,
        "velocity",
        jointName,
        localIndex,
        proxy.lowerVelocity,
        proxy.upperVelocity,
        proxy.initialVelocity);
  }

  if (hasElement(_dofElement, "acceleration")) {
    setDofLimitAttributes(
        _dofElement,
        "acceleration",
        jointName,
        localIndex,
        proxy.lowerAcceleration,
        proxy.upperAcceleration,
        proxy.initialAcceleration);
  }

  if (hasElement(_dofElement, "force")) {
    setDofLimitAttributes(
        _dofElement,
        "force",
        jointName,
        localIndex,
        proxy.lowerForce,
        proxy.upperForce,
        proxy.initialForce);
  }

  if (hasElement(_dofElement, "damping"))
    *proxy.dampingCoefficient = getValueDouble(_dofElement, "damping");

  if (hasElement(_dofElement, "friction"))
    *proxy.friction = getValueDouble(_dofElement, "friction");

  if (hasElement(_dofElement, "spring_rest_position"))
    *proxy.restPosition = getValueDouble(_dofElement, "spring_rest_position");

  if (hasElement(_dofElement, "spring_stiffness"))
    *proxy.springStiffness = getValueDouble(_dofElement, "spring_stiffness");
}

//==============================================================================
template <typename PropertyType>
void readJointDynamicsAndLimit(
    tinyxml2::XMLElement* _jointElement,
    PropertyType& _properties,
    SkelJoint& _joint,
    std::string_view _name,
    std::size_t _numAxis)
{
  // TODO(MXG): Consider printing warnings for these tags that recommends using
  // the dof tag instead, because all functionality of these tags have been
  // moved to the dof tag

  DART_ASSERT(_jointElement != nullptr);
  DART_ASSERT(_numAxis <= 6);

  std::string axisName = "axis";

  // axis
  for (std::size_t i = 0; i < _numAxis; ++i) {
    if (i != 0)
      axisName = "axis" + std::to_string(i + 1);

    if (hasElement(_jointElement, axisName)) {
      DofProxy proxy(_properties, _joint, i, _name);

      tinyxml2::XMLElement* axisElement = getElement(_jointElement, axisName);

      // damping
      if (hasElement(axisElement, "damping")) {
        DART_WARN(
            "[SkelParser] <damping> tag is now an element under the <dynamics> "
            "tag. Please see (https://github.com/dartsim/dart/wiki/) for more "
            "details.");
        double damping = getValueDouble(axisElement, "damping");
        *proxy.dampingCoefficient = damping;
      }

      // dynamics
      if (hasElement(axisElement, "dynamics")) {
        tinyxml2::XMLElement* dynamicsElement
            = getElement(axisElement, "dynamics");

        // damping
        if (hasElement(dynamicsElement, "damping")) {
          double val = getValueDouble(dynamicsElement, "damping");
          *proxy.dampingCoefficient = val;
        }

        // friction
        if (hasElement(dynamicsElement, "friction")) {
          double val = getValueDouble(dynamicsElement, "friction");
          *proxy.friction = val;
        }

        // spring_rest_position
        if (hasElement(dynamicsElement, "spring_rest_position")) {
          double val = getValueDouble(dynamicsElement, "spring_rest_position");
          *proxy.restPosition = val;
        }

        // spring_stiffness
        if (hasElement(dynamicsElement, "spring_stiffness")) {
          double val = getValueDouble(dynamicsElement, "spring_stiffness");
          *proxy.springStiffness = val;
        }
      }

      // limit
      if (hasElement(axisElement, "limit")) {
        tinyxml2::XMLElement* limitElement = getElement(axisElement, "limit");

        // lower
        if (hasElement(limitElement, "lower")) {
          double lower = getValueDouble(limitElement, "lower");
          *proxy.lowerPosition = lower;
        }

        // upper
        if (hasElement(limitElement, "upper")) {
          double upper = getValueDouble(limitElement, "upper");
          *proxy.upperPosition = upper;
        }
      }
    }
  }
}

//==============================================================================
JointPropPtr readWeldJoint(
    tinyxml2::XMLElement* /*_jointElement*/,
    SkelJoint& /*_joint*/,
    std::string_view)
{
  return dynamics::WeldJoint::Properties::createShared();
}

//==============================================================================
JointPropPtr readRevoluteJoint(
    tinyxml2::XMLElement* _jointElement,
    SkelJoint& _joint,
    std::string_view _name)
{
  DART_ASSERT(_jointElement != nullptr);

  dynamics::RevoluteJoint::Properties properties;

  //--------------------------------------------------------------------------
  // axis
  if (hasElement(_jointElement, "axis")) {
    tinyxml2::XMLElement* axisElement = getElement(_jointElement, "axis");

    // xyz
    Eigen::Vector3d xyz = getValueVector3d(axisElement, "xyz");
    properties.mAxis = xyz;
  } else {
    DART_ERROR(
        "[readRevoluteJoint] Revolute Joint named [{}] is missing axis "
        "information!",
        _name);
    DART_ASSERT(0);
  }

  readJointDynamicsAndLimit<dynamics::GenericJoint<math::R1Space>::Properties>(
      _jointElement, properties, _joint, _name, 1);

  //--------------------------------------------------------------------------
  // init_pos
  if (hasElement(_jointElement, "init_pos")) {
    double init_pos = getValueDouble(_jointElement, "init_pos");
    Eigen::VectorXd ipos = Eigen::VectorXd(1);
    ipos << init_pos;
    _joint.position = ipos;
    properties.mInitialPositions[0] = ipos[0];
  }

  //--------------------------------------------------------------------------
  // init_vel
  if (hasElement(_jointElement, "init_vel")) {
    double init_vel = getValueDouble(_jointElement, "init_vel");
    Eigen::VectorXd ivel = Eigen::VectorXd(1);
    ivel << init_vel;
    _joint.velocity = ivel;
    properties.mInitialVelocities[0] = ivel[0];
  }

  readAllDegreesOfFreedom<dynamics::GenericJoint<math::R1Space>::Properties>(
      _jointElement, properties, _joint, _name, 1);

  return dynamics::RevoluteJoint::Properties::createShared(properties);
}

//==============================================================================
JointPropPtr readPrismaticJoint(
    tinyxml2::XMLElement* _jointElement,
    SkelJoint& _joint,
    std::string_view _name)
{
  DART_ASSERT(_jointElement != nullptr);

  dynamics::PrismaticJoint::Properties properties;

  //--------------------------------------------------------------------------
  // axis
  if (hasElement(_jointElement, "axis")) {
    tinyxml2::XMLElement* axisElement = getElement(_jointElement, "axis");

    // xyz
    Eigen::Vector3d xyz = getValueVector3d(axisElement, "xyz");
    properties.mAxis = xyz;
  } else {
    DART_ERROR(
        "[readPrismaticJoint] Prismatic Joint named [{}] is missing axis "
        "information!",
        _name);
    DART_ASSERT(0);
  }

  readJointDynamicsAndLimit<dynamics::GenericJoint<math::R1Space>::Properties>(
      _jointElement, properties, _joint, _name, 1);

  //--------------------------------------------------------------------------
  // init_pos
  if (hasElement(_jointElement, "init_pos")) {
    double init_pos = getValueDouble(_jointElement, "init_pos");
    Eigen::VectorXd ipos = Eigen::VectorXd(1);
    ipos << init_pos;
    _joint.position = ipos;
    properties.mInitialPositions[0] = ipos[0];
  }

  //--------------------------------------------------------------------------
  // init_vel
  if (hasElement(_jointElement, "init_vel")) {
    double init_vel = getValueDouble(_jointElement, "init_vel");
    Eigen::VectorXd ivel = Eigen::VectorXd(1);
    ivel << init_vel;
    _joint.velocity = ivel;
    properties.mInitialVelocities[0] = ivel[0];
  }

  readAllDegreesOfFreedom<dynamics::GenericJoint<math::R1Space>::Properties>(
      _jointElement, properties, _joint, _name, 1);

  return dynamics::PrismaticJoint::Properties::createShared(properties);
}

//==============================================================================
JointPropPtr readScrewJoint(
    tinyxml2::XMLElement* _jointElement,
    SkelJoint& _joint,
    std::string_view _name)
{
  DART_ASSERT(_jointElement != nullptr);

  dynamics::ScrewJoint::Properties properties;

  //--------------------------------------------------------------------------
  // axis
  if (hasElement(_jointElement, "axis")) {
    tinyxml2::XMLElement* axisElement = getElement(_jointElement, "axis");

    // xyz
    Eigen::Vector3d xyz = getValueVector3d(axisElement, "xyz");
    properties.mAxis = xyz;

    // pitch
    if (hasElement(axisElement, "pitch")) {
      double pitch = getValueDouble(axisElement, "pitch");
      properties.mPitch = pitch;
    }
  } else {
    DART_ERROR(
        "[readScrewJoint] Screw Joint named [{}] is missing axis information!",
        _name);
    DART_ASSERT(0);
  }

  readJointDynamicsAndLimit<dynamics::GenericJoint<math::R1Space>::Properties>(
      _jointElement, properties, _joint, _name, 1);

  //--------------------------------------------------------------------------
  // init_pos
  if (hasElement(_jointElement, "init_pos")) {
    double init_pos = getValueDouble(_jointElement, "init_pos");
    Eigen::VectorXd ipos = Eigen::VectorXd(1);
    ipos << init_pos;
    _joint.position = ipos;
    properties.mInitialPositions[0] = ipos[0];
  }

  //--------------------------------------------------------------------------
  // init_vel
  if (hasElement(_jointElement, "init_vel")) {
    double init_vel = getValueDouble(_jointElement, "init_vel");
    Eigen::VectorXd ivel = Eigen::VectorXd(1);
    ivel << init_vel;
    _joint.velocity = ivel;
    properties.mInitialVelocities[0] = ivel[0];
  }

  readAllDegreesOfFreedom<dynamics::GenericJoint<math::R1Space>::Properties>(
      _jointElement, properties, _joint, _name, 1);

  return dynamics::ScrewJoint::Properties::createShared(properties);
}

//==============================================================================
JointPropPtr readUniversalJoint(
    tinyxml2::XMLElement* _jointElement,
    SkelJoint& _joint,
    std::string_view _name)
{
  DART_ASSERT(_jointElement != nullptr);

  dynamics::UniversalJoint::Properties properties;

  //--------------------------------------------------------------------------
  // axis
  if (hasElement(_jointElement, "axis")) {
    tinyxml2::XMLElement* axisElement = getElement(_jointElement, "axis");

    // xyz
    Eigen::Vector3d xyz = getValueVector3d(axisElement, "xyz");
    properties.mAxis[0] = xyz;
  } else {
    DART_ERROR(
        "[readUniversalJoint] Universal Joint named [{}] is missing axis "
        "information!",
        _name);
    DART_ASSERT(0);
  }

  //--------------------------------------------------------------------------
  // axis2
  if (hasElement(_jointElement, "axis2")) {
    tinyxml2::XMLElement* axis2Element = getElement(_jointElement, "axis2");

    // xyz
    Eigen::Vector3d xyz = getValueVector3d(axis2Element, "xyz");
    properties.mAxis[1] = xyz;
  } else {
    DART_ERROR(
        "[readUniversalJoint] Universal Joint named [{}] is missing axis2 "
        "information!",
        _name);
    DART_ASSERT(0);
  }

  readJointDynamicsAndLimit(_jointElement, properties, _joint, _name, 2);

  //--------------------------------------------------------------------------
  // init_pos
  if (hasElement(_jointElement, "init_pos")) {
    Eigen::Vector2d init_pos = getValueVector2d(_jointElement, "init_pos");
    _joint.position = init_pos;
    properties.mInitialPositions = init_pos;
  }

  //--------------------------------------------------------------------------
  // init_vel
  if (hasElement(_jointElement, "init_vel")) {
    Eigen::Vector2d init_vel = getValueVector2d(_jointElement, "init_vel");
    _joint.velocity = init_vel;
    properties.mInitialVelocities = init_vel;
  }

  readAllDegreesOfFreedom(_jointElement, properties, _joint, _name, 2);

  return dynamics::UniversalJoint::Properties::createShared(properties);
}

//==============================================================================
JointPropPtr readBallJoint(
    tinyxml2::XMLElement* _jointElement,
    SkelJoint& _joint,
    std::string_view _name)
{
  DART_ASSERT(_jointElement != nullptr);

  dynamics::BallJoint::Properties properties;

  //--------------------------------------------------------------------------
  // init_pos
  if (hasElement(_jointElement, "init_pos")) {
    Eigen::Vector3d init_pos = getValueVector3d(_jointElement, "init_pos");
    _joint.position = init_pos;
    properties.mInitialPositions = init_pos;
  }

  //--------------------------------------------------------------------------
  // init_vel
  if (hasElement(_jointElement, "init_vel")) {
    Eigen::Vector3d init_vel = getValueVector3d(_jointElement, "init_vel");
    _joint.velocity = init_vel;
    properties.mInitialVelocities = init_vel;
  }

  readAllDegreesOfFreedom(_jointElement, properties, _joint, _name, 3);

  return dynamics::BallJoint::Properties::createShared(properties);
}

//==============================================================================
JointPropPtr readEulerJoint(
    tinyxml2::XMLElement* _jointElement,
    SkelJoint& _joint,
    std::string_view _name)
{
  DART_ASSERT(_jointElement != nullptr);

  dynamics::EulerJoint::Properties properties;

  //--------------------------------------------------------------------------
  // axis order
  std::string order = getValueString(_jointElement, "axis_order");
  if (order == "xyz") {
    properties.mAxisOrder = dynamics::EulerJoint::AxisOrder::XYZ;
  } else if (order == "zyx") {
    properties.mAxisOrder = dynamics::EulerJoint::AxisOrder::ZYX;
  } else {
    DART_ERROR(
        "[readEulerJoint] Undefined Euler axis order for Euler Joint named "
        "[{}]",
        _name);
    DART_ASSERT(0);
  }

  //--------------------------------------------------------------------------
  // axis
  readJointDynamicsAndLimit(_jointElement, properties, _joint, _name, 3);

  //--------------------------------------------------------------------------
  // init_pos
  if (hasElement(_jointElement, "init_pos")) {
    Eigen::Vector3d init_pos = getValueVector3d(_jointElement, "init_pos");
    _joint.position = init_pos;
    properties.mInitialPositions = init_pos;
  }

  //--------------------------------------------------------------------------
  // init_vel
  if (hasElement(_jointElement, "init_vel")) {
    Eigen::Vector3d init_vel = getValueVector3d(_jointElement, "init_vel");
    _joint.velocity = init_vel;
    properties.mInitialVelocities = init_vel;
  }

  readAllDegreesOfFreedom(_jointElement, properties, _joint, _name, 3);

  return dynamics::EulerJoint::Properties::createShared(properties);
}

//==============================================================================
JointPropPtr readTranslationalJoint(
    tinyxml2::XMLElement* _jointElement,
    SkelJoint& _joint,
    std::string_view _name)
{
  DART_ASSERT(_jointElement != nullptr);

  dynamics::TranslationalJoint::Properties properties;

  //--------------------------------------------------------------------------
  // axis
  readJointDynamicsAndLimit(_jointElement, properties, _joint, _name, 3);

  //--------------------------------------------------------------------------
  // init_pos
  if (hasElement(_jointElement, "init_pos")) {
    Eigen::Vector3d init_pos = getValueVector3d(_jointElement, "init_pos");
    _joint.position = init_pos;
    properties.mInitialPositions = init_pos;
  }

  //--------------------------------------------------------------------------
  // init_vel
  if (hasElement(_jointElement, "init_vel")) {
    Eigen::Vector3d init_vel = getValueVector3d(_jointElement, "init_vel");
    _joint.velocity = init_vel;
    properties.mInitialVelocities = init_vel;
  }

  readAllDegreesOfFreedom(_jointElement, properties, _joint, _name, 3);

  return dynamics::TranslationalJoint::Properties::createShared(properties);
}

//==============================================================================
JointPropPtr readTranslationalJoint2D(
    tinyxml2::XMLElement* _jointElement,
    SkelJoint& _joint,
    std::string_view _name)
{
  DART_ASSERT(_jointElement != nullptr);

  dynamics::TranslationalJoint2D::Properties properties;

  //--------------------------------------------------------------------------
  // Plane
  if (hasElement(_jointElement, "plane")) {
    tinyxml2::XMLElement* planeElement = getElement(_jointElement, "plane");

    // Type attribute
    std::string type = getAttributeString(planeElement, "type");

    if (type == "xy") {
      properties.setXYPlane();
    } else if (type == "yz") {
      properties.setYZPlane();
    } else if (type == "zx") {
      properties.setZXPlane();
    } else if (type == "arbitrary") {
      const auto* transAxis1Element
          = getElement(planeElement, "translation_axis1");

      const auto* transAxis2Element
          = getElement(planeElement, "translation_axis2");

      properties.setArbitraryPlane(
          getValueVector3d(transAxis1Element, "xyz"),
          getValueVector3d(transAxis2Element, "xyz"));
    } else {
      DART_ERROR(
          "[readTranslationalJoint2D] TranslationalJoint2D named [{}] contains "
          "unsupported plane type. Defaulting to XY-Plane.",
          _name);
      properties.setXYPlane();
    }
  } else {
    DART_WARN(
        "[readTranslationalJoint2D] TranslationalJoint2D named [{}] doesn't "
        "contain plane element. Defaulting to XY-Plane.",
        _name);
    properties.setXYPlane();
  }

  //--------------------------------------------------------------------------
  // axis
  readJointDynamicsAndLimit(_jointElement, properties, _joint, _name, 2);

  //--------------------------------------------------------------------------
  // init_pos
  if (hasElement(_jointElement, "init_pos")) {
    Eigen::Vector2d init_pos = getValueVector2d(_jointElement, "init_pos");
    _joint.position = init_pos;
    properties.mInitialPositions = init_pos;
  }

  //--------------------------------------------------------------------------
  // init_vel
  if (hasElement(_jointElement, "init_vel")) {
    Eigen::Vector2d init_vel = getValueVector2d(_jointElement, "init_vel");
    _joint.velocity = init_vel;
    properties.mInitialVelocities = init_vel;
  }

  readAllDegreesOfFreedom(_jointElement, properties, _joint, _name, 2);

  return dynamics::TranslationalJoint2D::Properties::createShared(properties);
}

//==============================================================================
JointPropPtr readPlanarJoint(
    tinyxml2::XMLElement* _jointElement,
    SkelJoint& _joint,
    std::string_view _name)
{
  DART_ASSERT(_jointElement != nullptr);

  dynamics::PlanarJoint::Properties properties;

  //--------------------------------------------------------------------------
  // Plane
  if (hasElement(_jointElement, "plane")) {
    tinyxml2::XMLElement* planeElement = getElement(_jointElement, "plane");

    // Type attribute
    std::string type = getAttributeString(planeElement, "type");

    if (type == "xy") {
      properties.mPlaneType = dynamics::PlanarJoint::PlaneType::XY;
    } else if (type == "yz") {
      properties.mPlaneType = dynamics::PlanarJoint::PlaneType::YZ;
    } else if (type == "zx") {
      properties.mPlaneType = dynamics::PlanarJoint::PlaneType::ZX;
    } else if (type == "arbitrary") {
      properties.mPlaneType = dynamics::PlanarJoint::PlaneType::ARBITRARY;

      tinyxml2::XMLElement* transAxis1Element
          = getElement(planeElement, "translation_axis1");

      properties.mTransAxis1 = getValueVector3d(transAxis1Element, "xyz");

      tinyxml2::XMLElement* transAxis2Element
          = getElement(planeElement, "translation_axis2");

      properties.mTransAxis2 = getValueVector3d(transAxis2Element, "xyz");
    } else {
      DART_ERROR(
          "[readPlanarJoint] Planar Joint named [{}] is missing plane type "
          "information. Defaulting to XY-Plane.",
          _name);
      properties.mPlaneType = dynamics::PlanarJoint::PlaneType::XY;
    }
  } else {
    DART_WARN(
        "[readPlanarJoint] Planar Joint named [{}] is missing plane type "
        "information. Defaulting to XY-Plane.",
        _name);
    properties.mPlaneType = dynamics::PlanarJoint::PlaneType::XY;
  }

  //--------------------------------------------------------------------------
  // axis
  readJointDynamicsAndLimit(_jointElement, properties, _joint, _name, 3);

  //--------------------------------------------------------------------------
  // init_pos
  if (hasElement(_jointElement, "init_pos")) {
    Eigen::Vector3d init_pos = getValueVector3d(_jointElement, "init_pos");
    _joint.position = init_pos;
    properties.mInitialPositions = init_pos;
  }

  //--------------------------------------------------------------------------
  // init_vel
  if (hasElement(_jointElement, "init_vel")) {
    Eigen::Vector3d init_vel = getValueVector3d(_jointElement, "init_vel");
    _joint.velocity = init_vel;
    properties.mInitialVelocities = init_vel;
  }

  readAllDegreesOfFreedom(_jointElement, properties, _joint, _name, 3);

  return dynamics::PlanarJoint::Properties::createShared(properties);
}

//==============================================================================
JointPropPtr readFreeJoint(
    tinyxml2::XMLElement* _jointElement,
    SkelJoint& _joint,
    std::string_view _name)
{
  DART_ASSERT(_jointElement != nullptr);

  dynamics::FreeJoint::Properties properties;

  //--------------------------------------------------------------------------
  // init_pos
  if (hasElement(_jointElement, "init_pos")) {
    Eigen::Vector6d init_pos = getValueVector6d(_jointElement, "init_pos");
    _joint.position = init_pos;
    properties.mInitialPositions = init_pos;
  }

  //--------------------------------------------------------------------------
  // init_vel
  if (hasElement(_jointElement, "init_vel")) {
    Eigen::Vector6d init_vel = getValueVector6d(_jointElement, "init_vel");
    _joint.velocity = init_vel;
    properties.mInitialVelocities = init_vel;
  }

  readAllDegreesOfFreedom(_jointElement, properties, _joint, _name, 6);

  return dynamics::FreeJoint::Properties::createShared(properties);
}

//==============================================================================
common::ResourceRetrieverPtr getRetriever(
    const common::ResourceRetrieverPtr& _retriever)
{
  if (_retriever) {
    return _retriever;
  } else {
    auto newRetriever = std::make_shared<utils::CompositeResourceRetriever>();
    newRetriever->addSchemaRetriever(
        "file", std::make_shared<common::LocalResourceRetriever>());
    newRetriever->addSchemaRetriever("dart", DartResourceRetriever::create());
    return newRetriever;
  }
}

} // anonymous namespace

} // namespace utils
} // namespace dart
