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

#include <dart/gui/all.hpp>

#include <dart/all.hpp>

#include <random>

const double default_shape_density = 1000;  // kg/m^3
const double default_shape_height = 0.1;    // m
const double default_shape_width = 0.03;    // m
const double default_skin_thickness = 1e-3; // m

const double default_start_height = 0.4; // m

const double minimum_start_v = 2.5; // m/s
const double maximum_start_v = 4.0; // m/s
const double default_start_v = 3.5; // m/s

const double minimum_launch_angle = dart::math::toRadian(30.0); // rad
const double maximum_launch_angle = dart::math::toRadian(70.0); // rad
const double default_launch_angle = dart::math::toRadian(45.0); // rad

const double maximum_start_w = 6 * dart::math::pi; // rad/s
const double default_start_w = 3 * dart::math::pi; // rad/s

const double ring_spring_stiffness = 0.5;
const double ring_damping_coefficient = 0.05;
const double default_damping_coefficient = 0.001;

const double default_ground_width = 2;
const double default_wall_thickness = 0.1;
const double default_wall_height = 1;
const double default_spawn_range = 0.9 * default_ground_width / 2;

const double default_restitution = 0.6;

const double default_vertex_stiffness = 1000.0;
const double default_edge_stiffness = 1.0;
const double default_soft_damping = 5.0;

using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::gui;
using namespace dart::gui;

void setupRing(const SkeletonPtr& ring)
{
  // snippet:cpp-collisions-lesson4a-ring-stiffness-start
  // Set the spring and damping coefficients for the degrees of freedom
  for (std::size_t i = 6; i < ring->getNumDofs(); ++i) {
    DegreeOfFreedom* dof = ring->getDof(i);
    dof->setSpringStiffness(ring_spring_stiffness);
    dof->setDampingCoefficient(ring_damping_coefficient);
  }
  // snippet:cpp-collisions-lesson4a-ring-stiffness-end

  // snippet:cpp-collisions-lesson4b-ring-rest-start
  // Compute the joint angle needed to form a ring
  std::size_t numEdges = ring->getNumBodyNodes();
  double angle = 2 * dart::math::pi / numEdges;

  // Set the BallJoints so that they have the correct rest position angle
  for (std::size_t i = 1; i < ring->getNumJoints(); ++i) {
    Joint* joint = ring->getJoint(i);
    Eigen::AngleAxisd rotation(angle, Eigen::Vector3d(0, 1, 0));
    Eigen::Vector3d restPos
        = BallJoint::convertToPositions(Eigen::Matrix3d(rotation));

    for (std::size_t j = 0; j < 3; ++j) {
      joint->setRestPosition(j, restPos[j]);
    }
  }
  // snippet:cpp-collisions-lesson4b-ring-rest-end

  // snippet:cpp-collisions-lesson4c-ring-rest-state-start
  // Set the Joints to be in their rest positions
  for (std::size_t i = 6; i < ring->getNumDofs(); ++i) {
    DegreeOfFreedom* dof = ring->getDof(i);
    dof->setPosition(dof->getRestPosition());
  }
  // snippet:cpp-collisions-lesson4c-ring-rest-state-end
}

class CollisionsEventHandler : public ::osgGA::GUIEventHandler
{
public:
  CollisionsEventHandler(
      const WorldPtr& world,
      const SkeletonPtr& ball,
      const SkeletonPtr& softBody,
      const SkeletonPtr& hybridBody,
      const SkeletonPtr& rigidChain,
      const SkeletonPtr& rigidRing)
    : mWorld(world),
      mRandomize(true),
      mRD(),
      mMT(mRD()),
      mDistribution(-1.0, std::nextafter(1.0, 2.0)),
      mOriginalBall(ball),
      mOriginalSoftBody(softBody),
      mOriginalHybridBody(hybridBody),
      mOriginalRigidChain(rigidChain),
      mOriginalRigidRing(rigidRing),
      mSkelCount(0)
  {
  }

  bool handle(
      const ::osgGA::GUIEventAdapter& ea, ::osgGA::GUIActionAdapter&) override
  {
    if (ea.getEventType() == ::osgGA::GUIEventAdapter::KEYDOWN) {
      switch (ea.getKey()) {
        case '1':
          addObject(mOriginalBall->cloneSkeleton());
          return true;
        case '2':
          addObject(mOriginalSoftBody->cloneSkeleton());
          return true;
        case '3':
          addObject(mOriginalHybridBody->cloneSkeleton());
          return true;
        case '4':
          addObject(mOriginalRigidChain->cloneSkeleton());
          return true;
        case '5':
          addRing(mOriginalRigidRing->cloneSkeleton());
          return true;
        case 'd':
          if (mWorld->getNumSkeletons() > 2) {
            removeSkeleton(mWorld->getSkeleton(2));
          }
          std::cout << "Remaining objects: " << mWorld->getNumSkeletons() - 2
                    << std::endl;
          return true;
        case 'r':
          mRandomize = !mRandomize;
          std::cout << "Randomization: " << (mRandomize ? "on" : "off")
                    << std::endl;
          return true;
        default:
          return false;
      }
    }
    return false;
  }

protected:
  /// Add an object to the world and toss it at the wall
  bool addObject(const SkeletonPtr& object)
  {
    // snippet:cpp-collisions-lesson3a-initial-position-start
    // Set the starting position for the object
    Eigen::Vector6d positions(Eigen::Vector6d::Zero());

    // If randomization is on, we will randomize the starting y-location
    if (mRandomize) {
      positions[4] = default_spawn_range * mDistribution(mMT);
    }

    positions[5] = default_start_height;
    object->getJoint(0)->setPositions(positions);
    // snippet:cpp-collisions-lesson3a-initial-position-end

    // snippet:cpp-collisions-lesson3b-name-start
    // Add the object to the world
    object->setName(object->getName() + std::to_string(mSkelCount++));
    // snippet:cpp-collisions-lesson3b-name-end

    // snippet:cpp-collisions-lesson3c-collision-check-start
    // Look through the collisions to see if the new object would start in
    // collision with something
    auto collisionEngine = mWorld->getCollisionDetector();
    auto collisionGroup = mWorld->getConstraintSolver()->getCollisionGroup();
    auto newGroup = collisionEngine->createCollisionGroup(object.get());

    dart::collision::CollisionOption option;
    dart::collision::CollisionResult result;
    bool collision = collisionGroup->collide(newGroup.get(), option, &result);

    // If the new object is not in collision
    if (!collision) {
      mWorld->addSkeleton(object);
    } else {
      // or refuse to add the object if it is in collision
      std::cout << "The new object spawned in a collision. "
                << "It will not be added to the world." << std::endl;
      return false;
    }
    // snippet:cpp-collisions-lesson3c-collision-check-end

    // snippet:cpp-collisions-lesson3d-reference-frame-start
    // Create reference frames for setting the initial velocity
    Eigen::Isometry3d centerTf(Eigen::Isometry3d::Identity());
    centerTf.translation() = object->getCOM();
    SimpleFrame center(Frame::World(), "center", centerTf);
    // snippet:cpp-collisions-lesson3d-reference-frame-end

    // snippet:cpp-collisions-lesson3e-launch-velocity-start
    // Set the velocities of the reference frames so that we can easily give the
    // Skeleton the linear and angular velocities that we want
    double angle = default_launch_angle;
    double speed = default_start_v;
    double angular_speed = default_start_w;
    if (mRandomize) {
      angle = (mDistribution(mMT) + 1.0) / 2.0
                  * (maximum_launch_angle - minimum_launch_angle)
              + minimum_launch_angle;

      speed = (mDistribution(mMT) + 1.0) / 2.0
                  * (maximum_start_v - minimum_start_v)
              + minimum_start_v;

      angular_speed = mDistribution(mMT) * maximum_start_w;
    }

    Eigen::Vector3d v = speed * Eigen::Vector3d(cos(angle), 0.0, sin(angle));
    Eigen::Vector3d w = angular_speed * Eigen::Vector3d::UnitY();
    center.setClassicDerivatives(v, w);
    // snippet:cpp-collisions-lesson3e-launch-velocity-end

    // snippet:cpp-collisions-lesson3f-apply-velocity-start
    SimpleFrame ref(&center, "root_reference");
    ref.setRelativeTransform(object->getBodyNode(0)->getTransform(&center));

    // Use the reference frames to set the velocity of the Skeleton's root
    object->getJoint(0)->setVelocities(ref.getSpatialVelocity());
    // snippet:cpp-collisions-lesson3f-apply-velocity-end

    return true;
  }

  /// Add a ring to the world, and create a BallJoint constraint to ensure that
  /// it stays in a ring shape
  void addRing(const SkeletonPtr& ring)
  {
    setupRing(ring);

    if (!addObject(ring)) {
      return;
    }

    // snippet:cpp-collisions-lesson5-closed-chain-start
    // Create a closed loop to turn the chain into a ring
    BodyNode* head = ring->getBodyNode(0);
    BodyNode* tail = ring->getBodyNode(ring->getNumBodyNodes() - 1);

    // Compute the offset where the JointConstraint should be located
    Eigen::Vector3d offset = Eigen::Vector3d(0, 0, default_shape_height / 2.0);
    offset = tail->getWorldTransform() * offset;
    auto constraint = std::make_shared<dart::constraint::BallJointConstraint>(
        head, tail, offset);

    mWorld->getConstraintSolver()->addConstraint(constraint);
    mJointConstraints.push_back(constraint);
    // snippet:cpp-collisions-lesson5-closed-chain-end
  }

  /// Remove a Skeleton and get rid of the constraint that was associated with
  /// it, if one existed
  void removeSkeleton(const SkeletonPtr& skel)
  {
    for (std::size_t i = 0; i < mJointConstraints.size(); ++i) {
      const dart::constraint::DynamicJointConstraintPtr& constraint
          = mJointConstraints[i];

      if (constraint->getBodyNode1()->getSkeleton() == skel
          || constraint->getBodyNode2()->getSkeleton() == skel) {
        mWorld->getConstraintSolver()->removeConstraint(constraint);
        mJointConstraints.erase(mJointConstraints.begin() + i);
        break; // There should only be one constraint per skeleton
      }
    }

    mWorld->removeSkeleton(skel);
  }

  WorldPtr mWorld;

  /// Flag to keep track of whether or not we are randomizing the tosses
  bool mRandomize;

  // std library objects that allow us to generate high-quality random numbers
  std::random_device mRD;
  std::mt19937 mMT;
  std::uniform_real_distribution<double> mDistribution;

  /// History of the active JointConstraints so that we can properly delete them
  /// when a Skeleton gets removed
  std::vector<dart::constraint::DynamicJointConstraintPtr> mJointConstraints;

  /// A blueprint Skeleton that we will use to spawn balls
  SkeletonPtr mOriginalBall;

  /// A blueprint Skeleton that we will use to spawn soft bodies
  SkeletonPtr mOriginalSoftBody;

  /// A blueprint Skeleton that we will use to spawn hybrid bodies
  SkeletonPtr mOriginalHybridBody;

  /// A blueprint Skeleton that we will use to spawn rigid chains
  SkeletonPtr mOriginalRigidChain;

  /// A blueprint Skeleton that we will use to spawn rigid rings
  SkeletonPtr mOriginalRigidRing;

  /// Keep track of how many Skeletons we spawn to ensure we can give them all
  /// unique names
  std::size_t mSkelCount;
};

class CustomWorldNode : public RealTimeWorldNode
{
public:
  CustomWorldNode(const WorldPtr& world, CollisionsEventHandler* /*handler*/)
    : RealTimeWorldNode(world)
  {
  }
};

/// Add a rigid body with the specified Joint type to a chain
template <class JointType>
BodyNode* addRigidBody(
    const SkeletonPtr& chain,
    const std::string& name,
    Shape::ShapeType type,
    BodyNode* parent = nullptr)
{
  // snippet:cpp-collisions-lesson1a-properties-start
  // Set the Joint properties
  typename JointType::Properties properties;
  properties.mName = name + "_joint";
  if (parent) {
    // If the body has a parent, we should position the joint to be in the
    // middle of the centers of the two bodies
    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    tf.translation() = Eigen::Vector3d(0, 0, default_shape_height / 2.0);
    properties.mT_ParentBodyToJoint = tf;
    properties.mT_ChildBodyToJoint = tf.inverse();
  }
  // snippet:cpp-collisions-lesson1a-properties-end

  // snippet:cpp-collisions-lesson1b-joint-pair-start
  // Create the Joint and Body pair
  BodyNode* bn = chain
                     ->createJointAndBodyNodePair<JointType>(
                         parent, properties, BodyNode::AspectProperties(name))
                     .second;
  // snippet:cpp-collisions-lesson1b-joint-pair-end

  // snippet:cpp-collisions-lesson1c-shape-selection-start
  // Make the shape based on the requested Shape type
  ShapePtr shape;
  if (Shape::BOX == type) {
    shape = std::make_shared<BoxShape>(Eigen::Vector3d(
        default_shape_width, default_shape_width, default_shape_height));
  } else if (Shape::CYLINDER == type) {
    shape = std::make_shared<CylinderShape>(
        default_shape_width / 2.0, default_shape_height);
  } else if (Shape::ELLIPSOID == type) {
    shape = std::make_shared<EllipsoidShape>(
        default_shape_height * Eigen::Vector3d::Ones());
  }
  // snippet:cpp-collisions-lesson1c-shape-selection-end

  // snippet:cpp-collisions-lesson1c-shape-node-start
  auto shapeNode
      = bn->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(
          shape);
  // snippet:cpp-collisions-lesson1c-shape-node-end

  // snippet:cpp-collisions-lesson1d-inertia-start
  // Setup the inertia for the body
  Inertia inertia;
  double mass = default_shape_density * shape->getVolume();
  inertia.setMass(mass);
  inertia.setMoment(shape->computeInertia(mass));
  bn->setInertia(inertia);
  // snippet:cpp-collisions-lesson1d-inertia-end

  // snippet:cpp-collisions-lesson1e-restitution-start
  // Set the coefficient of restitution to make the body more bouncy
  shapeNode->getDynamicsAspect()->setRestitutionCoeff(default_restitution);
  // snippet:cpp-collisions-lesson1e-restitution-end

  // snippet:cpp-collisions-lesson1f-damping-start
  // Set damping to make the simulation more stable
  if (parent) {
    Joint* joint = bn->getParentJoint();
    for (std::size_t i = 0; i < joint->getNumDofs(); ++i) {
      joint->getDof(i)->setDampingCoefficient(default_damping_coefficient);
    }
  }
  // snippet:cpp-collisions-lesson1f-damping-end

  return bn;
}

enum SoftShapeType
{
  SOFT_BOX = 0,
  SOFT_CYLINDER,
  SOFT_ELLIPSOID
};

/// Add a soft body with the specified Joint type to a chain
template <class JointType>
BodyNode* addSoftBody(
    const SkeletonPtr& chain,
    const std::string& name,
    SoftShapeType type,
    BodyNode* parent = nullptr)
{
  // Set the Joint properties
  typename JointType::Properties joint_properties;
  joint_properties.mName = name + "_joint";
  if (parent) {
    // If the body has a parent, we should position the joint to be in the
    // middle of the centers of the two bodies
    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    tf.translation() = Eigen::Vector3d(0, 0, default_shape_height / 2.0);
    joint_properties.mT_ParentBodyToJoint = tf;
    joint_properties.mT_ChildBodyToJoint = tf.inverse();
  }

  // snippet:cpp-collisions-lesson2b-soft-properties-start
  // Set the properties of the soft body
  SoftBodyNode::UniqueProperties soft_properties;
  // Use the SoftBodyNodeHelper class to create the geometries for the
  // SoftBodyNode
  if (SOFT_BOX == type) {
    // Make a wide and short box
    double width = default_shape_height, height = 2 * default_shape_width;
    Eigen::Vector3d dims(width, width, height);

    double mass
        = 2 * dims[0] * dims[1] + 2 * dims[0] * dims[2] + 2 * dims[1] * dims[2];
    mass *= default_shape_density * default_skin_thickness;
    soft_properties = SoftBodyNodeHelper::makeBoxProperties(
        dims, Eigen::Isometry3d::Identity(), Eigen::Vector3i(4, 4, 4), mass);
  } else if (SOFT_CYLINDER == type) {
    // Make a wide and short cylinder
    double radius = default_shape_height / 2.0,
           height = 2 * default_shape_width;

    // Mass of center
    double mass = default_shape_density * height * 2 * dart::math::pi * radius
                  * default_skin_thickness;
    // Mass of top and bottom
    mass += 2 * default_shape_density * dart::math::pi * pow(radius, 2)
            * default_skin_thickness;
    soft_properties = SoftBodyNodeHelper::makeCylinderProperties(
        radius, height, 8, 3, 2, mass);
  } else if (SOFT_ELLIPSOID == type) {
    double radius = default_shape_height / 2.0;
    Eigen::Vector3d dims = 2 * radius * Eigen::Vector3d::Ones();
    double mass = default_shape_density * 4.0 * dart::math::pi * pow(radius, 2)
                  * default_skin_thickness;
    soft_properties
        = SoftBodyNodeHelper::makeEllipsoidProperties(dims, 6, 6, mass);
  }
  soft_properties.mKv = default_vertex_stiffness;
  soft_properties.mKe = default_edge_stiffness;
  soft_properties.mDampCoeff = default_soft_damping;
  // snippet:cpp-collisions-lesson2b-soft-properties-end

  // snippet:cpp-collisions-lesson2c-soft-node-start
  // Create the Joint and Body pair
  SoftBodyNode::Properties body_properties(
      BodyNode::AspectProperties(name), soft_properties);
  SoftBodyNode* bn = chain
                         ->createJointAndBodyNodePair<JointType, SoftBodyNode>(
                             parent, joint_properties, body_properties)
                         .second;
  // snippet:cpp-collisions-lesson2c-soft-node-end

  // snippet:cpp-collisions-lesson2d-soft-inertia-start
  // Zero out the inertia for the underlying BodyNode
  Inertia inertia;
  inertia.setMoment(1e-8 * Eigen::Matrix3d::Identity());
  inertia.setMass(1e-8);
  bn->setInertia(inertia);
  // snippet:cpp-collisions-lesson2d-soft-inertia-end

  // snippet:cpp-collisions-lesson2e-soft-alpha-start
  // Make the shape transparent
  bn->setAlpha(0.4);
  // snippet:cpp-collisions-lesson2e-soft-alpha-end

  return bn;
}

void setAllColors(const SkeletonPtr& object, const Eigen::Vector3d& color)
{
  // Set the color of all the shapes in the object
  object->setColor(color);
}

SkeletonPtr createBall()
{
  SkeletonPtr ball = Skeleton::create("rigid_ball");

  // Give the ball a body
  addRigidBody<FreeJoint>(ball, "rigid ball", Shape::ELLIPSOID);

  setAllColors(ball, dart::Color::Red());

  return ball;
}

SkeletonPtr createRigidChain()
{
  SkeletonPtr chain = Skeleton::create("rigid_chain");

  // Add bodies to the chain
  BodyNode* bn = addRigidBody<FreeJoint>(chain, "rigid box 1", Shape::BOX);
  bn = addRigidBody<BallJoint>(chain, "rigid cyl 2", Shape::CYLINDER, bn);
  bn = addRigidBody<BallJoint>(chain, "rigid box 3", Shape::BOX, bn);

  setAllColors(chain, dart::Color::Orange());

  return chain;
}

SkeletonPtr createRigidRing()
{
  SkeletonPtr ring = Skeleton::create("rigid_ring");

  // Add bodies to the ring
  BodyNode* bn = addRigidBody<FreeJoint>(ring, "rigid box 1", Shape::BOX);
  bn = addRigidBody<BallJoint>(ring, "rigid cyl 2", Shape::CYLINDER, bn);
  bn = addRigidBody<BallJoint>(ring, "rigid box 3", Shape::BOX, bn);
  bn = addRigidBody<BallJoint>(ring, "rigid cyl 4", Shape::CYLINDER, bn);
  bn = addRigidBody<BallJoint>(ring, "rigid box 5", Shape::BOX, bn);
  bn = addRigidBody<BallJoint>(ring, "rigid cyl 6", Shape::CYLINDER, bn);

  setAllColors(ring, dart::Color::Blue());

  return ring;
}

SkeletonPtr createSoftBody()
{
  SkeletonPtr soft = Skeleton::create("soft");

  // Add a soft body
  BodyNode* bn = addSoftBody<FreeJoint>(soft, "soft box", SOFT_BOX);

  // snippet:cpp-collisions-lesson2f-rigid-core-start
  // Add a rigid collision geometry and inertia
  double width = default_shape_height, height = 2 * default_shape_width;
  Eigen::Vector3d dims(width, width, height);
  dims *= 0.6;
  std::shared_ptr<BoxShape> box = std::make_shared<BoxShape>(dims);
  bn->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(box);

  Inertia inertia;
  inertia.setMass(default_shape_density * box->getVolume());
  inertia.setMoment(box->computeInertia(inertia.getMass()));
  bn->setInertia(inertia);
  // snippet:cpp-collisions-lesson2f-rigid-core-end

  setAllColors(soft, dart::Color::Fuchsia());

  return soft;
}

SkeletonPtr createHybridBody()
{
  SkeletonPtr hybrid = Skeleton::create("hybrid");

  // Add a soft body
  BodyNode* bn = addSoftBody<FreeJoint>(hybrid, "soft sphere", SOFT_ELLIPSOID);

  // snippet:cpp-collisions-lesson2g-welded-rigid-start
  // Add a rigid body attached by a WeldJoint
  bn = hybrid->createJointAndBodyNodePair<WeldJoint>(bn).second;
  bn->setName("rigid box");

  double box_shape_height = default_shape_height;
  std::shared_ptr<BoxShape> box
      = std::make_shared<BoxShape>(box_shape_height * Eigen::Vector3d::Ones());
  bn->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(box);

  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = Eigen::Vector3d(box_shape_height / 2.0, 0, 0);
  bn->getParentJoint()->setTransformFromParentBodyNode(tf);

  Inertia inertia;
  inertia.setMass(default_shape_density * box->getVolume());
  inertia.setMoment(box->computeInertia(inertia.getMass()));
  bn->setInertia(inertia);
  // snippet:cpp-collisions-lesson2g-welded-rigid-end

  setAllColors(hybrid, dart::Color::Green());

  return hybrid;
}

SkeletonPtr createGround()
{
  SkeletonPtr ground = Skeleton::create("ground");

  BodyNode* bn = ground->createJointAndBodyNodePair<WeldJoint>().second;

  std::shared_ptr<BoxShape> shape = std::make_shared<BoxShape>(Eigen::Vector3d(
      default_ground_width, default_ground_width, default_wall_thickness));
  auto shapeNode
      = bn->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(
          shape);
  shapeNode->getVisualAspect()->setColor(Eigen::Vector3d(1.0, 1.0, 1.0));

  return ground;
}

SkeletonPtr createWall()
{
  SkeletonPtr wall = Skeleton::create("wall");

  BodyNode* bn = wall->createJointAndBodyNodePair<WeldJoint>().second;

  std::shared_ptr<BoxShape> shape = std::make_shared<BoxShape>(Eigen::Vector3d(
      default_wall_thickness, default_ground_width, default_wall_height));
  auto shapeNode
      = bn->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(
          shape);
  shapeNode->getVisualAspect()->setColor(Eigen::Vector3d(0.8, 0.8, 0.8));

  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = Eigen::Vector3d(
      (default_ground_width + default_wall_thickness) / 2.0,
      0.0,
      (default_wall_height - default_wall_thickness) / 2.0);
  bn->getParentJoint()->setTransformFromParentBodyNode(tf);

  shapeNode->getDynamicsAspect()->setRestitutionCoeff(0.2);

  return wall;
}

int main()
{
  WorldPtr world = std::make_shared<World>();
  world->addSkeleton(createGround());
  world->addSkeleton(createWall());

  // Create event handler
  auto handler = new CollisionsEventHandler(
      world,
      createBall(),
      createSoftBody(),
      createHybridBody(),
      createRigidChain(),
      createRigidRing());

  // Create a WorldNode and wrap it around the world
  ::osg::ref_ptr<CustomWorldNode> node = new CustomWorldNode(world, handler);

  // Create a Viewer and set it up with the WorldNode
  auto viewer = Viewer();
  viewer.addWorldNode(node);
  viewer.addEventHandler(handler);

  // Print instructions
  viewer.addInstructionText("space bar: simulation on/off\n");
  viewer.addInstructionText("'1': toss a rigid ball\n");
  viewer.addInstructionText("'2': toss a soft body\n");
  viewer.addInstructionText("'3': toss a hybrid soft/rigid body\n");
  viewer.addInstructionText("'4': toss a rigid chain\n");
  viewer.addInstructionText("'5': toss a ring of rigid bodies\n");
  viewer.addInstructionText("'d': delete the oldest object\n");
  viewer.addInstructionText("'r': toggle randomness\n");
  viewer.addInstructionText(
      "\nWarning: Let objects settle before tossing a new one, or the "
      "simulation could explode.\n");
  viewer.addInstructionText(
      "         If the simulation freezes, you may need to force quit the "
      "application.\n");
  std::cout << viewer.getInstructions() << std::endl;

  // Set up the window to be 640x480
  viewer.setUpViewInWindow(0, 0, 640, 480);

  // Adjust the viewpoint of the Viewer
  viewer.getCameraManipulator()->setHomePosition(
      ::osg::Vec3(3.0f, 2.0f, 2.0f),
      ::osg::Vec3(0.0f, 0.0f, 0.0f),
      ::osg::Vec3(0.0f, 0.0f, 1.0f));

  // We need to re-dirty the CameraManipulator by passing it into the viewer
  // again, so that the viewer knows to update its HomePosition setting
  viewer.setCameraManipulator(viewer.getCameraManipulator());

  // Begin running the application loop
  viewer.run();

  return 0;
}
