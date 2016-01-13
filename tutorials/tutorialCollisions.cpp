/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <mxgrey@gatech.edu>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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

#include <random>
#include "dart/dart.h"

const double default_shape_density = 1000; // kg/m^3
const double default_shape_height  = 0.1;  // m
const double default_shape_width   = 0.03; // m
const double default_skin_thickness = 1e-3; // m

const double default_start_height = 0.4;  // m

const double minimum_start_v = 2.5; // m/s
const double maximum_start_v = 4.0; // m/s
const double default_start_v = 3.5; // m/s

const double minimum_launch_angle = 30.0*M_PI/180.0; // rad
const double maximum_launch_angle = 70.0*M_PI/180.0; // rad
const double default_launch_angle = 45.0*M_PI/180.0; // rad

const double maximum_start_w = 6*M_PI; // rad/s
const double default_start_w = 3*M_PI;  // rad/s

const double ring_spring_stiffness = 0.5;
const double ring_damping_coefficient = 0.05;
const double default_damping_coefficient = 0.001;

const double default_ground_width = 2;
const double default_wall_thickness = 0.1;
const double default_wall_height = 1;
const double default_spawn_range = 0.9*default_ground_width/2;

const double default_restitution = 0.6;

const double default_vertex_stiffness = 1000.0;
const double default_edge_stiffness = 1.0;
const double default_soft_damping = 5.0;

using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::gui;

void setupRing(const SkeletonPtr& ring)
{
  // Set the spring and damping coefficients for the degrees of freedom
  // Lesson 4a
  for (size_t i =6; i<ring->getNumDofs(); ++i)
  {
    DegreeOfFreedom* dof = ring->getDof(i);
    dof->setSpringStiffness(ring_spring_stiffness);
    dof->setDampingCoefficient(ring_damping_coefficient);
  }

  // Compute the joint angle needed to form a ring
  // Lesson 4b
  size_t numEdges = ring->getNumBodyNodes();
  double angle = 2*M_PI/numEdges;

  // Set the BallJoints so that they have the correct rest position angle
  // Lesson 4b
  for (size_t i=1; i<ring->getNumJoints();++i)
  {
    Joint* joint = ring->getJoint(i);
    Eigen::AngleAxisd rotation(angle, Eigen::Vector3d(0,1,0));
    Eigen::Vector3d restPos = BallJoint::convertToPositions(
            Eigen::Matrix3d(rotation));
    for (size_t j=0;j<3;++j)
    {
      joint->setRestPosition(j, restPos[j]);
    }

  }

  // Set the Joints to be in their rest positions
  // Lesson 4c
  for(size_t i = 6;i<ring->getNumDofs();++i)
  {
    DegreeOfFreedom* dof = ring->getDof(i);
    dof->setPosition(dof->getRestPosition());
  }

}

class MyWindow : public SimWindow
{
public:

  MyWindow(const WorldPtr& world, const SkeletonPtr& ball,
           const SkeletonPtr& softBody, const SkeletonPtr& hybridBody,
           const SkeletonPtr& rigidChain, const SkeletonPtr& rigidRing)
    : mRandomize(true),
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
    setWorld(world);
  }

  void keyboard(unsigned char key, int x, int y) override
  {
    switch(key)
    {
      case '1':
        addObject(mOriginalBall->clone());
        break;

      case '2':
        addObject(mOriginalSoftBody->clone());
        break;

      case '3':
        addObject(mOriginalHybridBody->clone());
        break;

      case '4':
        addObject(mOriginalRigidChain->clone());
        break;

      case '5':
        addRing(mOriginalRigidRing->clone());
        break;

      case 'd':

      // because the ground and wall are the first and second skeleton
        if(mWorld->getNumSkeletons() > 2)
          removeSkeleton(mWorld->getSkeleton(2));
        std::cout << "Remaining objects: " << mWorld->getNumSkeletons()-2
                  << std::endl;
        break;

      case 'r':
        mRandomize = !mRandomize;
        std::cout << "Randomization: " << (mRandomize? "on" : "off")
                  << std::endl;
        break;

      default:
        SimWindow::keyboard(key, x, y);
    }
  }

  void drawSkels() override
  {
    // Make sure lighting is turned on and that polygons get filled in
    glEnable(GL_LIGHTING);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    SimWindow::drawSkels();
  }

  void displayTimer(int _val) override
  {
    // We remove playback and baking, because we want to be able to add and
    // remove objects during runtime
    int numIter = mDisplayTimeout / (mWorld->getTimeStep() * 1000);
    if (mSimulating)
    {
      for (int i = 0; i < numIter; i++)
        timeStepping();
    }
    glutPostRedisplay();
    glutTimerFunc(mDisplayTimeout, refreshTimer, _val);
  }

protected:

  /// Add an object to the world and toss it at the wall
  bool addObject(const SkeletonPtr& object)
  {
    // Set the starting position for the object
    // Lesson 3a
    Eigen::Vector6d positions(Eigen::Vector6d::Zero());

    if (mRandomize)
    {
      positions[4] = default_spawn_range * mDistribution(mMT);
    }

    positions[5] = default_start_height;
    object->getJoint(0)->setPositions(positions);


    // Add the object to the world
    // Lesson 3b
    object->setName(object->getName() + std::to_string(mSkelCount++));
    mWorld->addSkeleton(object);

    // Compute collisions
    // Lesson 3c
    dart::collision::CollisionDetector* detector =
        mWorld->getConstraintSolver()->getCollisionDetector();
    detector->detectCollision(true, true);

    bool collision = false;
    size_t collisionCount = detector->getNumContacts();
    for(size_t i=0; i<collisionCount;i++)
    {
      const dart::collision::Contact& contact = detector->getContact(i);
      if(contact.bodyNode1.lock()->getSkeleton() == object 
          || contact.bodyNode2.lock()->getSkeleton() == object)
      {
        collision = true;
        break;
      }
    }

    if (collision)
    {
      mWorld->removeSkeleton(object);
      std::cout<<"The new object spawned in a collision. " <<"It will not be added to the world. "<<std::endl;
      return false;
    }


    // Refuse to add the object if it is in collision
    // Lesson 3c

    // Create reference frames for setting the initial velocity
    // Lesson 3d
    Eigen::Isometry3d centerTf(Eigen::Isometry3d::Identity());
    centerTf.translation() = object->getCOM();
    SimpleFrame center(Frame::World(), "center", centerTf);



    // Set the velocities of the reference frames so that we can easily give the
    // Skeleton the linear and angular velocities that we want
    // Lesson 3e
    double angle = default_launch_angle;
    double speed = default_start_v;
    double angular_speed = default_start_w;
    if (mRandomize)
    {
      angle = (mDistribution(mMT) + 1.0)/2.0 *
            (maximum_launch_angle - minimum_launch_angle) + minimum_launch_angle;

      speed = (mDistribution(mMT) + 1.0)/2.0 *
            (maximum_start_v - minimum_start_v) + minimum_start_v;

      angular_speed  = mDistribution(mMT) * maximum_start_w;
    }
    Eigen::Vector3d v = speed * Eigen::Vector3d(cos(angle),0.0,sin(angle));
    Eigen::Vector3d w = angular_speed * Eigen::Vector3d::UnitY();

    center.setClassicDerivatives(v,w);



    // Use the reference frames to set the velocity of the Skeleton's root
    // Lesson 3f
    SimpleFrame ref(&center,"root_reference");
    ref.setRelativeTransform(object->getBodyNode(0)->getTransform(&center));

    object->getJoint(0)->setVelocities(ref.getSpatialVelocity());

    return true;
  }

  /// Add a ring to the world, and create a BallJoint constraint to ensure that
  /// it stays in a ring shape
  void addRing(const SkeletonPtr& ring)
  {
    setupRing(ring);

    if(!addObject(ring))
      return;

    // Create a closed loop to turn the chain into a ring
    // Lesson 5
    BodyNode* head = ring->getBodyNode(0);
    BodyNode* tail = ring->getBodyNode(ring->getNumBodyNodes()-1);

    Eigen::Vector3d offset = Eigen::Vector3d(0,0, default_shape_height / 2.0);
    offset = tail->getWorldTransform() * offset;

    auto constraint = new dart::constraint::BallJointConstraint(head, tail, offset);
    mWorld->getConstraintSolver()->addConstraint(constraint);
    mJointConstraints.push_back(constraint);
  }

  /// Remove a Skeleton and get rid of the constraint that was associated with
  /// it, if one existed
  void removeSkeleton(const SkeletonPtr& skel)
  {
    for(size_t i=0; i<mJointConstraints.size(); ++i)
    {
      dart::constraint::JointConstraint* constraint = mJointConstraints[i];
      if(constraint->getBodyNode1()->getSkeleton() == skel
         || constraint->getBodyNode2()->getSkeleton() == skel)
      {
        mWorld->getConstraintSolver()->removeConstraint(constraint);
        mJointConstraints.erase(mJointConstraints.begin()+i);
        delete constraint;
        break; // There should only be one constraint per skeleton
      }
    }

    mWorld->removeSkeleton(skel);
  }

  /// Flag to keep track of whether or not we are randomizing the tosses
  bool mRandomize;

  // std library objects that allow us to generate high-quality random numbers
  std::random_device mRD;
  std::mt19937 mMT;
  std::uniform_real_distribution<double> mDistribution;

  /// History of the active JointConstraints so that we can properly delete them
  /// when a Skeleton gets removed
  std::vector<dart::constraint::JointConstraint*> mJointConstraints;

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
  size_t mSkelCount;

};

/// Add a rigid body with the specified Joint type to a chain
template<class JointType>
BodyNode* addRigidBody(const SkeletonPtr& chain, const std::string& name,
                       Shape::ShapeType type, BodyNode* parent = nullptr)
{
  // Set the Joint properties
  // Lesson 1a
  // typename keywords is for consideration of syntax of template
  typename JointType::Properties properties;
  properties.mName = name + "_joint";

  if (parent)
  {
    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    tf.translation() = Eigen::Vector3d(0,0,default_shape_height / 2.0);
    properties.mT_ParentBodyToJoint = tf;
    properties.mT_ChildBodyToJoint = tf.inverse();

  }

  // Create the Joint and Body pair
  // Lesson 1b
  //BodyNode* bn = nullptr;
  BodyNode* bn = chain->createJointAndBodyNodePair<JointType>(
        parent, properties, BodyNode::Properties(name)).second;

  // Make the shape based on the requested Shape type
  // Lesson 1c
  ShapePtr shape;
  if(Shape::BOX == type)
  {
    shape = std::make_shared<BoxShape>(Eigen::Vector3d(
                                          default_shape_width,
                                          default_shape_width,
                                          default_shape_height));
  }
  else if (Shape::CYLINDER == type)
  {
    shape = std::make_shared<CylinderShape>(default_shape_width/2.0,
                                          default_shape_height);
  }
  else if (Shape::ELLIPSOID == type)
  {
    shape = std::make_shared<EllipsoidShape>(default_shape_height*Eigen::Vector3d::Ones());
  }
  bn->addVisualizationShape(shape);
  bn->addCollisionShape(shape);

  // Setup the inertia for the body
  // Lesson 1d
  Inertia inertia;
  double mass = default_shape_density * shape->getVolume();
  inertia.setMass(mass);
  inertia.setMoment(shape->computeInertia(mass));
  bn->setInertia(inertia);

  // Set the coefficient of restitution to make the body more bouncy
  // Lesson 1e
  bn->setRestitutionCoeff(default_restitution);

  // Set damping to make the simulation more stable
  // Lesson 1f
  if (parent)
  {
    Joint* joint = bn->getParentJoint();
    for (size_t i=0;i<joint->getNumDofs();++i)
    {
      joint->getDof(i)->setDampingCoefficient(default_damping_coefficient);
    }
  }

  return bn;
}

enum SoftShapeType {
  SOFT_BOX = 0,
  SOFT_CYLINDER,
  SOFT_ELLIPSOID
};

/// Add a soft body with the specified Joint type to a chain
template<class JointType>
BodyNode* addSoftBody(const SkeletonPtr& chain, const std::string& name,
                      SoftShapeType type, BodyNode* parent = nullptr)
{
  // Set the Joint properties
  // Lesson 2a
  typename JointType::Properties properties;
  properties.mName = name + "_joint";

  if(parent)
  {
    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    tf.translation() = Eigen::Vector3d(0,0,default_shape_height/ 2.0);
    properties.mT_ParentBodyToJoint = tf;
    properties.mT_ChildBodyToJoint = tf.inverse();
  }

  // Set the properties of the soft body
  // Lesson 2b
  SoftBodyNode::UniqueProperties soft_properties;
  if (SOFT_BOX == type)
  {
    double width = default_shape_height, height = 2 * default_shape_width;
    Eigen::Vector3d dims(width, width, height);
    double mass = 2*dims[0]*dims[1] + 2*dims[0]*dims[2] + 2*dims[1]*dims[2];
    // mass above is the area of skin, mass above is the true mass
    mass *= default_shape_density * default_skin_thickness;
    soft_properties = SoftBodyNodeHelper::makeBoxProperties(
          dims, Eigen::Isometry3d::Identity(), Eigen::Vector3i(4,4,4),mass);
  }
  else if (SOFT_CYLINDER == type)
  {
    double radius = default_shape_height/2.0, height = 2*default_shape_width;
    double mass =  default_shape_density * height * 2 * M_PI * radius * default_skin_thickness;
    mass += 2*default_shape_density * M_PI * pow(radius,2)*default_skin_thickness;
    soft_properties = SoftBodyNodeHelper::makeCylinderProperties(
          radius, height, 8,3,2, mass);
  }
  else if (SOFT_ELLIPSOID == type)
  {
    double radius = default_shape_height /2.0;
    Eigen::Vector3d dims = 2*radius*Eigen::Vector3d::Ones();
    double mass = default_shape_density * 4.0 * M_PI*pow(radius,2)*default_skin_thickness;
    soft_properties = SoftBodyNodeHelper::makeEllipsoidProperties(
          dims,6,6,mass);
  }
  soft_properties.mKv = default_vertex_stiffness;
  soft_properties.mKe = default_edge_stiffness;
  soft_properties.mDampCoeff = default_soft_damping;

  // Create the Joint and Body pair
  // Lesson 2c
  //SoftBodyNode* bn = nullptr;
  SoftBodyNode::Properties body_properties(BodyNode::Properties(name), soft_properties);
  SoftBodyNode* bn = chain->createJointAndBodyNodePair<JointType, SoftBodyNode>(
        parent, properties, body_properties).second;

  // Zero out the inertia for the underlying BodyNode
  // Lesson 2d
  Inertia inertia;
  inertia.setMoment(1e-8*Eigen::Matrix3d::Identity());
  inertia.setMass(1e-8);
  bn->setInertia(inertia);


  // Make the shape transparent
  // Lesson 2e
  Eigen::Vector4d color = bn->getVisualizationShape(0)->getRGBA();
  color[3]=0.4;
  bn->getVisualizationShape(0)->setRGBA(color);


  return bn;
}

void setAllColors(const SkeletonPtr& object, const Eigen::Vector3d& color)
{
  // Set the color of all the shapes in the object
  for(size_t i=0; i < object->getNumBodyNodes(); ++i)
  {
    BodyNode* bn = object->getBodyNode(i);
    for(size_t j=0; j < bn->getNumVisualizationShapes(); ++j)
      bn->getVisualizationShape(j)->setColor(color);
  }
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

  // Add a rigid collision geometry and inertia
  // Lesson 2f
  double width = default_shape_height, height = 2*default_shape_width;
  Eigen::Vector3d dims(width,width,height);
  dims *= 0.6;
  std::shared_ptr<BoxShape> box = std::make_shared<BoxShape>(dims);

  bn->addCollisionShape(box);
  bn->addVisualizationShape(box);

  Inertia inertia;
  inertia.setMass(default_shape_density*box->getVolume());
  inertia.setMoment(box->computeInertia(inertia.getMass()));
  bn->setInertia(inertia);


  setAllColors(soft, dart::Color::Fuschia());

  return soft;
}

SkeletonPtr createHybridBody()
{
  SkeletonPtr hybrid = Skeleton::create("hybrid");

  // Add a soft body
  BodyNode* bn = addSoftBody<FreeJoint>(hybrid, "soft sphere", SOFT_ELLIPSOID);

  // Add a rigid body attached by a WeldJoint
  // Lesson 2g
  bn = hybrid->createJointAndBodyNodePair<WeldJoint>(bn).second;
  bn->setName("rigid box");

  double box_shape_height = default_shape_height;
  std::shared_ptr<BoxShape> box = std::make_shared<BoxShape>(
      box_shape_height * Eigen::Vector3d::Ones());

  bn->addCollisionShape(box);
  bn->addVisualizationShape(box);

  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = Eigen::Vector3d(box_shape_height/2.0, 0, 0);
  bn->getParentJoint()->setTransformFromParentBodyNode(tf);
  
  Inertia inertia;
  inertia.setMass(default_shape_density*box->getVolume());
  inertia.setMoment(box->computeInertia(inertia.getMass()));
  bn->setInertia(inertia);

  setAllColors(hybrid, dart::Color::Green());

  return hybrid;
}

SkeletonPtr createGround()
{
  SkeletonPtr ground = Skeleton::create("ground");

  BodyNode* bn = ground->createJointAndBodyNodePair<WeldJoint>().second;

  std::shared_ptr<BoxShape> shape = std::make_shared<BoxShape>(
        Eigen::Vector3d(default_ground_width, default_ground_width,
                        default_wall_thickness));
  shape->setColor(Eigen::Vector3d(1.0, 1.0, 1.0));

  bn->addCollisionShape(shape);
  bn->addVisualizationShape(shape);

  return ground;
}

SkeletonPtr createWall()
{
  SkeletonPtr wall = Skeleton::create("wall");

  BodyNode* bn = wall->createJointAndBodyNodePair<WeldJoint>().second;

  std::shared_ptr<BoxShape> shape = std::make_shared<BoxShape>(
        Eigen::Vector3d(default_wall_thickness, default_ground_width,
                        default_wall_height));
  shape->setColor(Eigen::Vector3d(0.8, 0.8, 0.8));

  bn->addCollisionShape(shape);
  bn->addVisualizationShape(shape);

  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = Eigen::Vector3d(
        (default_ground_width + default_wall_thickness)/2.0, 0.0,
        (default_wall_height  - default_wall_thickness)/2.0);
  bn->getParentJoint()->setTransformFromParentBodyNode(tf);

  bn->setRestitutionCoeff(0.2);

  return wall;
}

int main(int argc, char* argv[])
{
  WorldPtr world = std::make_shared<World>();
  world->addSkeleton(createGround());
  world->addSkeleton(createWall());

  MyWindow window(world, createBall(), createSoftBody(), createHybridBody(),
                  createRigidChain(), createRigidRing());

  std::cout << "space bar: simulation on/off" << std::endl;
  std::cout << "'1': toss a rigid ball" << std::endl;
  std::cout << "'2': toss a soft body" << std::endl;
  std::cout << "'3': toss a hybrid soft/rigid body" << std::endl;
  std::cout << "'4': toss a rigid chain" << std::endl;
  std::cout << "'5': toss a ring of rigid bodies" << std::endl;

  std::cout << "\n'd': delete the oldest object" << std::endl;
  std::cout <<   "'r': toggle randomness" << std::endl;

  std::cout << "\nWarning: Let objects settle before tossing a new one, or the simulation could explode." << std::endl;
  std::cout <<   "         If the simulation freezes, you may need to force quit the application.\n" << std::endl;

  glutInit(&argc, argv);
  window.initWindow(640, 480, "Collisions");
  glutMainLoop();
}
