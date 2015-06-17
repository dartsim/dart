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

#include "dart/dart.h"

const double default_domino_height = 0.2;
const double default_domino_width = 0.4*default_domino_height;
const double default_domino_depth = default_domino_width/5.0;

const double default_distance = default_domino_height/2.0;
const double default_angle = 20.0*M_PI/180.0;

const double default_push_force = 8.0; // N
const int default_push_duration = 200; // # iterations

using namespace dart::dynamics;
using namespace dart::simulation;

class Controller
{
public:

  Controller(const SkeletonPtr& manipulator, const SkeletonPtr& domino)
    : _manipulator(manipulator)
  {
    // Create a transform from the center of the domino to the top of the domino
    Eigen::Isometry3d target_offset(Eigen::Isometry3d::Identity());
    target_offset.translation() =
        default_domino_height/2.0 * Eigen::Vector3d::UnitZ();

    // Place the _target SimpleFrame at the top of the domino
    _target = std::make_shared<SimpleFrame>(
          domino->getBodyNode(0), "target", target_offset);
  }

  void computeForces()
  {
    if(nullptr == _manipulator)
      return;
  }

protected:

  SkeletonPtr _manipulator;

  SimpleFramePtr _target;

};

class MyWindow : public dart::gui::SimWindow
{
public:

  MyWindow(const WorldPtr& world)
    : _totalAngle(0.0),
      _hasEverRun(false),
      _pushCountDown(0)
  {
    setWorld(world);
    _firstDomino = world->getSkeleton("domino");
    _floor = world->getSkeleton("floor");
  }

  // Attempt to create a new domino. If the new domino would be in collision
  // with anything (other than the floor), then discard it.
  void attemptToCreateDomino(double angle)
  {
    const SkeletonPtr& lastDomino = _dominoes.size()>0?
          _dominoes.back() : _firstDomino;

    // Compute the position for the new domino
    Eigen::Vector3d dx = default_distance*Eigen::Vector3d(
          cos(_totalAngle), sin(_totalAngle), 0.0);

    Eigen::Vector6d x = lastDomino->getPositions();
    x.tail<3>() += dx;

    // Adjust the angle for the new domino
    x[2] = _totalAngle + angle;

    SkeletonPtr newDomino = _firstDomino->clone();
    newDomino->setName("domino #" + std::to_string(_dominoes.size()+1));
    newDomino->setPositions(x);

    mWorld->addSkeleton(newDomino);

    // Compute collisions
    dart::collision::CollisionDetector* detector =
        mWorld->getConstraintSolver()->getCollisionDetector();
    detector->detectCollision(true, true);

    // Look through the collisions to see if any dominoes are penetrating each
    // other
    bool dominoCollision = false;
    size_t collisionCount = detector->getNumContacts();
    for(size_t i=0; i < collisionCount; ++i)
    {
      // If either of the colliding BodyNodes belongs to the floor, then we know
      // that we have colliding dominoes
      const dart::collision::Contact& contact = detector->getContact(i);
      if(   contact.bodyNode1.lock()->getSkeleton() != _floor
         && contact.bodyNode2.lock()->getSkeleton() != _floor)
      {
        dominoCollision = true;
        break;
      }
    }

    if(dominoCollision)
    {
      // Remove the new domino, because it is penetrating an existing one
      mWorld->removeSkeleton(newDomino);
    }
    else
    {
      // Record the latest domino addition
      _angles.push_back(angle);
      _dominoes.push_back(newDomino);
      _totalAngle += angle;
    }
  }

  // Delete the last domino that was added to the scene. (Do not delete the
  // original domino)
  void deleteLastDomino()
  {
    if(_dominoes.size() > 0)
    {
      SkeletonPtr lastDomino = _dominoes.back();
      _dominoes.pop_back();
      mWorld->removeSkeleton(lastDomino);

      _totalAngle -= _angles.back();
      _angles.pop_back();
    }
  }

  void keyboard(unsigned char key, int x, int y) override
  {
    if(!_hasEverRun)
    {
      switch(key)
      {
        case 'q':
          attemptToCreateDomino( default_angle);
          break;
        case 'w':
          attemptToCreateDomino(0.0);
          break;
        case 'e':
          attemptToCreateDomino(-default_angle);
          break;
        case 'd':
          deleteLastDomino();
          break;
        case ' ':
          _hasEverRun = true;
          break;
      }
    }
    else
    {
      switch(key)
      {
        case 'f':
          _pushCountDown = default_push_duration;
          break;
      }
    }

    SimWindow::keyboard(key, x, y);
  }

  void timeStepping() override
  {
    // If the user has pressed the 'f' key, apply a force to the first domino in
    // order to push it over
    if(_pushCountDown > 0)
    {
      _firstDomino->getBodyNode(0)->addExtForce(
            default_push_force*Eigen::Vector3d::UnitX(),
            default_domino_height/2.0*Eigen::Vector3d::UnitZ());

      --_pushCountDown;
    }

    SimWindow::timeStepping();
  }

protected:

  /// Base domino. Used to clone new dominoes.
  SkeletonPtr _firstDomino;

  /// Floor of the scene
  SkeletonPtr _floor;

  /// History of the dominoes that have been created
  std::vector<SkeletonPtr> _dominoes;

  /// History of the angles that the user has specified
  std::vector<double> _angles;

  /// Sum of all angles so far
  double _totalAngle;

  /// Set to true the first time spacebar is pressed
  bool _hasEverRun;

  /// The first domino will be pushed on while the value of this is positive
  int _pushCountDown;

};

SkeletonPtr createDomino()
{
  /// Create a Skeleton with the name "domino"
  SkeletonPtr domino = Skeleton::create("domino");

  /// Create a body for the domino
  BodyNodePtr body =
      domino->createJointAndBodyNodePair<FreeJoint>(nullptr).second;

  std::shared_ptr<BoxShape> box(
        new BoxShape(Eigen::Vector3d(default_domino_depth,
                                     default_domino_width,
                                     default_domino_height)));
  body->addVisualizationShape(box);
  body->addCollisionShape(box);

  domino->getDof("Joint_pos_z")->setPosition(default_domino_height/2.0);

  return domino;
}

SkeletonPtr createFloor()
{
  SkeletonPtr floor = Skeleton::create("floor");

  // Give the floor a body
  BodyNodePtr body =
      floor->createJointAndBodyNodePair<WeldJoint>(nullptr).second;

  // Give the body a shape
  double floor_width = 10.0;
  double floor_height = 0.01;
  std::shared_ptr<BoxShape> box(
        new BoxShape(Eigen::Vector3d(floor_width, floor_width, floor_height)));
  box->setColor(dart::Color::Black());

  body->addVisualizationShape(box);
  body->addCollisionShape(box);

  // Put the body into position
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = Eigen::Vector3d(0.0, 0.0, -floor_height/2.0);
  body->getParentJoint()->setTransformFromParentBodyNode(tf);

  return floor;
}

SkeletonPtr createManipulator()
{
  // Load the Skeleton from a file
  dart::utils::DartLoader loader;
  SkeletonPtr manipulator =
      loader.parseSkeleton(DART_DATA_PATH"urdf/KR5/KR5 sixx R650.urdf");

  // Position its base in a reasonable way
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = Eigen::Vector3d(-0.75, 0.0, 0.0);
  manipulator->getJoint(0)->setTransformFromParentBodyNode(tf);

  return manipulator;
}

int main(int argc, char* argv[])
{
  SkeletonPtr domino = createDomino();
  SkeletonPtr floor = createFloor();
  SkeletonPtr manipulator = createManipulator();

  WorldPtr world(new World);
  world->addSkeleton(domino);
  world->addSkeleton(floor);
  world->addSkeleton(manipulator);

  MyWindow window(world);

  std::cout << "Before simulation has started, you can create new dominoes:" << std::endl;
  std::cout << "'w': Create new domino angled forward" << std::endl;
  std::cout << "'q': Create new domino angled to the left" << std::endl;
  std::cout << "'e': Create new domino angled to the right" << std::endl;
  std::cout << "'d': Delete the last domino that was created" << std::endl;
  std::cout << std::endl;
  std::cout << "spacebar: Begin simulation (you can no longer create or remove dominoes)" << std::endl;
  std::cout << "'f': Push the first domino so that it falls over" << std::endl;
  std::cout << "'v': Turn contact force visualization on/off" << std::endl;

  glutInit(&argc, argv);
  window.initWindow(640, 480, "Dominoes");
  glutMainLoop();
}
