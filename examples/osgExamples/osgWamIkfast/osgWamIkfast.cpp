/*
 * Copyright (c) 2015-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2015-2017, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016-2017, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/utils.hpp>
#include <dart/utils/urdf/urdf.hpp>

using namespace dart::dynamics;
using namespace dart::simulation;

class RelaxedPosture : public dart::optimizer::Function
{
public:

  RelaxedPosture(const Eigen::VectorXd& idealPosture,
                 const Eigen::VectorXd& lower, const Eigen::VectorXd& upper,
                 const Eigen::VectorXd& weights, bool enforceIdeal = false)
    : enforceIdealPosture(enforceIdeal),
      mIdeal(idealPosture),
      mLower(lower),
      mUpper(upper),
      mWeights(weights)
  {
    int dofs = mIdeal.size();
    if(mLower.size() != dofs || mWeights.size() != dofs || mUpper.size() != dofs)
    {
      dterr << "[RelaxedPose::RelaxedPose] Dimension mismatch:\n"
            << "  ideal:   " << mIdeal.size()   << "\n"
            << "  lower:   " << mLower.size()   << "\n"
            << "  upper:   " << mUpper.size()   << "\n"
            << "  weights: " << mWeights.size() << "\n";
    }
    mResultVector.setZero(dofs);
  }

  double eval(const Eigen::VectorXd& _x) override
  {
    computeResultVector(_x);
    return 0.5 * mResultVector.dot(mResultVector);
  }

  void evalGradient(const Eigen::VectorXd& _x,
                    Eigen::Map<Eigen::VectorXd> _grad) override
  {
    computeResultVector(_x);

    _grad.setZero();
    int smaller = std::min(mResultVector.size(), _grad.size());
    for(int i=0; i < smaller; ++i)
      _grad[i] = mResultVector[i];
  }

  void computeResultVector(const Eigen::VectorXd& _x)
  {
    mResultVector.setZero();

    if(enforceIdealPosture)
    {
      for(int i=0; i < _x.size(); ++i)
      {
        if(mIdeal.size() <= i)
          break;

        mResultVector[i] = mWeights[i]*(_x[i] - mIdeal[i]);
      }
    }
    else
    {
      for(int i=0; i < _x.size(); ++i)
      {
        if(mIdeal.size() <= i)
          break;

        if(_x[i] < mLower[i])
          mResultVector[i] = mWeights[i]*(_x[i] - mLower[i]);
        else if(mUpper[i] < _x[i])
          mResultVector[i] = mWeights[i]*(_x[i] - mUpper[i]);
      }
    }
  }

  bool enforceIdealPosture;

protected:

  Eigen::VectorXd mResultVector;

  Eigen::VectorXd mIdeal;

  Eigen::VectorXd mLower;

  Eigen::VectorXd mUpper;

  Eigen::VectorXd mWeights;
};

static inline bool checkDist(Eigen::Vector3d& p, double a, double b)
{
  double d = p.norm();
  double dmax = a+b;
  double dmin = fabs(a-b);

  if (d > dmax)
  {
    p *= dmax/d;
    return false;
  }
  else if (d < dmin)
  {
    p *= dmin/d;
    return false;
  }
  else
  {
    return true;
  }
}

static inline void clamp_sincos(double& sincos, bool& valid)
{
  if (sincos < -1)
  {
    valid = false;
    sincos = -1;
  }
  else if (sincos > 1)
  {
    valid = false;
    sincos = 1;
  }
}

static inline Eigen::Vector3d flipEuler3Axis(const Eigen::Vector3d& u)
{
  Eigen::Vector3d v;
  v[0] = u[0] - M_PI;
  v[1] = M_PI - u[1];
  v[2] = u[2] - M_PI;
  return v;
}

class TeleoperationWorld : public dart::gui::osg::WorldNode
{
public:

  enum MoveEnum_t
  {
    MOVE_Q = 0,
    MOVE_W,
    MOVE_E,
    MOVE_A,
    MOVE_S,
    MOVE_D,
    MOVE_F,
    MOVE_Z,

    NUM_MOVE
  };

  TeleoperationWorld(WorldPtr _world, SkeletonPtr _robot)
    : dart::gui::osg::WorldNode(_world),
      mHubo(_robot),
      iter(0),
      l_foot(_robot->getEndEffector("l_foot")),
      r_foot(_robot->getEndEffector("r_foot")),
      l_hand(_robot->getEndEffector("l_hand")),
      r_hand(_robot->getEndEffector("r_hand"))
  {
    mMoveComponents.resize(NUM_MOVE, false);
    mAnyMovement = false;
    mAmplifyMovement = false;
  }

  void setMovement(const std::vector<bool>& moveComponents)
  {
    mMoveComponents = moveComponents;

    mAnyMovement = false;

    for(bool move : mMoveComponents)
    {
      if(move)
      {
        mAnyMovement = true;
        break;
      }
    }
  }

  void customPreRefresh() override
  {
    if(mAnyMovement)
    {
      Eigen::Isometry3d old_tf = mHubo->getBodyNode(0)->getWorldTransform();
      Eigen::Isometry3d new_tf = Eigen::Isometry3d::Identity();
      Eigen::Vector3d forward = old_tf.linear().col(0);
      forward[2] = 0.0;
      if(forward.norm() > 1e-10)
        forward.normalize();
      else
        forward.setZero();

      Eigen::Vector3d left = old_tf.linear().col(1);
      left[2] = 0.0;
      if(left.norm() > 1e-10)
        left.normalize();
      else
        left.setZero();

      const Eigen::Vector3d& up = Eigen::Vector3d::UnitZ();

      double linearStep = 0.01;
      double elevationStep = 0.2*linearStep;
      double rotationalStep = 2.0*M_PI/180.0;

      if(mAmplifyMovement)
      {
        linearStep *= 2.0;
        elevationStep *= 2.0;
        rotationalStep *= 2.0;
      }

      if(mMoveComponents[MOVE_W])
        new_tf.translate( linearStep*forward);

      if(mMoveComponents[MOVE_S])
        new_tf.translate(-linearStep*forward);

      if(mMoveComponents[MOVE_A])
        new_tf.translate( linearStep*left);

      if(mMoveComponents[MOVE_D])
        new_tf.translate(-linearStep*left);

      if(mMoveComponents[MOVE_F])
        new_tf.translate( elevationStep*up);

      if(mMoveComponents[MOVE_Z])
        new_tf.translate(-elevationStep*up);

      if(mMoveComponents[MOVE_Q])
        new_tf.rotate(Eigen::AngleAxisd( rotationalStep, up));

      if(mMoveComponents[MOVE_E])
        new_tf.rotate(Eigen::AngleAxisd(-rotationalStep, up));

      new_tf.pretranslate(old_tf.translation());
      new_tf.rotate(old_tf.rotation());

      mHubo->getJoint(0)->setPositions(FreeJoint::convertToPositions(new_tf));
    }

    mHubo->getIK(true)->solve();
  }

  bool mAmplifyMovement;

protected:

  SkeletonPtr mHubo;
  std::size_t iter;

  EndEffectorPtr l_foot;
  EndEffectorPtr r_foot;

  EndEffectorPtr l_hand;
  EndEffectorPtr r_hand;

  std::vector<IK::Analytical::Solution> mSolutions;

  Eigen::VectorXd grad;

  // Order: q, w, e, a, s, d
  std::vector<bool> mMoveComponents;

  bool mAnyMovement;
};

class InputHandler : public ::osgGA::GUIEventHandler
{
public:

  InputHandler(dart::gui::osg::Viewer* viewer, TeleoperationWorld* teleop,
               const SkeletonPtr& hubo, const WorldPtr& world)
    : mViewer(viewer),
      mTeleop(teleop),
      mHubo(hubo),
      mWorld(world)
  {
    initialize();
  }

  void initialize()
  {
    mRestConfig = mHubo->getPositions();

    for(std::size_t i=0; i < mHubo->getNumEndEffectors(); ++i)
    {
      const InverseKinematicsPtr ik = mHubo->getEndEffector(i)->getIK();
      if(ik)
      {
        mDefaultBounds.push_back(ik->getErrorMethod().getBounds());
        mDefaultTargetTf.push_back(ik->getTarget()->getRelativeTransform());
        mConstraintActive.push_back(false);
        mEndEffectorIndex.push_back(i);
      }
    }

    mPosture = std::dynamic_pointer_cast<RelaxedPosture>(
          mHubo->getIK(true)->getObjective());

    mBalance = std::dynamic_pointer_cast<dart::constraint::BalanceConstraint>(
          mHubo->getIK(true)->getProblem()->getEqConstraint(1));

    mOptimizationKey = 'r';

    mMoveComponents.resize(TeleoperationWorld::NUM_MOVE, false);
  }

  virtual bool handle(const ::osgGA::GUIEventAdapter& ea,
                      ::osgGA::GUIActionAdapter&) override
  {
    if(nullptr == mHubo)
    {
      return false;
    }

    if( ::osgGA::GUIEventAdapter::KEYDOWN == ea.getEventType() )
    {
      if( ea.getKey() == 'p' )
      {
        for(std::size_t i=0; i < mHubo->getNumDofs(); ++i)
          std::cout << mHubo->getDof(i)->getName() << ": "
                    << mHubo->getDof(i)->getPosition() << std::endl;
        return true;
      }

      if( ea.getKey() == 't' )
      {
        // Reset all the positions except for x, y, and yaw
        for(std::size_t i=0; i < mHubo->getNumDofs(); ++i)
        {
          if( i < 2 || 4 < i )
            mHubo->getDof(i)->setPosition(mRestConfig[i]);
        }
        return true;
      }

      if( '1' <= ea.getKey() && ea.getKey() <= '9' )
      {
        std::size_t index = ea.getKey() - '1';
        if(index < mConstraintActive.size())
        {
          EndEffector* ee = mHubo->getEndEffector(mEndEffectorIndex[index]);
          const InverseKinematicsPtr& ik = ee->getIK();
          if(ik && mConstraintActive[index])
          {
            mConstraintActive[index] = false;

            ik->getErrorMethod().setBounds(mDefaultBounds[index]);
            ik->getTarget()->setRelativeTransform(mDefaultTargetTf[index]);
            mWorld->removeSimpleFrame(ik->getTarget());
          }
          else if(ik)
          {
            mConstraintActive[index] = true;

            // Use the standard default bounds instead of our custom default
            // bounds
            ik->getErrorMethod().setBounds();
            ik->getTarget()->setTransform(ee->getTransform());
            mWorld->addSimpleFrame(ik->getTarget());
          }
        }
        return true;
      }

      if( 'x' == ea.getKey() )
      {
        EndEffector* ee = mHubo->getEndEffector("l_foot");
        ee->getSupport()->setActive(!ee->getSupport()->isActive());
        return true;
      }

      if( 'c' == ea.getKey() )
      {
        EndEffector* ee = mHubo->getEndEffector("r_foot");
        ee->getSupport()->setActive(!ee->getSupport()->isActive());
        return true;
      }

      if(ea.getKey() == ::osgGA::GUIEventAdapter::KEY_Shift_L)
        mTeleop->mAmplifyMovement = true;

      switch(ea.getKey())
      {
        case 'w': case 'W': mMoveComponents[TeleoperationWorld::MOVE_W] = true; break;
        case 'a': case 'A': mMoveComponents[TeleoperationWorld::MOVE_A] = true; break;
        case 's': case 'S': mMoveComponents[TeleoperationWorld::MOVE_S] = true; break;
        case 'd': case 'D': mMoveComponents[TeleoperationWorld::MOVE_D] = true; break;
        case 'q': case 'Q': mMoveComponents[TeleoperationWorld::MOVE_Q] = true; break;
        case 'e': case 'E': mMoveComponents[TeleoperationWorld::MOVE_E] = true; break;
        case 'f': case 'F': mMoveComponents[TeleoperationWorld::MOVE_F] = true; break;
        case 'z': case 'Z': mMoveComponents[TeleoperationWorld::MOVE_Z] = true; break;
      }

      switch(ea.getKey())
      {
        case 'w': case 'a': case 's': case 'd': case 'q': case 'e': case 'f': case 'z':
        case 'W': case 'A': case 'S': case 'D': case 'Q': case 'E': case 'F': case 'Z':
        {
          mTeleop->setMovement(mMoveComponents);
          return true;
        }
      }

      if(mOptimizationKey == ea.getKey())
      {
        if(mPosture)
          mPosture->enforceIdealPosture = true;

        if(mBalance)
          mBalance->setErrorMethod(dart::constraint::BalanceConstraint::OPTIMIZE_BALANCE);

        return true;
      }
    }

    if( ::osgGA::GUIEventAdapter::KEYUP == ea.getEventType() )
    {
      if(ea.getKey() == mOptimizationKey)
      {
        if(mPosture)
          mPosture->enforceIdealPosture = false;

        if(mBalance)
          mBalance->setErrorMethod(dart::constraint::BalanceConstraint::FROM_CENTROID);

        return true;
      }

      if(ea.getKey() == ::osgGA::GUIEventAdapter::KEY_Shift_L)
        mTeleop->mAmplifyMovement = false;

      switch(ea.getKey())
      {
        case 'w': case 'W': mMoveComponents[TeleoperationWorld::MOVE_W] = false; break;
        case 'a': case 'A': mMoveComponents[TeleoperationWorld::MOVE_A] = false; break;
        case 's': case 'S': mMoveComponents[TeleoperationWorld::MOVE_S] = false; break;
        case 'd': case 'D': mMoveComponents[TeleoperationWorld::MOVE_D] = false; break;
        case 'q': case 'Q': mMoveComponents[TeleoperationWorld::MOVE_Q] = false; break;
        case 'e': case 'E': mMoveComponents[TeleoperationWorld::MOVE_E] = false; break;
        case 'f': case 'F': mMoveComponents[TeleoperationWorld::MOVE_F] = false; break;
        case 'z': case 'Z': mMoveComponents[TeleoperationWorld::MOVE_Z] = false; break;
      }

      switch(ea.getKey())
      {
        case 'w': case 'a': case 's': case 'd': case 'q': case'e': case 'f': case 'z':
        case 'W': case 'A': case 'S': case 'D': case 'Q': case 'E': case 'F': case 'Z':
        {
          mTeleop->setMovement(mMoveComponents);
          return true;
        }
      }
    }

    return false;
  }

protected:

  dart::gui::osg::Viewer* mViewer;

  TeleoperationWorld* mTeleop;

  SkeletonPtr mHubo;

  WorldPtr mWorld;

  Eigen::VectorXd mRestConfig;

  std::vector<bool> mConstraintActive;

  std::vector<std::size_t> mEndEffectorIndex;

  std::vector< std::pair<Eigen::Vector6d, Eigen::Vector6d> > mDefaultBounds;

  Eigen::aligned_vector<Eigen::Isometry3d> mDefaultTargetTf;

  std::shared_ptr<RelaxedPosture> mPosture;

  std::shared_ptr<dart::constraint::BalanceConstraint> mBalance;

  char mOptimizationKey;

  std::vector<bool> mMoveComponents;
};

//==============================================================================
SkeletonPtr createGround()
{
  // Create a Skeleton to represent the ground
  SkeletonPtr ground = Skeleton::create("ground");
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  double thickness = 0.01;
  tf.translation() = Eigen::Vector3d(0,0,-thickness/2.0);
  WeldJoint::Properties joint;
  joint.mT_ParentBodyToJoint = tf;
  ground->createJointAndBodyNodePair<WeldJoint>(nullptr, joint);
  ShapePtr groundShape =
      std::make_shared<BoxShape>(Eigen::Vector3d(10,10,thickness));

  auto shapeNode = ground->getBodyNode(0)->createShapeNodeWith<
      VisualAspect, CollisionAspect, DynamicsAspect>(groundShape);
  shapeNode->getVisualAspect()->setColor(dart::Color::Blue(0.2));

  return ground;
}

//==============================================================================
SkeletonPtr createWam()
{
  dart::utils::DartLoader urdfParser;
  urdfParser.addPackageDirectory("herb_description", DART_DATA_PATH"/urdf/wam");
  SkeletonPtr wam = urdfParser.parseSkeleton(DART_DATA_PATH"/urdf/wam/wam.urdf");

  return wam;
}

//==============================================================================
void setStartupConfiguration(const SkeletonPtr& wam)
{
  wam->getDof("/j1")->setPosition(0.0);
  wam->getDof("/j2")->setPosition(0.0);
  wam->getDof("/j3")->setPosition(0.0);
  wam->getDof("/j4")->setPosition(0.0);
  wam->getDof("/j5")->setPosition(0.0);
  wam->getDof("/j6")->setPosition(0.0);
  wam->getDof("/j7")->setPosition(0.0);
}

//==============================================================================
void setupEndEffectors(const SkeletonPtr& wam)
{
  Eigen::VectorXd rootjoint_weights = Eigen::VectorXd::Ones(7);
  rootjoint_weights = 0.01*rootjoint_weights;

  double extra_error_clamp = 0.1;

  Eigen::Vector3d linearBounds =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());

  Eigen::Vector3d angularBounds =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());

  Eigen::Isometry3d tf_hand(Eigen::Isometry3d::Identity());
  tf_hand.translate(Eigen::Vector3d(0.0, 0.0, -0.09));

  EndEffector* ee = wam->getBodyNode("/wam7")->createEndEffector("ee");
  ee->setDefaultRelativeTransform(tf_hand, true);

  dart::gui::osg::InteractiveFramePtr lh_target(new dart::gui::osg::InteractiveFrame(
                                           Frame::World(), "lh_target"));

  ee->getIK(true)->setTarget(lh_target);
  ee->getIK()->useWholeBody();

  ee->getIK()->setGradientMethod<dart::dynamics::ImportedIkfast>("libwamIkfast");
  ee->getIK()->getAnalytical()->setExtraDofUtilization(IK::Analytical::POST_ANALYTICAL);
  ee->getIK()->getAnalytical()->setExtraErrorLengthClamp(extra_error_clamp);
  ee->getIK()->getGradientMethod().setComponentWeights(rootjoint_weights);

  ee->getIK()->getErrorMethod().setLinearBounds(-linearBounds, linearBounds);
  ee->getIK()->getErrorMethod().setAngularBounds(-angularBounds, angularBounds);

  Eigen::Isometry3d tf_foot =Eigen::Isometry3d::Identity();
  double ground_dist = 0.01;
  tf_foot.translation() = Eigen::Vector3d(0.14, 0.0, -0.136+ground_dist);

  linearBounds[2] = 1e-8;

  angularBounds[0] = 1e-8;
  angularBounds[1] = 1e-8;
}

void enableDragAndDrops(dart::gui::osg::Viewer& viewer, const SkeletonPtr& hubo)
{
  // Turn on drag-and-drop for the whole Skeleton
  for(std::size_t i=0; i < hubo->getNumBodyNodes(); ++i)
    viewer.enableDragAndDrop(hubo->getBodyNode(i), false, false);

  for(std::size_t i=0; i < hubo->getNumEndEffectors(); ++i)
  {
    EndEffector* ee = hubo->getEndEffector(i);
    if(!ee->getIK())
      continue;

    // Check whether the target is an interactive frame, and add it if it is
    if(const auto& frame = std::dynamic_pointer_cast<dart::gui::osg::InteractiveFrame>(
         ee->getIK()->getTarget()))
      viewer.enableDragAndDrop(frame.get());
  }
}

void setupWholeBodySolver(const SkeletonPtr& hubo)
{
  std::shared_ptr<dart::optimizer::GradientDescentSolver> solver =
      std::dynamic_pointer_cast<dart::optimizer::GradientDescentSolver>(
        hubo->getIK(true)->getSolver());

  std::size_t nDofs = hubo->getNumDofs();

  double default_weight = 0.01;
  Eigen::VectorXd weights = default_weight * Eigen::VectorXd::Ones(nDofs);
  weights[2] = 0.0;
  weights[3] = 0.0;
  weights[4] = 0.0;

  Eigen::VectorXd lower_posture = Eigen::VectorXd::Constant(nDofs,
        -std::numeric_limits<double>::infinity());
  lower_posture[0] = -0.35;
  lower_posture[1] = -0.35;
  lower_posture[5] =  0.55;

  Eigen::VectorXd upper_posture = Eigen::VectorXd::Constant(nDofs,
         std::numeric_limits<double>::infinity());
  upper_posture[0] =  0.35;
  upper_posture[1] =  0.50;
  upper_posture[5] =  0.95;

  std::shared_ptr<RelaxedPosture> objective = std::make_shared<RelaxedPosture>(
        hubo->getPositions(), lower_posture, upper_posture, weights);

  hubo->getIK()->setObjective(objective);

  std::shared_ptr<dart::constraint::BalanceConstraint> balance =
      std::make_shared<dart::constraint::BalanceConstraint>(hubo->getIK());
  hubo->getIK()->getProblem()->addEqConstraint(balance);

  balance->setErrorMethod(dart::constraint::BalanceConstraint::FROM_CENTROID);
  balance->setBalanceMethod(dart::constraint::BalanceConstraint::SHIFT_SUPPORT);

  solver->setNumMaxIterations(5);
}

int main()
{
  dart::simulation::WorldPtr world(new dart::simulation::World);

  SkeletonPtr wam = createWam();
  setStartupConfiguration(wam);
  setupEndEffectors(wam);

  Eigen::VectorXd positions = wam->getPositions();
  // We make a clone to test whether the cloned version behaves the exact same
  // as the original version.
  wam = wam->clone("wam_copy");
  wam->setPositions(positions);

  world->addSkeleton(wam);
  world->addSkeleton(createGround());

  setupWholeBodySolver(wam);

  ::osg::ref_ptr<TeleoperationWorld> node = new TeleoperationWorld(world, wam);

  dart::gui::osg::Viewer viewer;
  viewer.allowSimulation(false);
  viewer.addWorldNode(node);

  enableDragAndDrops(viewer, wam);

  viewer.addEventHandler(new InputHandler(&viewer, node, wam, world));

  double display_elevation = 0.05;
  viewer.addAttachment(new dart::gui::osg::SupportPolygonVisual(
                         wam, display_elevation));

  std::cout << viewer.getInstructions() << std::endl;

  std::cout << "Alt + Click:   Try to translate a body without changing its orientation\n"
            << "Ctrl + Click:  Try to rotate a body without changing its translation\n"
            << "Shift + Click: Move a body using only its parent joint\n"
            << "1 -> 6:        Toggle the interactive target of an EndEffector\n"
            << "W A S D:       Move the robot around the scene\n"
            << "Q E:           Rotate the robot counter-clockwise and clockwise\n"
            << "F Z:           Shift the robot's elevation up and down\n"
            << "X C:           Toggle support on the left and right foot\n"
            << "R:             Optimize the robot's posture\n"
            << "T:             Reset the robot to its relaxed posture\n\n"
            << "  The green polygon is the support polygon of the robot, and the blue/red ball is\n"
            << "  the robot's center of mass. The green ball is the centroid of the polygon.\n\n"
            << "Note that this is purely kinematic. Physical simulation is not allowed in this app.\n"
            << std::endl;

  // Set up the window
  viewer.setUpViewInWindow(0, 0, 1280, 960);

  // Set up the default viewing position
  viewer.getCameraManipulator()->setHomePosition(::osg::Vec3( 5.34,  3.00, 1.91),
                                                 ::osg::Vec3( 0.00,  0.00, 0.50),
                                                 ::osg::Vec3(-0.20, -0.08, 0.98));

  // Reset the camera manipulator so that it starts in the new viewing position
  viewer.setCameraManipulator(viewer.getCameraManipulator());

  viewer.run();
}
