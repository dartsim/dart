/*
 * Copyright (c) 2011-2019, The DART development contributors
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

#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/io/io.hpp>
#include <dart/io/urdf/urdf.hpp>

using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::io;
using namespace dart::math;

const double display_elevation = 0.05;

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

  double eval(const Eigen::VectorXd& _x) const override
  {
    computeResultVector(_x);
    return 0.5 * mResultVector.dot(mResultVector);
  }

  void evalGradient(const Eigen::VectorXd& _x,
                    Eigen::Map<Eigen::VectorXd> _grad) const override
  {
    computeResultVector(_x);

    _grad.setZero();
    int smaller = std::min(mResultVector.size(), _grad.size());
    for(int i=0; i < smaller; ++i)
      _grad[i] = mResultVector[i];
  }

  void computeResultVector(const Eigen::VectorXd& _x) const
  {
    mResultVector.setZero();

    if(enforceIdealPosture)
    {
      // Try to get the robot into the best possible posture
      for(int i=0; i < _x.size(); ++i)
      {
        if(mIdeal.size() <= i)
          break;

        mResultVector[i] = mWeights[i]*(_x[i] - mIdeal[i]);
      }
    }
    else
    {
      // Only adjust the posture if it is really bad
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

  mutable Eigen::VectorXd mResultVector;

  Eigen::VectorXd mIdeal;

  Eigen::VectorXd mLower;

  Eigen::VectorXd mUpper;

  Eigen::VectorXd mWeights;
};

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
      mAtlas(_robot),
      iter(0),
      l_foot(_robot->getEndEffector("l_foot")),
      r_foot(_robot->getEndEffector("r_foot"))
  {
    mMoveComponents.resize(NUM_MOVE, false);
    mAnyMovement = false;
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
      Eigen::Isometry3d old_tf = mAtlas->getBodyNode(0)->getWorldTransform();
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

      const double linearStep = 0.01;
      const double elevationStep = 0.2*linearStep;
      const double rotationalStep = 2.0*M_PI/180.0;

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

      mAtlas->getJoint(0)->setPositions(FreeJoint::convertToPositions(new_tf));
    }

    bool solved = mAtlas->getIK(true)->solve();

    if(!solved)
      ++iter;
    else
      iter = 0;

    if(iter == 1000)
    {
      std::cout << "Failing!" << std::endl;
    }
  }

protected:

  SkeletonPtr mAtlas;
  std::size_t iter;

  EndEffectorPtr l_foot;
  EndEffectorPtr r_foot;

  Eigen::VectorXd grad;

  std::vector<bool> mMoveComponents;

  bool mAnyMovement;
};

class InputHandler : public ::osgGA::GUIEventHandler
{
public:

  InputHandler(dart::gui::osg::Viewer* viewer, TeleoperationWorld* teleop,
               const SkeletonPtr& atlas, const WorldPtr& world)
    : mViewer(viewer),
      mTeleop(teleop),
      mAtlas(atlas),
      mWorld(world)
  {
    initialize();
  }

  void initialize()
  {
    mRestConfig = mAtlas->getPositions();

    mLegs.reserve(12);
    for(std::size_t i=0; i<mAtlas->getNumDofs(); ++i)
    {
      if(mAtlas->getDof(i)->getName().substr(1, 5) == "_leg_")
        mLegs.push_back(mAtlas->getDof(i)->getIndexInSkeleton());
    }
    // We should also adjust the pelvis when detangling the legs
    mLegs.push_back(mAtlas->getDof("rootJoint_rot_x")->getIndexInSkeleton());
    mLegs.push_back(mAtlas->getDof("rootJoint_rot_y")->getIndexInSkeleton());
    mLegs.push_back(mAtlas->getDof("rootJoint_pos_z")->getIndexInSkeleton());

    for(std::size_t i=0; i < mAtlas->getNumEndEffectors(); ++i)
    {
      const InverseKinematicsPtr ik = mAtlas->getEndEffector(i)->getIK();
      if(ik)
      {
        mDefaultBounds.push_back(ik->getErrorMethod().getBounds());
        mDefaultTargetTf.push_back(ik->getTarget()->getRelativeTransform());
        mConstraintActive.push_back(false);
        mEndEffectorIndex.push_back(i);
      }
    }

    mPosture = std::dynamic_pointer_cast<RelaxedPosture>(
          mAtlas->getIK(true)->getObjective());

    mBalance = std::dynamic_pointer_cast<dart::constraint::BalanceConstraint>(
          mAtlas->getIK(true)->getProblem()->getEqConstraint(1));

    mOptimizationKey = 'r';

    mMoveComponents.resize(TeleoperationWorld::NUM_MOVE, false);
  }

  virtual bool handle(const ::osgGA::GUIEventAdapter& ea,
                      ::osgGA::GUIActionAdapter&) override
  {
    if(nullptr == mAtlas)
    {
      return false;
    }

    if( ::osgGA::GUIEventAdapter::KEYDOWN == ea.getEventType() )
    {
      if( ea.getKey() == 'p' )
      {
        for(std::size_t i=0; i < mAtlas->getNumDofs(); ++i)
          std::cout << mAtlas->getDof(i)->getName() << ": "
                    << mAtlas->getDof(i)->getPosition() << std::endl;
        std::cout << "  -- -- -- -- -- " << std::endl;
        return true;
      }

      if( ea.getKey() == 't' )
      {
        // Reset all the positions except for x, y, and yaw
        for(std::size_t i=0; i < mAtlas->getNumDofs(); ++i)
        {
          if( i < 2 || 4 < i )
            mAtlas->getDof(i)->setPosition(mRestConfig[i]);
        }
        return true;
      }

      if( '1' <= ea.getKey() && ea.getKey() <= '9' )
      {
        std::size_t index = ea.getKey() - '1';
        if(index < mConstraintActive.size())
        {
          EndEffector* ee = mAtlas->getEndEffector(mEndEffectorIndex[index]);
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
        EndEffector* ee = mAtlas->getEndEffector("l_foot");
        ee->getSupport()->setActive(!ee->getSupport()->isActive());
        return true;
      }

      if( 'c' == ea.getKey() )
      {
        EndEffector* ee = mAtlas->getEndEffector("r_foot");
        ee->getSupport()->setActive(!ee->getSupport()->isActive());
        return true;
      }

      switch(ea.getKey())
      {
        case 'w': mMoveComponents[TeleoperationWorld::MOVE_W] = true; break;
        case 'a': mMoveComponents[TeleoperationWorld::MOVE_A] = true; break;
        case 's': mMoveComponents[TeleoperationWorld::MOVE_S] = true; break;
        case 'd': mMoveComponents[TeleoperationWorld::MOVE_D] = true; break;
        case 'q': mMoveComponents[TeleoperationWorld::MOVE_Q] = true; break;
        case 'e': mMoveComponents[TeleoperationWorld::MOVE_E] = true; break;
        case 'f': mMoveComponents[TeleoperationWorld::MOVE_F] = true; break;
        case 'z': mMoveComponents[TeleoperationWorld::MOVE_Z] = true; break;
      }

      switch(ea.getKey())
      {
        case 'w': case 'a': case 's': case 'd': case 'q': case'e': case 'f': case 'z':
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


      switch(ea.getKey())
      {
        case 'w': mMoveComponents[TeleoperationWorld::MOVE_W] = false; break;
        case 'a': mMoveComponents[TeleoperationWorld::MOVE_A] = false; break;
        case 's': mMoveComponents[TeleoperationWorld::MOVE_S] = false; break;
        case 'd': mMoveComponents[TeleoperationWorld::MOVE_D] = false; break;
        case 'q': mMoveComponents[TeleoperationWorld::MOVE_Q] = false; break;
        case 'e': mMoveComponents[TeleoperationWorld::MOVE_E] = false; break;
        case 'f': mMoveComponents[TeleoperationWorld::MOVE_F] = false; break;
        case 'z': mMoveComponents[TeleoperationWorld::MOVE_Z] = false; break;
      }

      switch(ea.getKey())
      {
        case 'w': case 'a': case 's': case 'd': case 'q': case'e': case 'f': case 'z':
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

  SkeletonPtr mAtlas;

  WorldPtr mWorld;

  Eigen::VectorXd mRestConfig;

  std::vector<std::size_t> mLegs;

  std::vector<bool> mConstraintActive;

  std::vector<std::size_t> mEndEffectorIndex;

  std::vector< std::pair<Eigen::Vector6d, Eigen::Vector6d> > mDefaultBounds;

  dart::common::aligned_vector<Eigen::Isometry3d> mDefaultTargetTf;

  std::shared_ptr<RelaxedPosture> mPosture;

  std::shared_ptr<dart::constraint::BalanceConstraint> mBalance;

  char mOptimizationKey;

  std::vector<bool> mMoveComponents;
};

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

SkeletonPtr createAtlas()
{
  // Parse in the atlas model
  DartLoader urdf;
  SkeletonPtr atlas =
      urdf.parseSkeleton("dart://sample/sdf/atlas/atlas_v3_no_head.urdf");

  // Add a box to the root node to make it easier to click and drag
  double scale = 0.25;
  ShapePtr boxShape =
      std::make_shared<BoxShape>(scale*Eigen::Vector3d(1.0, 1.0, 0.5));

  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = Eigen::Vector3d(0.1*Eigen::Vector3d(0.0, 0.0, 1.0));

  auto shapeNode =
      atlas->getBodyNode(0)->createShapeNodeWith<VisualAspect>(boxShape);
  shapeNode->getVisualAspect()->setColor(dart::Color::Black());
  shapeNode->setRelativeTransform(tf);

  return atlas;
}

void setupStartConfiguration(const SkeletonPtr& atlas)
{
  // Squat with the right leg
  atlas->getDof("r_leg_hpy")->setPosition(-45.0*M_PI/180.0);
  atlas->getDof("r_leg_kny")->setPosition( 90.0*M_PI/180.0);
  atlas->getDof("r_leg_aky")->setPosition(-45.0*M_PI/180.0);

  // Squat with the left left
  atlas->getDof("l_leg_hpy")->setPosition(-45.0*M_PI/180.0);
  atlas->getDof("l_leg_kny")->setPosition( 90.0*M_PI/180.0);
  atlas->getDof("l_leg_aky")->setPosition(-45.0*M_PI/180.0);

  // Get the right arm into a comfortable position
  atlas->getDof("r_arm_shx")->setPosition( 65.0*M_PI/180.0);
  atlas->getDof("r_arm_ely")->setPosition( 90.0*M_PI/180.0);
  atlas->getDof("r_arm_elx")->setPosition(-90.0*M_PI/180.0);
  atlas->getDof("r_arm_wry")->setPosition( 65.0*M_PI/180.0);

  // Get the left arm into a comfortable position
  atlas->getDof("l_arm_shx")->setPosition(-65.0*M_PI/180.0);
  atlas->getDof("l_arm_ely")->setPosition( 90.0*M_PI/180.0);
  atlas->getDof("l_arm_elx")->setPosition( 90.0*M_PI/180.0);
  atlas->getDof("l_arm_wry")->setPosition( 65.0*M_PI/180.0);

  // Prevent the knees from bending backwards
  atlas->getDof("r_leg_kny")->setPositionLowerLimit( 10*M_PI/180.0);
  atlas->getDof("l_leg_kny")->setPositionLowerLimit( 10*M_PI/180.0);
}

void setupEndEffectors(const SkeletonPtr& atlas)
{
  // Apply very small weights to the gradient of the root joint in order to
  // encourage the arms to use arm joints instead of only moving around the root
  // joint
  Eigen::VectorXd rootjoint_weights = Eigen::VectorXd::Ones(6);
  rootjoint_weights = 0.01*rootjoint_weights;

  // Setting the bounds to be infinite allows the end effector to be implicitly
  // unconstrained
  Eigen::Vector3d linearBounds =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());

  Eigen::Vector3d angularBounds =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());

  // -- Set up the left hand --

  // Create a relative transform for the EndEffector frame. This is the
  // transform that the left hand will have relative to the BodyNode that it is
  // attached to
  Eigen::Isometry3d tf_hand(Eigen::Isometry3d::Identity());
  tf_hand.translation() = Eigen::Vector3d(0.0009, 0.1254, 0.012);
  tf_hand.rotate(Eigen::AngleAxisd(90.0*M_PI/180.0, Eigen::Vector3d::UnitZ()));

  // Create the left hand's end effector and set its relative transform
  EndEffector* l_hand = atlas->getBodyNode("l_hand")->createEndEffector("l_hand");
  l_hand->setDefaultRelativeTransform(tf_hand, true);

  // Create an interactive frame to use as the target for the left hand
  dart::gui::osg::InteractiveFramePtr lh_target(new dart::gui::osg::InteractiveFrame(
                                           Frame::World(), "lh_target"));

  // Set the target of the left hand to the interactive frame. We pass true into
  // the function to tell it that it should create the IK module if it does not
  // already exist. If we don't do that, then calling getIK() could return a
  // nullptr if the IK module was never created.
  l_hand->getIK(true)->setTarget(lh_target);

  // Tell the left hand to use the whole body for its IK
  l_hand->getIK()->useWholeBody();

  // Set the weights for the gradient
  l_hand->getIK()->getGradientMethod().setComponentWeights(rootjoint_weights);

  // Set the bounds for the IK to be infinite so that the hands start out
  // unconstrained
  l_hand->getIK()->getErrorMethod().setLinearBounds(
        -linearBounds, linearBounds);

  l_hand->getIK()->getErrorMethod().setAngularBounds(
        -angularBounds, angularBounds);

  // -- Set up the right hand --

  // The orientation of the right hand frame is different than the left, so we
  // need to adjust the signs of the relative transform
  tf_hand.translation()[0] = -tf_hand.translation()[0];
  tf_hand.translation()[1] = -tf_hand.translation()[1];
  tf_hand.linear() = tf_hand.linear().inverse().eval();

  // Create the right hand's end effector and set its relative transform
  EndEffector* r_hand = atlas->getBodyNode("r_hand")->createEndEffector("r_hand");
  r_hand->setDefaultRelativeTransform(tf_hand, true);

  // Create an interactive frame to use as the target for the right hand
  dart::gui::osg::InteractiveFramePtr rh_target(new dart::gui::osg::InteractiveFrame(
                                           Frame::World(), "rh_target"));

  // Create the right hand's IK and set its target
  r_hand->getIK(true)->setTarget(rh_target);

  // Tell the right hand to use the whole body for its IK
  r_hand->getIK()->useWholeBody();

  // Set the weights for the gradient
  r_hand->getIK()->getGradientMethod().setComponentWeights(rootjoint_weights);

  // Set the bounds for the IK to be infinite so that the hands start out
  // unconstrained
  r_hand->getIK()->getErrorMethod().setLinearBounds(
        -linearBounds, linearBounds);

  r_hand->getIK()->getErrorMethod().setAngularBounds(
        -angularBounds, angularBounds);


  // Define the support geometry for the feet. These points will be used to
  // compute the convex hull of the robot's support polygon
  dart::math::SupportGeometry support;
  const double sup_pos_x =  0.10-0.186;
  const double sup_neg_x = -0.03-0.186;
  const double sup_pos_y =  0.03;
  const double sup_neg_y = -0.03;
  support.push_back(Eigen::Vector3d(sup_neg_x, sup_neg_y, 0.0));
  support.push_back(Eigen::Vector3d(sup_pos_x, sup_neg_y, 0.0));
  support.push_back(Eigen::Vector3d(sup_pos_x, sup_pos_y, 0.0));
  support.push_back(Eigen::Vector3d(sup_neg_x, sup_pos_y, 0.0));

  // Create a relative transform that goes from the center of the feet to the
  // bottom of the feet
  Eigen::Isometry3d tf_foot(Eigen::Isometry3d::Identity());
  tf_foot.translation() = Eigen::Vector3d(0.186, 0.0, -0.08);

  // Constrain the feet to snap to the ground
  linearBounds[2] = 1e-8;

  // Constrain the feet to lie flat on the ground
  angularBounds[0] = 1e-8;
  angularBounds[1] = 1e-8;

  // Create an end effector for the left foot and set its relative transform
  EndEffector* l_foot = atlas->getBodyNode("l_foot")->createEndEffector("l_foot");
  l_foot->setRelativeTransform(tf_foot);

  // Create an interactive frame to use as the target for the left foot
  dart::gui::osg::InteractiveFramePtr lf_target(new dart::gui::osg::InteractiveFrame(
                                           Frame::World(), "lf_target"));

  // Create the left foot's IK and set its target
  l_foot->getIK(true)->setTarget(lf_target);

  // Set the left foot's IK hierarchy level to 1. This will project its IK goals
  // through the null space of any IK modules that are on level 0. This means
  // that it will try to accomplish its goals while also accommodating the goals
  // of other modules.
  l_foot->getIK()->setHierarchyLevel(1);

  // Use the bounds defined above
  l_foot->getIK()->getErrorMethod().setLinearBounds(
        -linearBounds, linearBounds);
  l_foot->getIK()->getErrorMethod().setAngularBounds(
        -angularBounds, angularBounds);

  // Create Support for the foot and give it geometry
  l_foot->getSupport(true)->setGeometry(support);

  // Turn on support mode so that it can be used as a foot
  l_foot->getSupport()->setActive();

  // Create an end effector for the right foot and set its relative transform
  EndEffector* r_foot = atlas->getBodyNode("r_foot")->createEndEffector("r_foot");
  r_foot->setRelativeTransform(tf_foot);

  // Create an interactive frame to use as the target for the right foot
  dart::gui::osg::InteractiveFramePtr rf_target(new dart::gui::osg::InteractiveFrame(
                                           Frame::World(), "rf_target"));

  // Create the right foot's IK module and set its target
  r_foot->getIK(true)->setTarget(rf_target);

  // Set the right foot's IK hierarchy level to 1
  r_foot->getIK()->setHierarchyLevel(1);

  // Use the bounds defined above
  r_foot->getIK()->getErrorMethod().setLinearBounds(
        -linearBounds, linearBounds);
  r_foot->getIK()->getErrorMethod().setAngularBounds(
        -angularBounds, angularBounds);

  // Create Support for the foot and give it geometry
  r_foot->getSupport(true)->setGeometry(support);

  // Turn on support mode so that it can be used as a foot
  r_foot->getSupport()->setActive();

  // Move atlas to the ground so that it starts out squatting with its feet on
  // the ground
  double heightChange = -r_foot->getWorldTransform().translation()[2];
  atlas->getDof(5)->setPosition(heightChange);

  // Now that the feet are on the ground, we should set their target transforms
  l_foot->getIK()->getTarget()->setTransform(l_foot->getTransform());
  r_foot->getIK()->getTarget()->setTransform(r_foot->getTransform());
}

void setupWholeBodySolver(const SkeletonPtr& atlas)
{
  // The default
  std::shared_ptr<dart::optimizer::GradientDescentSolver> solver =
      std::dynamic_pointer_cast<dart::optimizer::GradientDescentSolver>(
      atlas->getIK(true)->getSolver());
  solver->setNumMaxIterations(10);

  std::size_t nDofs = atlas->getNumDofs();

  double default_weight = 0.01;
  Eigen::VectorXd weights = default_weight * Eigen::VectorXd::Ones(nDofs);
  weights[2] = 0.0;
  weights[3] = 0.0;
  weights[4] = 0.0;

  weights[6] *= 0.2;
  weights[7] *= 0.2;
  weights[8] *= 0.2;

  Eigen::VectorXd lower_posture = Eigen::VectorXd::Constant(
        nDofs, -std::numeric_limits<double>::infinity());
  lower_posture[0] = -0.35;
  lower_posture[1] = -0.35;
  lower_posture[5] =  0.600;

  lower_posture[6] = -0.1;
  lower_posture[7] = -0.1;
  lower_posture[8] = -0.1;

  Eigen::VectorXd upper_posture = Eigen::VectorXd::Constant(
        nDofs, std::numeric_limits<double>::infinity());
  upper_posture[0] =  0.35;
  upper_posture[1] =  0.35;
  upper_posture[5] =  0.885;

  upper_posture[6] =  0.1;
  upper_posture[7] =  0.1;
  upper_posture[8] =  0.1;

  std::shared_ptr<RelaxedPosture> objective = std::make_shared<RelaxedPosture>(
        atlas->getPositions(), lower_posture, upper_posture, weights);
  atlas->getIK()->setObjective(objective);

  std::shared_ptr<dart::constraint::BalanceConstraint> balance =
      std::make_shared<dart::constraint::BalanceConstraint>(atlas->getIK());
  atlas->getIK()->getProblem()->addEqConstraint(balance);

//  // Shift the center of mass towards the support polygon center while trying
//  // to keep the support polygon where it is
//  balance->setErrorMethod(dart::constraint::BalanceConstraint::FROM_CENTROID);
//  balance->setBalanceMethod(dart::constraint::BalanceConstraint::SHIFT_COM);

//  // Keep shifting the center of mass towards the center of the support
//  // polygon, even if it is already inside. This is useful for trying to
//  // optimize a stance
//  balance->setErrorMethod(dart::constraint::BalanceConstraint::OPTIMIZE_BALANCE);
//  balance->setBalanceMethod(dart::constraint::BalanceConstraint::SHIFT_COM);

//  // Try to leave the center of mass where it is while moving the support
//  // polygon to be under the current center of mass location
  balance->setErrorMethod(dart::constraint::BalanceConstraint::FROM_CENTROID);
  balance->setBalanceMethod(dart::constraint::BalanceConstraint::SHIFT_SUPPORT);

//  // Try to leave the center of mass where it is while moving the support
//  // point that is closest to the center of mass
//  balance->setErrorMethod(dart::constraint::BalanceConstraint::FROM_EDGE);
//  balance->setBalanceMethod(dart::constraint::BalanceConstraint::SHIFT_SUPPORT);

  // Note that using the FROM_EDGE error method is liable to leave the center of
  // mass visualization red even when the constraint was successfully solved.
  // This is because the constraint solver has a tiny bit of tolerance that
  // allows the Problem to be considered solved when the center of mass is
  // microscopically outside of the support polygon. This is an inherent risk of
  // using FROM_EDGE instead of FROM_CENTROID.

  // Feel free to experiment with the different balancing methods. You will find
  // that some work much better for user interaction than others.
}

void enableDragAndDrops(dart::gui::osg::Viewer& viewer, const SkeletonPtr& atlas)
{
  // Turn on drag-and-drop for the whole Skeleton
  for(std::size_t i=0; i < atlas->getNumBodyNodes(); ++i)
    viewer.enableDragAndDrop(atlas->getBodyNode(i), false, false);

  for(std::size_t i=0; i < atlas->getNumEndEffectors(); ++i)
  {
    EndEffector* ee = atlas->getEndEffector(i);
    if(!ee->getIK())
      continue;

    // Check whether the target is an interactive frame, and add it if it is
    if(const auto& frame = std::dynamic_pointer_cast<dart::gui::osg::InteractiveFrame>(
         ee->getIK()->getTarget()))
      viewer.enableDragAndDrop(frame.get());
  }
}

int main()
{
  WorldPtr world = World::create();

  SkeletonPtr atlas = createAtlas();
  world->addSkeleton(atlas);

  SkeletonPtr ground = createGround();
  world->addSkeleton(ground);

  setupStartConfiguration(atlas);

  setupEndEffectors(atlas);

  setupWholeBodySolver(atlas);

  ::osg::ref_ptr<TeleoperationWorld> node = new TeleoperationWorld(world, atlas);

  dart::gui::osg::Viewer viewer;

  // Prevent this World from simulating
  viewer.allowSimulation(false);
  viewer.addWorldNode(node);

  // Add our custom input handler to the Viewer
  viewer.addEventHandler(new InputHandler(&viewer, node, atlas, world));

  enableDragAndDrops(viewer, atlas);

  // Attach a support polygon visualizer
  viewer.addAttachment(new dart::gui::osg::SupportPolygonVisual(
                         atlas, display_elevation));

  // Print out instructions for the viewer
  std::cout << viewer.getInstructions() << std::endl;

  std::cout << "Alt + Click:   Try to translate a body without changing its orientation\n"
            << "Ctrl + Click:  Try to rotate a body without changing its translation\n"
            << "Shift + Click: Move a body using only its parent joint\n"
            << "1 -> 4:        Toggle the interactive target of an EndEffector\n"
            << "W A S D:       Move the robot around the scene\n"
            << "Q E:           Rotate the robot counter-clockwise and clockwise\n"
            << "F Z:           Shift the robot's elevation up and down\n"
            << "X C:           Toggle support on the left and right foot\n"
            << "R:             Optimize the robot's posture\n"
            << "T:             Reset the robot to its relaxed posture\n\n"
            << "  Because this uses iterative Jacobian methods, the solver can get finicky,\n"
            << "  and the robot can get tangled up. Use 'R' and 'T' keys when the robot is\n"
            << "  in a messy configuration\n\n"
            << "  The green polygon is the support polygon of the robot, and the blue/red ball is\n"
            << "  the robot's center of mass. The green ball is the centroid of the polygon.\n\n"
            << "Note that this is purely kinematic. Physical simulation is not allowed in this app.\n"
            << std::endl;


  // Set up the window
  viewer.setUpViewInWindow(0, 0, 1280, 960);

  // Set up the default viewing position
  viewer.getCameraManipulator()->setHomePosition(::osg::Vec3( 5.34,  3.00, 2.41),
                                                 ::osg::Vec3( 0.00,  0.00, 1.00),
                                                 ::osg::Vec3(-0.20, -0.08, 0.98));

  // Reset the camera manipulator so that it starts in the new viewing position
  viewer.setCameraManipulator(viewer.getCameraManipulator());

  // Run the Viewer
  viewer.run();
}
