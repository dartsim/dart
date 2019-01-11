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

#ifndef DART_EXAMPLE_OSG_OSGATLASSIMBICON_TINKERTOYWORLDNODE_HPP_
#define DART_EXAMPLE_OSG_OSGATLASSIMBICON_TINKERTOYWORLDNODE_HPP_

#include <dart/dart.hpp>
#include <dart/io/io.hpp>
#include <dart/gui/osg/osg.hpp>

const double DefaultBlockLength = 0.5;
const double DefaultBlockWidth = 0.075;
const double DefaultJointRadius = 1.5*DefaultBlockWidth/2.0;
const double BalsaWoodDensity = 0.16*10e3; // kg/m^3
const double DefaultBlockMass = BalsaWoodDensity * DefaultBlockLength * pow(DefaultBlockWidth,2);
const double DefaultDamping = 0.4;

const Eigen::Vector4d DefaultSimulationColor = Eigen::Vector4d(0.5, 0.5, 1.0, 1.0);
const Eigen::Vector4d DefaultPausedColor = Eigen::Vector4d(0xEE, 0xC9, 0x00, 0.0)/255.0 + Eigen::Vector4d(0,0,0,1.0);
const Eigen::Vector4d DefaultSelectedColor = dart::Color::Red(1.0);
const Eigen::Vector4d DefaultForceBodyColor = dart::Color::Fuchsia(1.0);
const Eigen::Vector4d DefaultForceLineColor = Eigen::Vector4d(1.0, 0.63, 0.0, 1.0);

const double MaxForce = 200.0;
const double DefaultForceCoeff = 100.0;
const double MaxForceCoeff = 1000.0;
const double MinForceCoeff = 10.0;
const double ForceIncrement = 10.0;

//==============================================================================
class TinkertoyWorldNode : public dart::gui::osg::RealTimeWorldNode
{
public:

  TinkertoyWorldNode(const dart::simulation::WorldPtr& world)
    : dart::gui::osg::RealTimeWorldNode(world),
      mForceCoeff(DefaultForceCoeff),
      mWasSimulating(false)
  {
    mTarget = dart::gui::osg::InteractiveFrame::createShared(
          dart::dynamics::Frame::World());
    getWorld()->addSimpleFrame(mTarget);

    createShapes();
    createInitialToy1();
    createInitialToy2();
    createForceLine();
  }

  void setAllBodyColors(const Eigen::Vector4d& color)
  {
    for(size_t i=0; i < getWorld()->getNumSkeletons(); ++i)
    {
      const dart::dynamics::SkeletonPtr& skel = getWorld()->getSkeleton(i);
      for(size_t j=0; j < skel->getNumBodyNodes(); ++j)
      {
        dart::dynamics::BodyNode* bn = skel->getBodyNode(j);
        for(size_t k=0; k < bn->getNumShapeNodes(); ++k)
          bn->getShapeNode(k)->getVisualAspect()->setColor(color);
      }
    }
  }

  void setPickedNodeColor(const Eigen::Vector4d& color)
  {
    if(!mPickedNode)
      return;

    for(size_t i=0; i < mPickedNode->getNumShapeNodes(); ++i)
      mPickedNode->getShapeNode(i)->getVisualAspect()->setColor(color);
  }

  void resetForceLine()
  {
    if(mPickedNode)
    {
      mForceLine->setVertex(0, mPickedNode->getWorldTransform()*mPickedPoint);
      mForceLine->setVertex(1, mTarget->getWorldTransform().translation());
    }
    else
    {
      mForceLine->setVertex(0, Eigen::Vector3d::Zero());
      mForceLine->setVertex(1, Eigen::Vector3d::Zero());
    }
  }

  void customPreRefresh() override
  {
    if(isSimulating())
    {
      setAllBodyColors(DefaultSimulationColor);
      setPickedNodeColor(DefaultForceBodyColor);
    }
    else
    {
      setAllBodyColors(DefaultPausedColor);
      setPickedNodeColor(DefaultSelectedColor);
    }

    resetForceLine();
  }

  void customPreStep() override
  {
    if(mPickedNode)
    {
      Eigen::Vector3d F = mForceCoeff *
          (mTarget->getWorldTransform().translation()
           - mPickedNode->getWorldTransform()*mPickedPoint);

      const double F_norm = F.norm();
      if(F_norm > MaxForce)
        F = MaxForce*F/F_norm;

      mPickedNode->addExtForce(F, mPickedPoint);
    }
  }

  void handlePick(const dart::gui::osg::PickInfo& pick)
  {
    dart::dynamics::BodyNode* bn = dynamic_cast<dart::dynamics::BodyNode*>(
          pick.frame->getParentFrame());

    if(!bn)
      return;

    mPickedNode = bn;
    mPickedPoint = bn->getWorldTransform().inverse() * pick.position;

    Eigen::Isometry3d tf = bn->getWorldTransform();
    tf.translation() = pick.position +
        pick.normal.normalized()*DefaultBlockWidth/2.0;

    mTarget->setTransform(tf);
  }

  void clearPick()
  {
    mPickedNode = nullptr;
    mTarget->setTransform(Eigen::Isometry3d::Identity());
  }

  void deletePick()
  {
    if(!mPickedNode)
      return;

    if(isSimulating())
    {
      std::cout << " -- Please pause simulation [using the Spacebar] before "
                << "attempting to delete blocks." << std::endl;
      return;
    }

    dart::dynamics::SkeletonPtr temporary = mPickedNode->remove();
    for(size_t i=0; i < temporary->getNumBodyNodes(); ++i)
    {
      mViewer->disableDragAndDrop(mViewer->enableDragAndDrop(
                                    temporary->getBodyNode(i)));
    }

    getWorld()->getConstraintSolver()->getCollisionGroup()->
        removeShapeFramesOf(temporary.get());


    clearPick();
  }

  void createShapes()
  {
    createWeldJointShape();
    createRevoluteJointShape();
    createBallJointShape();
    createBlockShape();
  }

  void createWeldJointShape()
  {
    mWeldJointShape = std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d(2.0*DefaultJointRadius,
                          DefaultBlockWidth,
                          DefaultBlockWidth));

    mWeldJointShape->addDataVariance(dart::dynamics::Shape::DYNAMIC_COLOR);
  }

  void createRevoluteJointShape()
  {
    mRevoluteJointShape = std::make_shared<dart::dynamics::CylinderShape>(
          DefaultJointRadius, 1.5*DefaultBlockWidth);

    mRevoluteJointShape->addDataVariance(dart::dynamics::Shape::DYNAMIC_COLOR);
  }

  void createBallJointShape()
  {
    mBallJointShape = std::make_shared<dart::dynamics::SphereShape>(
          DefaultJointRadius);

    mBallJointShape->addDataVariance(dart::dynamics::Shape::DYNAMIC_COLOR);
  }

  void createBlockShape()
  {
    mBlockShape = std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d(DefaultBlockLength,
                          DefaultBlockWidth,
                          DefaultBlockWidth));

    mBlockShape->addDataVariance(dart::dynamics::Shape::DYNAMIC_COLOR);

    mBlockOffset = Eigen::Isometry3d::Identity();
    mBlockOffset.translation()[0] = DefaultBlockLength/2.0;
  }

  template<class JointType>
  std::pair<JointType*, dart::dynamics::BodyNode*> addBlock(
      dart::dynamics::BodyNode* parent,
      const Eigen::Isometry3d& relTf,
      const dart::dynamics::ShapePtr& jointShape)
  {
    if(isSimulating())
    {
      std::cout << " -- Please pause simulation [using the Spacebar] before "
                << "attempting to add new bodies" << std::endl;
      return std::make_pair(nullptr, nullptr);
    }

    dart::dynamics::SkeletonPtr skel;
    if(parent)
    {
      skel = parent->getSkeleton();
    }
    else
    {
      skel = dart::dynamics::Skeleton::create(
            "toy_#" + std::to_string(getWorld()->getNumSkeletons()+1));
      getWorld()->addSkeleton(skel);
    }

    auto pair = skel->createJointAndBodyNodePair<JointType>(parent);
    JointType* joint = pair.first;
    dart::dynamics::BodyNode* bn = pair.second;
    bn->setName("block_#" + std::to_string(skel->getNumBodyNodes()));
    joint->setName("joint_#" + std::to_string(skel->getNumJoints()));

    joint->setTransformFromParentBodyNode(relTf);
    for(size_t i=0; i < joint->getNumDofs(); ++i)
      joint->getDof(i)->setDampingCoefficient(DefaultDamping);

    bn->createShapeNodeWith<
        dart::dynamics::VisualAspect,
        dart::dynamics::CollisionAspect,
        dart::dynamics::DynamicsAspect>(jointShape);

    dart::dynamics::ShapeNode* block = bn->createShapeNodeWith<
        dart::dynamics::VisualAspect,
        dart::dynamics::CollisionAspect,
        dart::dynamics::DynamicsAspect>(mBlockShape);
    block->setRelativeTransform(mBlockOffset);

    dart::dynamics::Inertia inertia = bn->getInertia();
    inertia.setMass(DefaultBlockMass);
    inertia.setMoment(mBlockShape->computeInertia(DefaultBlockMass));
    inertia.setLocalCOM(DefaultBlockLength/2.0*Eigen::Vector3d::UnitX());
    bn->setInertia(inertia);

    getWorld()->getConstraintSolver()->
        getCollisionGroup()->addShapeFramesOf(bn);

    clearPick();

    return std::make_pair(joint, bn);
  }

  Eigen::Isometry3d getRelTf() const
  {
    return mPickedNode? mTarget->getTransform(mPickedNode)
                      : mTarget->getWorldTransform();
  }

  void addWeldJointBlock()
  {
    addWeldJointBlock(mPickedNode, getRelTf());
  }

  dart::dynamics::BodyNode* addWeldJointBlock(
      dart::dynamics::BodyNode* parent, const Eigen::Isometry3d& relTf)
  {
    return addBlock<dart::dynamics::WeldJoint>(
          parent, relTf, mWeldJointShape).second;
  }

  void addRevoluteJointBlock()
  {
    addRevoluteJointBlock(mPickedNode, getRelTf());
  }

  dart::dynamics::BodyNode* addRevoluteJointBlock(
      dart::dynamics::BodyNode* parent, const Eigen::Isometry3d& relTf)
  {
    auto pair = addBlock<dart::dynamics::RevoluteJoint>(
          parent, relTf, mRevoluteJointShape);

    if(pair.first)
      pair.first->setAxis(Eigen::Vector3d::UnitZ());

    return pair.second;
  }

  void addBallJointBlock()
  {
    addBallJointBlock(mPickedNode, getRelTf());
  }

  dart::dynamics::BodyNode* addBallJointBlock(
      dart::dynamics::BodyNode* parent, const Eigen::Isometry3d& relTf)
  {
    return addBlock<dart::dynamics::BallJoint>(
          parent, relTf, mBallJointShape).second;
  }


  void createInitialToy1()
  {
    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    tf.rotate(Eigen::AngleAxisd(45.0*M_PI/180.0, Eigen::Vector3d::UnitY()));
    dart::dynamics::BodyNode* bn = addBallJointBlock(nullptr, tf);

    tf = Eigen::Isometry3d::Identity();
    tf.translation()[0] = DefaultBlockLength;
    tf.linear() = Eigen::Matrix3d::Identity();
    tf.prerotate(Eigen::AngleAxisd(90.0*M_PI/180.0, Eigen::Vector3d::UnitX()));
    bn = addRevoluteJointBlock(bn, tf);

    tf = Eigen::Isometry3d::Identity();
    tf.rotate(Eigen::AngleAxisd(90.0*M_PI/180.0, Eigen::Vector3d::UnitZ()));
    bn = addWeldJointBlock(bn, tf);

    tf = Eigen::Isometry3d::Identity();
    tf.translation()[0] = DefaultBlockLength/2.0;
    tf.translation()[2] = DefaultBlockWidth;
    tf.rotate(Eigen::AngleAxisd(-30.0*M_PI/180.0, Eigen::Vector3d::UnitZ()));
    bn = addBallJointBlock(bn, tf);
  }

  void createInitialToy2()
  {
    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    tf.rotate(Eigen::AngleAxisd(90.0*M_PI/180.0, Eigen::Vector3d::UnitY()));
    tf.pretranslate(-1.0*Eigen::Vector3d::UnitX());
    dart::dynamics::BodyNode* bn = addBallJointBlock(nullptr, tf);

    tf = Eigen::Isometry3d::Identity();
    tf.translation()[0] = DefaultBlockLength;
    tf.translation()[2] = DefaultBlockLength/2.0;
    tf.rotate(Eigen::AngleAxisd(90.0*M_PI/180.0, Eigen::Vector3d::UnitY()));
    bn = addWeldJointBlock(bn, tf);

    tf = Eigen::Isometry3d::Identity();
    tf.rotate(Eigen::AngleAxisd(-90.0*M_PI/180.0, Eigen::Vector3d::UnitX()));
    tf.rotate(Eigen::AngleAxisd(-90.0*M_PI/180.0, Eigen::Vector3d::UnitZ()));
    tf.translation()[2] = DefaultBlockWidth/2.0;
    addRevoluteJointBlock(bn, tf);

    tf.translation()[0] = DefaultBlockLength;
    bn = addRevoluteJointBlock(bn, tf);

    tf = Eigen::Isometry3d::Identity();
    tf.translation()[0] = DefaultBlockLength;
    addBallJointBlock(bn, tf);
  }

  void createForceLine()
  {
    dart::dynamics::SimpleFramePtr lineFrame =
        std::make_shared<dart::dynamics::SimpleFrame>(
          dart::dynamics::Frame::World());

    mForceLine = std::make_shared<dart::dynamics::LineSegmentShape>(
          Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 3.0);
    mForceLine->addDataVariance(dart::dynamics::Shape::DYNAMIC_VERTICES);

    lineFrame->setShape(mForceLine);
    lineFrame->createVisualAspect();
    lineFrame->getVisualAspect()->setColor(DefaultForceLineColor);

    getWorld()->addSimpleFrame(lineFrame);
  }

  void setForceCoeff(double coeff)
  {
    mForceCoeff = coeff;

    if(mForceCoeff > MaxForceCoeff)
      mForceCoeff = MaxForceCoeff;
    else if(mForceCoeff < MinForceCoeff)
      mForceCoeff = MinForceCoeff;
  }

  double getForceCoeff() const
  {
    return mForceCoeff;
  }

  void incrementForceCoeff()
  {
    mForceCoeff += ForceIncrement;
    if(mForceCoeff > MaxForceCoeff)
      mForceCoeff = MaxForceCoeff;

    std::cout << "[Force Coefficient: " << mForceCoeff << "]" << std::endl;
  }

  void decrementForceCoeff()
  {
    mForceCoeff -= ForceIncrement;
    if(mForceCoeff < MinForceCoeff)
      mForceCoeff = MinForceCoeff;

    std::cout << "[Force Coefficient: " << mForceCoeff << "]" << std::endl;
  }

  void reorientTarget()
  {
    Eigen::Isometry3d tf = mTarget->getWorldTransform();
    tf.linear() = Eigen::Matrix3d::Identity();
    mTarget->setTransform(tf);
  }

  float mForceCoeff;

protected:

  void setupViewer() override
  {
    mViewer->enableDragAndDrop(mTarget.get());
  }

  dart::dynamics::ShapePtr mWeldJointShape;
  dart::dynamics::ShapePtr mRevoluteJointShape;
  dart::dynamics::ShapePtr mBallJointShape;
  dart::dynamics::ShapePtr mBlockShape;
  Eigen::Isometry3d mBlockOffset;

  dart::dynamics::BodyNode* mPickedNode;
  Eigen::Vector3d mPickedPoint;
  dart::gui::osg::InteractiveFramePtr mTarget;

  dart::dynamics::LineSegmentShapePtr mForceLine;

  bool mWasSimulating;
};

#endif // DART_EXAMPLE_OSG_OSGATLASSIMBICON_TINKERTOYWORLDNODE_HPP_
