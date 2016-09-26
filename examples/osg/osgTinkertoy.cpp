/*
 * Copyright (c) 2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
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

const double DefaultBlockLength = 0.5;
const double DefaultBlockWidth = 0.05;
const double DefaultJointRadius = DefaultBlockWidth/2.0;
const double BalsaWoodDensity = 0.16*10e3; // kg/m^3
const double DefaultBlockMass = BalsaWoodDensity * DefaultBlockLength * pow(DefaultBlockWidth,2);

const Eigen::Vector4d DefaultNewColor = dart::Color::Orange(1.0);
const Eigen::Vector4d DefaultSimulationColor = dart::Color::Blue(1.0);
const Eigen::Vector4d DefaultPausedColor = Eigen::Vector4d(1.0, 1.0, 0.0, 1.0);
const Eigen::Vector4d DefaultSelectedColor = dart::Color::Red(1.0);
const Eigen::Vector4d DefaultForceColor = dart::Color::Fuchsia(1.0);

//==============================================================================
class TinkertoyWorldNode : public dart::gui::osg::WorldNode
{
public:

  TinkertoyWorldNode(const dart::simulation::WorldPtr& world)
    : dart::gui::osg::WorldNode(world)
  {
    createShapes();
    createInitialToy();
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
          DefaultJointRadius, DefaultBlockWidth);

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

    dart::dynamics::ShapeNode* j_node = bn->createShapeNodeWith<
        dart::dynamics::VisualAspect,
        dart::dynamics::CollisionAspect,
        dart::dynamics::DynamicsAspect>(jointShape);
    j_node->getVisualAspect()->setColor(DefaultNewColor);

    dart::dynamics::ShapeNode* block = bn->createShapeNodeWith<
        dart::dynamics::VisualAspect,
        dart::dynamics::CollisionAspect,
        dart::dynamics::DynamicsAspect>(mBlockShape);
    block->setRelativeTransform(mBlockOffset);
    block->getVisualAspect()->setColor(DefaultNewColor);

    dart::dynamics::Inertia inertia = bn->getInertia();
    inertia.setMass(DefaultBlockMass);
    inertia.setMoment(mBlockShape->computeInertia(DefaultBlockMass));
    inertia.setLocalCOM(DefaultBlockLength*Eigen::Vector3d::UnitX());
    bn->setInertia(inertia);

    if(mViewer)
      mViewer->enableDragAndDrop(bn);

    return std::make_pair(joint, bn);
  }

  dart::dynamics::BodyNode* addWeldJointBlock(
      dart::dynamics::BodyNode* parent, const Eigen::Isometry3d& relTf)
  {
    return addBlock<dart::dynamics::WeldJoint>(
          parent, relTf, mWeldJointShape).second;
  }

  dart::dynamics::BodyNode* addRevoluteJointBlock(
      dart::dynamics::BodyNode* parent, const Eigen::Isometry3d& relTf)
  {
    auto pair = addBlock<dart::dynamics::RevoluteJoint>(
          parent, relTf, mRevoluteJointShape);

    pair.first->setAxis(Eigen::Vector3d::UnitZ());
    return pair.second;
  }

  dart::dynamics::BodyNode* addBallJointBlock(
      dart::dynamics::BodyNode* parent, const Eigen::Isometry3d& relTf)
  {
    return addBlock<dart::dynamics::BallJoint>(
          parent, relTf, mBallJointShape).second;
  }


  void createInitialToy()
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
    tf.rotate(Eigen::AngleAxisd(-30.0*M_PI/180.0, Eigen::Vector3d::UnitZ()));
    bn = addBallJointBlock(bn, tf);
  }


protected:

  void setupViewer() override
  {
    for(size_t i=0; i < getWorld()->getNumSkeletons(); ++i)
    {
      const dart::dynamics::SkeletonPtr& skel = getWorld()->getSkeleton(i);
      for(size_t j=0; j < skel->getNumBodyNodes(); ++j)
        mViewer->enableDragAndDrop(skel->getBodyNode(j));
    }
  }

  dart::dynamics::ShapePtr mWeldJointShape;
  dart::dynamics::ShapePtr mRevoluteJointShape;
  dart::dynamics::ShapePtr mBallJointShape;
  dart::dynamics::ShapePtr mBlockShape;
  Eigen::Isometry3d mBlockOffset;



};

//==============================================================================
class TinkertoyInputHandler : public osgGA::GUIEventHandler
{
public:

  TinkertoyInputHandler(dart::gui::osg::Viewer* viewer,
                        TinkertoyWorldNode* node)
    : mViewer(viewer),
      mNode(node)
  {
    mTarget = std::make_shared<dart::gui::osg::InteractiveFrame>(
          dart::dynamics::Frame::World());

    mNode->getWorld()->addSimpleFrame(mTarget);
    mViewer->enableDragAndDrop(mTarget.get());
  }



  dart::gui::osg::Viewer* mViewer;
  dart::gui::osg::InteractiveFramePtr mTarget;
  TinkertoyWorldNode* mNode;

};


//==============================================================================
int main()
{
  // Create a world
  dart::simulation::WorldPtr world(new dart::simulation::World);

  osg::ref_ptr<TinkertoyWorldNode> node = new TinkertoyWorldNode(world);
  node->setNumStepsPerCycle(20);

  dart::gui::osg::Viewer viewer;
  viewer.addWorldNode(node);
  viewer.addEventHandler(new TinkertoyInputHandler(&viewer, node));

  viewer.setUpViewInWindow(0, 0, 640, 480);

  viewer.run();
}
