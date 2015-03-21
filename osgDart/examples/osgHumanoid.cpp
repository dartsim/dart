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

#include "osgDart/osgDart.h"

#include "dart/dart.h"

using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::utils;
using namespace dart::math;

Eigen::Vector3d clamped_mag(const Eigen::Vector3d& v, double clamp=0.1)
{
  if(v.norm() > clamp)
    return v.normalized()*clamp;
  return v;
}

void clamp_components(Eigen::VectorXd& v, double clamp=0.1)
{
  clamp = fabs(clamp);
  for(int i=0; i<v.size(); ++i)
  {
    if(fabs(v[i]) > clamp)
      v[i] = v[i] > 0? clamp : -clamp;
  }
}

class TeleoperationWorld : public osgDart::WorldNode
{
public:

  TeleoperationWorld(World* _world, Skeleton* _robot)
    : osgDart::WorldNode(_world),
      mRobot(_robot)
  {
    effectors.push_back(mRobot->getBodyNode("l_hand"));
    initialTfs.push_back(effectors.back()->getWorldTransform());
    dofs.push_back(effectors.back()->getDependentGenCoordIndices());
    targets.push_back(new osgDart::InteractiveFrame(Frame::World(), "l_target",
                                  effectors.back()->getWorldTransform(), 0.3));
    mWorld->addFrame(targets.back());


    effectors.push_back(mRobot->getBodyNode("r_hand"));
    initialTfs.push_back(effectors.back()->getWorldTransform());
    dofs.push_back(effectors.back()->getDependentGenCoordIndices());
    targets.push_back(new osgDart::InteractiveFrame(Frame::World(), "r_target",
                                  effectors.back()->getWorldTransform(), 0.3));
    mWorld->addFrame(targets.back());


    effectors.push_back(mRobot->getBodyNode("l_foot"));
    initialTfs.push_back(effectors.back()->getWorldTransform());
    dofs.push_back(effectors.back()->getDependentGenCoordIndices());
    targets.push_back(nullptr);


    effectors.push_back(mRobot->getBodyNode("r_foot"));
    initialTfs.push_back(effectors.back()->getWorldTransform());
    dofs.push_back(effectors.back()->getDependentGenCoordIndices());
    targets.push_back(nullptr);


    qs.resize(effectors.size());
    dqs.resize(effectors.size());
    inv_J.resize(effectors.size());
    errs.resize(effectors.size());

    l_knee = mRobot->getDof("l_leg_kny");
    r_knee = mRobot->getDof("r_leg_kny");
    min_knee_angle = 40.0*M_PI/180.0;
  }

  void customPreRefresh() override
  {
    for(size_t i=0; i<100; ++i)
    {
      // compute errors
      for(size_t e=0; e<effectors.size(); ++e)
      {
        BodyNode* bn = effectors[e];
        const Eigen::Isometry3d& B = bn->getWorldTransform();
        Eigen::Isometry3d T = (targets[e] == nullptr)?
              initialTfs[e] : targets[e]->getWorldTransform();
        Eigen::AngleAxisd aa(T.rotation()*B.rotation().transpose());
        errs[e].head<3>() = clamped_mag(aa.angle()*aa.axis());
        errs[e].tail<3>() = clamped_mag(T.translation() - B.translation());
        if(nullptr == targets[e])
        {
          errs[e][3] = 0; errs[e][4] = 0;
        }
      }

      bool finished = true;
      for(size_t e=0; e<effectors.size(); ++e)
        if(errs[e].norm() > 1e-8)
          finished = false;

      if(finished)
        return;

      for(size_t e=0; e<effectors.size(); ++e)
      {
        qs[e] = mRobot->getPositionSegment(dofs[e]);
        const Jacobian& J = effectors[e]->getWorldJacobian();
        inv_J[e] = J.transpose()*(J*J.transpose()
                                + 0.0025*Eigen::Matrix6d::Identity()).inverse();
        dqs[e] = inv_J[e]*errs[e];
        clamp_components(dqs[e]);
      }

      q = mRobot->getPositions();
      for(size_t e=0; e<effectors.size(); ++e)
      {
        const std::vector<size_t>& d = dofs[e];
        for(size_t j=0; j<d.size(); ++j)
          q[d[j]] += dqs[e][j];
      }

      mRobot->setPositions(q);

      if(l_knee->getPosition() < min_knee_angle)
        l_knee->setPosition(min_knee_angle);
      if(r_knee->getPosition() < min_knee_angle)
        r_knee->setPosition(min_knee_angle);
    }
  }

protected:

  void setupViewer() override
  {
    if(mViewer)
    {
      for(size_t i=0; i<effectors.size(); ++i)
        mViewer->enableDragAndDrop(targets[i]);
    }
  }

  Skeleton* mRobot;

  std::vector< BodyNode* > effectors;
  std::vector< std::vector<size_t> > dofs;
  std::vector< osgDart::InteractiveFrame* > targets;
  std::vector< Eigen::Isometry3d > initialTfs;

  std::vector< Eigen::VectorXd > qs;
  std::vector< Eigen::VectorXd > dqs;
  std::vector< Eigen::MatrixXd > inv_J;
  std::vector< Eigen::Vector6d > errs;

  double min_knee_angle;
  DegreeOfFreedom* l_knee;
  DegreeOfFreedom* r_knee;

  Eigen::VectorXd q;
};

int main()
{
  World* world = new World;

  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = Eigen::Vector3d(0,0,-0.95);
  SimpleFrame ground(Frame::World(), "ground", tf);
  ground.addVisualizationShape(new BoxShape(Eigen::Vector3d(10,10,0.01)));
  world->addFrame(&ground);

  DartLoader urdf;
  Skeleton* atlas =
      urdf.parseSkeleton(DART_DATA_PATH"sdf/atlas/atlas_v3_no_head.urdf");
  world->addSkeleton(atlas);

  osg::ref_ptr<osgDart::WorldNode> node = new TeleoperationWorld(world, atlas);

  osgDart::Viewer viewer;
  viewer.addWorldNode(node);

  std::cout << viewer.getInstructions() << std::endl;

  for(size_t i=0; i<atlas->getNumDofs(); ++i)
    std::cout << atlas->getDof(i)->getName()
              << " : " << atlas->getDof(i)->getPosition() << std::endl;

  viewer.setUpViewInWindow(0, 0, 640, 480);

  viewer.run();
}
