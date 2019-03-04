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

#ifndef EXAMPLES_JOINTCONSTRAINTS_MYWINDOW_HPP_
#define EXAMPLES_JOINTCONSTRAINTS_MYWINDOW_HPP_

#include <Eigen/Dense>
#include <stdarg.h>

#include "Controller.hpp"

#include <dart/dart.hpp>
#include <dart/gui/gui.hpp>

class MyWindow : public dart::gui::glut::SimWindow
{
public:
  MyWindow(): SimWindow()
  {
    mForce = Eigen::Vector3d::Zero();
    mController = nullptr;
    mWeldJoint = nullptr;
    mImpulseDuration = 0;
    mHarnessOn = false;

  }
  virtual ~MyWindow() {}

  void timeStepping() override;
  void drawSkels() override;
  //  void displayTimer(int _val) override;
  //  void draw() override;
  void keyboard(unsigned char key, int x, int y) override;

  void setController(Controller* _controller)
  {
    mController = _controller;
  }

private:
  Eigen::Vector3d mForce;
  Controller* mController;
  dart::constraint::WeldJointConstraintPtr mWeldJoint;
  int mImpulseDuration;
  bool mHarnessOn;

};

#endif  // EXAMPLES_JOINTCONSTRAINTS_MYWINDOW_HPP_

/*
#include <stdarg.h>
#include "yui/Win3D.hpp"
#include "Controller.hpp"
#include "integration/EulerIntegrator.hpp"
#include "integration/RK4Integrator.hpp"
#include "dynamics/SkeletonDynamics.hpp"

class MyWindow : public yui::Win3D, public integration::IntegrableSystem {
public:
    //    MyWindow(dynamics::SkeletonDynamics* _m1, dynamics::SkeletonDynamics* _m2)
 MyWindow(dynamics::SkeletonDynamics* _mList = 0, ...): Win3D() {
        mBackground[0] = 1.0;
        mBackground[1] = 1.0;
        mBackground[2] = 1.0;
        mBackground[3] = 1.0;

        mSim = false;
        mPlay = false;
        mSimFrame = 0;
        mPlayFrame = 0;
        mShowMarkers = true;
        mImpulseDuration = 0;


        mPersp = 45.f;
        mTrans[1] = 300.f;

        mGravity = Eigen::Vector3d(0.0, -9.8, 0.0);
        mTimeStep = 1.0/1000.0;
        mForce = Eigen::Vector3d::Zero();

        if (_mList) {
            mSkels.push_back(_mList);
            va_list ap;
            va_start(ap, _mList);
            while (true) {
                dynamics::SkeletonDynamics *skel = va_arg(ap, dynamics::SkeletonDynamics*);
                if(skel)
                    mSkels.push_back(skel);
                else
                    break;
            }
            va_end(ap);
        }
        
        int sumNDofs = 0;
        mIndices.push_back(sumNDofs);
        for (unsigned int i = 0; i < mSkels.size(); i++) {
            int nDofs = mSkels[i]->getNumDofs();
            sumNDofs += nDofs;
            mIndices.push_back(sumNDofs);
        }
        initDyn();
    }

    virtual void draw();
    virtual void keyboard(unsigned char key, int x, int y);
    virtual void displayTimer(int _val);

    // Needed for integration
    virtual Eigen::VectorXd getState();
    virtual Eigen::VectorXd evalDeriv();
    virtual void setState(const Eigen::VectorXd &state);

 protected:
    int mSimFrame;
    bool mSim;
    int mPlayFrame;
    bool mPlay;
    bool mShowMarkers;
    integration::EulerIntegrator mIntegrator;
    //    integration::RK4Integrator mIntegrator;
    std::vector<Eigen::VectorXd> mBakedStates;

    std::vector<dynamics::SkeletonDynamics*> mSkels;
    dynamics::ContactDynamics *mCollisionHandle;
    std::vector<Eigen::VectorXd> mDofVels;
    std::vector<Eigen::VectorXd> mDofs;
    double mTimeStep;
    Eigen::Vector3d mGravity;
    Eigen::Vector3d mForce;
    std::vector<int> mIndices;
    Controller *mController;
    int mImpulseDuration;

    void initDyn();
    void setConfig();
    void bake();
};

#endif
*/
