/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>
 * Date: 06/12/2011
 *
 * Geoorgia Tech Graphics Lab and Humanoid Robotics Lab
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

#include "Shape.h"
using namespace Eigen;


#define PRIMITIVE_MAGIC_NUMBER 1000

namespace kinematics {

    // initialize in the same order as declaration
    Shape::Shape(ShapeType _type, double _mass) :
          mType(_type),
          mDim(0, 0, 0),
          mMass(_mass),
          mVolume(0), 
          mInertia(Matrix3d::Zero()),
          mMassTensor(Matrix4d::Zero()),
          mOffset(0, 0, 0),
	  mVisTransform( Matrix4d::Identity() ),
          mVizMesh(NULL),
          mCollisionMesh(NULL),
          mID(mCounter++),
          mColor(0.5, 0.5, 1.0) {
    }

    void Shape::setInertia(const Eigen::Matrix3d& _inertia) {
        mInertia = _inertia;
        setMassTensorFromInertia();
    }

    void Shape::setDim(const Eigen::Vector3d& _dim) {
        mDim = _dim;
        computeVolume();
        computeMassTensor();
        computeInertiaFromMassTensor();
    }

    void Shape::setMass(double _m) {
        mMass = _m;
        computeMassTensor();
        computeInertiaFromMassTensor();
    }

    int Shape::mCounter = PRIMITIVE_MAGIC_NUMBER;

    void Shape::setMassTensorFromInertia() {
        // compute the half trace of mat = row*integral((x*x+y*y+z*z)dxdydz) = sum of moment of inertia along all 3 axes
	
        double halftrace = 0.5* mInertia.trace();
        for(int i=0; i<3; i++) {
            for(int j=0; j<3; j++) {
                mMassTensor(i, j) = -mInertia(i, j);
            }
        }
        mMassTensor(0, 0) += halftrace;
        mMassTensor(1, 1) += halftrace;
        mMassTensor(2, 2) += halftrace;
        mMassTensor(3, 3) = mMass;
    }

    void Shape::computeInertiaFromMassTensor(){
        double trace = mMassTensor(0, 0) + mMassTensor(1, 1) + mMassTensor(2, 2);
        for(int i=0; i<3; i++)
            for(int j=0; j<3; j++)
                mInertia(i, j) = -mMassTensor(i, j);
        mInertia(0, 0) += trace;
        mInertia(1, 1) += trace;
        mInertia(2, 2) += trace;
    }

} // namespace kinematics
