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

#include "ShapeEllipsoid.h"
#include "renderer/RenderInterface.h"

using namespace Eigen;

namespace kinematics {

    ShapeEllipsoid::ShapeEllipsoid(Vector3d _dim, double _mass){
        mType = P_ELLIPSOID;
        mDim = _dim;
        mMass = _mass;
        initMeshes();
        if (mDim != Vector3d::Zero())
            computeVolume();
        if (mMass != 0){
            computeMassTensor();
            computeInertiaFromMassTensor();
        }
    }

    void ShapeEllipsoid::draw(renderer::RenderInterface* _ri, const Vector4d& _color, bool _useDefaultColor) const {
        if (!_ri)
            return;
        if (!_useDefaultColor)
            _ri->setPenColor(_color);
        else
            _ri->setPenColor(mColor);
        _ri->pushMatrix();
        _ri->drawEllipsoid(mDim);
        _ri->popMatrix();
    }

    void ShapeEllipsoid::computeMassTensor() {
        mMassTensor(0, 0) = (mDim(0)*mDim(0))/20;
        mMassTensor(1, 1) = (mDim(1)*mDim(1))/20;
        mMassTensor(2, 2) = (mDim(2)*mDim(2))/20;
        mMassTensor(3, 3) = 1;
        mMassTensor *= mMass;
    }

    void ShapeEllipsoid::computeVolume() {
        mVolume = M_PI * mDim(0) * mDim(1) *mDim(2) /6;	//	4/3* Pi* a/2* b/2* c/2
    }
    
    void ShapeEllipsoid::initMeshes() {
        mVizMesh = NULL;
        mCollisionMesh = NULL;
    }

} // namespace kinematics
