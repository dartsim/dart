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

ShapeEllipsoid::ShapeEllipsoid(Vector3d _dim)
    : Shape(P_ELLIPSOID)
{
    mDim = _dim;
    initMeshes();
    if (mDim != Vector3d::Zero())
        computeVolume();
}

void ShapeEllipsoid::draw(renderer::RenderInterface* _ri, const Vector4d& _color, bool _useDefaultColor) const {
    if (!_ri)
        return;
    if (!_useDefaultColor)
        _ri->setPenColor(_color);
    else
        _ri->setPenColor(mColor);
    _ri->pushMatrix();
    _ri->transform(mTransform);
    _ri->drawEllipsoid(mDim);
    _ri->popMatrix();
}

Matrix3d ShapeEllipsoid::computeInertia(double _mass) {
    Matrix3d inertia = Matrix3d::Zero();
    inertia(0, 0) = _mass / 20.0 * (mDim(1) * mDim(1) + mDim(2) * mDim(2));
    inertia(1, 1) = _mass / 20.0 * (mDim(0) * mDim(0) + mDim(2) * mDim(2));
    inertia(2, 2) = _mass / 20.0 * (mDim(0) * mDim(0) + mDim(1) * mDim(1));

    return inertia;
}

bool ShapeEllipsoid::isSphere() const {
    if (mDim[0] == mDim[1] && mDim[1] == mDim[2])
        return true;
    else
        return false;
}

void ShapeEllipsoid::computeVolume() {
    mVolume = M_PI * mDim(0) * mDim(1) *mDim(2) /6;	//	4/3* Pi* a/2* b/2* c/2
}
void ShapeEllipsoid::initMeshes() {
}

} // namespace kinematics
