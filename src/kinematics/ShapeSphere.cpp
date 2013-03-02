/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Tobias Kunz <tobias@gatech.edu>
 * Date: 01/29/2013
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

#include "ShapeSphere.h"
#include "renderer/RenderInterface.h"

using namespace std;
using namespace Eigen;

namespace kinematics {

    ShapeSphere::ShapeSphere(double _radius)
    	: Shape(P_SPHERE, 0),
    	  mRadius(_radius)
    {
        if (_radius > 0.0) {
            computeVolume();
        }
    }

    ShapeSphere::ShapeSphere(double _radius, double _mass)
		: Shape(P_SPHERE, _mass),
		  mRadius(_radius)
	{
		if (_radius > 0.0) {
			computeVolume();
		}
	}

    void ShapeSphere::draw(renderer::RenderInterface* _ri, const Vector4d& _color, bool _useDefaultColor) const {
        // TODO
    }

    void ShapeSphere::computeVolume() {
        mVolume = 4.0 / 3.0 * M_PI * mRadius * mRadius * mRadius;
    }

    Matrix3d ShapeSphere::computeInertia(double _mass) {
        return 0.4 * _mass * mRadius * mRadius * Matrix3d::Identity();
    }

} // namespace kinematics
