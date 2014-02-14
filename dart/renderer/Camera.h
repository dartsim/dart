/*
 * Copyright (c) 2011-2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jie (Jay) Tan <jtan34@cc.gatech.edu>
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

#ifndef DART_RENDERER_CAMERA_H
#define DART_RENDERER_CAMERA_H

#include <Eigen/Core>

namespace dart {
namespace renderer {

#define DEFAULT_NEAR_PLANE 0.001
#define DEFAULT_FAR_PLANE 15

enum AXIS {
    AXIS_X,
    AXIS_Y,
    AXIS_Z
};

class Camera {
public:
    Camera() {}
    virtual ~Camera() {}
    virtual void set(const Eigen::Vector3d& _eye, const Eigen::Vector3d& _look, const Eigen::Vector3d& _up);
    virtual void slide(double _delX, double _delY, double _delZ, bool _bLocal = false);
    virtual void setFrustum(float _vAng, float _asp, float _nearD, float _farD);
    virtual void setOrtho(float _width, float _height, float _nearD, float _farD);


    virtual void roll(float _angle);
    virtual void pitch(float _angle);
    virtual void yaw(float _angle);
    virtual void localRotate(float _angle, AXIS _axis);
    virtual void globalRotate(float _angle, AXIS _axis);

    virtual Eigen::Vector3d getEye(void) const {
        return Eigen::Vector3d(0.0f, 0.0f, 1.0f);
    }
    virtual Eigen::Vector3d getLookAtDir(void) const {
        return Eigen::Vector3d(0.0f, 0.0f, 0.0f);
    }
    virtual Eigen::Vector3d getUpDir(void) const {
        return Eigen::Vector3d(0.0f, 1.0f, 0.0f);

    }
    virtual bool isOrthogonal(void) const {
        return mIsOrthognal;
    }

    virtual float getVerticalViewAngle(void) const {
        return mViewAngle;
    }
    virtual float getNearDistance() const {
        return mNearDist;
    }
    virtual float getFarDistance() const {
        return mFarDist;
    }
    virtual float getWidth() const {
        return mWidth;
    }
    virtual float getHeight() const {
        return mHeight;
    }

protected:
    float mViewAngle, mAspect, mNearDist, mFarDist, mWidth, mHeight;
    bool mIsOrthognal;
};

} // namespace renderer
} // namespace dart

#endif // #ifndef DART_RENDERER_CAMERA_H
