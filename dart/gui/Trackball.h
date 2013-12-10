/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sumit Jain <sumit@cc.gatech.edu>
 * Date: 07/21/2011
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

#ifndef DART_YUI_TRACKBALL_H
#define DART_YUI_TRACKBALL_H

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace dart {
namespace yui {

class Trackball
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Trackball(): mCenter(0.0,0.0), mRadius(1.0), mStartPos(0.0,0.0,0.0),mCurrQuat(1.0,0.0,0.0,0.0) {}
    Trackball(const Eigen::Vector2d& center, double radius)
        : mCenter(center), mRadius(radius), mStartPos(0.0,0.0,0.0), mCurrQuat(1.0,0.0,0.0,0.0) {}

    // set the starting position to the project of (x,y) on the trackball
    void startBall(double x, double y);
    // update the current rotation to rotate from mStartPos to the projection of (x,y) on trackball, then update mStartPos
    void updateBall(double x, double y);

    // apply the current rotation to openGL env
    void applyGLRotation();
    // draw the trackball on screen
    void draw(int winWidth, int winHeight);

    void setTrackball(const Eigen::Vector2d& center, const double radius){ mCenter = center, mRadius = radius; }
    void setCenter(const Eigen::Vector2d& center){ mCenter = center; }
    void setRadius( const double radius ){ mRadius = radius; }
    void setQuaternion(const Eigen::Quaterniond& q){ mCurrQuat = q; }
    Eigen::Quaterniond getCurrQuat() const { return mCurrQuat; }
    Eigen::Matrix3d getRotationMatrix() const { return mCurrQuat.toRotationMatrix(); }
    Eigen::Vector2d getCenter() const { return mCenter; }
    double getRadius() const { return mRadius; }

private:
    // project screen coordinate (x,y) to the trackball
    Eigen::Vector3d mouseOnSphere(double mouseX, double mouseY) const;
    // compute the quaternion that rotates from vector "from" to vector "to"
    Eigen::Quaterniond quatFromVectors(const Eigen::Vector3d& from, const Eigen::Vector3d& to) const;

    Eigen::Vector2d mCenter;
    double mRadius;
    Eigen::Vector3d mStartPos;
    Eigen::Quaterniond mCurrQuat;
};

} // namespace yui
} // namespace dart

#endif // #ifndef DART_YUI_TRACKBALL_H
