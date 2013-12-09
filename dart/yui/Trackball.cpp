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

#include "Trackball.h"
#include "renderer/LoadOpengl.h"

namespace dart {
namespace yui {

void Trackball::startBall(double x, double y)
{
    mStartPos = mouseOnSphere(x,y);
}

void Trackball::updateBall(double x, double y)
{
    Eigen::Vector3d toPos = mouseOnSphere(x,y);
    Eigen::Quaterniond newQuat(quatFromVectors(mStartPos, toPos));
    mStartPos = toPos;
    mCurrQuat = newQuat*mCurrQuat;
}

void Trackball::applyGLRotation()
{
    Eigen::Transform<double, 3, Eigen::Affine> t(mCurrQuat);
    glMultMatrixd(t.data());
}

void Trackball::draw(int winWidth, int winHeight)
{
    glDisable( GL_LIGHTING );
    glDisable( GL_TEXTURE_2D );

    glPushMatrix();

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glViewport( 0, 0, winWidth, winHeight );
    gluOrtho2D(0.0, (GLdouble)winWidth, 0.0, (GLdouble)winHeight);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glLineWidth(4.0);
    glColor3f( 1.0f, 1.0f, 0.0f );
    glBegin( GL_LINE_LOOP);
    for(int i = 0; i < 360; i+=4){
        double theta = i / 180.0 * M_PI;
        double x = mRadius * cos(theta);
        double y = mRadius * sin(theta);
        glVertex2d( (GLdouble)((winWidth >> 1) + x), (GLdouble)((winHeight >> 1) + y));
    }
    glEnd();

    glPopMatrix();
}

Eigen::Vector3d Trackball::mouseOnSphere(double mouseX, double mouseY) const
{
    double mag;
    Eigen::Vector3d pointOnSphere;

    pointOnSphere(0) = (mouseX - mCenter(0)) / mRadius;
    pointOnSphere(1) = (mouseY - mCenter(1)) / mRadius;

    mag = pointOnSphere(0) * pointOnSphere(0) + pointOnSphere(1)*pointOnSphere(1);
    if (mag > 1.0)
    {
        register double scale = 1.0/sqrt(mag);
        pointOnSphere(0) *= scale;
        pointOnSphere(1) *= scale;
        pointOnSphere(2) = 0.0;
    }
    else
        pointOnSphere(2) = sqrt(1 - mag);

    return pointOnSphere;
}

Eigen::Quaterniond Trackball::quatFromVectors(const Eigen::Vector3d& from, const Eigen::Vector3d& to) const
{
    Eigen::Quaterniond quat;
    quat.x() = from(1)*to(2) - from(2)*to(1);
    quat.y() = from(2)*to(0) - from(0)*to(2);
    quat.z() = from(0)*to(1) - from(1)*to(0);
    quat.w() = from(0)*to(0) + from(1)*to(1) + from(2)*to(2);
    return quat;
}

} // namespace yui
} // namespace dart
