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

#include "GLFuncs.h"
#include "renderer/LoadOpengl.h"
#include <string>
#include <iostream>
#include <Eigen/Eigen>
#include <cstdio>

namespace dart {
namespace yui {

void drawStringOnScreen(float x, float y, const std::string& s)
{ // draws text on the screen
    GLint oldMode;
    glGetIntegerv(GL_MATRIX_MODE, &oldMode);
    glMatrixMode( GL_PROJECTION );

    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D( 0.0, 1.0, 0.0, 1.0 );

    glMatrixMode( GL_MODELVIEW );
    glPushMatrix();
    glLoadIdentity();
    glRasterPos2f(x, y);
    unsigned int length = s.length();
    for (unsigned int c=0; c < length; c++)
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, s.at(c) );
    glPopMatrix();

    glMatrixMode( GL_PROJECTION );
    glPopMatrix();
    glMatrixMode(oldMode);
}

// draw a 3D arrow starting from pt along dir, the arrowhead is on the other end
void drawArrow3D(const Eigen::Vector3d& pt, const Eigen::Vector3d& dir, const double length, const double thickness, const double arrowThickness){
    Eigen::Vector3d normDir = dir;
    normDir.normalize();

    double arrowLength;
    if(arrowThickness == -1)
        arrowLength = 4*thickness;
    else arrowLength = 2*arrowThickness;

    // draw the arrow body as a cylinder
    GLUquadricObj *c;
    c = gluNewQuadric();
    gluQuadricDrawStyle(c, GLU_FILL);
    gluQuadricNormals(c, GLU_SMOOTH);

    glPushMatrix();
    glTranslatef(pt[0], pt[1], pt[2]);
    glRotated(acos(normDir[2])*180/M_PI, -normDir[1], normDir[0], 0);
    gluCylinder(c, thickness, thickness, length-arrowLength, 16, 16);

    // draw the arrowhed as a cone
    glPushMatrix();
    glTranslatef(0, 0, length-arrowLength);
    gluCylinder(c, arrowLength*0.5, 0.0, arrowLength, 10, 10);
    glPopMatrix();

    glPopMatrix();

    gluDeleteQuadric(c);
}

// draw a 2D arrow starting from pt along vec, the arrow head is on the other end
void drawArrow2D(const Eigen::Vector2d& pt, const Eigen::Vector2d& vec, double thickness)
{
    // draw the arrow body as a thick line
    glLineWidth(thickness);
    glBegin(GL_LINES);
    glVertex2f(pt[0],pt[1]);
    glVertex2f(pt[0]+vec[0],pt[1]+vec[1]);
    glEnd();

    // draw arrowhead as a triangle
    double theta = atan2(vec[1],vec[0]);
    glPushMatrix();
    glTranslatef(pt[0]+vec[0],pt[1]+vec[1],0.0);
    glRotatef(theta*180.0/M_PI,0.0,0.0,1.0);
    glTranslatef(thickness,0.0,0.0);
    glBegin(GL_TRIANGLES);
    glVertex2f(0.0,thickness);
    glVertex2f(2*thickness,0.0);
    glVertex2f(0.0,-thickness);
    glEnd();
    glPopMatrix();
}

void drawProgressBar(int currFrame, int totalFrame)
{
    GLint oldMode;
    glGetIntegerv(GL_MATRIX_MODE, &oldMode);
    glMatrixMode( GL_PROJECTION );

    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D( 0.0, 1.0, 0.0, 1.0 );

    glMatrixMode( GL_MODELVIEW );
    glPushMatrix();
    glLoadIdentity();

    glPolygonMode(GL_FRONT, GL_LINE);
    glColor4d(0.0,0.0,0.0,0.5);
    glBegin(GL_QUADS);
    glVertex2f(0.15f,0.02f);
    glVertex2f(0.85f,0.02f);
    glVertex2f(0.85f,0.08f);
    glVertex2f(0.15f,0.08f);
    glEnd();

    float portion = (float)currFrame/totalFrame;
    float end = 0.15f+portion*0.7f;
    glPolygonMode(GL_FRONT, GL_FILL);
    glColor4d(0.3,0.3,0.3,0.5);
    glBegin(GL_QUADS);
    glVertex2f(0.15f,0.02f);
    glVertex2f(end, 0.02f);
    glVertex2f(end, 0.08f);
    glVertex2f(0.15f,0.08f);
    glEnd();

    glPopMatrix();

    glMatrixMode( GL_PROJECTION );
    glPopMatrix();
    glMatrixMode(oldMode);
}

} // namespace yui
} // namespace dart
