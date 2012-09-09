/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jie (Jay) Tan <jtan34@cc.gatech.edu>
 * Date: 07/18/2011
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

#include "OpenGLRenderInterface.h"
#include "utils/LoadOpengl.h"
#include "geometry/Mesh3D.h"
#include <iostream>

using namespace std;
using namespace Eigen;
using namespace geometry;

namespace renderer {

    void OpenGLRenderInterface::initialize() {
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glCullFace(GL_FRONT);
        glDisable(GL_LIGHTING);
        glEnable(GL_DEPTH_TEST);
        //glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
        glShadeModel(GL_SMOOTH);
        clear(Vector3d(1.0, 1.0, 1.0));
    }

    void OpenGLRenderInterface::destroy() {

    }

    void OpenGLRenderInterface::setViewport(int _x,int _y,int _width,int _height) {
        glViewport(_x, _y, _width, _height);
        mViewportX = _x;
        mViewportY = _y;
        mViewportWidth = _width;
        mViewportHeight = _height;
    }

    void OpenGLRenderInterface::getViewport(int& _x, int& _y, int& _width, int& _height) const {
        _x = mViewportX;
        _y = mViewportY;
        _width = mViewportWidth;
        _height =mViewportHeight;
    }

    void OpenGLRenderInterface::clear(const Vector3d& _color) {
        glClearColor((GLfloat)_color[0], (GLfloat)_color[1], (GLfloat)_color[2], 1.0);
        glClear(GL_COLOR_BUFFER_BIT);
    }

    void OpenGLRenderInterface::setDefaultLight() {

    }

    void OpenGLRenderInterface::turnLightsOff() {
        glDisable(GL_LIGHTING);
    }

    void OpenGLRenderInterface::turnLightsOn() {
        //not finished yet
        glEnable(GL_LIGHTING);	
    }

    void OpenGLRenderInterface::setMaterial(const Vector3d& _diffuse, const Vector3d& _specular, double _cosinePow) {
        
    }

    void OpenGLRenderInterface::getMaterial(Vector3d& _diffuse, Vector3d& _specular, double& _cosinePow) const {
        
    }

    void OpenGLRenderInterface::setDefaultMaterial() {
        
    }

    void OpenGLRenderInterface::pushMatrix() {
        glPushMatrix();
    }

    void OpenGLRenderInterface::popMatrix() {
        glPopMatrix();
    }

    void OpenGLRenderInterface::pushName(int _id) {
        glPushName(_id);
    }

    void OpenGLRenderInterface::popName() {
        glPopName();
    }

    void OpenGLRenderInterface::translate(const Vector3d& _offset) {
        glTranslated(_offset[0], _offset[1], _offset[2]);
    }

    void OpenGLRenderInterface::rotate(const Vector3d& _axis, double _rad) {
        glRotated(_rad, _axis[0], _axis[1], _axis[2]);
    }

    void OpenGLRenderInterface::scale(const Vector3d& _scale) {
        glScaled(_scale[0], _scale[1], _scale[2]);
    }

    void OpenGLRenderInterface::drawEllipsoid(const Vector3d& _size) {
        glScaled(_size(0), _size(1), _size(2));
        glutSolidSphere(0.5, 16, 16);
    }

    void OpenGLRenderInterface::drawCube(const Vector3d& _size) {
        glScaled(_size(0), _size(1), _size(2));
        glutSolidCube(1.0);
    }
    

    void OpenGLRenderInterface::drawMesh(const Vector3d& _size, const geometry::Mesh3D *_mesh) {        

        glBegin(GL_TRIANGLES);
        for (unsigned int i = 0; i < _mesh->mNumFaces; i++) {
            if (_mesh->mVertexNormals.size() > 0) {
                int ni0 = _mesh->mFaceNormalIndices[i][0] - 1;
                int ni1 = _mesh->mFaceNormalIndices[i][1] - 1;
                int ni2 = _mesh->mFaceNormalIndices[i][2] - 1;
                Vector3d n0 = _mesh->mVertexNormals[ni0];
                Vector3d n1 = _mesh->mVertexNormals[ni1];
                Vector3d n2 = _mesh->mVertexNormals[ni2];

                int pi0 = _mesh->mFaces[i * 3];
                int pi1 = _mesh->mFaces[i * 3 + 1];
                int pi2 = _mesh->mFaces[i * 3 + 2];

                Vector3d p0(_mesh->mVertexPos[pi0 * 3] * _size[0], _mesh->mVertexPos[pi0 * 3 + 1] * _size[1], _mesh->mVertexPos[pi0 * 3 + 2] * _size[2]);
                Vector3d p1(_mesh->mVertexPos[pi1 * 3] * _size[0], _mesh->mVertexPos[pi1 * 3 + 1] * _size[1], _mesh->mVertexPos[pi1 * 3 + 2] * _size[2]);
                Vector3d p2(_mesh->mVertexPos[pi2 * 3] * _size[0], _mesh->mVertexPos[pi2 * 3 + 1] * _size[1], _mesh->mVertexPos[pi2 * 3 + 2] * _size[2]);

                glNormal3f(n0[0], n0[1], n0[2]);
                glVertex3f(p0[0], p0[1], p0[2]);
                glNormal3f(n1[0], n1[1], n1[2]);
                glVertex3f(p1[0], p1[1], p1[2]);
                glNormal3f(n2[0], n2[1], n2[2]);
                glVertex3f(p2[0], p2[1], p2[2]);
            } else {
                int pi0 = _mesh->mFaces[i * 3];
                int pi1 = _mesh->mFaces[i * 3 + 1];
                int pi2 = _mesh->mFaces[i * 3 + 2];

                Vector3d p0(_mesh->mVertexPos[pi0 * 3] * _size[0], _mesh->mVertexPos[pi0 * 3 + 1] * _size[1], _mesh->mVertexPos[pi0 * 3 + 2] * _size[2]);
                Vector3d p1(_mesh->mVertexPos[pi1 * 3] * _size[0], _mesh->mVertexPos[pi1 * 3 + 1] * _size[1], _mesh->mVertexPos[pi1 * 3 + 2] * _size[2]);
                Vector3d p2(_mesh->mVertexPos[pi2 * 3] * _size[0], _mesh->mVertexPos[pi2 * 3 + 1] * _size[1], _mesh->mVertexPos[pi2 * 3 + 2] * _size[2]);


                glVertex3f(p0[0], p0[1], p0[2]);
                glVertex3f(p1[0], p1[1], p1[2]);
                glVertex3f(p2[0], p2[1], p2[2]);
            }
        }
        glEnd();       
    }


    void OpenGLRenderInterface::setPenColor(const Vector4d& _col) {
        glColor4d(_col[0], _col[1], _col[2], _col[3]);
    }

    void OpenGLRenderInterface::setPenColor(const Vector3d& _col) {
        glColor4d(_col[0], _col[1], _col[2], 1.0);
    }

    void OpenGLRenderInterface::readFrameBuffer(DecoBufferType _buffType, DecoColorChannel _ch, void *_pixels) {

    }
    
    void OpenGLRenderInterface::saveToImage(const char *_filename, DecoBufferType _buffType) {

    }

} // namespace renderer
