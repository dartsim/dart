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
#include <iostream>

using namespace std;
using namespace Eigen;

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

    void color4_to_float4(const aiColor4D *c, float f[4])
    {
        f[0] = c->r;
        f[1] = c->g;
        f[2] = c->b;
        f[3] = c->a;
    }

    void set_float4(float f[4], float a, float b, float c, float d)
    {
        f[0] = a;
        f[1] = b;
        f[2] = c;
        f[3] = d;
    }

    void applyMaterial(const struct aiMaterial *mtl)
    {
        float c[4];

        GLenum fill_mode;
        int ret1, ret2;
        aiColor4D diffuse;
        aiColor4D specular;
        aiColor4D ambient;
        aiColor4D emission;
        float shininess, strength;
        int two_sided;
        int wireframe;
        unsigned int max;

        set_float4(c, 0.8f, 0.8f, 0.8f, 1.0f);
        if(AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_DIFFUSE, &diffuse))
            color4_to_float4(&diffuse, c);
        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, c);

        set_float4(c, 0.0f, 0.0f, 0.0f, 1.0f);
        if(AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_SPECULAR, &specular))
            color4_to_float4(&specular, c);
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, c);

        set_float4(c, 0.2f, 0.2f, 0.2f, 1.0f);
        if(AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_AMBIENT, &ambient))
            color4_to_float4(&ambient, c);
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, c);

        set_float4(c, 0.0f, 0.0f, 0.0f, 1.0f);
        if(AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_EMISSIVE, &emission))
            color4_to_float4(&emission, c);
        glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, c);

        max = 1;
        ret1 = aiGetMaterialFloatArray(mtl, AI_MATKEY_SHININESS, &shininess, &max);
        if(ret1 == AI_SUCCESS) {
            max = 1;
            ret2 = aiGetMaterialFloatArray(mtl, AI_MATKEY_SHININESS_STRENGTH, &strength, &max);
            if(ret2 == AI_SUCCESS)
                glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shininess * strength);
            else
                glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shininess);
        }
        else {
            glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 0.0f);
            set_float4(c, 0.0f, 0.0f, 0.0f, 0.0f);
            glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, c);
        }

        max = 1;
        if(AI_SUCCESS == aiGetMaterialIntegerArray(mtl, AI_MATKEY_ENABLE_WIREFRAME, &wireframe, &max))
            fill_mode = wireframe ? GL_LINE : GL_FILL;
        else
            fill_mode = GL_FILL;
        glPolygonMode(GL_FRONT_AND_BACK, fill_mode);

        max = 1;
        if((AI_SUCCESS == aiGetMaterialIntegerArray(mtl, AI_MATKEY_TWOSIDED, &two_sided, &max)) && two_sided)
            glDisable(GL_CULL_FACE);
        else 
            glEnable(GL_CULL_FACE);
    }

    void OpenGLRenderInterface::drawMesh(const Vector3d& _size, const aiScene *_mesh) {
        const aiScene* scene = _mesh;
        glBegin(GL_TRIANGLES);
        for (unsigned int i = 0; i < scene->mNumMeshes; i++) {
            const aiMesh* mesh = scene->mMeshes[i];
            applyMaterial(scene->mMaterials[mesh->mMaterialIndex]);
            if(mesh->mNormals == NULL)
                glDisable(GL_LIGHTING);
            else
                glEnable(GL_LIGHTING);
            for (unsigned int j = 0; j < mesh->mNumFaces; j++) {
                const aiFace* face = &mesh->mFaces[j];
                for(unsigned int k = 0; k < face->mNumIndices; k++) {
                    int index = face->mIndices[k];
                    if(mesh->mColors[0] != NULL)
                        glColor4fv((GLfloat*)&mesh->mColors[0][index]);
                    if(mesh->mNormals != NULL) 
                        glNormal3fv(&mesh->mNormals[index].x);
                    glVertex3fv(&mesh->mVertices[index].x);
                }
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
