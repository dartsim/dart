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

#ifndef DART_RENDERER_OPENGLRENDERINTERFACE_H
#define DART_RENDERER_OPENGLRENDERINTERFACE_H

#include <list>
#include <vector>
#include "RenderInterface.h"
#include "dart/renderer/LoadOpengl.h"

namespace dart {

namespace dynamics {
class Skeleton;
class BodyNode;
class Shape;
}

namespace renderer {
class OpenGLRenderInterface : public RenderInterface {

public:
    OpenGLRenderInterface() {}
    virtual ~OpenGLRenderInterface(){}

    virtual void initialize();
    virtual void destroy();

    virtual void setViewport(int _x,int _y,int _width,int _height);
    virtual void getViewport(int& _x, int& _y, int& _width, int& _height) const;

    virtual void clear(const Eigen::Vector3d& _color);

    virtual void setDefaultLight();
    virtual void turnLightsOff();
    virtual void turnLightsOn();

    virtual void setMaterial(const Eigen::Vector3d& _diffuse, const Eigen::Vector3d& _specular, double _cosinePow);
    virtual void getMaterial(Eigen::Vector3d& _diffuse, Eigen::Vector3d& _specular, double& _cosinePow) const;
    virtual void setDefaultMaterial();

    virtual void pushMatrix();
    virtual void popMatrix();
    virtual void pushName(int _id);
    virtual void popName();

    virtual void translate(const Eigen::Vector3d& _offset); //glTranslate
    virtual void rotate(const Eigen::Vector3d& _axis, double _rad); //glRotate
    virtual void transform(const Eigen::Isometry3d& _transform); //glMultMatrix
    virtual void scale(const Eigen::Vector3d& _scale); //glScale

    void compileList(dynamics::Skeleton *_skel);
    void compileList(dynamics::BodyNode *_node);
    void compileList(dynamics::Shape *_shape);
    GLuint compileList(const Eigen::Vector3d& _scale, const aiScene *_mesh);

    virtual void draw(dynamics::Skeleton *_skel, bool _vizCol = false, bool _colMesh = false);
    virtual void draw(dynamics::BodyNode *_node, bool _vizCol = false, bool _colMesh = false);
    virtual void draw(dynamics::Shape *_shape);

    virtual void drawEllipsoid(const Eigen::Vector3d& _size);
    virtual void drawCube(const Eigen::Vector3d& _size);
    virtual void drawCylinder(double _radius, double _height);
    virtual void drawMesh(const Eigen::Vector3d& _scale, const aiScene *_mesh);
    virtual void drawList(GLuint index);

    virtual void setPenColor(const Eigen::Vector4d& _col);
    virtual void setPenColor(const Eigen::Vector3d& _col);

    virtual void saveToImage(const char *_filename, DecoBufferType _buffType = BT_Back);
    virtual void readFrameBuffer(DecoBufferType _buffType, DecoColorChannel _ch, void *_pixels);

private:
    void color4_to_float4(const aiColor4D *c, float f[4]);
    void set_float4(float f[4], float a, float b, float c, float d);
    void applyMaterial(const struct aiMaterial *mtl);
    void recursiveRender(const struct aiScene *sc, const struct aiNode* nd);

    int mViewportX, mViewportY, mViewportWidth, mViewportHeight;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace renderer
} // namespace dart

#endif // #ifndef DART_RENDERER_OPENGLRENDERINTERFACE_H
