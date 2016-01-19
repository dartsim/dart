/*
 * Copyright (c) 2011-2015, Georgia Tech Research Corporation
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

#ifndef KIDO_RENDERER_OPENGLRENDERINTERFACE_H
#define KIDO_RENDERER_OPENGLRENDERINTERFACE_H

#include <list>
#include <vector>
#include "RenderInterface.hpp"
#include "kido/renderer/LoadOpengl.hpp"

namespace kido {

namespace dynamics {
class Skeleton;
class BodyNode;
class Shape;
}

namespace renderer {
class OpenGLRenderInterface : public RenderInterface {

public:
    OpenGLRenderInterface() : mViewportX(0.0), mViewportY(0.0), mViewportWidth(0.0), mViewportHeight(0.0) {}
    virtual ~OpenGLRenderInterface(){}

    virtual void initialize() override;
    virtual void destroy() override;

    virtual void setViewport(int _x,int _y,int _width,int _height) override;
    virtual void getViewport(int& _x, int& _y, int& _width, int& _height) const override;

    virtual void clear(const Eigen::Vector3d& _color) override;

    virtual void setMaterial(const Eigen::Vector3d& _diffuse, const Eigen::Vector3d& _specular, double _cosinePow) override;
    virtual void getMaterial(Eigen::Vector3d& _diffuse, Eigen::Vector3d& _specular, double& _cosinePow) const override;
    virtual void setDefaultMaterial() override;

    virtual void pushMatrix() override;
    virtual void popMatrix() override;
    virtual void pushName(int _id) override;
    virtual void popName() override;

    virtual void translate(const Eigen::Vector3d& _offset) override; //glTranslate
    virtual void rotate(const Eigen::Vector3d& _axis, double _rad) override; //glRotate
    virtual void transform(const Eigen::Isometry3d& _transform) override; //glMultMatrix
    virtual void scale(const Eigen::Vector3d& _scale) override; //glScale

    void compileList(dynamics::Skeleton* _skel);
    void compileList(dynamics::BodyNode* _node);
    void compileList(dynamics::Shape* _shape);
    GLuint compileList(const Eigen::Vector3d& _scale, const aiScene* _mesh);

    virtual void draw(dynamics::Skeleton* _skel, bool _vizCol = false, bool _colMesh = false);
    virtual void draw(dynamics::BodyNode* _node, bool _vizCol = false, bool _colMesh = false);
    virtual void draw(dynamics::Shape* _shape);

    virtual void drawEllipsoid(const Eigen::Vector3d& _size) override;
    virtual void drawCube(const Eigen::Vector3d& _size) override;
    virtual void drawCylinder(double _radius, double _height) override;
    virtual void drawMesh(const Eigen::Vector3d& _scale, const aiScene* _mesh) override;
    virtual void drawList(GLuint index) override;
    virtual void drawLineSegments(const std::vector<Eigen::Vector3d>& _vertices,
                                  const Eigen::aligned_vector<Eigen::Vector2i>& _connections) override;

    virtual void setPenColor(const Eigen::Vector4d& _col) override;
    virtual void setPenColor(const Eigen::Vector3d& _col) override;

    virtual void setLineWidth(float _width) override;

    virtual void saveToImage(const char* _filename, DecoBufferType _buffType = BT_Back) override;
    virtual void readFrameBuffer(DecoBufferType _buffType, DecoColorChannel _ch, void* _pixels) override;

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
} // namespace kido

#endif // #ifndef KIDO_RENDERER_OPENGLRENDERINTERFACE_H
