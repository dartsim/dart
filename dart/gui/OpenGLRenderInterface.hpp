/*
 * Copyright (c) 2011-2019, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#ifndef DART_GUI_OPENGLRENDERINTERFACE_HPP_
#define DART_GUI_OPENGLRENDERINTERFACE_HPP_

#include <list>
#include <vector>
#include "dart/gui/RenderInterface.hpp"
#include "dart/gui/LoadOpengl.hpp"

namespace dart {

namespace dynamics {
class Skeleton;
class BodyNode;
class Shape;
class ShapeFrame;
}

namespace gui {
class OpenGLRenderInterface : public RenderInterface {

public:
    OpenGLRenderInterface() : mViewportX(0.0), mViewportY(0.0), mViewportWidth(0.0), mViewportHeight(0.0) {}
    virtual ~OpenGLRenderInterface(){}

    void initialize() override;
    void destroy() override;

    void setViewport(int _x,int _y,int _width,int _height) override;
    void getViewport(int& _x, int& _y, int& _width, int& _height) const override;

    void clear(const Eigen::Vector3d& _color) override;

    void setMaterial(const Eigen::Vector3d& _diffuse, const Eigen::Vector3d& _specular, double _cosinePow) override;
    void getMaterial(Eigen::Vector3d& _diffuse, Eigen::Vector3d& _specular, double& _cosinePow) const override;
    void setDefaultMaterial() override;

    void pushMatrix() override;
    void popMatrix() override;
    void pushName(int _id) override;
    void popName() override;

    void translate(const Eigen::Vector3d& _offset) override; //glTranslate
    void rotate(const Eigen::Vector3d& _axis, double _rad) override; //glRotate
    void transform(const Eigen::Isometry3d& _transform) override; //glMultMatrix
    void scale(const Eigen::Vector3d& _scale) override; //glScale

    void compileList(dynamics::Skeleton* _skel);
    void compileList(dynamics::BodyNode* _node);
    void compileList(dynamics::Shape* _shape);
    GLuint compileList(const Eigen::Vector3d& _scale, const aiScene* _mesh);

    void drawSphere(double radius, int slices = 16, int stacks = 16) override;
    void drawMultiSphere(const std::vector<std::pair<double, Eigen::Vector3d>>& spheres, int slices = 16, int stacks = 16) override;
    void drawEllipsoid(const Eigen::Vector3d& _diameters) override;
    void drawCube(const Eigen::Vector3d& _size) override;
    void drawOpenCylinder(double baseRadius, double topRadius, double height, int slices = 16, int stacks = 16) override;
    void drawCylinder(double _radius, double _height, int slices = 16, int stacks = 16) override;
    void drawCapsule(double radius, double height) override;
    void drawCone(double radius, double height) override;
    void drawMesh(const Eigen::Vector3d& _scale, const aiScene* _mesh) override;
    void drawSoftMesh(const aiMesh* mesh) override;
    void drawList(GLuint index) override;
    void drawLineSegments(const std::vector<Eigen::Vector3d>& _vertices,
                          const common::aligned_vector<Eigen::Vector2i>& _connections) override;

    void setPenColor(const Eigen::Vector4d& _col) override;
    void setPenColor(const Eigen::Vector3d& _col) override;

    void setLineWidth(float _width) override;

    void saveToImage(const char* _filename, DecoBufferType _buffType = BT_Back) override;
    void readFrameBuffer(DecoBufferType _buffType, DecoColorChannel _ch, void* _pixels) override;

private:
    void color4_to_float4(const aiColor4D *c, float f[4]);
    void set_float4(float f[4], float a, float b, float c, float d);
    void applyMaterial(const struct aiMaterial *mtl);
    void recursiveRender(const struct aiScene *sc, const struct aiNode* nd);

    int mViewportX, mViewportY, mViewportWidth, mViewportHeight;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace gui
} // namespace dart

#endif // #ifndef DART_GUI_OPENGLRENDERINTERFACE_HPP_
