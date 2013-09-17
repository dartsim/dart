/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>
 * Date: 06/12/2011
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

#ifndef DART_DYNAMICS_SHAPE_H
#define DART_DYNAMICS_SHAPE_H

#include <Eigen/Dense>

#include "math/Geometry.h"

extern "C" { struct aiScene; }

namespace dart {
namespace renderer { class RenderInterface; }
namespace dynamics {

/// @brief
class Shape
{
public:
    /// @brief
    enum ShapeType {
        P_UNDEFINED,
        P_BOX,
        P_ELLIPSOID,
        P_CYLINDER,
        P_MESH
    };

    /// @brief
    Shape(ShapeType _type = P_UNDEFINED);

    /// @brief
    virtual ~Shape();

    /// @brief
    void setColor(const Eigen::Vector3d& _color);

    /// @brief
    const Eigen::Vector3d& getColor() const;

    /// @brief
    void setDim(const Eigen::Vector3d& _dim);

    /// @brief
    const Eigen::Vector3d& getDim() const;

    /// @brief Set local transformation of the Shape w.r.t. parent frame.
    void setLocalTransform(const Eigen::Isometry3d& _Transform);

    /// @brief Get local transformation of the Shape w.r.t. parent frame.
    const Eigen::Isometry3d& getLocalTransform() const;

    /// @brief
    void setOffset(const Eigen::Vector3d& _offset);

    /// @brief
    Eigen::Vector3d getOffset() const;

    /// @brief
    virtual Eigen::Matrix3d computeInertia(double _mass) const = 0;

    /// @brief
    void setVolume(double _v);

    /// @brief
    double getVolume() const;

    /// @brief
    int getID() const;

    /// @brief
    ShapeType getShapeType() const;

    /// @brief
    virtual void draw(renderer::RenderInterface* _ri = NULL,
                      const Eigen::Vector4d& _color = Eigen::Vector4d::Ones(),
                      bool _useDefaultColor = true) const = 0;

protected:
    /// @brief
    virtual void computeVolume() = 0;

    /// @brief
    virtual void initMeshes() {}

    /// @brief Type of primitive; unknown in the general case.
    ShapeType mType;

    /// @brief Dimensions for bounding box.
    Eigen::Vector3d mDim;

    /// @brief Volume enclosed by the geometry.
    double mVolume;

    /// @brief Unique id.
    int mID;

    /// @brief Color for the primitive.
    Eigen::Vector3d mColor;

    /// @brief The origin of this primitive in the bodynode frame.
    Eigen::Vector3d mOffset;

    /// @brief Local geometric transformation of the Shape w.r.t. parent frame.
    Eigen::Isometry3d mTransform;

    /// @brief
    static int mCounter;

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace dynamics
} // namespace dart

#endif // #ifndef DART_DYNAMICS_SHAPE_H

