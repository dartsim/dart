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

#ifndef DART_RENDERER_LIGHT_H
#define DART_RENDERER_LIGHT_H

#include <Eigen/Core>

namespace dart {
namespace renderer {

enum LightType {
    LT_PointLight,
    LT_DirectionLight,
};

class Light {
public:
    /* construction function */
    Light();
    Light(LightType type);
    Light(const Eigen::Vector3d& _pos, const Eigen::Vector3d& _diffuse, const Eigen::Vector3d& _specular, LightType _type);

    LightType GetType() const;
    void SetPosition(const Eigen::Vector3d& _p);
    void GetPosition(float *_pos) const;
    void GetSpecular(float *_specular) const;
    void GetDiffuse(float *_diffuse) const;
    Eigen::Vector3d GetPosition() const;
    Eigen::Vector3d GetSpecular() const;
    Eigen::Vector3d GetDiffuse() const;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
        Eigen::Vector3d mPosition;
    Eigen::Vector3d mSpecular;
    Eigen::Vector3d mDiffuse;
    LightType mType;
};

} // namespace renderer
} // namespace dart

#endif // #ifndef DART_RENDERER_LIGHT_H
