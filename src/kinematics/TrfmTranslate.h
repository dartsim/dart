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

#ifndef KINEMATICS_TRFM_TRANSLATE_H
#define KINEMATICS_TRFM_TRANSLATE_H

#include <cassert>
#include "Transformation.h"

namespace kinematics {
    class TrfmTranslate : public Transformation {
    public:
        TrfmTranslate(Dof *x, Dof *y, Dof *z, const char *_name = NULL);

        Eigen::Matrix4d getInvTransform();
        void applyTransform(Eigen::Vector3d&);
        void applyTransform(Eigen::Matrix4d&);
        void applyInvTransform(Eigen::Vector3d&);
        void applyInvTransform(Eigen::Matrix4d&);

        void applyGLTransform(renderer::RenderInterface* _ri) const;
        void computeTransform();

        Eigen::Matrix4d getDeriv(const Dof *q);
        void applyDeriv(const Dof* q, Eigen::Vector3d& v);
        void applyDeriv(const Dof* q, Eigen::Matrix4d& m);

        Eigen::Matrix4d getSecondDeriv(const Dof *q1, const Dof *q2);
        void applySecondDeriv(const Dof* q1, const Dof* q2, Eigen::Vector3d& v);
        void applySecondDeriv(const Dof* q1, const Dof* q2, Eigen::Matrix4d& m);
    };

    class TrfmTranslateX : public Transformation {
    public:
        TrfmTranslateX(Dof *x, const char *_name = NULL);

        Eigen::Matrix4d getInvTransform();
        void applyTransform(Eigen::Vector3d&);
        void applyTransform(Eigen::Matrix4d&);
        void applyInvTransform(Eigen::Vector3d&);
        void applyInvTransform(Eigen::Matrix4d&);

        void applyGLTransform(renderer::RenderInterface* _ri) const;
        void computeTransform();

        Eigen::Matrix4d getDeriv(const Dof *q);
        void applyDeriv(const Dof* q, Eigen::Vector3d& v);
        void applyDeriv(const Dof* q, Eigen::Matrix4d& m);

        Eigen::Matrix4d getSecondDeriv(const Dof *q1, const Dof *q2);
        void applySecondDeriv(const Dof* q1, const Dof* q2, Eigen::Vector3d& v);
        void applySecondDeriv(const Dof* q1, const Dof* q2, Eigen::Matrix4d& m);
    };

    class TrfmTranslateY : public Transformation {
    public:
        TrfmTranslateY(Dof *y, const char *_name = NULL);

        Eigen::Matrix4d getInvTransform();
        void applyTransform(Eigen::Vector3d&);
        void applyTransform(Eigen::Matrix4d&);
        void applyInvTransform(Eigen::Vector3d&);
        void applyInvTransform(Eigen::Matrix4d&);

        void applyGLTransform(renderer::RenderInterface* _ri) const;
        void computeTransform();

        Eigen::Matrix4d getDeriv(const Dof *q);
        void applyDeriv(const Dof* q, Eigen::Vector3d& v);
        void applyDeriv(const Dof* q, Eigen::Matrix4d& m);

        Eigen::Matrix4d getSecondDeriv(const Dof *q1, const Dof *q2);
        void applySecondDeriv(const Dof* q1, const Dof* q2, Eigen::Vector3d& v);
        void applySecondDeriv(const Dof* q1, const Dof* q2, Eigen::Matrix4d& m);
    };

    class TrfmTranslateZ : public Transformation {
    public:
        TrfmTranslateZ(Dof *z, const char *_name = NULL);

        Eigen::Matrix4d getInvTransform();
        void applyTransform(Eigen::Vector3d&);
        void applyTransform(Eigen::Matrix4d&);
        void applyInvTransform(Eigen::Vector3d&);
        void applyInvTransform(Eigen::Matrix4d&);

        void applyGLTransform(renderer::RenderInterface* _ri) const;
        void computeTransform();

        Eigen::Matrix4d getDeriv(const Dof *q);
        void applyDeriv(const Dof* q, Eigen::Vector3d& v);
        void applyDeriv(const Dof* q, Eigen::Matrix4d& m);

        Eigen::Matrix4d getSecondDeriv(const Dof *q1, const Dof *q2);
        void applySecondDeriv(const Dof* q1, const Dof* q2, Eigen::Vector3d& v);
        void applySecondDeriv(const Dof* q1, const Dof* q2, Eigen::Matrix4d& m);
    };
} // namespace kinematics

#endif // #ifndef KINEMATICS_TRFM_TRANSLATE_H

