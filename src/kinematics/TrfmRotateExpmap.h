/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/

#ifndef KINEMATICS_TRFM_ROTATE_EXPMAP_H
#define KINEMATICS_TRFM_ROTATE_EXPMAP_H

#include "Transformation.h"

#include <cmath>
#include <cassert>

namespace kinematics {

    class TrfmRotateExpMap: public Transformation{
    public:
        TrfmRotateExpMap(Dof *x, Dof *y, Dof *z, const char* name = NULL);

        Eigen::Matrix4d getInvTransform();

        void applyGLTransform(renderer::RenderInterface* _ri) const;
        void computeTransform();
        Eigen::Matrix4d getDeriv(const Dof *);
        Eigen::Matrix4d getSecondDeriv(const Dof *q1, const Dof *q2);

    protected:

    };

} // namespace kinematics

#endif // KINEMATICS_TRFM_ROTATE_EXPMAP_H
