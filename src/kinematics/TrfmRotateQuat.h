/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/

#ifndef KINEMATICS_TRFM_ROTATE_QUAT_H
#define KINEMATICS_TRFM_ROTATE_QUAT_H

#include "Transformation.h"

namespace kinematics {

    class TrfmRotateQuat: public Transformation {
    public:
        TrfmRotateQuat(Dof *w, Dof *x, Dof *y, Dof *z, const char *_name = NULL);
	
		void applyGLTransform(renderer::RenderInterface* _ri) const;
        void computeTransform();
        Eigen::Matrix4d getDeriv(const Dof *);
        Eigen::Matrix4d getSecondDeriv(const Dof *, const Dof *);
    };

} // namespace kinematics

#endif // #ifndef KINEMATICS_TRFM_ROTATE_QUAT_H

