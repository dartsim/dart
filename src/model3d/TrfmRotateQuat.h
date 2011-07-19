/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/

#ifndef MODEL3D_TRFM_ROTATE_QUAT_H
#define MODEL3D_TRFM_ROTATE_QUAT_H

#include "Transformation.h"

namespace model3d {

    class TrfmRotateQuat: public Transformation {
    public:
        TrfmRotateQuat(Dof *w, Dof *x, Dof *y, Dof *z, char *_name = NULL);
	
		void applyGLTransform(renderer::RenderInterface* _ri) const;
        void evalTransform();
        Eigen::Matrix4d getDeriv(const Dof *);
        Eigen::Matrix4d getSecondDeriv(const Dof *, const Dof *);
    };

} // namespace model3d

#endif // #ifndef MODEL3D_TRFM_ROTATE_QUAT_H

