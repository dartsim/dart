/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/

#ifndef KINEMATICS_TRFM_ROTATE_EULER_H
#define KINEMATICS_TRFM_ROTATE_EULER_H

#include "Transformation.h"
#include <cassert>

namespace kinematics {
    // rotate about x axis
    class TrfmRotateEulerX: public Transformation {
    public:
        TrfmRotateEulerX(Dof *x, const char *_name=NULL);

        Eigen::Matrix4d getInvTransform();
	
		void applyGLTransform(renderer::RenderInterface* _ri) const;
        void computeTransform();
        Eigen::Matrix4d getDeriv(const Dof *q);
        Eigen::Matrix4d getSecondDeriv(const Dof *q1, const Dof *q2);
    };

    // rotate about y axis
    class TrfmRotateEulerY: public Transformation {
    public:
        TrfmRotateEulerY(Dof *y, const char *_name=NULL);

        Eigen::Matrix4d getInvTransform();
	
        void applyGLTransform(renderer::RenderInterface* _ri) const;
        void computeTransform();
        Eigen::Matrix4d getDeriv(const Dof *q);
        Eigen::Matrix4d getSecondDeriv(const Dof *q1, const Dof *q2);
    };


// rotate about z axis
    class TrfmRotateEulerZ: public Transformation {
    public:
        TrfmRotateEulerZ(Dof *z, const char *_name=NULL);

        Eigen::Matrix4d getInvTransform();
	
        void applyGLTransform(renderer::RenderInterface* _ri) const;
        void computeTransform();
        Eigen::Matrix4d getDeriv(const Dof *q);
        Eigen::Matrix4d getSecondDeriv(const Dof *q1, const Dof *q2);
    };
} // namespace kinematics

#endif // #ifndef KINEMATICS_ROTATE_EULER_H

