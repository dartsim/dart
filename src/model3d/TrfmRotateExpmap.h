/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/

#ifndef MODEL3D_TRFM_ROTATE_EXPMAP_H
#define MODEL3D_TRFM_ROTATE_EXPMAP_H

#include "Transformation.h"

#include <cmath>
#include <cassert>

namespace model3d {

    class TrfmRotateExpMap: public Transformation{
    public:
        TrfmRotateExpMap(Dof *x, Dof *y, Dof *z, char* name = NULL);

        Eigen::Matrix4d getInvTransform();

        void applyGLTransform() const;
        void evalTransform();
        Eigen::Matrix4d getDeriv(const Dof *);
        Eigen::Matrix4d getDeriv2(const Dof *q1, const Dof *q2);
	
    private:
        // some math utility functions
        double get_dq_dv(int i, int j, double theta, Eigen::Vector3d v, Eigen::Vector3d vhat);
        double get_dq_dv_approx(int i, int j, double theta, Eigen::Vector3d v);
        double get_dq_dv_dv(int i, int j, int k, double theta, Eigen::Vector3d vhat);
        double get_dq_dv_dv_approx(int i, int j, int k, double theta, Eigen::Vector3d v);
    };
} // namespace model3d

#endif // MODEL3D_TRFM_ROTATE_EXPMAP_H
