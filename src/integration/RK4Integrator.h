/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Kristin Siu (kasiu)
  Date		09/16/2011
*/

#ifndef RK4_INTEGRATOR_H
#define RK4_INTEGRATOR_H

#include <Eigen/Dense>
#include "Integrator.h"

namespace integration {
    class RK4Integrator : public Integrator {
    public:
        RK4Integrator(){}
        ~RK4Integrator(){}
        void integrate(IntegrableSystem* system, double dt) const;
    private:
        mutable Eigen::VectorXd k1, k2, k3, k4;
    };
} // namespace integration

#endif // RK4_INTEGRATOR_H