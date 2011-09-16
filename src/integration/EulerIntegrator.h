/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Kristin Siu (kasiu)
  Date		09/16/2011
*/

#ifndef EULER_INTEGRATOR_H
#define EULER_INTEGRATOR_H

#include <Eigen/Dense>
#include "Integrator.h"

namespace integration {
    class EulerIntegrator : public Integrator {
    public:
        EulerIntegrator(){}
        ~EulerIntegrator(){}
        void integrate(IntegrableSystem* system, double dt) const;
    };
} // namespace integration

#endif // EULER_INTEGRATOR_H
