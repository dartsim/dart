/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Kristin Siu (kasiu)
  Date		09/16/2011
*/

#include "EulerIntegrator.h"

using namespace Eigen;

namespace integration {
    void EulerIntegrator::integrate(IntegrableSystem* system, double dt) const {
        VectorXd deriv = system->evalDeriv();
        system->setState(system->getState() + (dt * deriv));
//        system->setState(x + (dt * system->evalDeriv()));
    }
} // namespace integration