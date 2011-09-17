/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Kristin Siu (kasiu)
  Date		09/16/2011
*/

#include "RK4Integrator.h"

using namespace Eigen;

namespace integration {
    // TODO (kasiu): Slow. Needs to be optimized.
    void RK4Integrator::integrate(IntegrableSystem* system, double dt) const {
        // calculates the four weighted deltas
        VectorXd deriv = system->evalDeriv();
        VectorXd x = system->getState();
        k1 = deriv * dt;

        system->setState(x + (k1 * 0.5));
        deriv = system->evalDeriv();
        k2 = deriv * dt;

        system->setState(x + (k2 * 0.5));
        deriv = system->evalDeriv();
        k3 = deriv * dt;

        system->setState(x + k3);
        deriv = system->evalDeriv();
        k4 = deriv * dt;

        system->setState(x + ((1.0/6.0) * (k1 + (2.0 * k2) + (2.0 * k3) + k4)));
    }
} // namespace integration