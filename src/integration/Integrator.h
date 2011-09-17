/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Kristin Siu (kasiu)
  Date		09/16/2011
*/

#ifndef INTEGRATOR_H
#define INTEGRATOR_H

#include <vector>
#include <Eigen/Dense>

namespace integration {
    // Any class that uses an integrator should implement this interface
    class IntegrableSystem {
    public:
        virtual Eigen::VectorXd getState() = 0;
        virtual void setState(Eigen::VectorXd) = 0;
        virtual Eigen::VectorXd evalDeriv() = 0;
    };

    // TODO (kasiu) Consider templating the class (which currently only works on arbitrarily-sized vectors of doubles)
    class Integrator {
    public:
        virtual void integrate(IntegrableSystem* system, double dt) const = 0;
    };
} // namespace integration

#endif // INTEGRATOR_H