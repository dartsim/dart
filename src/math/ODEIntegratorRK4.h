/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sumit Jain
  Date		07/28/2011
*/

#ifndef MATH_ODEINTEGRATORRK4_H
#define MATH_ODEINTEGRATORRK4_H

#include "ODEIntegrator.h"

namespace math{
    template<typename T_STATE> 
    class ODEIntegratorRK4: public ODEIntegrator<T_STATE>{
    private:
    public:
        ODEIntegratorRK4(){}
        ~ODEIntegratorRK4(){}

        // integrates over one time step dt at time t
        // takes in some auxData as input to be passed in deriv and/or used in step
        // takes the derivative as dqdt which saves a call of deriv
        // writes out the result in qnext
        // deriv writes out the result in dqdt
        void step(T_STATE &qcurr, T_STATE &qnext, T_STATE &dqdt, double t, double dt, void *_auxData, void (*deriv)(double t, T_STATE &q, T_STATE &dqdt, void *_auxData));
    };

}   // namespace math

#endif  // #ifndef MATH_ODEINTEGRATORRK2_H
