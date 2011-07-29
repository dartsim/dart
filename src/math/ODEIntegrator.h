/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sumit Jain
  Date		07/28/2011
*/
#ifndef MATH_ODEINTEGRATOR_H
#define MATH_ODEINTEGRATOR_H

namespace math{

    template<typename T_STATE> 
    class ODEIntegrator{
    protected:
    public:
        ODEIntegrator(){}
        virtual ~ODEIntegrator(){}

        // integrates over one time step dt at time t
        // takes in some auxData as input to be passed in deriv and/or used in step
        // takes the derivative as dqdt which saves a call of deriv
        // writes out the result in qnext
        // deriv writes out the result in dqdt
        virtual void step(T_STATE &qcurr, T_STATE &qnext, T_STATE &dqdt, double t, double dt, void *_auxData, void (*deriv)(double t, T_STATE &q, T_STATE &dqdt, void *_auxData))=0;
    };

}   // namespace math

#endif  // MATH_ODEINTEGRATOR_H
