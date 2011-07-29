/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sumit Jain
  Date		07/28/2011
*/
#include "ODEIntegratorRK4.h"

#define ONE_BY_THREE 1.0/3.0
#include <Eigen/Dense>
using namespace Eigen;

namespace math{

    template<typename T_STATE>
    void ODEIntegratorRK4<T_STATE>::step(T_STATE &qcurr, T_STATE &qnext, T_STATE &dqdt, double t, double dt, void *_auxData, void (*deriv)(double t, T_STATE &q, T_STATE &dqdt, void *_auxData)){
        double dt_half = dt*0.5;
        double tnext= t+dt;
        double tnext_half = t+dt_half;

        // first step - derivs provided as dqdt
        T_STATE k1_half = dqdt*dt_half;
        // second step
        T_STATE k2_half(qcurr);
        T_STATE temp=qcurr+k1_half;	// just to pass as ref
        (*deriv)(tnext_half, temp, k2_half, _auxData);
        k2_half*=dt_half;
        // third step
        T_STATE k3(qcurr);
        temp=qcurr+k2_half;	// just to pass as ref
        (*deriv)(tnext_half, temp, k3, _auxData);
        k3*=dt;
        // fourth step
        T_STATE k4_half(qcurr);
        temp=qcurr+k3;	// just to pass as ref
        (*deriv)(tnext, temp, k4_half, _auxData);
        k4_half*=dt_half;

        // calculate the final result
        qnext = qcurr + (k1_half + k2_half*2 + k3 + k4_half)*ONE_BY_THREE;
    }

    template class ODEIntegratorRK4<double>;
    template class ODEIntegratorRK4<VectorXd>;
    template class ODEIntegratorRK4<MatrixXd>;
}   // namespace math
