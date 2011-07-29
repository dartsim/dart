/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sumit Jain
  Date		07/28/2011
*/

#include "ODEIntegratorRK2.h"

#include <Eigen/Dense>
using namespace Eigen;

namespace math{
    template<typename T_STATE>
    void ODEIntegratorRK2<T_STATE>::step(T_STATE &qcurr, T_STATE &qnext, T_STATE &dqdt, double t, double dt, void *_auxData, void (*deriv)(double t, T_STATE &q, T_STATE &dqdt, void *_auxData)){
        double dt_half = dt*0.5;
        double tnext= t+dt;
        double tnext_half = t+dt_half;

        // first step - derivs provided as dqdt
        T_STATE k1_half = dqdt*dt_half;
        // second step
        T_STATE k2(qcurr);
        T_STATE temp=qcurr+k1_half;	// just to pass as ref
        (*deriv)(tnext_half, temp, k2, _auxData);
        k2*=dt;

        // calculate the final result
        qnext = qcurr + k2;
    }

    template class ODEIntegratorRK2<double>;
    template class ODEIntegratorRK2<VectorXd>;
    template class ODEIntegratorRK2<MatrixXd>;

} // namespace math
