#ifndef _ODEINT_
#define _ODEINT_

template<typename T_STATE> 
class ODEInt{
protected:
public:
	ODEInt(){}
	virtual ~ODEInt(){}

	// integrates over one time step dt at time t
	// takes in some auxData as input to be passed in deriv and/or used in step
	// takes the derivative as dqdt which saves a call of deriv
	// writes out the result in qnext
	// deriv writes out the result in dqdt
	virtual void step(T_STATE &qcurr, T_STATE &qnext, T_STATE &dqdt, double t, double dt, void *_auxData, void (*deriv)(double t, T_STATE &q, T_STATE &dqdt, void *_auxData))=0;
};

#endif