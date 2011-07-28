#ifndef _SPLINEBUNIFORM_
#define _SPLINEBUNIFORM_

#include "Spline.h"
#include "moremath.h"
#include <cassert>

class SplineBUniform: public Spline{
protected:
	Mat4d mBasisMat;

	void getParameter(double p, Vec2d *p0, Vec2d *p1, Vec2d *p2, Vec2d *p3, double *t);
	Vec2d evalCurvePoint(Vec2d *p0, Vec2d *p1, Vec2d *p2, Vec2d *p3, double t);
	Vec2d evalCurveDer(Vec2d *p0, Vec2d *p1, Vec2d *p2, Vec2d *p3, double t);
	Vec2d evalCurveSecondDer(Vec2d *p0, Vec2d *p1, Vec2d *p2, Vec2d *p3, double t);

public:
	SplineBUniform(vector<Vec2d> ctrl, bool _needpad=true);
	SplineBUniform(vector<double> ctrl, vector<double> times, bool _needpad=true);
	SplineBUniform(int _numPts, double _t0, double _tf);

	int getUpperBound(double p);

	double getValue(double p);
	double getPartialDerivative(double p, int npt);
	double getSlope(double p);
	double getSlopePartialDerivative(double p, int npt);
	// slope of the slope
	double getSecondSlope(double p);
	double getSecondSlopePartialDerivative(double p, int npt);

	double getValuePartialCtrlGlobal(double _t, int _ctrlIndex);

	void superSample(vector<double> newctrlpts);
	void subSample(vector<double> newctrlpts);

	// inline access functions
	inline Matd getBasisMat(){return mBasisMat;}

	inline vector<Vec2d> getSplineCtrlPts(){return mCtrlPts;}
};

#endif
