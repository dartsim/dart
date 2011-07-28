#ifndef _SPLINECATMULLROM_
#define _SPLINECATMULLROM_

#include "Spline.h"
#include "moremath.h"
#include <cassert>

class SplineCatmullRom: public Spline{
protected:
	Mat4d mBasisMat;

	void getParameter(double p, Vec2d *p0, Vec2d *p1, Vec2d *p2, Vec2d *p3, double *t);
	Vec2d evalCurvePoint(Vec2d *p0, Vec2d *p1, Vec2d *p2, Vec2d *p3, double t);
	Vec2d evalCurveDer(Vec2d *p0, Vec2d *p1, Vec2d *p2, Vec2d *p3, double t);
	Vec2d evalCurveSecondDer(Vec2d *p0, Vec2d *p1, Vec2d *p2, Vec2d *p3, double t);

public:
	SplineCatmullRom(vector<Vec2d> ctrl, bool padded=true);

	int getUpperBound(double p);

	double getValue(double p);
	double getPartialDerivative(double p, int npt);
	double getSlope(double p);
	double getSlopePartialDerivative(double p, int npt);
	// slope of the slope
	double getSecondSlope(double p);

	void superSample(vector<double> newctrlpts);
	void subSample(vector<double> newctrlpts);

	// inline access functions
	inline Matd getBasisMat(){return mBasisMat;}
};

#endif

