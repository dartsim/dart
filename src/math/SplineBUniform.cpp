#include "SplineBUniform.h"

#define ONE_BY_SIX 1.0/6.0

SplineBUniform::SplineBUniform(int _numPts, double _t0, double _tf){
	// add the first and the last point to pad the ends
	mCtrlPts.clear();
	double diff = (_tf-_t0)/(_numPts-1);
	for(int i=0; i<_numPts; i++){
		mCtrlPts.push_back(Vec2d(_t0+diff*i, 0));
	}

	// add padding
	Vec2d pend = mCtrlPts[mCtrlPts.size()-1];
	pend[0] = 2*pend[0] - mCtrlPts[mCtrlPts.size()-2][0]; 
	addCtrlPt(pend);
	Vec2d pend2 = mCtrlPts[mCtrlPts.size()-1];
	pend2[0] = 2*pend[0] - mCtrlPts[mCtrlPts.size()-2][0]; 
	addCtrlPt(pend2);
	// start
	Vec2d pstart= mCtrlPts[0];
	pstart[0] = 2*pstart[0] - mCtrlPts[1][0]; 
	addCtrlPt(pstart);

	// assumes pre multiplied by [t3 t2 t 1]
	mBasisMat = ONE_BY_SIX*Mat4d(-1, 3, -3, 1, 
		3, -6, 3, 0, 
		-3, 0, 3, 0, 
		1, 4, 1, 0);
}

SplineBUniform::SplineBUniform(vector<double> ctrl, vector<double> times, bool _needpad){
	mCtrlPts.resize(ctrl.size());
	for(int i=0; i<mCtrlPts.size(); i++) mCtrlPts[i] = Vec2d(times[i], ctrl[i]);

	// add the first and the last point to pad the ends
	if(_needpad){
		Vec2d pend = mCtrlPts[mCtrlPts.size()-1];
		pend[0] = 2*pend[0] - mCtrlPts[mCtrlPts.size()-2][0]; 
		addCtrlPt(pend);
		Vec2d pend2 = mCtrlPts[mCtrlPts.size()-1];
		pend2[0] = 2*pend[0] - mCtrlPts[mCtrlPts.size()-1][0]; 
		addCtrlPt(pend2);
		Vec2d pstart= mCtrlPts[0];
		pstart[0] = 2*pstart[0] - mCtrlPts[1][0]; 
		addCtrlPt(pstart);
	}

	// assumes pre multiplied by [t3 t2 t 1]
	mBasisMat = ONE_BY_SIX*Mat4d(-1, 3, -3, 1, 
		3, -6, 3, 0, 
		-3, 0, 3, 0, 
		1, 4, 1, 0);
}

SplineBUniform::SplineBUniform(vector<Vec2d> _ctrlpts, bool _needpad): Spline(_ctrlpts){
	// add the first and the last point to pad the ends
	if(_needpad){
		Vec2d pend = mCtrlPts[mCtrlPts.size()-1];
		pend[0] = 2*pend[0] - mCtrlPts[mCtrlPts.size()-2][0]; 
		addCtrlPt(pend);
		Vec2d pend2 = mCtrlPts[mCtrlPts.size()-1];
		pend2[0] = 2*pend[0] - mCtrlPts[mCtrlPts.size()-1][0]; 
		addCtrlPt(pend2);
		Vec2d pstart= mCtrlPts[0];
		pstart[0] = 2*pstart[0] - mCtrlPts[1][0]; 
		addCtrlPt(pstart);
	}

	// assumes pre multiplied by [t3 t2 t 1]
	mBasisMat = ONE_BY_SIX*Mat4d(-1, 3, -3, 1, 
		3, -6, 3, 0, 
		-3, 0, 3, 0, 
		1, 4, 1, 0);
}

Vec2d SplineBUniform::evalCurvePoint(Vec2d *p0, Vec2d *p1, Vec2d *p2, Vec2d *p3, double t){
	Vec2d p;
	double t2 = t * t;
	double t3 = t2 * t;

	Matd geom(4,2,vl_0);
	geom[0] = *p0;
	geom[1] = *p1;
	geom[2] = *p2;
	geom[3] = *p3;

	Vec4d blendfunc4 = Vec4d(t3, t2, t, 1)*mBasisMat;
	Vecd blendfunc(4, blendfunc4[0], blendfunc4[1], blendfunc4[2], blendfunc4[3]);
	Vecd pt = blendfunc*geom;
	return Vec2d(pt[0], pt[1]);
}

Vec2d SplineBUniform::evalCurveDer(Vec2d *p0, Vec2d *p1, Vec2d *p2, Vec2d *p3, double t){
	Vec2d p;
	double t2 = t * t;

	Matd geom(4,2,vl_0);
	geom[0] = *p0;
	geom[1] = *p1;
	geom[2] = *p2;
	geom[3] = *p3;

	Vec4d blendfunc4 = Vec4d(3*t2, 2*t, 1, 0)*mBasisMat;
	Vecd blendfunc(4, blendfunc4[0], blendfunc4[1], blendfunc4[2], blendfunc4[3]);
	Vecd pt = blendfunc*geom;
	return Vec2d(pt[0], pt[1]);
}

Vec2d SplineBUniform::evalCurveSecondDer(Vec2d *p0, Vec2d *p1, Vec2d *p2, Vec2d *p3, double t){
	Vec2d p;

	Matd geom(4,2,vl_0);
	geom[0] = *p0;
	geom[1] = *p1;
	geom[2] = *p2;
	geom[3] = *p3;

	Vec4d blendfunc4 = Vec4d(6*t, 2, 0, 0)*mBasisMat;
	Vecd blendfunc(4, blendfunc4[0], blendfunc4[1], blendfunc4[2], blendfunc4[3]);
	Vecd pt = blendfunc*geom;
	return Vec2d(pt[0], pt[1]);
}

// return integer index of the upperbound iterator
// => 4 meaningful indices are uindex-2, uindex-1, uindex, uindex+1
int SplineBUniform::getUpperBound(double p){
	vector <Vec2d>::iterator uindex = upper_bound(mCtrlPts.begin(), mCtrlPts.end(), Vec2d(p,0), lessThan());
	// assert that the interval is surrounded by other intervals - reqd for b spline
	// i.e b spline has to be _needpad at both ends ALWAYS 
	//cout<<"value: "<<p<<", ctrl :"<<*uindex<<endl;
	if(uindex>=mCtrlPts.end()-1){
		cout<<"should not come here as 2 padded points in the end now\n";
		getchar();
		uindex = lower_bound(mCtrlPts.begin(), mCtrlPts.end(), Vec2d(p,0), lessThan());
		//cout<<"new value: "<<p<<", ctrl :"<<*uindex<<endl;
	}

	// convert the iterator into int value
	int uindex_int=0;
	for(;uindex!=mCtrlPts.end(); uindex_int++, uindex++);
	int ub = mCtrlPts.size()-uindex_int;
	if(ub<2) {
		printf("upper bound: %d for input %lf\n", ub, p);
	}
	return mCtrlPts.size()-uindex_int;
}

// calculate the t value in appropriate interval and also the control points
void SplineBUniform::getParameter(double p, Vec2d *p0, Vec2d *p1, Vec2d *p2, Vec2d *p3, double *t){
	// determine in which interval does p lie
	int uindex = getUpperBound(p);
	int lindex=uindex-1;
	*p0 = mCtrlPts[lindex-1];
	*p1 = mCtrlPts[lindex];
	*p2 = mCtrlPts[uindex];
	*p3 = mCtrlPts[uindex+1];
	//printf("indices for %lf: %d, %d, %d, %d\n", p, lindex-1, lindex, uindex, uindex+1);

	// solve or actual cubic equation to get t corresponding to p
	Vec4d geom((*p0)[0], (*p1)[0], (*p2)[0], (*p3)[0]);
	Vec4d c = mBasisMat*geom;
	c[3]-=p;	// make the cubic eqn =0
	vector<double> solution = moremath::solveCubic(c[0], c[1], c[2], c[3]);
	for(int i=0; i<solution.size(); i++){
		if(solution[i]>=-SPLINE_EPSILON && solution[i]<=1+SPLINE_EPSILON) {
			*t=solution[i];
			break;
		}
	}
}

double SplineBUniform::getValue(double p){
	if(p<mCtrlPts[1][0]) p = mCtrlPts[1][0];
	else if(p>mCtrlPts[mCtrlPts.size()-3][0]) p = mCtrlPts[mCtrlPts.size()-3][0];

	Vec2d p0, p1, p2, p3;
	double t;
	// fill the values for 4 control points and the t value corresponding to p
	getParameter(p, &p0, &p1, &p2, &p3, &t);
	Vec2d vp = evalCurvePoint(&p0, &p1, &p2, &p3, t);
	return vp[1];
}

// computes partial derivative of spline (wrt to one of the control points) at a point p
double SplineBUniform::getPartialDerivative(double p, int npt){
	if(p<mCtrlPts[1][0]) p = mCtrlPts[1][0];
	else if(p>mCtrlPts[mCtrlPts.size()-3][0]) p = mCtrlPts[mCtrlPts.size()-3][0];

	// compute the t value and discard all the pts values as unnecessary
	Vec2d p0, p1, p2, p3;
	double t;
	getParameter(p, &p0, &p1, &p2, &p3, &t);
	vector<Vec2d> cp(4, Vec2d(0,0));
	cp[npt][1]=1;
	Vec2d der = evalCurvePoint(&cp[0], &cp[1], &cp[2], &cp[3], t);
	
	return der[1];
}


double SplineBUniform::getValuePartialCtrlGlobal(double _t, int _ctrlIndex){
	int uindex = getUpperBound(_t);
	int lindex=uindex-1;
	if(_ctrlIndex>=lindex-1 && _ctrlIndex<=uindex+1){
		Vec2d p0, p1, p2, p3;
		p0 = mCtrlPts[lindex-1];
		p1 = mCtrlPts[lindex];
		p2 = mCtrlPts[uindex];
		p3 = mCtrlPts[uindex+1];
		// solve or actual cubic equation to get t corresponding to p
		Vec4d geom((p0)[0], (p1)[0], (p2)[0], (p3)[0]);
		Vec4d c = mBasisMat*geom;
		c[3]-=_t;	// make the cubic eqn =0
		double tlocal;
		vector<double> solution = moremath::solveCubic(c[0], c[1], c[2], c[3]);
		for(int i=0; i<solution.size(); i++){
			if(solution[i]>=-SPLINE_EPSILON && solution[i]<=1+SPLINE_EPSILON) {
				tlocal=solution[i];
				break;
			}
		}
		double v = 0;
		double tp=1;
		for(int i=0; i<4; i++) {
			v += tp*mBasisMat[3-i][_ctrlIndex-lindex+1];
			tp *= tlocal;
		}
		return v;
	}
	else return 0;
}

// computes slope of the curve and point p
double SplineBUniform::getSlope(double p){
	if(p<mCtrlPts[1][0]) p = mCtrlPts[1][0];
	else if(p>mCtrlPts[mCtrlPts.size()-3][0]) p = mCtrlPts[mCtrlPts.size()-3][0];

	Vec2d p0, p1, p2, p3;
	double t;
	// fill the values for 4 control points and the t value corresponding to p
	getParameter(p, &p0, &p1, &p2, &p3, &t);
	// evaluate the value at t
	Vec2d vp = evalCurveDer(&p0, &p1, &p2, &p3, t);

	//return vp[1]/vp[0];	// coincidence these are same?
	return vp[1]/(p2[0]-p1[0]);
}

// computes partial derivative (wrt to one of the control points) of spline slope (derivative wrt t) at a point p
double SplineBUniform::getSlopePartialDerivative(double _pt, int npt){
	if(_pt<mCtrlPts[1][0]) _pt = mCtrlPts[1][0];
	else if(_pt>mCtrlPts[mCtrlPts.size()-3][0]) _pt = mCtrlPts[mCtrlPts.size()-3][0];
	// compute the t value and discard all the pts values as unnecessary
	//Vec2d p0, p1, p2, p3;
	vector<Vec2d> p(4);
	double t;
	getParameter(_pt, &p[0], &p[1], &p[2], &p[3], &t);
	vector<Vec2d> cp(4, Vec2d(0,0));
	cp[npt][1]=1;
	Vec2d der = evalCurveDer(&cp[0], &cp[1], &cp[2], &cp[3], t);

	return der[1]/(p[2][0]-p[1][0]);	// also the denominator can be found using den of getslope as it is constant
}


// computes slope of the slope of the curve and point p
double SplineBUniform::getSecondSlope(double p){
	if(p<mCtrlPts[1][0]) p = mCtrlPts[1][0];
	else if(p>mCtrlPts[mCtrlPts.size()-3][0]) p = mCtrlPts[mCtrlPts.size()-3][0];

	Vec2d p0, p1, p2, p3;
	double t;
	// fill the values for 4 control points and the t value corresponding to p
	getParameter(p, &p0, &p1, &p2, &p3, &t);
	// evaluate the value at t
	Vec2d vp = evalCurveSecondDer(&p0, &p1, &p2, &p3, t);

	//return vp[1]/vp[0];	// denom always 0 ??
	return vp[1]/moremath::sqr(p2[0]-p1[0]);
}

// computes partial derivative (wrt to one of the control points) of spline slope (derivative wrt t) at a point p
double SplineBUniform::getSecondSlopePartialDerivative(double _pt, int npt){
	if(_pt<mCtrlPts[1][0]) _pt = mCtrlPts[1][0];
	else if(_pt>mCtrlPts[mCtrlPts.size()-3][0]) _pt = mCtrlPts[mCtrlPts.size()-3][0];
	// compute the t value and discard all the pts values as unnecessary
	//Vec2d p0, p1, p2, p3;
	vector<Vec2d> p(4);
	double t;
	getParameter(_pt, &p[0], &p[1], &p[2], &p[3], &t);
	vector<Vec2d> cp(4, Vec2d(0,0));
	cp[npt][1]=1;
	Vec2d der = evalCurveSecondDer(&cp[0], &cp[1], &cp[2], &cp[3], t);

	return der[1]/moremath::sqr(p[2][0]-p[1][0]);
}

void SplineBUniform::superSample(vector<double> newsamplepts){
	//vector<Vec2d> newctrlpts;
	//for(int i=0; i<newsamplepts.size(); i++){
	//	double val = getValue(newsamplepts[i]);
	//	newctrlpts.push_back(Vec2d(newsamplepts[i], val));
	//}
	//cout<<"Warning: not the correct way of resampling a b spline\n";
	//// check if _needpad or not
	////if(mCtrlPts[0][0]==mCtrlPts[1][0]){
	////	newctrlpts.push_back(newctrlpts[0]);
	////	newctrlpts.push_back(newctrlpts[newctrlpts.size()-1]);
	////	sort(newctrlpts.begin(), newctrlpts.end(), lessThan());
	////}
	//mCtrlPts.clear();
	//mCtrlPts=newctrlpts;
}

void SplineBUniform::subSample(vector<double> newctrlpts){
	superSample(newctrlpts);
}
