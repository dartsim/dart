#include "Spline.h"

bool lessThan::operator ()(const Vec2d p1, const Vec2d p2){
	return p1[0]<p2[0];
}

Spline::Spline(vector<Vec2d> _ctrlpts){
    mCtrlPts=_ctrlpts;
}

void Spline::addCtrlPt(Vec2d p){
	mCtrlPts.push_back(p);
	// sort the points
	sort(mCtrlPts.begin(), mCtrlPts.end(), lessThan());
}

