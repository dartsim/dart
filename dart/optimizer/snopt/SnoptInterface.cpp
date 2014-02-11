/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/19/2011
*/

#include <iostream>
#include "SnoptInterface.h"

namespace dart {
namespace optimizer {
namespace snopt {
#include <string.h>
#include <assert.h>
//#include <FL/Fl.H>

SnoptInterface* SnoptInterface::mRef = NULL;

SnoptInterface::SnoptInterface(int constr_tot, int coef_tot, int nonlin_constr_tot,
                               int nonlin_obj, int nonlin_jac, int *c_eqns, int has_obj,
                               VVD J, VVB JMap, std::vector<double> *constraints,
                               double *objective, std::vector<double> *gradient,
                               SnoptInterface::updateFunc update_f, void *update_d) {
    mHasObjective = has_obj;
    mNumConstr = constr_tot;
    mNumCoef = coef_tot;
    mNumNonlinConstr = nonlin_constr_tot;
    mNumNonlinObjCoef = nonlin_obj;
    mNumNonlinJacCoef = nonlin_jac;

    // init constraint type for each constraint
    if(mNumConstr != 0)
        mConstrEqns = new int[mNumConstr];
    else
        mConstrEqns = NULL;

    for(int i = 0; i < mNumConstr; i++)
        mConstrEqns[i] = c_eqns[i];

    mSolverX = new double[mNumCoef];
    mProblemX = new double[mNumCoef];

    mdConstrdCoef = J;
    mCoefMap = JMap;
    mConstr = constraints;

    mObj = objective;
    mdObjdCoef = gradient;

    mUpdateFunc = update_f;
    mUpdateData = update_d;

    mBoundsLo = NULL;
    mBoundsHi = NULL;

    mOutput = 0;
    mSum = 0;
    mCheckTerm = true;
    mTermination = false;
    mAbnormal = NONE;
    mBreak = -1;

    mRef = this;
}

SnoptInterface::SnoptInterface(int has_obj, VVD J, VVB JMap, std::vector<double> *constraints,
                               double *objective, std::vector<double> *gradient,
                               SnoptInterface::updateFunc update_f, void *update_d) {
    mHasObjective = has_obj;

    mConstrEqns = NULL;
    mSolverX = NULL;
    mProblemX = NULL;

    mdConstrdCoef = J;
    mCoefMap = JMap;
    mConstr = constraints;

    mObj = objective;
    mdObjdCoef = gradient;

    mUpdateFunc = update_f;
    mUpdateData = update_d;

    mOutput = 0;
    mSum = 0;
    mCheckTerm = true;
    mTermination = false;
    mAbnormal = NONE;
    mBreak = -1;


    mRef = this;
}

SnoptInterface::~SnoptInterface() {

    delete[] mConstrEqns;
    delete[] mSolverX;
    delete[] mProblemX;
    // delete[] lo_bounds;
    // delete[] hi_bounds;

    mRef = NULL;
}

void SnoptInterface::clear(long mask, int compute_derivs) {
    if(mask & SnoptInterface::Obj) {
        *mObj = 0.0;
        if (compute_derivs) {
            for(unsigned int i = 0; i < mdObjdCoef->size(); i++) {
                mdObjdCoef->at(i) = 0.0;
            }
        }
    }

    if(mask & SnoptInterface::Constr) {
        for(unsigned int i = 0; i < mConstr->size(); i++)
            mConstr->at(i) = 0.0;
        if(compute_derivs) {
            for(unsigned int i = 0; i < mdConstrdCoef->size(); i++)
                for(unsigned int j = 0; j < mdConstrdCoef->at(i)->size(); j++)
                    mdConstrdCoef->at(i)->at(j) = 0.0;
            for(unsigned int i = 0; i < mCoefMap->size(); i++)
                for(unsigned int j = 0; j < mCoefMap->at(i)->size(); j++)
                    mCoefMap->at(i)->at(j) = 0;
        }

    }
}


void SnoptInterface::resizeJacobian(int coef_tot, int nonlin_coef_tot,
                                    int constr_tot, int nonlin_constr_tot) {
    mNumConstr = constr_tot;
    mNumNonlinConstr = nonlin_constr_tot;

    mNumCoef = coef_tot;
    mNumNonlinObjCoef = nonlin_coef_tot;
    mNumNonlinJacCoef = nonlin_coef_tot;

    // modify constraint type for each constraint
    delete[] mConstrEqns;

    if(mNumConstr != 0)
        mConstrEqns = new int[mNumConstr];
    else
        mConstrEqns = NULL;

    for(int i = 0; i < mNumConstr; i++)
        mConstrEqns[i] = 0;

    mConstr->resize(mNumConstr);

    for(unsigned int i = 0; i < mdConstrdCoef->size(); i++)
        delete mdConstrdCoef->at(i);

    mdConstrdCoef->resize(mNumConstr);

    for(int i = 0; i < mNumConstr; i++){
        mdConstrdCoef->at(i) = new std::vector<double>;
        mdConstrdCoef->at(i)->resize(mNumCoef);
    }

    for(unsigned int i = 0; i < mCoefMap->size(); i++)
        delete mCoefMap->at(i);

    mCoefMap->resize(mNumConstr);
    for(int i = 0; i < mNumConstr; i++){
        mCoefMap->at(i) = new std::vector<bool>;
        mCoefMap->at(i)->resize(mNumCoef);
    }

    mdObjdCoef->resize(mNumCoef);

    delete[] mSolverX;
    delete[] mProblemX;

    mSolverX = new double[mNumCoef];
    mProblemX = new double[mNumCoef];

}

void SnoptInterface::update(long mask, int compute_derivs, double *x) {
    for(int i = 0; i < mNumCoef; i++){
        mProblemX[i] = x[i];
    }

    mUpdateFunc(mask, compute_derivs, mProblemX, mUpdateData);
}

void SnoptInterface::updateSolverX() {
    for(int i = 0; i < mNumCoef; i++)
        mSolverX[i] = mProblemX[i];
}

void SnoptInterface::scaleValues(long mask, int compute_derivs) {
    if(mask & SnoptInterface::Obj){
        if(compute_derivs)
            for(int i = 0; i < mNumCoef; i++){
                mdObjdCoef->at(i) *= mCoefScale[i];
            }
    }

    if (mask & SnoptInterface::Constr) {
        for(int i = 0; i < mNumConstr; i++)
            mConstr->at(i) *= mConstrScale[i];
        if (compute_derivs) {
            for(int i = 0; i < mNumConstr; i++)
                for(int j = 0; j < mNumCoef; j++){
                    double scl = mConstrScale[i] * mCoefScale[j];
                    mdConstrdCoef->at(i)->at(j) *= scl;
                }
        }
    }
}

#ifndef WIN32

#define SNINIT sninit_
#define SNOPT snopt_
#define SNSPEC snspec_
#define S1USER s1user_

#endif
extern "C" {
void SNINIT(int *iprint, int *isum, char *cw, int *lencw,
            int *iw, int *leniw, double *rw, int *lenrw);
void SNOPT(char *start, int *m, int *n, int *ne,
           int *nName, int *nnCon, int *nnObj, int *nnJac,
           int *iObj, double *ObjAdd, char *Prob,
           void (*funCon)(int *mode, int *nnCon, int *nnJac, int *neJac,
                          double *x, double *fCon, double *gCon, int *nState,
                          char *cu, int *lencu, int *iu, int *leniu,
                          double *ru, int *lenru),
           void (*funObj)(int *mode, int *nnObj, double *x,
                          double *fObj, double *gObj, int *nState,
                          char *cu, int *lencu, int *iu, int *leniu,
                          double *ru, int *lenru),
           double *a, int *ha, int *ka, double *bl, double *bu,
           char *Names, int *hs, double *xs, double *pi, double *rc,
           int *inform, int *mincw, int *miniw, int *minrw,
           int *nS, int *nInf, double *sInf, double *Obj,
           char *cu,int *lencu, int *iu,int *leniu, double *ru,int *lenru,
           char *cw,int *lencw, int *iw,int *leniw, double *rw,int *lenrw,
           int start_len);
void SNSPEC(int *ispecs, int *inform, char *cw, int *lencw,
            int *iw, int *leniw, double *rw, int *lenrw);


void
S1USER(int *iAbort, char *MjrMsg, int *KTcond,
       int *m, int *n, int *nb, int *nR, int *nS,
       int *nMajor, int *nMinor, int *nSwap,
       double *condHz, double *duInf, double *emaxS, double *fObj,
       double *fMrt, double gMrt, double *PenNrm, double *prInf, double *step,
       double *vimax, double *dxnrm, double *dxrel,
       int *ne, int *nka, double *a, int *ha, int *ka,
       int *hs, double *bl, double *bu, double *pi, double *rc, double *xs,
       char *cu, int *lencu, int *iu, int *leniu, double *ru, int *lenru,
       char *cw, int *lencw, int *iw, int *leniw, double *rw, int *lenrw)
{

    SnoptInterface *s = SnoptInterface::mRef;
    /*
                char *found;
                  if((found = strstr(MjrMsg, "i")) != NULL){
                  cout << "Infeasible QP found" << endl;
                  s->mAbnormal = SnoptInterface::INFEASIBLE;
                  s->mTermination = true;
                  SnoptInterface::checkTermination(iAbort, xs);

                  }
                */
    /*    if((found = strstr(MjrMsg, "n")) != NULL){
                      cout << "Hessian does not update" << endl;
                      s->mAbnormal = SnoptInterface::HESSIAN_UPDATE;
                      }
                      if((found = strstr(MjrMsg, "sR")) != NULL){
                      cout << "Hessian reset abnormally" << endl;
                      s->mAbnormal = SnoptInterface::HESSIAN_RESET;
                      }*/
    if(s->mCheckTerm)
        SnoptInterface::checkTermination(iAbort, xs);

    if(s->mBreak == *nMajor){
        s->mTermination = true;
        SnoptInterface::checkTermination(iAbort, xs);
    }

}

} // extern "C"


SnoptInterface::Return SnoptInterface::solve(double *x, double *lo_bounds,
                                             double *hi_bounds, int unit) {
    //	printf("Solver: mObj=%d, nconstrs=%d, ncoefs=%d, nl_constrs=%d, nl_coefs=%d\n"
    //		, mHasObjective, mNumConstr, mNumCoef,
    //	mNumNonlinConstr, nonlin_coef_total);

    mAbnormal = NONE;
    mTermination = false;

    int constr_count = (1 > mNumConstr)? 1 : mNumConstr;
    int nm = mNumCoef + constr_count;

    for(int i = 0; i < mNumCoef; i++)
        mProblemX[i] = x[i];

    updateSolverX();
    double *xs = new double[nm];
    for(int i = 0; i < mNumCoef; i++)
        xs[i] = mSolverX[i];

    double *bu = new double[nm];
    double *bl = new double[nm];
    int *hs = new int[nm];
    double *pi = new double[constr_count];
    // set the bounds for the unknowns, and init the unknowns vector
    for(int i = 0; i < mNumCoef; i++){
        bl[i] = lo_bounds[i];
        bu[i] = hi_bounds[i];
        hs[i] = 2;
    }

    // set bounds for the constraints
    for(int i = 0; i < mNumConstr; i++){
        if(mConstrEqns[i] == 0){
            bl[mNumCoef + i] = -x[mNumCoef + i];
            bu[mNumCoef + i] = x[mNumCoef + i];
        }else if(mConstrEqns[i] == 1){
            bl[mNumCoef + i] = x[mNumCoef + i];
            bu[mNumCoef + i] = x[mNumCoef + i] + 1e7;
        }else if(mConstrEqns[i] == 2){
            bl[mNumCoef + i] = -1e7 + x[mNumCoef + i];
            bu[mNumCoef + i] = -0.0 + x[mNumCoef + i];
        }else if(mConstrEqns[i] == 3){
            bl[mNumCoef + i] = -1e-2 + x[mNumCoef + i];
            bu[mNumCoef + i] = 1e-2 + x[mNumCoef + i];
        }else if(mConstrEqns[i] == 4){
            bl[mNumCoef + i] = -1e-2 + x[mNumCoef + i];
            bu[mNumCoef + i] = 1e-2 + x[mNumCoef + i];
        }else if(mConstrEqns[i] == 5){
            bl[mNumCoef + i] = -1e3 + x[mNumCoef + i];
            bu[mNumCoef + i] = 1e3 + x[mNumCoef + i];
        }

        pi[i] = 0;
        xs[mNumCoef + i] = x[mNumCoef + i];
        hs[mNumCoef + i] = 2;
    }

    if(mNumConstr == 0){
        pi[0] = 0;
        xs[mNumCoef] = 0.0;
        bl[mNumCoef] = -1.e7;
        bu[mNumCoef] = 1.e7;
    }
    double *rc = new double[nm];

    // initialize best solution and update function so that jacobian
    // can be frozen
    update(SnoptInterface::Obj | SnoptInterface::Constr, 1, mSolverX);

    // set the jacobian
    double *a = NULL;
    int *ha = NULL;
    int *ka = NULL;
    fillUpSnoptFormat(mdConstrdCoef, &a, &ha, &ka);

    static int lencw = 1000 * sizeof(char[8]);
    static int leniw = 800 * nm;
    static int lenrw = 6400 * nm;
    static char* cw = new char[lencw];
    static int* iw = new int[leniw];
    static double* rw = new double[lenrw];

    int iprint = mOutput;
    int ispec = unit; //if spcname is not specified, snopt will load in fort.4 as default
    int isum = mSum; // 5 == stdin

    //  	fprintf(stderr,"cw: %d, iw: %d, rw: %d\n",cw,iw,rw);

    int inform;
    static int spec_loaded = 0;
    //  if(unit != 4){
    spec_loaded = 0;	//reset when a spacetime problem is fomulated
    lencw = 1000 * sizeof(char[8]);
    leniw = 800 * nm;
    lenrw = 6400 * nm;
    delete[] cw;
    delete[] iw;
    delete[] rw;

    cw = new char[lencw];
    iw = new int[leniw];
    rw = new double[lenrw];
    //  }

    //  if(mOutput == 9 ||!spec_loaded){
    SNINIT(&iprint, &isum, cw, &lencw, iw, &leniw, rw, &lenrw);
    SNSPEC(&ispec, &inform, cw, &lencw, iw, &leniw, rw, &lenrw);
    spec_loaded = 1;
    // }

    //	char *startup = "Warm";
    char startup[16] = "Cold";
    int m = constr_count;
    int n = mNumCoef;
    int ne = 0;
    for(int j = 0; j < mNumCoef; j++){
        int nonZeroCount = sparseCount(j);
        ne += (1 > nonZeroCount) ? 1 : nonZeroCount;
    }

    int nName = 1;
    int nnCon = mNumNonlinConstr;
    int nnObj = mNumNonlinObjCoef;
    int nnJac = (nnCon == 0) ? 0 : mNumNonlinJacCoef;
    int iObj = 0;
    double ObjAdd = 0;
    char problem_name[16] = "MOM";
    char names[] = " ";
    int mincw, miniw, minrw;
    int nS;
    int nInf;
    double sInf, Obj;

    SNOPT(startup, &m, &n, &ne, &nName, &nnCon, &nnObj, &nnJac,
          &iObj, &ObjAdd, problem_name,
          SnoptInterface::snoptJac, SnoptInterface::snoptObj,
          a, ha, ka, bl, bu, names, hs, xs, pi, rc,
          &inform, &mincw, &miniw, &minrw, &nS, &nInf, &sInf, &Obj,
          cw, &lencw, iw, &leniw, rw, &lenrw,
          cw, &lencw, iw, &leniw, rw, &lenrw,
          strlen(startup));

    mReturnedObj = Obj;

    update(SnoptInterface::Obj | SnoptInterface::Constr, 1, xs);

    for(int i = 0; i < mNumCoef; i++)
        x[i] = xs[i];

    delete[] xs;
    delete[] bu;
    delete[] bl;
    delete[] hs;
    delete[] pi;
    delete[] rc;
    //  delete[] cw;
    //delete[] iw;
    //delete[] rw;

    delete[] a;
    delete[] ha;
    delete[] ka;

    if(inform != 0)
        std::cout << "[VLOG(1)]" << "inform = " << inform << std::endl;

    // cout << "objective = " << mReturnedObj << endl;

    switch(inform){
    case 0:
        return SnoptInterface::Solution;
    case 1:
        return SnoptInterface::Infeasible;
    case 3:
        return SnoptInterface::Stop;
    case 6:
    case 12:
        return SnoptInterface::UserStop;
    default:
        return SnoptInterface::Error;
    };

}

/* ARGSUSED */
void  SnoptInterface::snoptObj(int *mode, int *nn_obj, double *x,
                               double *f_obj, double *g_obj, int *nstate,
                               char *cu, int *lencu,
                               int *iu, int *leniu,
                               double *ru, int *lenru) {

    SnoptInterface *s = SnoptInterface::mRef;
    assert(s != NULL);

    s->update(SnoptInterface::Obj, *mode, x);

    *f_obj = s->mObj[0];
    if(*mode == 2){
        for(int i = 0; i < *nn_obj; i++){
            g_obj[i] = s->mdObjdCoef->at(i);
        }
    }
}

/* ARGSUSED */
void SnoptInterface::snoptJac(int *mode, int *nn_con, int *nn_jac, int *ne_jac,
                              double *x, double *f_con, double *g_con, int *nstate,
                              char *cu, int *lencu,
                              int *iu, int *leniu,
                              double *ru, int *lenru) {
    SnoptInterface *s = SnoptInterface::mRef;
    assert(s != NULL);
    if(s->mNumConstr == 0){
        f_con[0] = 0.0;
        return;
    }

    s->update(SnoptInterface::Constr, *mode, x);

    for(int i = 0; i < *nn_con; i++){
        f_con[i] = s->mConstr->at(i);
    }


    if(*mode == 2){
        int nElt = 0;
        for(int j = 0; j < s->mNumCoef; j++){
            if(s->sparseCount(j) == 0){
                g_con[nElt++] = 0.0;
                continue;
            }
            for(int i = 0; i < *nn_con; i++){
                if(s->mCoefMap->at(i)->at(j))
                    g_con[nElt++] = s->mdConstrdCoef->at(i)->at(j);
            }
        }
    }
}

void SnoptInterface::fillUpSnoptFormat(VVD jacobian, double **a, int **ha, int **ka) {
    //	Count up the nonzero elements and allocate memory for a, ha, and ka
    int nElts = 0;
    int cols = mNumCoef;
    int *nonZeroInCol = new int[mNumCoef];

    for(int j = 0; j < cols; j++){
        nonZeroInCol[j] = sparseCount(j);
        nElts += (1 > nonZeroInCol[j]) ? 1 : nonZeroInCol[j];
    }

    delete[] *a;
    delete[] *ha;
    delete[] *ka;
    *a = new double[nElts];
    *ha = new int[nElts];
    *ka = new int[mNumCoef + 1];

    //	Fillup the SNOPT sparse structure
    nElts = 0;

    for(int j = 0; j < cols; j++){
        (*ka)[j] = nElts + 1;	 //Fortran starts from 1
        if(nonZeroInCol[j] == 0){ //Deal with empty column
            (*a)[nElts] = 0.0;
            (*ha)[nElts] = 1;
            nElts++;
            continue;
        }

        for(int i = 0; i < mNumConstr; i++){
            if(mCoefMap->at(i)->at(j)){
                (*a)[nElts] = jacobian->at(i)->at(j);
                (*ha)[nElts] = i + 1;	//Fortran starts from 1
                nElts++;
            }
        }
    }

    (*ka)[mNumCoef] = nElts + 1;	//Last entry is special
    delete[] nonZeroInCol;
}

//int SnoptInterface::sparseCount()
//{
//	int numNonZero = 0;
//	for(int i = 0; i < mNumConstr; i++)
//		for(int j = 0; j < mNumCoef; j++)
//			if(mCoefMap->at(i)->at(j))
//				numNonZero++;
//	return numNonZero;
//}

int SnoptInterface::sparseCount(int col) {
    int numNonZero = 0;
    for(int i = 0; i < mNumConstr; i++)
        if(mCoefMap->at(i)->at(col))
            numNonZero++;

    return numNonZero;
}


void SnoptInterface::checkTermination(int *iAbort, double *xs) {
    SnoptInterface *s = SnoptInterface::mRef;
    // cout << "check " << s->mTermination << endl;
    if(s->mTermination){
        *iAbort = 1;
        s->mTermination = false;
    }
}

} // namespace snopt
} // namespace optimizer
} // namespace dart
