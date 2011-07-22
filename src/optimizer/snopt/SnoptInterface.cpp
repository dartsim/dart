/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/19/2011
*/

#include "SnoptInterface.h"
using namespace std;
using namespace Eigen;

#include <glog/logging.h>
using namespace google;


namespace optimizer {
    namespace snopt {
#include <string.h>
#include <assert.h>
//#include <FL/Fl.H>

        using namespace std;

        SnoptInterface* SnoptInterface::ref = NULL;


        SnoptInterface::SnoptInterface(int constr_tot, int coef_tot, int nonlin_constr_tot,
                                       int nonlin_obj, int nonlin_jac, int *c_eqns, int has_obj,
                                       VVD J, VVB JMap, std::vector<double> *constraints,
                                       double *objective, std::vector<double> *gradient,
                                       SnoptInterface::UpdateF update_f, void *update_d) {
            has_objective = has_obj;
            constr_total = constr_tot;
            coef_total = coef_tot;
            nonlin_constr_total = nonlin_constr_tot;
            nonlin_obj_coef = nonlin_obj;
            nonlin_jac_coef = nonlin_jac;

            // init constraint type for each constraint
            if(constr_total != 0)
                constr_eqns = new int[constr_total];
            else
                constr_eqns = NULL;

            for(int i = 0; i < constr_total; i++)
                constr_eqns[i] = c_eqns[i];

            solver_x = new double[coef_total];
            problem_x = new double[coef_total];
	
            dConstr_dCoef = J;
            coefMap = JMap;
            constr = constraints;
  
            obj = objective;
            dObj_dCoef = gradient;

            update_func = update_f;
            update_data = update_d;
	
            lo_bounds = NULL;
            hi_bounds = NULL;

            mOutput = 0;
            mSum = 0;
            mCheckTerm = true;
            mTermination = false;
            mAbnormal = NONE;
            mBreak = -1;

            ref = this;
        }

        SnoptInterface::SnoptInterface(int has_obj, VVD J, VVB JMap, std::vector<double> *constraints,
                                       double *objective, std::vector<double> *gradient,
                                       SnoptInterface::UpdateF update_f, void *update_d) {
            has_objective = has_obj;

            constr_eqns = NULL;
            solver_x = NULL;
            problem_x = NULL;
	
            dConstr_dCoef = J;
            coefMap = JMap;
            constr = constraints;
  
            obj = objective;
            dObj_dCoef = gradient;

            update_func = update_f;
            update_data = update_d;

            mOutput = 0;
            mSum = 0;
            mCheckTerm = true;
            mTermination = false;
            mAbnormal = NONE;
            mBreak = -1;


            ref = this;
        }

        SnoptInterface::~SnoptInterface() {

            delete[] constr_eqns;
            delete[] solver_x;
            delete[] problem_x;
            // delete[] lo_bounds;
            // delete[] hi_bounds;

            ref = NULL;
        }

        void SnoptInterface::clear(long mask, int compute_derivs) {
            if(mask & SnoptInterface::Obj) {
                *obj = 0.0;
                if (compute_derivs) {
                    for(unsigned int i = 0; i < dObj_dCoef->size(); i++) {
                        dObj_dCoef->at(i) = 0.0;
                    }
                }
            }

            if(mask & SnoptInterface::Constr) {
                for(unsigned int i = 0; i < constr->size(); i++)
                    constr->at(i) = 0.0;
                if(compute_derivs) {
                    for(unsigned int i = 0; i < dConstr_dCoef->size(); i++)
                        for(unsigned int j = 0; j < dConstr_dCoef->at(i)->size(); j++)
                            dConstr_dCoef->at(i)->at(j) = 0.0;
                    for(unsigned int i = 0; i < coefMap->size(); i++)
                        for(unsigned int j = 0; j < coefMap->at(i)->size(); j++)
                            coefMap->at(i)->at(j) = 0;
                }

            }
        }


        void SnoptInterface::ResizeJacobian(int coef_tot, int nonlin_coef_tot,
                                            int constr_tot, int nonlin_constr_tot) {
            constr_total = constr_tot;
            nonlin_constr_total = nonlin_constr_tot;

            coef_total = coef_tot;
            nonlin_obj_coef = nonlin_coef_tot;
            nonlin_jac_coef = nonlin_coef_tot;

            // modify constraint type for each constraint
            if (constr_eqns != NULL)
                delete[] constr_eqns;

            if(constr_total != 0)
                constr_eqns = new int[constr_total];
            else
                constr_eqns = NULL;

            for(int i = 0; i < constr_total; i++)
                constr_eqns[i] = 0;

            constr->resize(constr_total);
  
            for(unsigned int i = 0; i < dConstr_dCoef->size(); i++)
                delete dConstr_dCoef->at(i);

            dConstr_dCoef->resize(constr_total);
  
            for(int i = 0; i < constr_total; i++){
                dConstr_dCoef->at(i) = new std::vector<double>;
                dConstr_dCoef->at(i)->resize(coef_total);
            }

            for(unsigned int i = 0; i < coefMap->size(); i++)
                delete coefMap->at(i);
  
            coefMap->resize(constr_total);
            for(int i = 0; i < constr_total; i++){
                coefMap->at(i) = new std::vector<bool>;
                coefMap->at(i)->resize(coef_total);
            }

            dObj_dCoef->resize(coef_total);

            if(solver_x)
                delete[] solver_x;
            if(problem_x)
                delete[] problem_x;

            solver_x = new double[coef_total];
            problem_x = new double[coef_total];

        }

        void SnoptInterface::update(long mask, int compute_derivs, double *x) {
            for(int i = 0; i < coef_total; i++){
                problem_x[i] = x[i];
            }

            update_func(mask, compute_derivs, problem_x, update_data);
        }

        void SnoptInterface::updateSolverX() {
            for(int i = 0; i < coef_total; i++)
                solver_x[i] = problem_x[i];
        }

        void SnoptInterface::scaleValues(long mask, int compute_derivs) {
            if(mask & SnoptInterface::Obj){
                if(compute_derivs) 
                    for(int i = 0; i < coef_total; i++){
                        dObj_dCoef->at(i) *= coef_scale[i];
                    }
            }

            if (mask & SnoptInterface::Constr) {
                for(int i = 0; i < constr_total; i++)
                    constr->at(i) *= constr_scale[i];
                if (compute_derivs) {
                    for(int i = 0; i < constr_total; i++)
                        for(int j = 0; j < coef_total; j++){
                            double scl = constr_scale[i] * coef_scale[j];
                            dConstr_dCoef->at(i)->at(j) *= scl;
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

                SnoptInterface *s = SnoptInterface::ref;
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
            //	printf("Solver: obj=%d, nconstrs=%d, ncoefs=%d, nl_constrs=%d, nl_coefs=%d\n"
            //		, has_objective, constr_total, coef_total, 
            //	nonlin_constr_total, nonlin_coef_total);

            mAbnormal = NONE;
            mTermination = false;

            int constr_count = (1 > constr_total)? 1 : constr_total;
            int nm = coef_total + constr_count;

            for(int i = 0; i < coef_total; i++)
                problem_x[i] = x[i];

            updateSolverX();
            double *xs = new double[nm];
            for(int i = 0; i < coef_total; i++)
                xs[i] = solver_x[i];

            double *bu = new double[nm];
            double *bl = new double[nm];
            int *hs = new int[nm];
            double *pi = new double[constr_count];
            // set the bounds for the unknowns, and init the unknowns vector
            for(int i = 0; i < coef_total; i++){
                bl[i] = lo_bounds[i];
                bu[i] = hi_bounds[i];
                hs[i] = 2;
            }

            // set bounds for the constraints
            for(int i = 0; i < constr_total; i++){
                if(constr_eqns[i] == 0){
                    bl[coef_total + i] = -x[coef_total + i];
                    bu[coef_total + i] = x[coef_total + i];
                }else if(constr_eqns[i] == 1){
                    bl[coef_total + i] = x[coef_total + i];
                    bu[coef_total + i] = x[coef_total + i] + 1e7;
                }else if(constr_eqns[i] == 2){
                    bl[coef_total + i] = -1e7 + x[coef_total + i];
                    bu[coef_total + i] = -0.0 + x[coef_total + i];
                }else if(constr_eqns[i] == 3){
                    bl[coef_total + i] = -1e-2 + x[coef_total + i];
                    bu[coef_total + i] = 1e-2 + x[coef_total + i];
                }else if(constr_eqns[i] == 4){
                    bl[coef_total + i] = -1e-2 + x[coef_total + i];
                    bu[coef_total + i] = 1e-2 + x[coef_total + i];
                }else if(constr_eqns[i] = 5){
                    bl[coef_total + i] = -1e3 + x[coef_total + i];
                    bu[coef_total + i] = 1e3 + x[coef_total + i];
                }

                pi[i] = 0;
                xs[coef_total + i] = x[coef_total + i];
                hs[coef_total + i] = 2;
            }

            if(constr_total == 0){
                pi[0] = 0;
                xs[coef_total] = 0.0;
                bl[coef_total] = -1.e7;
                bu[coef_total] = 1.e7;
            }
            double *rc = new double[nm];

            // initialize best solution and update function so that jacobian
            // can be frozen
            update(SnoptInterface::Obj | SnoptInterface::Constr, 1, solver_x);

            // set the jacobian
            double *a = NULL;
            int *ha = NULL;
            int *ka = NULL;
            FillUpSnoptFormat(dConstr_dCoef, &a, &ha, &ka);

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
            if(cw)
                delete[] cw;
            if(iw)
                delete[] iw;
            if(rw)
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
            int n = coef_total;
            int ne = 0;
            for(int j = 0; j < coef_total; j++){
                int nonZeroCount = SparseCount(j);	
                ne += (1 > nonZeroCount) ? 1 : nonZeroCount;
            }

            int nName = 1;
            int nnCon = nonlin_constr_total;
            int nnObj = nonlin_obj_coef;
            int nnJac = (nnCon == 0) ? 0 : nonlin_jac_coef;
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

            returnedObj = Obj;

            update(SnoptInterface::Obj | SnoptInterface::Constr, 1, xs);

            for(int i = 0; i < coef_total; i++)
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
                VLOG(1) << "inform = " << inform << endl;
  
            // cout << "objective = " << returnedObj << endl;

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

            SnoptInterface *s = SnoptInterface::ref;
            assert(s != NULL);

            s->update(SnoptInterface::Obj, *mode, x);

            *f_obj = s->obj[0];
            if(*mode == 2){
                for(int i = 0; i < *nn_obj; i++){
                    g_obj[i] = s->dObj_dCoef->at(i);
                }
            }
        }

/* ARGSUSED */
        void SnoptInterface::snoptJac(int *mode, int *nn_con, int *nn_jac, int *ne_jac,
                                   double *x, double *f_con, double *g_con, int *nstate, 
                                   char *cu, int *lencu, 
                                   int *iu, int *leniu, 
                                   double *ru, int *lenru) {
            SnoptInterface *s = SnoptInterface::ref;
            assert(s != NULL);
            if(s->constr_total == 0){
                f_con[0] = 0.0;
                return;
            }

            s->update(SnoptInterface::Constr, *mode, x);

            for(int i = 0; i < *nn_con; i++){
                f_con[i] = s->constr->at(i);
            }


            if(*mode == 2){
                int nElt = 0;
                for(int j = 0; j < s->coef_total; j++){
                    if(s->SparseCount(j) == 0){
                        g_con[nElt++] = 0.0;
                        continue;
                    }
                    for(int i = 0; i < *nn_con; i++){
                        if(s->coefMap->at(i)->at(j))
                            g_con[nElt++] = s->dConstr_dCoef->at(i)->at(j);
                    }
                }
            }
        }

        void SnoptInterface::FillUpSnoptFormat(VVD jacobian, double **a, int **ha, int **ka) {
            //	Count up the nonzero elements and allocate memory for a, ha, and ka
            int nElts = 0;
            int cols = coef_total;
            int *nonZeroInCol = new int[coef_total];

            for(int j = 0; j < cols; j++){
                nonZeroInCol[j] = SparseCount(j);
                nElts += (1 > nonZeroInCol[j]) ? 1 : nonZeroInCol[j];
            }

            if(*a != NULL)
                delete[] *a;
            if(*ha != NULL)
                delete[] *ha;
            if(*ka != NULL)
                delete[] *ka;
            *a = new double[nElts];
            *ha = new int[nElts];
            *ka = new int[coef_total + 1];

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

                for(int i = 0; i < constr_total; i++){
                    if(coefMap->at(i)->at(j)){
                        (*a)[nElts] = jacobian->at(i)->at(j);
                        (*ha)[nElts] = i + 1;	//Fortran starts from 1
                        nElts++;
                    }
                }
            }

            (*ka)[coef_total] = nElts + 1;	//Last entry is special	
            delete[] nonZeroInCol;
        }

//int SnoptInterface::SparseCount()
//{
//	int numNonZero = 0;
//	for(int i = 0; i < constr_total; i++)
//		for(int j = 0; j < coef_total; j++)
//			if(coefMap->at(i)->at(j))
//				numNonZero++;
//	return numNonZero;
//}

        int SnoptInterface::SparseCount(int col) {
            int numNonZero = 0;
            for(int i = 0; i < constr_total; i++)
                if(coefMap->at(i)->at(col))
                    numNonZero++;
	
            return numNonZero;
        }


        void SnoptInterface::checkTermination(int *iAbort, double *xs) {
            SnoptInterface *s = SnoptInterface::ref;
            // cout << "check " << s->mTermination << endl;
            if(s->mTermination){
                *iAbort = 1;
                s->mTermination = false;
            }
        }
        
    } // namespace snopt
} // namespace optimizer
