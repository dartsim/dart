#include "Lemke.h"
#include <iostream>
#include <cmath>
using namespace std;

#include <math.h>

#ifndef isnan
# define isnan(x) \
    (sizeof (x) == sizeof (long double) ? isnan_ld (x) \
    : sizeof (x) == sizeof (double) ? isnan_d (x) \
    : isnan_f (x))
static inline int isnan_f  (float       x) { return x != x; }
static inline int isnan_d  (double      x) { return x != x; }
static inline int isnan_ld (long double x) { return x != x; }
#endif

#ifndef isinf
# define isinf(x) \
    (sizeof (x) == sizeof (long double) ? isinf_ld (x) \
    : sizeof (x) == sizeof (double) ? isinf_d (x) \
    : isinf_f (x))
static inline int isinf_f  (float       x)
{ return !isnan (x) && isnan (x - x); }
static inline int isinf_d  (double      x)
{ return !isnan (x) && isnan (x - x); }
static inline int isinf_ld (long double x)
{ return !isnan (x) && isnan (x - x); }
#endif

namespace lcpsolver {

    double RandDouble(double _low, double _high)
    {
        double temp;

        /* swap low & high around if the user makes no sense */
        if (_low > _high)
        {
            temp = _low;
            _low = _high;
            _high = temp;
        }

        /* calculate the random number & return it */
        temp = (rand() / (static_cast<double>(RAND_MAX) + 1.0))
            * (_high - _low) + _low;
        return temp;
    }


    int Lemke(const MatrixXd& _M, const VectorXd& _q, VectorXd& _z)
    {
        int n = _q.size();
        CHECK(_M.rows() == n && _M.cols() == n) << "Matrices are not compatible";
        
	    const double zer_tol = 1e-5;
	    const double piv_tol = 1e-8;
	    int maxiter = 1000;
	    int err = 0;

	    if (_q.minCoeff() > 0)
	    {
		    LOG(INFO) << "Trivial solution exists.";
		    _z = VectorXd::Zero(n);
		    return err;	
	    }

	    _z = VectorXd::Zero(2 * n);
            //            cout << "check 1" << endl;
	    int iter = 0;
	    double theta = 0;
	    double ratio = 0;
	    int leaving  = 0;
	    VectorXd Be = VectorXd::Constant(n, 1);
	    VectorXd x = _q;
	    vector<int> bas;
	    vector<int> nonbas;

	    int t = 2 * n + 1;
	    int entering = t;


	    bas.clear();
	    for (int i = 0; i < n; ++i)
	    {
		    nonbas.push_back(i);
		    bas.push_back(i + n);
	    }
            //            cout << "check 2" << endl;

	    MatrixXd B = -MatrixXd::Identity(n, n);
	    VectorXd minuxX = -x;
	    int lvindex;
	    double tval = minuxX.maxCoeff(&lvindex);
	    leaving = bas[lvindex];

	    bas[lvindex] = t;
	    VectorXd U = VectorXd::Zero(n);
	    for (int i = 0; i < n; ++i)
	    {
		    if (x[i] < 0)
			    U[i] = 1;
	    }
	    Be = -(B * U);
	    x += tval * U;
	    x[lvindex] = tval;
	    B.col(lvindex) = Be;

	    for (iter = 0; iter < maxiter; ++iter)
	    {
		    if (leaving == t)
		    {
			    break;
		    }
		    else if (leaving < n)
		    {
			    entering = n + leaving;
			    Be = VectorXd::Zero(n);
			    Be[leaving] = -1;
		    }
		    else
		    {
			    entering = leaving - n;
			    Be = _M.col(entering);
		    }
		    VectorXd d = B.fullPivLu().solve(Be);

		    vector<int> j;
		    for (int i = 0; i < n; ++i)
		    {
			    if (d[i] > piv_tol)
				    j.push_back(i);
		    }
		    if (j.empty())
		    {
			    err = 2;
			    break;
		    }
		    int jSize = static_cast<int>(j.size());
		    VectorXd minRatio(jSize);
		    for (int i = 0; i < jSize; ++i)
		    {
			    minRatio[i] = (x[j[i]] + zer_tol) / d[j[i]];
		    }
		    double theta = minRatio.minCoeff();
		    vector<int> tmpJ;
                    vector<double> tmpMinRatio;
		    for (int i = 0; i < jSize; ++i)
		    {
                if (x[j[i]] / d[j[i]] <= theta)
                {
                    tmpJ.push_back(j[i]);
                    tmpMinRatio.push_back(minRatio[i]);
                }
		    }
//             if (tmpJ.empty())
//             {
//                 LOG(WARNING) << "tmpJ should never be empty!!!";
//                 LOG(WARNING) << "dumping data:";
//                 LOG(WARNING) << "theta:" << theta;
//                 for (int i = 0; i < jSize; ++i)
//                 {
//                     LOG(WARNING) << "x(" << j[i] << "): " << x[j[i]] << "d: " << d[j[i]];
//                 }
//             }

		    j = tmpJ;
		    jSize = static_cast<int>(j.size());
            if (jSize == 0)
            {
                err = 4;
                break;
            }
		    lvindex = -1;
		    for (int i = 0; i < jSize; ++i)
		    {
			    if (bas[j[i]] == t)
				    lvindex = i;
		    }
		    if (lvindex != -1)
		    {
			    lvindex = j[lvindex]; 
		    }
		    else
		    {
                theta = tmpMinRatio[0];
                for (int i = 0; i < jSize; ++i)
                {
                    if (tmpMinRatio[i] <= theta)
                    {
                        theta = tmpMinRatio[i];
                        lvindex = i;
                    }
                }
// 			    VectorXd dj(jSize);
// 			    for (int i = 0; i < jSize; ++i)
// 			    {
// 				    dj[i] = d[j[i]];
// 			    }
// 			    theta = dj.maxCoeff(&lvindex);
// 			    vector<int> lvindexSet;
// 			    for (int i = 0; i < jSize; ++i)
// 			    {
// 				    if (dj[i] == theta)
// 					    lvindexSet.push_back(i);
// 			    }
//			    lvindex = lvindexSet[static_cast<int>((lvindexSet.size() * RandDouble(0, 1)))];
			    lvindex = j[lvindex];

		    }

		    leaving = bas[lvindex];

		    ratio = x[lvindex] / d[lvindex];

            bool bDiverged = false;
            for (int i = 0; i < n; ++i)
            {
                if (isnan(x[i]) || isinf(x[i]))
                {
                    bDiverged = true;
                    break;
                }
            }
            if (bDiverged)
            {
                err = 4;
                break;
            }
		    x = x - ratio * d;
		    x[lvindex] = ratio;
		    B.col(lvindex) = Be;
		    bas[lvindex] = entering;
    		
	    }
            //            cout << "check 6" << endl;

	    if (iter >= maxiter && leaving != t)
		    err = 1;

	    //for (size_t i = 0; i < bas.size(); ++i)
     //           cout << "bas[i] = " << bas[i] << " ";
     //       cout << endl;

		if (err == 0)
		{
			for (size_t i = 0; i < bas.size(); ++i) {
				_z[bas[i]] = x[i];
			}

			VectorXd realZ = _z.segment(0, n);
			_z = realZ;

            if (!validate(_M, _z, _q))
            {
                _z = VectorXd::Zero(n);
                err = 3;
            }
		}
		else
		{
			_z = VectorXd::Zero(n); //solve failed, return a 0 vector
		}

	    if (err == 1)
		    LOG(ERROR) << "LCP Solver: Iterations exceeded limit";
	    else if (err == 2)
		    LOG(ERROR) << "LCP Solver: Unbounded ray";
        else if (err == 3)
            LOG(ERROR) << "LCP Solver: Solver converged with numerical issues. Validation failed.";
        else if (err == 4)
            LOG(ERROR) << "LCP Solver: Iteration diverged.";
	    return err;
    }

    bool validate(const MatrixXd& _M, const VectorXd& _z, const VectorXd& _q)
    {
        const double threshold = 1e-4;
        int n = _z.size();

        VectorXd w = _M * _z + _q;
        for (int i = 0; i < n; ++i)
        {
            if (w(i) < -threshold || _z(i) < -threshold)
                return false;
            if (abs(w(i) * _z(i)) > threshold)
                return false;
        }
        return true;
    }

} //namespace lcpsolver
