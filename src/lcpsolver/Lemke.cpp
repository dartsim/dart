#include "Lemke.h"
#include <iostream>

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
	    int maxiter = min(1000, 50 * n);
	    int err = 0;

	    if (_q.minCoeff() > 0)
	    {
		    LOG(INFO) << "Trivial solution exists.";
		    _z = VectorXd::Zero(n);
		    return err;	
	    }

	    _z = VectorXd::Zero(2 * n);

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
		    for (int i = 0; i < jSize; ++i)
		    {
			    if (x[j[i]] / d[j[i]] <= theta)
				    tmpJ.push_back(j[i]);
		    }
		    j = tmpJ;
		    jSize = static_cast<int>(j.size());
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
			    VectorXd dj(jSize);
			    for (int i = 0; i < jSize; ++i)
			    {
				    dj[i] = d[j[i]];
			    }
			    theta = dj.maxCoeff(&lvindex);
			    vector<int> lvindexSet;
			    for (int i = 0; i < jSize; ++i)
			    {
				    if (dj[i] == theta)
					    lvindexSet.push_back(i);
			    }

			    lvindex = lvindexSet[static_cast<int>((lvindexSet.size() * RandDouble(0, 1)))];
			    lvindex = j[lvindex];

		    }
		    leaving = bas[lvindex];

		    ratio = x[lvindex] / d[lvindex];
		    x = x - ratio * d;
		    x[lvindex] = ratio;
		    B.col(lvindex) = Be;
		    bas[lvindex] = entering;
    		
	    }

	    if (iter >= maxiter && leaving != t)
		    err = 1;

	    for (size_t i = 0; i < bas.size(); ++i)
                cout << "bas[i] = " << bas[i] << " ";
            cout << endl;
	    for (size_t i = 0; i < bas.size(); ++i) {
		    _z[bas[i]] = x[i];
	    }

	    VectorXd realZ = _z.segment(0, n);
	    _z = realZ;

	    if (err == 1)
		    LOG(ERROR) << "LCP Solver: Iterations exceeded limit";
	    else if (err == 2)
		    LOG(ERROR) << "LCP Solver: Unbounded ray";
	    return err;
    }

} //namespace lcpsolver
