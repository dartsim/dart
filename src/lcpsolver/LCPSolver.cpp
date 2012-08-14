#include "LCPSolver.h"
#include "Lemke.h"
#include <cstdio>
#include "lcp.h"
#include "misc.h"

namespace lcpsolver {

    LCPSolver::LCPSolver()
    {

    }
    LCPSolver::~LCPSolver()
    {

    }

    bool LCPSolver::Solve(const MatrixXd& _A, const VectorXd& _b, VectorXd& _x, double mu, int numDir, bool bUseODESolver)
    {
		if (!bUseODESolver)
		{
			int err = Lemke(_A, _b, _x);
			return (err == 0);
		}
		else
		{
			assert(numDir >= 4);
			MatrixXd AODE;
			VectorXd bODE;
			transferToODEFormulation(_A, _b, AODE, bODE, numDir);
			double* A, *b, *x, *w, *lo, *hi;
			int n = AODE.rows();
			int nContacts = n / 3;

			int nSkip = dPAD(n);

			A = new double[n * nSkip];
			b = new double[n];
			x = new double[n];
			w = new double[n];
			lo = new double[n];
			hi = new double[n];
			int* findex = new int[n];

			memset(A, 0, n * nSkip * sizeof(double));
			for (int i = 0; i < n; ++i)
			{
				for (int j = 0; j < n; ++j)
				{
					A[i * nSkip + j] = AODE(i, j);
				}
			}
			for (int i = 0; i < n; ++i)
			{
				b[i] = -bODE[i];
				x[i] = w[i] = lo[i] = 0;
				hi[i] = dInfinity;
			}
			for (int i = 0; i < nContacts; ++i)
			{
				findex[i] = -1;
				findex[nContacts + i * 2 + 0] = i;
				findex[nContacts + i * 2 + 1] = i;

				lo[nContacts + i * 2 + 0] = -mu;
				lo[nContacts + i * 2 + 1] = -mu;

				hi[nContacts + i * 2 + 0] = mu;
				hi[nContacts + i * 2 + 1] = mu;

			}
			//		dClearUpperTriangle (A,n);
			dSolveLCP (n,A,x,b,w,0,lo,hi,findex);

			VectorXd xODE = VectorXd::Zero(n);
			for (int i = 0; i < n; ++i)
			{
				xODE[i] = x[i];
			}
			transferSolFromODEFormulation(xODE, _x, numDir);

			//		checkIfSolution(reducedA, reducedb, _x);

			delete[] A;
			delete[] b;
			delete[] x;
			delete[] w;
			delete[] lo;
			delete[] hi;
			delete[] findex;
			return 1;
		}

    }

	void LCPSolver::transferToODEFormulation(const MatrixXd& _A, const VectorXd& _b, MatrixXd& AOut, VectorXd& bOut, int numDir)
	{
		int numContacts = _A.rows() / (2 + numDir);
		MatrixXd AIntermediate = MatrixXd::Zero(numContacts * 3, _A.cols());
		AOut = MatrixXd::Zero(numContacts * 3, numContacts * 3);
		bOut = VectorXd::Zero(numContacts * 3);
		int offset = numDir / 4;
		for (int i = 0; i < numContacts; ++i)
		{
			AIntermediate.row(i) = _A.row(i);
			bOut[i] = _b[i];

			AIntermediate.row(numContacts + i * 2 + 0) = _A.row(numContacts + i * numDir + 0);
			AIntermediate.row(numContacts + i * 2 + 1) = _A.row(numContacts + i * numDir + offset);
			bOut[numContacts + i * 2 + 0] = _b[numContacts + i * numDir + 0];
			bOut[numContacts + i * 2 + 1] = _b[numContacts + i * numDir + offset];
		}
		for (int i = 0; i < numContacts; ++i)
		{
			AOut.col(i) = AIntermediate.col(i);
			AOut.col(numContacts + i * 2 + 0) = AIntermediate.col(numContacts + i * numDir + 0);
			AOut.col(numContacts + i * 2 + 1) = AIntermediate.col(numContacts + i * numDir + offset);
		}
		
	}
	void LCPSolver::transferSolFromODEFormulation(const VectorXd& _x, VectorXd& xOut, int numDir)
	{
		int numContacts = _x.size() / 3;
		xOut = VectorXd::Zero(numContacts * (2 + numDir));

		xOut.head(numContacts) = _x.head(numContacts);

		int offset = numDir / 4;
		for (int i = 0; i < numContacts; ++i)
		{
			xOut[numContacts + i * numDir + 0] = _x[numContacts + i * 2 + 0];
			xOut[numContacts + i * numDir + offset] = _x[numContacts + i * 2 + 1];
		}
	}
    bool LCPSolver::checkIfSolution(const MatrixXd& _A, const VectorXd& _b, const VectorXd& _x)
    {
		const double threshold = 1e-4;
		int n = _x.size();

		VectorXd w = _A * _x + _b;
		for (int i = 0; i < n; ++i)
		{
			if (w(i) < -threshold || _x(i) < -threshold)
				return false;
			if (abs(w(i) * _x(i)) > threshold)
				return false;
		}
		return true;
    }



} //namespace lcpsolver
