/*
 * Copyright (c) 2011-2025, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "dart/math/lcp/ODELCPSolver.hpp"

#include "dart/common/Macros.hpp"
#include "dart/math/lcp/Lemke.hpp"
#include "dart/math/lcp/dantzig/Lcp.hpp"

#include <cstdio>

namespace dart::math {

//==============================================================================
ODELCPSolver::ODELCPSolver()
{
  // Do nothing
}

//==============================================================================
ODELCPSolver::~ODELCPSolver()
{
  // Do nothing
}

//==============================================================================
bool ODELCPSolver::Solve(
    const Eigen::MatrixXd& _A,
    const Eigen::VectorXd& _b,
    Eigen::VectorXd* _x,
    int _numContacts,
    double _mu,
    int _numDir,
    bool _bUseODESolver)
{
  if (!_bUseODESolver) {
    int err = Lemke(_A, _b, _x);
    return (err == 0);
  } else {
    DART_ASSERT(_numDir >= 4);
    DART_UNUSED(_numDir);

    double *A, *b, *x, *w, *lo, *hi;
    int n = _A.rows();

    int nSkip = padding(n);

    A = new double[n * nSkip];
    b = new double[n];
    x = new double[n];
    w = new double[n];
    lo = new double[n];
    hi = new double[n];
    int* findex = new int[n];

    memset(A, 0, n * nSkip * sizeof(double));
    for (int i = 0; i < n; ++i) {
      for (int j = 0; j < n; ++j) {
        A[i * nSkip + j] = _A(i, j);
      }
    }
    const double inf = ScalarTraits<double>::inf();
    for (int i = 0; i < n; ++i) {
      b[i] = -_b[i];
      x[i] = w[i] = lo[i] = 0;
      hi[i] = inf;
      findex[i] = -1;
    }
    for (int i = 0; i < _numContacts; ++i) {
      findex[_numContacts + i * 2 + 0] = i;
      findex[_numContacts + i * 2 + 1] = i;

      lo[_numContacts + i * 2 + 0] = -_mu;
      lo[_numContacts + i * 2 + 1] = -_mu;

      hi[_numContacts + i * 2 + 0] = _mu;
      hi[_numContacts + i * 2 + 1] = _mu;
    }
    // dClearUpperTriangle (A,n);
    SolveLCP<double>(n, A, x, b, w, 0, lo, hi, findex, false);

    //    for (int i = 0; i < n; i++) {
    //      if (w[i] < 0.0 && abs(x[i] - hi[i]) > 0.000001)
    //        cout << "w[" << i << "] is negative, but x is " << x[i] << endl;
    //      else if (w[i] > 0.0 && abs(x[i] - lo[i]) > 0.000001)
    //        cout << "w[" << i << "] is positive, but x is " << x[i]
    //                << " lo is " <<  lo[i] << endl;
    //      else if (abs(w[i]) < 0.000001 && (x[i] > hi[i] || x[i] < lo[i]))
    //        cout << "w[i] " << i << " is zero, but x is " << x[i] << endl;
    //    }

    *_x = Eigen::VectorXd(n);
    for (int i = 0; i < n; ++i) {
      (*_x)[i] = x[i];
    }

    // checkIfSolution(reducedA, reducedb, _x);

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

//==============================================================================
void ODELCPSolver::transferToODEFormulation(
    const Eigen::MatrixXd& _A,
    const Eigen::VectorXd& _b,
    Eigen::MatrixXd* _AOut,
    Eigen::VectorXd* _bOut,
    int _numDir,
    int _numContacts)
{
  const Eigen::Index numContacts = static_cast<Eigen::Index>(_numContacts);
  const Eigen::Index numDir = static_cast<Eigen::Index>(_numDir);
  const Eigen::Index numOtherConstrs = _A.rows() - numContacts * (2 + numDir);
  const Eigen::Index n = numContacts * 3 + numOtherConstrs;
  Eigen::MatrixXd AIntermediate = Eigen::MatrixXd::Zero(n, _A.cols());
  *_AOut = Eigen::MatrixXd::Zero(n, n);
  *_bOut = Eigen::VectorXd::Zero(n);
  const Eigen::Index offset = numDir / 4;
  for (Eigen::Index i = 0; i < numContacts; ++i) {
    AIntermediate.row(i) = _A.row(i);
    (*_bOut)[i] = _b[i];

    AIntermediate.row(numContacts + i * 2 + 0)
        = _A.row(numContacts + i * numDir + 0);
    AIntermediate.row(numContacts + i * 2 + 1)
        = _A.row(numContacts + i * numDir + offset);
    (*_bOut)[numContacts + i * 2 + 0] = _b[numContacts + i * numDir + 0];
    (*_bOut)[numContacts + i * 2 + 1] = _b[numContacts + i * numDir + offset];
  }
  for (Eigen::Index i = 0; i < numOtherConstrs; ++i) {
    AIntermediate.row(numContacts * 3 + i)
        = _A.row(numContacts * (numDir + 2) + i);
    (*_bOut)[numContacts * 3 + i] = _b[numContacts * (numDir + 2) + i];
  }
  for (Eigen::Index i = 0; i < numContacts; ++i) {
    _AOut->col(i) = AIntermediate.col(i);
    _AOut->col(numContacts + i * 2 + 0)
        = AIntermediate.col(numContacts + i * numDir + 0);
    _AOut->col(numContacts + i * 2 + 1)
        = AIntermediate.col(numContacts + i * numDir + offset);
  }
  for (Eigen::Index i = 0; i < numOtherConstrs; ++i)
    _AOut->col(numContacts * 3 + i)
        = AIntermediate.col(numContacts * (numDir + 2) + i);
}

//==============================================================================
void ODELCPSolver::transferSolFromODEFormulation(
    const Eigen::VectorXd& _x,
    Eigen::VectorXd* _xOut,
    int _numDir,
    int _numContacts)
{
  const Eigen::Index numContacts = static_cast<Eigen::Index>(_numContacts);
  const Eigen::Index numDir = static_cast<Eigen::Index>(_numDir);
  const Eigen::Index numOtherConstrs = _x.size() - numContacts * 3;
  *_xOut = Eigen::VectorXd::Zero(numContacts * (2 + numDir) + numOtherConstrs);

  _xOut->head(numContacts) = _x.head(numContacts);

  const Eigen::Index offset = numDir / 4;
  for (Eigen::Index i = 0; i < numContacts; ++i) {
    (*_xOut)[numContacts + i * numDir + 0] = _x[numContacts + i * 2 + 0];
    (*_xOut)[numContacts + i * numDir + offset] = _x[numContacts + i * 2 + 1];
  }
  for (Eigen::Index i = 0; i < numOtherConstrs; ++i)
    (*_xOut)[numContacts * (2 + numDir) + i] = _x[numContacts * 3 + i];
}

//==============================================================================
bool ODELCPSolver::checkIfSolution(
    const Eigen::MatrixXd& _A,
    const Eigen::VectorXd& _b,
    const Eigen::VectorXd& _x)
{
  const double threshold = 1e-4;
  int n = _x.size();

  Eigen::VectorXd w = _A * _x + _b;
  for (int i = 0; i < n; ++i) {
    if (w(i) < -threshold || _x(i) < -threshold)
      return false;
    if (std::abs(w(i) * _x(i)) > threshold)
      return false;
  }
  return true;
}

} // namespace dart::math
