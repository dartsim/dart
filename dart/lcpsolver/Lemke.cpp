/*
 * Copyright (c) 2011-2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jie (Jay) Tan <jtan34@cc.gatech.edu>,
 *            Yunfei Bai <byf1658@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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

#include "dart/lcpsolver/Lemke.h"

#include <cmath>
#include <iostream>
#include <vector>

#ifndef isnan
# define isnan(x) \
  (sizeof (x) == sizeof (long double) ? isnan_ld (x) \
  : sizeof (x) == sizeof (double) ? isnan_d (x) \
  : isnan_f(x))
static inline int isnan_f  (float       _x) { return _x != _x; }
static inline int isnan_d  (double      _x) { return _x != _x; }
static inline int isnan_ld (long double _x) { return _x != _x; }
#endif

#ifndef isinf
# define isinf(x) \
  (sizeof (x) == sizeof (long double) ? isinf_ld (x) \
  : sizeof (x) == sizeof (double) ? isinf_d (x) \
  : isinf_f(x))
static inline int isinf_f(float       _x)
{ return !isnan (_x) && isnan (_x - _x); }
static inline int isinf_d(double      _x)
{ return !isnan (_x) && isnan (_x - _x); }
static inline int isinf_ld(long double _x)
{ return !isnan (_x) && isnan (_x - _x); }
#endif

namespace dart {
namespace lcpsolver {

// double RandDouble(double _low, double _high) {
//  double temp;

//  /* swap low & high around if the user makes no sense */
//  if (_low > _high) {
//    temp = _low;
//    _low = _high;
//    _high = temp;
//  }

//  /* calculate the random number & return it */
//  temp = (rand() / (static_cast<double>(RAND_MAX) + 1.0))
//         * (_high - _low) + _low;
//  return temp;
// }

int Lemke(const Eigen::MatrixXd& _M, const Eigen::VectorXd& _q,
          Eigen::VectorXd* _z) {
  int n = _q.size();

  const double zer_tol = 1e-5;
  const double piv_tol = 1e-8;
  int maxiter = 1000;
  int err = 0;

  if (_q.minCoeff() > 0) {
    // LOG(INFO) << "Trivial solution exists.";
    *_z = Eigen::VectorXd::Zero(n);
    return err;
  }

  *_z = Eigen::VectorXd::Zero(2 * n);
  int iter = 0;
  double theta = 0;
  double ratio = 0;
  int leaving  = 0;
  Eigen::VectorXd Be = Eigen::VectorXd::Constant(n, 1);
  Eigen::VectorXd x = _q;
  std::vector<int> bas;
  std::vector<int> nonbas;

  int t = 2 * n + 1;
  int entering = t;

  bas.clear();
  nonbas.clear();

  for (int i = 0; i < n; ++i) {
    bas.push_back(i);
  }

  Eigen::MatrixXd B = -Eigen::MatrixXd::Identity(n, n);

  for (int i = 0; i < bas.size(); ++i) {
    B.col(bas[i]) = _M.col(bas[i]);
  }

  x = -B.partialPivLu().solve(_q);

  Eigen::VectorXd minuxX = -x;
  int lvindex;
  double tval = minuxX.maxCoeff(&lvindex);
  leaving = bas[lvindex];
  bas[lvindex] = t;

  Eigen::VectorXd U = Eigen::VectorXd::Zero(n);
  for (int i = 0; i < n; ++i) {
    if (x[i] < 0)
      U[i] = 1;
  }
  Be = -(B * U);
  x += tval * U;
  x[lvindex] = tval;
  B.col(lvindex) = Be;

  for (iter = 0; iter < maxiter; ++iter) {
    if (leaving == t) {
      break;
    } else if (leaving < n) {
      entering = n + leaving;
      Be = Eigen::VectorXd::Zero(n);
      Be[leaving] = -1;
    } else {
      entering = leaving - n;
      Be = _M.col(entering);
    }

    Eigen::VectorXd d = B.partialPivLu().solve(Be);

    std::vector<int> j;
    for (int i = 0; i < n; ++i) {
      if (d[i] > piv_tol)
        j.push_back(i);
    }
    if (j.empty()) {
      // err = 2;
      break;
    }

    int jSize = static_cast<int>(j.size());
    Eigen::VectorXd minRatio(jSize);
    for (int i = 0; i < jSize; ++i) {
      minRatio[i] = (x[j[i]] + zer_tol) / d[j[i]];
    }
    double theta = minRatio.minCoeff();

    std::vector<int> tmpJ;
    std::vector<double> tmpMinRatio;
    for (int i = 0; i < jSize; ++i) {
      if (x[j[i]] / d[j[i]] <= theta) {
        tmpJ.push_back(j[i]);
        tmpMinRatio.push_back(minRatio[i]);
      }
    }
//    if (tmpJ.empty())
//    {
//      LOG(WARNING) << "tmpJ should never be empty!!!";
//      LOG(WARNING) << "dumping data:";
//      LOG(WARNING) << "theta:" << theta;
//      for (int i = 0; i < jSize; ++i)
//      {
//        LOG(WARNING) << "x(" << j[i] << "): " << x[j[i]] << "d: " << d[j[i]];
//      }
//    }

    j = tmpJ;
    jSize = static_cast<int>(j.size());
    if (jSize == 0) {
      err = 4;
      break;
    }
    lvindex = -1;

    for (int i = 0; i < jSize; ++i) {
      if (bas[j[i]] == t)
        lvindex = i;
    }
    if (lvindex != -1) {
      lvindex = j[lvindex];
    } else {
      theta = tmpMinRatio[0];
      lvindex = 0;

      for (int i = 0; i < jSize; ++i) {
        if (tmpMinRatio[i]-theta > piv_tol) {
          theta = tmpMinRatio[i];
          lvindex = i;
        }
      }
      lvindex = j[lvindex];
    }

    leaving = bas[lvindex];

    ratio = x[lvindex] / d[lvindex];

    bool bDiverged = false;
    for (int i = 0; i < n; ++i) {
      if (isnan(x[i]) || isinf(x[i])) {
        bDiverged = true;
        break;
      }
    }
    if (bDiverged) {
      err = 4;
      break;
    }
    x = x - ratio * d;
    x[lvindex] = ratio;
    B.col(lvindex) = Be;
    bas[lvindex] = entering;
  }

  if (iter >= maxiter && leaving != t)
    err = 1;

  if (err == 0) {
    for (size_t i = 0; i < bas.size(); ++i) {
      if (bas[i] < _z->size()) {
        (*_z)[bas[i]] = x[i];
      }
    }

    Eigen::VectorXd realZ = _z->segment(0, n);
    *_z = realZ;

    if (!validate(_M, *_z, _q)) {
      // _z = VectorXd::Zero(n);
      err = 3;
    }
  } else {
    *_z = Eigen::VectorXd::Zero(n);  // solve failed, return a 0 vector
  }

//  if (err == 1)
//    LOG(ERROR) << "LCP Solver: Iterations exceeded limit";
//  else if (err == 2)
//    LOG(ERROR) << "LCP Solver: Unbounded ray";
//  else if (err == 3)
//    LOG(ERROR) << "LCP Solver: Solver converged with numerical issues. "
//               << "Validation failed.";
//  else if (err == 4)
//    LOG(ERROR) << "LCP Solver: Iteration diverged.";

  return err;
}

bool validate(const Eigen::MatrixXd& _M, const Eigen::VectorXd& _z,
              const Eigen::VectorXd& _q) {
  const double threshold = 1e-4;
  int n = _z.size();

  Eigen::VectorXd w = _M * _z + _q;
  for (int i = 0; i < n; ++i) {
    if (w(i) < -threshold || _z(i) < -threshold)
      return false;
    if (abs(w(i) * _z(i)) > threshold)
      return false;
  }
  return true;
}

}  // namespace lcpsolver
}  // namespace dart
