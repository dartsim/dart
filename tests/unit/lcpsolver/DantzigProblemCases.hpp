#ifndef DART_TEST_UNIT_LCPSOLVER_DANTZIGPROBLEMCASES_HPP_
#define DART_TEST_UNIT_LCPSOLVER_DANTZIGPROBLEMCASES_HPP_

#include "dart/lcpsolver/dantzig/DantzigCommon.hpp"
#include "dart/lcpsolver/dantzig/DantzigLcp.hpp"
#include "tests/baseline/odelcpsolver/Lcp.h"

#include <algorithm>
#include <cstddef>
#include <vector>

namespace dart::test {

struct DantzigProblemCase
{
  const char* name = "";
  int n = 0;
  int nub = 0;
  std::vector<double> A;
  std::vector<double> b;
  std::vector<double> lo;
  std::vector<double> hi;
  std::vector<int> findex;
};

struct DantzigProblemWorkspace
{
  DantzigProblemWorkspace() = default;

  explicit DantzigProblemWorkspace(const DantzigProblemCase& problem)
  {
    resize(problem);
  }

  void resize(const DantzigProblemCase& problem)
  {
    A.resize(problem.A.size());
    b.resize(problem.b.size());
    lo.resize(problem.lo.size());
    hi.resize(problem.hi.size());
    findex.resize(problem.findex.size());
    x.resize(static_cast<std::size_t>(problem.n));
    w.resize(static_cast<std::size_t>(problem.n));
  }

  void reset(const DantzigProblemCase& problem)
  {
    if (A.size() != problem.A.size()) {
      resize(problem);
    }

    std::copy(problem.A.begin(), problem.A.end(), A.begin());
    std::copy(problem.b.begin(), problem.b.end(), b.begin());
    std::copy(problem.lo.begin(), problem.lo.end(), lo.begin());
    std::copy(problem.hi.begin(), problem.hi.end(), hi.begin());
    std::copy(problem.findex.begin(), problem.findex.end(), findex.begin());
    std::fill(x.begin(), x.end(), 0.0);
    std::fill(w.begin(), w.end(), 0.0);
  }

  std::vector<double> A;
  std::vector<double> b;
  std::vector<double> lo;
  std::vector<double> hi;
  std::vector<int> findex;
  std::vector<double> x;
  std::vector<double> w;
};

inline int dantzigStride(int n)
{
  return dart::lcpsolver::dantzig::padding(n);
}

inline DantzigProblemCase makeDantzigProblemCase(
    const char* name, int n, int nub)
{
  DantzigProblemCase problem;
  problem.name = name;
  problem.n = n;
  problem.nub = nub;
  problem.A.assign(
      static_cast<std::size_t>(n * dantzigStride(n)), 0.0);
  problem.b.assign(static_cast<std::size_t>(n), 0.0);
  problem.lo.assign(static_cast<std::size_t>(n), 0.0);
  problem.hi.assign(static_cast<std::size_t>(n), 0.0);
  problem.findex.assign(static_cast<std::size_t>(n), -1);
  return problem;
}

inline void setMatrixEntry(
    DantzigProblemCase& problem, int row, int col, double value)
{
  problem.A[static_cast<std::size_t>(row * dantzigStride(problem.n) + col)]
      = value;
}

inline void setSymmetricMatrixEntry(
    DantzigProblemCase& problem, int row, int col, double value)
{
  setMatrixEntry(problem, row, col, value);
  setMatrixEntry(problem, col, row, value);
}

inline DantzigProblemCase makeUnboundedDantzigCase(int n)
{
  auto problem = makeDantzigProblemCase("unbounded", n, n);
  for (int row = 0; row < n; ++row) {
    problem.b[static_cast<std::size_t>(row)] = 1.0 + 0.25 * row;
    problem.lo[static_cast<std::size_t>(row)]
        = -dart::lcpsolver::dantzig::kInfinity;
    problem.hi[static_cast<std::size_t>(row)]
        = dart::lcpsolver::dantzig::kInfinity;
    for (int col = 0; col <= row; ++col) {
      const double value = (row == col) ? 4.0 + 0.1 * row : 0.03 * (col + 1);
      setSymmetricMatrixEntry(problem, row, col, value);
    }
  }
  return problem;
}

inline DantzigProblemCase makeUnboundedDiagonalDantzigCase(int n)
{
  auto problem = makeDantzigProblemCase("unbounded_diagonal", n, n);
  for (int row = 0; row < n; ++row) {
    setMatrixEntry(problem, row, row, 2.0 + 0.125 * row);
    problem.b[static_cast<std::size_t>(row)] = 1.0 + 0.2 * row;
    problem.lo[static_cast<std::size_t>(row)]
        = -dart::lcpsolver::dantzig::kInfinity;
    problem.hi[static_cast<std::size_t>(row)]
        = dart::lcpsolver::dantzig::kInfinity;
  }
  return problem;
}

inline DantzigProblemCase makeBoxedDiagonalDantzigCase(int n)
{
  auto problem = makeDantzigProblemCase("boxed_diagonal", n, 0);
  for (int row = 0; row < n; ++row) {
    setMatrixEntry(problem, row, row, 2.0 + 0.125 * row);
    problem.b[static_cast<std::size_t>(row)] = 1.0 + 0.2 * row;
    problem.lo[static_cast<std::size_t>(row)] = 0.0;
    problem.hi[static_cast<std::size_t>(row)] = 5.0;
  }
  return problem;
}

inline DantzigProblemCase makePartiallyUnboundedDiagonalDantzigCase()
{
  auto problem = makeDantzigProblemCase("partially_unbounded_diagonal", 3, 1);

  setMatrixEntry(problem, 0, 0, 2.0);
  setMatrixEntry(problem, 1, 1, 3.0);
  setMatrixEntry(problem, 2, 2, 4.0);

  problem.b = {4.0, 6.0, -2.0};
  problem.lo = {0.0, 0.0, -1.0};
  problem.hi = {0.0, 1.0, 1.0};

  return problem;
}

inline DantzigProblemCase makeBoxedCoupledDantzigCase(int n)
{
  auto problem = makeDantzigProblemCase("boxed_coupled", n, 0);
  for (int row = 0; row < n; ++row) {
    problem.b[static_cast<std::size_t>(row)]
        = (row % 2 == 0) ? 0.6 + 0.1 * row : -0.35 - 0.05 * row;
    problem.lo[static_cast<std::size_t>(row)] = (row % 3 == 0) ? -1.0 : 0.0;
    problem.hi[static_cast<std::size_t>(row)] = 3.0 + 0.25 * (row % 4);
    for (int col = 0; col <= row; ++col) {
      const double value
          = (row == col) ? 5.0 + 0.2 * row : 0.04 / (1 + row - col);
      setSymmetricMatrixEntry(problem, row, col, value);
    }
  }
  return problem;
}

inline DantzigProblemCase makeFrictionDantzigCase(int contacts)
{
  const int normalCount = contacts;
  const int tangentCount = contacts * 2;
  const int n = normalCount + tangentCount;
  auto problem = makeDantzigProblemCase("friction", n, normalCount);
  for (int row = 0; row < n; ++row) {
    problem.b[static_cast<std::size_t>(row)] = 0.4 + 0.07 * row;
    problem.lo[static_cast<std::size_t>(row)] = 0.0;
    problem.hi[static_cast<std::size_t>(row)] = 4.0;
    for (int col = 0; col <= row; ++col) {
      const double value
          = (row == col) ? 6.0 + 0.1 * row : 0.02 / (1 + row - col);
      setSymmetricMatrixEntry(problem, row, col, value);
    }
  }

  for (int row = 0; row < normalCount; ++row) {
    problem.lo[static_cast<std::size_t>(row)]
        = -dart::lcpsolver::dantzig::kInfinity;
    problem.hi[static_cast<std::size_t>(row)]
        = dart::lcpsolver::dantzig::kInfinity;
  }

  for (int contact = 0; contact < contacts; ++contact) {
    for (int dir = 0; dir < 2; ++dir) {
      const int index = normalCount + contact * 2 + dir;
      problem.lo[static_cast<std::size_t>(index)] = -0.8;
      problem.hi[static_cast<std::size_t>(index)] = 0.8;
      problem.findex[static_cast<std::size_t>(index)] = contact;
    }
  }

  return problem;
}

inline std::vector<DantzigProblemCase> makeDantzigCorrectnessCases()
{
  return {
      makeUnboundedDantzigCase(1),
      makeUnboundedDantzigCase(8),
      makeBoxedDiagonalDantzigCase(4),
      makePartiallyUnboundedDiagonalDantzigCase(),
      makeBoxedCoupledDantzigCase(8),
      makeFrictionDantzigCase(3),
  };
}

inline std::vector<DantzigProblemCase> makeDantzigPerformanceCases()
{
  return {
      makeUnboundedDiagonalDantzigCase(96),
      makeBoxedDiagonalDantzigCase(96),
      makeBoxedCoupledDantzigCase(96),
      makeFrictionDantzigCase(32),
  };
}

inline bool solveDantzigNative(
    DantzigProblemCase problem,
    std::vector<double>* x,
    dart::lcpsolver::dantzig::DantzigLcpScratch<double>* scratch = nullptr)
{
  x->assign(static_cast<std::size_t>(problem.n), 0.0);
  std::vector<double> w(static_cast<std::size_t>(problem.n), 0.0);
  if (scratch) {
    return dart::lcpsolver::dantzig::solveLcpWithScratch<double>(
        problem.n,
        problem.A.data(),
        x->data(),
        problem.b.data(),
        w.data(),
        problem.nub,
        problem.lo.data(),
        problem.hi.data(),
        problem.findex.data(),
        *scratch);
  }

  return dart::lcpsolver::dantzig::solveLcp<double>(
      problem.n,
      problem.A.data(),
      x->data(),
      problem.b.data(),
      w.data(),
      problem.nub,
      problem.lo.data(),
      problem.hi.data(),
      problem.findex.data());
}

inline bool solveDantzigNativeWorkspace(
    const DantzigProblemCase& problem,
    DantzigProblemWorkspace* workspace,
    dart::lcpsolver::dantzig::DantzigLcpScratch<double>* scratch = nullptr)
{
  workspace->reset(problem);
  if (scratch) {
    return dart::lcpsolver::dantzig::solveLcpWithScratch<double>(
        problem.n,
        workspace->A.data(),
        workspace->x.data(),
        workspace->b.data(),
        workspace->w.data(),
        problem.nub,
        workspace->lo.data(),
        workspace->hi.data(),
        workspace->findex.data(),
        *scratch);
  }

  return dart::lcpsolver::dantzig::solveLcp<double>(
      problem.n,
      workspace->A.data(),
      workspace->x.data(),
      workspace->b.data(),
      workspace->w.data(),
      problem.nub,
      workspace->lo.data(),
      workspace->hi.data(),
      workspace->findex.data());
}

inline bool solveDantzigBaseline(
    DantzigProblemCase problem, std::vector<double>* x)
{
  x->assign(static_cast<std::size_t>(problem.n), 0.0);
  std::vector<double> w(static_cast<std::size_t>(problem.n), 0.0);
  return dart::baseline::ode::dSolveLCP(
      problem.n,
      problem.A.data(),
      x->data(),
      problem.b.data(),
      w.data(),
      problem.nub,
      problem.lo.data(),
      problem.hi.data(),
      problem.findex.data());
}

inline bool solveDantzigBaselineWorkspace(
    const DantzigProblemCase& problem, DantzigProblemWorkspace* workspace)
{
  workspace->reset(problem);
  return dart::baseline::ode::dSolveLCP(
      problem.n,
      workspace->A.data(),
      workspace->x.data(),
      workspace->b.data(),
      workspace->w.data(),
      problem.nub,
      workspace->lo.data(),
      workspace->hi.data(),
      workspace->findex.data());
}

inline double solutionChecksum(const std::vector<double>& x)
{
  double checksum = 0.0;
  for (std::size_t i = 0; i < x.size(); ++i) {
    checksum += (1.0 + static_cast<double>(i)) * x[i];
  }
  return checksum;
}

} // namespace dart::test

#endif // DART_TEST_UNIT_LCPSOLVER_DANTZIGPROBLEMCASES_HPP_
