/*
 * Copyright (c) 2011, The DART development contributors
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

#include <dart/gui/im_gui_handler.hpp>
#include <dart/gui/im_gui_viewer.hpp>
#include <dart/gui/im_gui_widget.hpp>
#include <dart/gui/include_im_gui.hpp>

#include <dart/simulation/world.hpp>

#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/simple_frame.hpp>

#include <dart/math/lcp/all.hpp>
#include <dart/math/lcp/lcp_validation.hpp>

#include <CLI/CLI.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/QR>

#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>
#include <numeric>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

#include <cmath>

namespace {

constexpr int kMaxHistory = 120;

struct ContractCheck
{
  bool finiteOk{false};
  bool boundsOk{false};
  bool residualOk{false};
  bool complementarityOk{false};
  bool validated{false};
  bool ok{false};

  double tol{std::numeric_limits<double>::quiet_NaN()};
  double compTol{std::numeric_limits<double>::quiet_NaN()};
  double residual{std::numeric_limits<double>::quiet_NaN()};
  double complementarity{std::numeric_limits<double>::quiet_NaN()};
  double boundViolation{std::numeric_limits<double>::quiet_NaN()};

  std::string message;
};

struct RunStats
{
  bool hasResult{false};
  dart::math::LcpResult result;
  ContractCheck check;
  double avgMs{0.0};
  double minMs{0.0};
  double maxMs{0.0};
  std::vector<float> historyMs;
};

double MatrixInfinityNorm(const Eigen::MatrixXd& m)
{
  if (m.size() == 0)
    return 0.0;
  return m.cwiseAbs().rowwise().sum().maxCoeff();
}

double VectorInfinityNorm(const Eigen::VectorXd& v)
{
  return v.size() > 0 ? v.cwiseAbs().maxCoeff() : 0.0;
}

double ComputeBoundViolation(
    const Eigen::VectorXd& lo,
    const Eigen::VectorXd& hi,
    const Eigen::VectorXd& x)
{
  double violation = 0.0;
  for (Eigen::Index i = 0; i < x.size(); ++i) {
    if (std::isfinite(lo[i]))
      violation = std::max(violation, lo[i] - x[i]);
    if (std::isfinite(hi[i]))
      violation = std::max(violation, x[i] - hi[i]);
  }
  return std::max(0.0, violation);
}

ContractCheck CheckSolution(
    const dart::math::LcpProblem& problem,
    const Eigen::VectorXd& x,
    const dart::math::LcpOptions& options)
{
  ContractCheck report;

  if (x.size() != problem.b.size()) {
    report.message = "Solution size does not match problem dimension";
    return report;
  }

  const Eigen::VectorXd w = problem.A * x - problem.b;
  report.finiteOk = x.allFinite() && w.allFinite();
  if (!report.finiteOk) {
    report.message = "Non-finite values in solution";
    return report;
  }

  Eigen::VectorXd loEff;
  Eigen::VectorXd hiEff;
  std::string boundsMessage;
  if (!dart::math::detail::computeEffectiveBounds(
          problem.lo,
          problem.hi,
          problem.findex,
          x,
          loEff,
          hiEff,
          &boundsMessage)) {
    report.message = boundsMessage;
    return report;
  }

  const double absTol
      = (options.absoluteTolerance > 0.0) ? options.absoluteTolerance : 1e-6;
  const double relTol
      = (options.relativeTolerance > 0.0) ? options.relativeTolerance : 1e-4;
  const double compTolOpt = (options.complementarityTolerance > 0.0)
                                ? options.complementarityTolerance
                                : absTol;

  const double scale = std::max(
      1.0,
      std::max(
          VectorInfinityNorm(problem.b),
          MatrixInfinityNorm(problem.A) * VectorInfinityNorm(x)));
  report.tol = std::max(absTol, relTol * scale);
  report.compTol = std::max(compTolOpt, relTol * scale);

  report.residual
      = dart::math::detail::naturalResidualInfinityNorm(x, w, loEff, hiEff);
  report.complementarity = dart::math::detail::complementarityInfinityNorm(
      x, w, loEff, hiEff, report.compTol);
  report.boundViolation = ComputeBoundViolation(loEff, hiEff, x);

  report.residualOk = report.residual <= report.tol;
  report.complementarityOk = report.complementarity <= report.compTol;
  report.boundsOk = report.boundViolation <= report.tol;

  std::string validationMessage;
  const double validationTol = std::max(report.tol, report.compTol);
  report.validated = dart::math::detail::validateSolution(
      x, w, loEff, hiEff, validationTol, &validationMessage);
  if (!report.validated && report.message.empty())
    report.message = validationMessage;

  report.ok = report.finiteOk && report.boundsOk && report.residualOk
              && report.complementarityOk && report.validated;

  if (!report.ok && report.message.empty())
    report.message = "Solution violates comparison contract";

  return report;
}

Eigen::MatrixXd MakeSpdMatrix(int n, unsigned seed, double diagShift)
{
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> dist(-1.0, 1.0);

  Eigen::MatrixXd M(n, n);
  for (int r = 0; r < n; ++r) {
    for (int c = 0; c < n; ++c)
      M(r, c) = dist(rng);
  }

  Eigen::MatrixXd A = M.transpose() * M;
  A += diagShift * Eigen::MatrixXd::Identity(n, n);
  return A;
}

Eigen::MatrixXd MakeIllConditionedSpd(int n, unsigned seed)
{
  std::mt19937 rng(seed);
  std::normal_distribution<double> dist(0.0, 1.0);

  Eigen::MatrixXd M(n, n);
  for (int r = 0; r < n; ++r) {
    for (int c = 0; c < n; ++c)
      M(r, c) = dist(rng);
  }

  Eigen::HouseholderQR<Eigen::MatrixXd> qr(M);
  Eigen::MatrixXd Q = qr.householderQ() * Eigen::MatrixXd::Identity(n, n);

  Eigen::VectorXd eigs(n);
  const double minExp = -6.0;
  const double maxExp = 1.5;
  for (int i = 0; i < n; ++i) {
    const double t = (n > 1) ? static_cast<double>(i) / (n - 1) : 0.0;
    eigs[i] = std::pow(10.0, minExp + (maxExp - minExp) * t);
  }

  return Q * eigs.asDiagonal() * Q.transpose();
}

Eigen::MatrixXd MakeMassRatioSpd(int n, unsigned seed)
{
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> dist(-0.1, 0.1);

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);
  for (int i = 0; i < n; ++i) {
    const double t = (n > 1) ? static_cast<double>(i) / (n - 1) : 0.0;
    A(i, i) = std::pow(10.0, -3.0 + 6.0 * t);
  }

  for (int r = 0; r < n; ++r) {
    for (int c = r + 1; c < n; ++c) {
      const double v = 0.01 * dist(rng);
      A(r, c) += v;
      A(c, r) += v;
    }
  }

  A += 0.05 * Eigen::MatrixXd::Identity(n, n);
  return A;
}

Eigen::MatrixXd MakeContactSpdMatrix(int contacts, unsigned seed)
{
  const int n = contacts * 3;
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);

  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> dist(-0.15, 0.15);

  for (int i = 0; i < contacts; ++i) {
    Eigen::Matrix3d block;
    block << 3.0 + dist(rng), 0.2, 0.0, 0.2, 2.6 + dist(rng), 0.1, 0.0, 0.1,
        2.2 + dist(rng);
    A.block<3, 3>(3 * i, 3 * i) = block;
  }

  for (int i = 0; i + 1 < contacts; ++i) {
    const double coupling = 0.05 + 0.02 * dist(rng);
    const int a = 3 * i;
    const int b = 3 * (i + 1);
    A(a, b) += coupling;
    A(b, a) += coupling;
    A(a + 1, b + 1) += 0.5 * coupling;
    A(b + 1, a + 1) += 0.5 * coupling;
    A(a + 2, b + 2) += 0.5 * coupling;
    A(b + 2, a + 2) += 0.5 * coupling;
  }

  A += 0.3 * Eigen::MatrixXd::Identity(n, n);
  return A;
}

Eigen::VectorXd MakePositiveVector(
    int n, unsigned seed, double minVal, double maxVal)
{
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> dist(minVal, maxVal);
  Eigen::VectorXd x(n);
  for (int i = 0; i < n; ++i)
    x[i] = dist(rng);
  return x;
}

std::vector<int> BuildUniformBlockSizes(int n, int blockSize)
{
  std::vector<int> sizes;
  if (blockSize <= 0 || n <= 0)
    return sizes;

  int remaining = n;
  while (remaining > 0) {
    const int size = std::min(blockSize, remaining);
    sizes.push_back(size);
    remaining -= size;
  }
  return sizes;
}

std::vector<int> ComputeFindexBlockSizes(const Eigen::VectorXi& findex)
{
  const int n = static_cast<int>(findex.size());
  if (n == 0)
    return {};

  std::vector<int> parent(n);
  std::iota(parent.begin(), parent.end(), 0);

  auto findRoot = [&](int v) {
    while (parent[v] != v) {
      parent[v] = parent[parent[v]];
      v = parent[v];
    }
    return v;
  };

  auto unite = [&](int a, int b) {
    const int rootA = findRoot(a);
    const int rootB = findRoot(b);
    if (rootA != rootB)
      parent[rootB] = rootA;
  };

  for (int i = 0; i < n; ++i) {
    const int ref = findex[i];
    if (ref >= 0)
      unite(i, ref);
  }

  std::unordered_map<int, int> counts;
  for (int i = 0; i < n; ++i)
    ++counts[findRoot(i)];

  std::vector<int> sizes;
  sizes.reserve(counts.size());
  for (const auto& entry : counts)
    sizes.push_back(entry.second);
  std::ranges::sort(sizes);
  return sizes;
}

struct ExampleCase
{
  std::string label;
  std::string category;
  std::string description;
  std::vector<std::string> references;
  unsigned seed{0};
  bool isBoxed{false};
  bool hasFindex{false};
  dart::math::LcpProblem problem;

  ExampleCase(
      std::string labelText,
      std::string categoryText,
      std::string descriptionText,
      std::vector<std::string> referenceTags,
      unsigned seedValue,
      bool boxed,
      bool findex,
      dart::math::LcpProblem lcpProblem)
    : label(std::move(labelText)),
      category(std::move(categoryText)),
      description(std::move(descriptionText)),
      references(std::move(referenceTags)),
      seed(seedValue),
      isBoxed(boxed),
      hasFindex(findex),
      problem(std::move(lcpProblem))
  {
  }
};

enum class SolverType
{
  Dantzig,
  Lemke,
  Baraff,
  Direct,
  Pgs,
  SymmetricPsor,
  Jacobi,
  RedBlack,
  BlockedJacobi,
  Bgs,
  Nncg,
  SubspaceMinimization,
  MinimumMapNewton,
  FischerBurmeisterNewton,
  PenalizedFischerBurmeisterNewton,
  InteriorPoint,
  Mprgp,
  ShockPropagation,
  Staggering
};

struct SolverInfo
{
  SolverType type;
  std::string name;
  std::string category;
  bool supportsStandard{true};
  bool supportsBoxed{true};
  bool supportsFindex{true};
  std::string fallbackNote;
  std::vector<std::string> pros;
  std::vector<std::string> cons;
};

std::unique_ptr<dart::math::LcpSolver> CreateSolver(SolverType type)
{
  using namespace dart::math;
  switch (type) {
    case SolverType::Dantzig:
      return std::make_unique<DantzigSolver>();
    case SolverType::Lemke:
      return std::make_unique<LemkeSolver>();
    case SolverType::Baraff:
      return std::make_unique<BaraffSolver>();
    case SolverType::Direct:
      return std::make_unique<DirectSolver>();
    case SolverType::Pgs:
      return std::make_unique<PgsSolver>();
    case SolverType::SymmetricPsor:
      return std::make_unique<SymmetricPsorSolver>();
    case SolverType::Jacobi:
      return std::make_unique<JacobiSolver>();
    case SolverType::RedBlack:
      return std::make_unique<RedBlackGaussSeidelSolver>();
    case SolverType::BlockedJacobi:
      return std::make_unique<BlockedJacobiSolver>();
    case SolverType::Bgs:
      return std::make_unique<BgsSolver>();
    case SolverType::Nncg:
      return std::make_unique<NncgSolver>();
    case SolverType::SubspaceMinimization:
      return std::make_unique<SubspaceMinimizationSolver>();
    case SolverType::MinimumMapNewton:
      return std::make_unique<MinimumMapNewtonSolver>();
    case SolverType::FischerBurmeisterNewton:
      return std::make_unique<FischerBurmeisterNewtonSolver>();
    case SolverType::PenalizedFischerBurmeisterNewton:
      return std::make_unique<PenalizedFischerBurmeisterNewtonSolver>();
    case SolverType::InteriorPoint:
      return std::make_unique<InteriorPointSolver>();
    case SolverType::Mprgp:
      return std::make_unique<MprgpSolver>();
    case SolverType::ShockPropagation:
      return std::make_unique<ShockPropagationSolver>();
    case SolverType::Staggering:
      return std::make_unique<StaggeringSolver>();
  }
  return nullptr;
}

std::vector<SolverInfo> BuildSolvers()
{
  std::vector<SolverInfo> solvers;

  solvers.push_back(
      {SolverType::Dantzig,
       "Dantzig",
       "Pivoting",
       true,
       true,
       true,
       "",
       {"Exact pivoting, handles boxed and findex", "Solid for small systems"},
       {"High cost; scales poorly"}});
  solvers.push_back(
      {SolverType::Lemke,
       "Lemke",
       "Pivoting",
       true,
       false,
       false,
       "Fallback to Dantzig for boxed or findex problems",
       {"Exact for standard LCP", "Useful for verification"},
       {"Standard only; not ideal for large problems"}});
  solvers.push_back(
      {SolverType::Baraff,
       "Baraff",
       "Pivoting",
       true,
       false,
       false,
       "Fallback to Dantzig for boxed or findex problems",
       {"Incremental pivoting for contact", "Keeps complementarity explicit"},
       {"Standard only; expensive for large systems"}});
  solvers.push_back(
      {SolverType::Direct,
       "Direct",
       "Pivoting",
       true,
       false,
       false,
       "Fallback to Dantzig for boxed/findex or size > 3",
       {"Enumerates active sets for tiny problems"},
       {"Exponential cost; limited to n <= 3"}});

  solvers.push_back(
      {SolverType::Pgs,
       "PGS",
       "Projection",
       true,
       true,
       true,
       "",
       {"Fast per-iteration; common in real time", "Handles boxed and findex"},
       {"Slow convergence; sensitive to conditioning"}});
  solvers.push_back(
      {SolverType::SymmetricPsor,
       "Symmetric PSOR",
       "Projection",
       true,
       true,
       true,
       "",
       {"Forward and backward sweep can reduce error"},
       {"Iterative; needs tuning"}});
  solvers.push_back(
      {SolverType::Jacobi,
       "Jacobi",
       "Projection",
       true,
       true,
       true,
       "",
       {"Parallel-friendly iterations"},
       {"Slow convergence; many iterations"}});
  solvers.push_back(
      {SolverType::RedBlack,
       "Red-Black GS",
       "Projection",
       true,
       true,
       true,
       "",
       {"Parallel-friendly ordering"},
       {"Ordering sensitive; convergence varies"}});
  solvers.push_back(
      {SolverType::BlockedJacobi,
       "Blocked Jacobi",
       "Projection",
       true,
       true,
       true,
       "",
       {"Per-block solves can improve convergence"},
       {"Block partition affects quality"}});
  solvers.push_back(
      {SolverType::Bgs,
       "BGS",
       "Projection",
       true,
       true,
       true,
       "",
       {"Contact block structure improves stability"},
       {"Block choice affects speed and accuracy"}});
  solvers.push_back(
      {SolverType::Nncg,
       "NNCG",
       "Projection",
       true,
       true,
       true,
       "",
       {"Improves convergence over plain PGS on tough contacts"},
       {"Parameter sensitive; higher per-iteration cost"}});
  solvers.push_back(
      {SolverType::SubspaceMinimization,
       "Subspace Minimization",
       "Projection",
       true,
       true,
       true,
       "",
       {"Active-set prediction with local direct solves"},
       {"Extra tuning; still iterative"}});

  solvers.push_back(
      {SolverType::MinimumMapNewton,
       "Minimum Map Newton",
       "Newton",
       true,
       false,
       false,
       "Fallback to Dantzig for boxed or findex problems",
       {"Fast local convergence; accurate"},
       {"Standard only; needs good initial guess"}});
  solvers.push_back(
      {SolverType::FischerBurmeisterNewton,
       "Fischer-Burmeister Newton",
       "Newton",
       true,
       false,
       false,
       "Fallback to Dantzig for boxed or findex problems",
       {"NCP reformulation enables Newton steps"},
       {"Standard only; line search overhead"}});
  solvers.push_back(
      {SolverType::PenalizedFischerBurmeisterNewton,
       "Penalized FB Newton",
       "Newton",
       true,
       false,
       false,
       "Fallback to Dantzig for boxed or findex problems",
       {"Smoothing can improve convergence"},
       {"Parameter sensitive; standard only"}});

  solvers.push_back(
      {SolverType::InteriorPoint,
       "Interior Point",
       "Other",
       true,
       false,
       false,
       "Fallback to Dantzig for boxed or findex problems",
       {"Robust for ill-conditioned standard LCPs"},
       {"Heavy linear algebra; standard only"}});
  solvers.push_back(
      {SolverType::Mprgp,
       "MPRGP",
       "Other",
       true,
       false,
       false,
       "Fallback to Dantzig for boxed or findex problems",
       {"Efficient for SPD bound-constrained problems"},
       {"Requires SPD and symmetry; standard only"}});
  solvers.push_back(
      {SolverType::ShockPropagation,
       "Shock Propagation",
       "Other",
       true,
       true,
       true,
       "",
       {"Layered ordering for stacks and contacts"},
       {"Order dependent; block setup matters"}});
  solvers.push_back(
      {SolverType::Staggering,
       "Staggering",
       "Other",
       true,
       true,
       true,
       "",
       {"Splits normal and friction updates"},
       {"More iterations; sensitive to coupling"}});

  return solvers;
}

std::vector<ExampleCase> BuildExamples()
{
  const double kInf = std::numeric_limits<double>::infinity();
  std::vector<ExampleCase> examples;

  {
    const unsigned seed = 1001;
    Eigen::MatrixXd A(2, 2);
    A << 2.0, 0.5, 0.5, 1.5;
    Eigen::VectorXd xStar(2);
    xStar << 0.4, 0.2;
    Eigen::VectorXd b = A * xStar;
    Eigen::VectorXd lo = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd hi = Eigen::VectorXd::Constant(2, kInf);
    Eigen::VectorXi findex = Eigen::VectorXi::Constant(2, -1);
    examples.emplace_back(
        "2x2 Standard LCP",
        "Toy / Didactic",
        "Minimal standard LCP for pivoting basics.",
        std::vector<std::string>{"Lemke1965", "CottleDantzig1968"},
        seed,
        false,
        false,
        dart::math::LcpProblem(A, b, lo, hi, findex));
  }

  {
    const unsigned seed = 1002;
    Eigen::MatrixXd A = MakeSpdMatrix(3, seed, 1.0);
    Eigen::VectorXd xStar(3);
    xStar << -1.0, 0.1, 0.8;
    Eigen::VectorXd b = A * xStar;
    Eigen::VectorXd lo(3);
    Eigen::VectorXd hi(3);
    lo << -1.0, -0.5, 0.0;
    hi << 0.5, 0.5, 0.8;
    Eigen::VectorXi findex = Eigen::VectorXi::Constant(3, -1);
    examples.emplace_back(
        "3x3 Boxed LCP (active bounds)",
        "Toy / Didactic",
        "Boxed LCP with mixed active bounds.",
        std::vector<std::string>{"Murty1988", "CottlePangStone1992"},
        seed,
        true,
        false,
        dart::math::LcpProblem(A, b, lo, hi, findex));
  }

  {
    const unsigned seed = 1003;
    Eigen::Matrix3d A;
    A << 4.0, 0.5, 0.0, 0.5, 3.0, 0.25, 0.0, 0.25, 2.5;
    Eigen::Vector3d xStar;
    xStar << 1.0, 0.2, -0.1;
    Eigen::Vector3d b = A * xStar;
    const double mu = 0.5;
    Eigen::Vector3d lo;
    Eigen::Vector3d hi;
    lo << 0.0, -mu, -mu;
    hi << kInf, mu, mu;
    Eigen::Vector3i findex;
    findex << -1, 0, 0;
    examples.emplace_back(
        "Single-contact friction pyramid",
        "Toy / Didactic",
        "One normal + two tangents using findex coupling.",
        std::vector<std::string>{
            "Smith2000", "Smith2006", "StewartTrinkle1996"},
        seed,
        true,
        true,
        dart::math::LcpProblem(A, b, lo, hi, findex));
  }

  {
    const unsigned seed = 2001;
    const int n = 8;
    Eigen::MatrixXd A = MakeIllConditionedSpd(n, seed);
    Eigen::VectorXd xStar = MakePositiveVector(n, seed + 1, 0.1, 1.0);
    Eigen::VectorXd b = A * xStar;
    Eigen::VectorXd lo = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd hi = Eigen::VectorXd::Constant(n, kInf);
    Eigen::VectorXi findex = Eigen::VectorXi::Constant(n, -1);
    examples.emplace_back(
        "Near-singular standard LCP",
        "Intermediate / Stress",
        "Ill-conditioned SPD matrix to stress robustness.",
        std::vector<std::string>{"Kojima1991", "Wright1997"},
        seed,
        false,
        false,
        dart::math::LcpProblem(A, b, lo, hi, findex));
  }

  {
    const unsigned seed = 2002;
    const int n = 12;
    Eigen::MatrixXd A = MakeMassRatioSpd(n, seed);
    Eigen::VectorXd xStar = MakePositiveVector(n, seed + 1, 0.05, 0.6);
    Eigen::VectorXd b = A * xStar;
    Eigen::VectorXd lo = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd hi = Eigen::VectorXd::Constant(n, kInf);
    Eigen::VectorXi findex = Eigen::VectorXi::Constant(n, -1);
    examples.emplace_back(
        "Large mass-ratio block",
        "Intermediate / Stress",
        "Diagonal dominance with large scale variation.",
        std::vector<std::string>{"Silcowitz2009", "Silcowitz2010"},
        seed,
        false,
        false,
        dart::math::LcpProblem(A, b, lo, hi, findex));
  }

  auto addContactExample = [&](const std::string& label,
                               const std::string& category,
                               const std::string& description,
                               const std::vector<std::string>& refs,
                               unsigned seed,
                               int contacts,
                               double mu) {
    const int n = contacts * 3;
    Eigen::MatrixXd A = MakeContactSpdMatrix(contacts, seed);
    Eigen::VectorXd xStar = Eigen::VectorXd::Zero(n);
    for (int i = 0; i < contacts; ++i) {
      const double normal = 0.6 + 0.05 * i;
      const double tangentScale = 0.3 * mu * normal;
      xStar[3 * i] = normal;
      xStar[3 * i + 1] = tangentScale * ((i % 2 == 0) ? 1.0 : -1.0);
      xStar[3 * i + 2] = -tangentScale * 0.5;
    }
    Eigen::VectorXd b = A * xStar;

    Eigen::VectorXd lo = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd hi = Eigen::VectorXd::Zero(n);
    Eigen::VectorXi findex = Eigen::VectorXi::Constant(n, -1);
    for (int i = 0; i < contacts; ++i) {
      lo[3 * i] = 0.0;
      hi[3 * i] = kInf;
      lo[3 * i + 1] = -mu;
      hi[3 * i + 1] = mu;
      findex[3 * i + 1] = 3 * i;
      lo[3 * i + 2] = -mu;
      hi[3 * i + 2] = mu;
      findex[3 * i + 2] = 3 * i;
    }

    examples.emplace_back(
        label,
        category,
        description,
        refs,
        seed,
        true,
        true,
        dart::math::LcpProblem(A, b, lo, hi, findex));
  };

  addContactExample(
      "Rigid-body time-stepping contact",
      "Real-world inspired",
      "Multi-contact friction using a time-stepping style LCP.",
      {"StewartTrinkle1996", "AnitescuPotra1997"},
      3001,
      6,
      0.6);

  addContactExample(
      "Granular pile / multi-contact blocks",
      "Real-world inspired",
      "Many contact blocks motivated by NSCD-style solvers.",
      {"Moreau1988", "Jean1999", "AcaryBrogliato2008"},
      3002,
      10,
      0.5);

  addContactExample(
      "Stacking / shock propagation layers",
      "Real-world inspired",
      "Layered contact ordering for stacking scenarios.",
      {"Guendelman2003"},
      3003,
      4,
      0.4);

  addContactExample(
      "Staggered normal/tangent solve",
      "Real-world inspired",
      "Normal vs tangent split for staggered updates.",
      {"Kaufman2008", "Tournier2015"},
      3004,
      5,
      0.7);

  {
    const unsigned seed = 3005;
    const int n = 10;
    Eigen::MatrixXd A = MakeSpdMatrix(n, seed, 2.5);
    Eigen::VectorXd xStar = MakePositiveVector(n, seed + 1, 0.1, 0.8);
    Eigen::VectorXd b = A * xStar;
    Eigen::VectorXd lo = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd hi = Eigen::VectorXd::Constant(n, kInf);
    Eigen::VectorXi findex = Eigen::VectorXi::Constant(n, -1);
    examples.emplace_back(
        "Newton accuracy-focused case",
        "Real-world inspired",
        "Moderate SPD system for Newton-style accuracy.",
        std::vector<std::string>{"Fischer1992", "QiSun1993"},
        seed,
        false,
        false,
        dart::math::LcpProblem(A, b, lo, hi, findex));
  }

  return examples;
}

const std::unordered_map<std::string, std::string>& ReferenceMap()
{
  static const std::unordered_map<std::string, std::string> refs{
      {"Lemke1965",
       "Lemke (1965) Bimatrix equilibrium points and mathematical programming"},
      {"CottleDantzig1968",
       "Cottle and Dantzig (1968) Complementary pivot theory"},
      {"Murty1988",
       "Murty (1988) Linear complementarity, linear and nonlinear programming"},
      {"CottlePangStone1992",
       "Cottle, Pang, Stone (1992) The linear complementarity problem"},
      {"Smith2000", "Smith (2000) Open Dynamics Engine user guide"},
      {"Smith2006", "Smith (2006) Open Dynamics Engine"},
      {"StewartTrinkle1996",
       "Stewart and Trinkle (1996) Implicit time-stepping with friction"},
      {"Kojima1991", "Kojima et al. (1991) Interior point algorithms for LCP"},
      {"Wright1997", "Wright (1997) Primal-dual interior-point methods"},
      {"Silcowitz2009", "Silcowitz et al. (2009) NNCG for interactive contact"},
      {"Silcowitz2010",
       "Silcowitz et al. (2010) NNCG for interactive contact (extended)"},
      {"AnitescuPotra1997", "Anitescu and Potra (1997) Multi-body contact LCP"},
      {"Moreau1988", "Moreau (1988) Unilateral contact and dry friction"},
      {"Jean1999", "Jean (1999) Non-smooth contact dynamics"},
      {"AcaryBrogliato2008",
       "Acary and Brogliato (2008) Numerical methods for nonsmooth dynamics"},
      {"Guendelman2003",
       "Guendelman et al. (2003) Nonconvex rigid bodies with stacking"},
      {"Kaufman2008",
       "Kaufman et al. (2008) Staggered projections for frictional contact"},
      {"Tournier2015", "Tournier et al. (2015) Stable constrained dynamics"},
      {"Fischer1992", "Fischer (1992) Special Newton-type optimization method"},
      {"QiSun1993", "Qi and Sun (1993) Nonsmooth Newton method"}};
  return refs;
}

void AddReferenceScene(const dart::simulation::WorldPtr& world)
{
  const double groundSize = 4.0;
  const double groundThickness = 0.1;
  const double axisLength = 1.5;
  const double axisThickness = 0.05;

  auto ground = std::make_shared<dart::dynamics::SimpleFrame>(
      dart::dynamics::Frame::World(), "ground");
  ground->setShape(
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d(groundSize, groundSize, groundThickness)));
  Eigen::Isometry3d groundTf = Eigen::Isometry3d::Identity();
  groundTf.translation() = Eigen::Vector3d(0.0, 0.0, -0.5 * groundThickness);
  ground->setRelativeTransform(groundTf);
  ground->getVisualAspect(true)->setColor(Eigen::Vector3d(0.85, 0.85, 0.85));
  world->addSimpleFrame(ground);

  auto addAxis = [&world](
                     const std::string& name,
                     const Eigen::Vector3d& size,
                     const Eigen::Vector3d& center,
                     const Eigen::Vector3d& color) {
    auto axis = std::make_shared<dart::dynamics::SimpleFrame>(
        dart::dynamics::Frame::World(), name);
    axis->setShape(std::make_shared<dart::dynamics::BoxShape>(size));
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    tf.translation() = center;
    axis->setRelativeTransform(tf);
    axis->getVisualAspect(true)->setColor(color);
    world->addSimpleFrame(axis);
  };

  addAxis(
      "axis_x",
      Eigen::Vector3d(axisLength, axisThickness, axisThickness),
      Eigen::Vector3d(0.5 * axisLength, 0.0, 0.5 * axisThickness),
      Eigen::Vector3d(0.9, 0.2, 0.2));
  addAxis(
      "axis_y",
      Eigen::Vector3d(axisThickness, axisLength, axisThickness),
      Eigen::Vector3d(0.0, 0.5 * axisLength, 0.5 * axisThickness),
      Eigen::Vector3d(0.2, 0.8, 0.2));
  addAxis(
      "axis_z",
      Eigen::Vector3d(axisThickness, axisThickness, axisLength),
      Eigen::Vector3d(0.0, 0.0, 0.5 * axisLength),
      Eigen::Vector3d(0.2, 0.4, 0.9));
}

struct SolverParameters
{
  dart::math::PgsSolver::Parameters pgs;
  dart::math::SymmetricPsorSolver::Parameters symPsor;
  dart::math::JacobiSolver::Parameters jacobi;
  dart::math::RedBlackGaussSeidelSolver::Parameters redBlack;
  dart::math::BlockedJacobiSolver::Parameters blockedJacobi;
  dart::math::BgsSolver::Parameters bgs;
  dart::math::NncgSolver::Parameters nncg;
  dart::math::SubspaceMinimizationSolver::Parameters subspace;
  dart::math::MinimumMapNewtonSolver::Parameters minMap;
  dart::math::FischerBurmeisterNewtonSolver::Parameters fb;
  dart::math::PenalizedFischerBurmeisterNewtonSolver::Parameters pfb;
  dart::math::InteriorPointSolver::Parameters interior;
  dart::math::MprgpSolver::Parameters mprgp;
  dart::math::ShockPropagationSolver::Parameters shock;
};

class LcpDashboardWidget : public dart::gui::ImGuiWidget
{
public:
  LcpDashboardWidget(dart::gui::ImGuiViewer* viewer, double guiScale)
    : mViewer(viewer),
      mGuiScale(guiScale),
      mExamples(BuildExamples()),
      mSolvers(BuildSolvers()),
      mSolverStats(mSolvers.size())
  {
    mOptions.maxIterations = 100;
    mOptions.absoluteTolerance = 1e-6;
    mOptions.relativeTolerance = 1e-4;
    mOptions.complementarityTolerance = 1e-6;
    mOptions.relaxation = 1.0;
    mOptions.warmStart = false;
    mOptions.validateSolution = false;
    mOptions.earlyTermination = false;
  }

  void render() override
  {
    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    const float scale = static_cast<float>(mGuiScale);
    const float padding = 10.0f * scale;
    const ImVec2 baseSize(660.0f * scale, 760.0f * scale);
    const ImVec2 maxSize(
        std::max(240.0f * scale, viewport->WorkSize.x - 2.0f * padding),
        std::max(240.0f * scale, viewport->WorkSize.y - 2.0f * padding));
    const ImVec2 windowSize(
        std::min(baseSize.x, maxSize.x), std::min(baseSize.y, maxSize.y));
    const ImVec2 windowPos(
        viewport->WorkPos.x + padding, viewport->WorkPos.y + padding);

    if (mApplyInitialLayout) {
      ImGui::SetNextWindowPos(windowPos, ImGuiCond_Always);
      ImGui::SetNextWindowSize(windowSize, ImGuiCond_Always);
      mApplyInitialLayout = false;
    } else {
      ImGui::SetNextWindowPos(windowPos, ImGuiCond_FirstUseEver);
      ImGui::SetNextWindowSize(windowSize, ImGuiCond_FirstUseEver);
    }
    ImGui::SetNextWindowBgAlpha(0.6f);

    if (!ImGui::Begin(
            "LCP Solvers",
            nullptr,
            ImGuiWindowFlags_MenuBar | ImGuiWindowFlags_NoCollapse)) {
      ImGui::End();
      return;
    }

    if (ImGui::BeginMenuBar()) {
      if (ImGui::BeginMenu("Menu")) {
        if (ImGui::MenuItem("Exit"))
          mViewer->setDone(true);
        ImGui::EndMenu();
      }
      if (ImGui::BeginMenu("Help")) {
        if (ImGui::MenuItem("About DART"))
          mViewer->showAbout();
        ImGui::EndMenu();
      }
      ImGui::EndMenuBar();
    }

    RenderExamplePanel();
    ImGui::Separator();
    RenderSolverPanel();
    ImGui::Separator();
    RenderOptionsPanel();
    ImGui::Separator();
    RenderRunPanel();
    ImGui::Separator();
    RenderResultsPanel();
    ImGui::Separator();
    RenderPerformancePanel();

    ImGui::End();
  }

private:
  void RenderExamplePanel()
  {
    ImGui::Text("Example");
    const ExampleCase& current = mExamples[mExampleIndex];
    if (ImGui::BeginCombo("Scenario", current.label.c_str())) {
      std::string lastCategory;
      for (int i = 0; i < static_cast<int>(mExamples.size()); ++i) {
        const auto& ex = mExamples[i];
        if (ex.category != lastCategory) {
          ImGui::SeparatorText(ex.category.c_str());
          lastCategory = ex.category;
        }
        const bool selected = (mExampleIndex == i);
        if (ImGui::Selectable(ex.label.c_str(), selected)) {
          mExampleIndex = i;
          ClearResults();
        }
        if (selected)
          ImGui::SetItemDefaultFocus();
      }
      ImGui::EndCombo();
    }

    const ExampleCase& selected = mExamples[mExampleIndex];
    ImGui::TextWrapped("%s", selected.description.c_str());
    const char* typeText = selected.hasFindex
                               ? "Boxed + friction index"
                               : (selected.isBoxed ? "Boxed" : "Standard");
    ImGui::Text("Problem type: %s", typeText);
    ImGui::Text("Dimension: %ld", static_cast<long>(selected.problem.b.size()));
    ImGui::Text("Seed: %u", selected.seed);

    RenderReferenceTags(selected.references);
  }

  void RenderSolverPanel()
  {
    ImGui::Text("Solver");
    const SolverInfo& current = mSolvers[mSolverIndex];

    if (ImGui::BeginCombo("Solver", current.name.c_str())) {
      std::string lastCategory;
      for (int i = 0; i < static_cast<int>(mSolvers.size()); ++i) {
        const auto& solver = mSolvers[i];
        if (solver.category != lastCategory) {
          ImGui::SeparatorText(solver.category.c_str());
          lastCategory = solver.category;
        }
        const bool selected = (mSolverIndex == i);
        if (ImGui::Selectable(solver.name.c_str(), selected))
          mSolverIndex = i;
        if (selected)
          ImGui::SetItemDefaultFocus();
      }
      ImGui::EndCombo();
    }

    if (ImGui::CollapsingHeader(
            "Capabilities", ImGuiTreeNodeFlags_DefaultOpen)) {
      const SolverInfo& solver = mSolvers[mSolverIndex];
      ImGui::Text("Standard: %s", solver.supportsStandard ? "yes" : "no");
      ImGui::Text("Boxed: %s", solver.supportsBoxed ? "yes" : "no");
      ImGui::Text("Findex: %s", solver.supportsFindex ? "yes" : "no");
      if (!solver.fallbackNote.empty())
        ImGui::TextWrapped("%s", solver.fallbackNote.c_str());
    }

    if (ImGui::CollapsingHeader(
            "Pros and Cons", ImGuiTreeNodeFlags_DefaultOpen)) {
      const SolverInfo& solver = mSolvers[mSolverIndex];
      ImGui::Text("Pros");
      if (solver.pros.empty()) {
        ImGui::TextDisabled("No notes");
      } else {
        for (const auto& item : solver.pros)
          ImGui::BulletText("%s", item.c_str());
      }
      ImGui::Text("Cons");
      if (solver.cons.empty()) {
        ImGui::TextDisabled("No notes");
      } else {
        for (const auto& item : solver.cons)
          ImGui::BulletText("%s", item.c_str());
      }
    }
  }

  void RenderOptionsPanel()
  {
    if (!ImGui::CollapsingHeader(
            "Solver Options", ImGuiTreeNodeFlags_DefaultOpen))
      return;

    ImGui::TextWrapped(
        "Shared options apply to all solvers for apples-to-apples comparison.");

    ImGui::InputInt("Max iterations", &mOptions.maxIterations);
    ImGui::InputDouble(
        "Abs tolerance", &mOptions.absoluteTolerance, 0.0, 0.0, "%.2e");
    ImGui::InputDouble(
        "Rel tolerance", &mOptions.relativeTolerance, 0.0, 0.0, "%.2e");
    ImGui::InputDouble(
        "Complementarity tol",
        &mOptions.complementarityTolerance,
        0.0,
        0.0,
        "%.2e");
    ImGui::InputDouble("Relaxation", &mOptions.relaxation, 0.0, 0.0, "%.2f");
    ImGui::Checkbox("Warm start", &mOptions.warmStart);
    ImGui::Checkbox("Validate solution", &mOptions.validateSolution);
    ImGui::Checkbox("Early termination", &mOptions.earlyTermination);

    RenderSolverSpecificOptions();
  }

  void RenderSolverSpecificOptions()
  {
    if (!ImGui::CollapsingHeader(
            "Solver-specific options", ImGuiTreeNodeFlags_DefaultOpen)) {
      return;
    }

    const SolverInfo& solver = mSolvers[mSolverIndex];
    const ExampleCase& example = mExamples[mExampleIndex];

    switch (solver.type) {
      case SolverType::Pgs:
        ImGui::InputDouble(
            "Division epsilon",
            &mSolverParams.pgs.epsilonForDivision,
            0.0,
            0.0,
            "%.1e");
        ImGui::Checkbox(
            "Randomize order", &mSolverParams.pgs.randomizeConstraintOrder);
        break;
      case SolverType::SymmetricPsor:
        ImGui::InputDouble(
            "Division epsilon",
            &mSolverParams.symPsor.epsilonForDivision,
            0.0,
            0.0,
            "%.1e");
        break;
      case SolverType::Jacobi:
        ImGui::InputDouble(
            "Division epsilon",
            &mSolverParams.jacobi.epsilonForDivision,
            0.0,
            0.0,
            "%.1e");
        break;
      case SolverType::RedBlack:
        ImGui::InputDouble(
            "Division epsilon",
            &mSolverParams.redBlack.epsilonForDivision,
            0.0,
            0.0,
            "%.1e");
        break;
      case SolverType::BlockedJacobi:
        RenderBlockOptions(
            "Blocked Jacobi",
            mBlockedJacobiManualBlocks,
            mBlockedJacobiBlockSize,
            example);
        break;
      case SolverType::Bgs:
        RenderBlockOptions("BGS", mBgsManualBlocks, mBgsBlockSize, example);
        break;
      case SolverType::Nncg:
        ImGui::InputInt("PGS iterations", &mSolverParams.nncg.pgsIterations);
        ImGui::InputInt(
            "Restart interval", &mSolverParams.nncg.restartInterval);
        ImGui::InputDouble(
            "Restart threshold",
            &mSolverParams.nncg.restartThreshold,
            0.0,
            0.0,
            "%.2f");
        break;
      case SolverType::SubspaceMinimization:
        ImGui::InputInt(
            "PGS iterations", &mSolverParams.subspace.pgsIterations);
        ImGui::InputDouble(
            "Active-set tolerance",
            &mSolverParams.subspace.activeSetTolerance,
            0.0,
            0.0,
            "%.2e");
        break;
      case SolverType::MinimumMapNewton:
        ImGui::InputInt(
            "Max line search", &mSolverParams.minMap.maxLineSearchSteps);
        ImGui::InputDouble(
            "Step reduction",
            &mSolverParams.minMap.stepReduction,
            0.0,
            0.0,
            "%.2f");
        ImGui::InputDouble(
            "Sufficient decrease",
            &mSolverParams.minMap.sufficientDecrease,
            0.0,
            0.0,
            "%.2e");
        ImGui::InputDouble(
            "Min step", &mSolverParams.minMap.minStep, 0.0, 0.0, "%.2e");
        break;
      case SolverType::FischerBurmeisterNewton:
        ImGui::InputDouble(
            "Smoothing epsilon",
            &mSolverParams.fb.smoothingEpsilon,
            0.0,
            0.0,
            "%.2e");
        ImGui::InputInt(
            "Max line search", &mSolverParams.fb.maxLineSearchSteps);
        ImGui::InputDouble(
            "Step reduction",
            &mSolverParams.fb.stepReduction,
            0.0,
            0.0,
            "%.2f");
        ImGui::InputDouble(
            "Sufficient decrease",
            &mSolverParams.fb.sufficientDecrease,
            0.0,
            0.0,
            "%.2e");
        ImGui::InputDouble(
            "Min step", &mSolverParams.fb.minStep, 0.0, 0.0, "%.2e");
        break;
      case SolverType::PenalizedFischerBurmeisterNewton:
        ImGui::InputDouble(
            "Smoothing epsilon",
            &mSolverParams.pfb.smoothingEpsilon,
            0.0,
            0.0,
            "%.2e");
        ImGui::InputDouble(
            "Penalty lambda", &mSolverParams.pfb.lambda, 0.0, 0.0, "%.2f");
        ImGui::InputInt(
            "Max line search", &mSolverParams.pfb.maxLineSearchSteps);
        ImGui::InputDouble(
            "Step reduction",
            &mSolverParams.pfb.stepReduction,
            0.0,
            0.0,
            "%.2f");
        ImGui::InputDouble(
            "Sufficient decrease",
            &mSolverParams.pfb.sufficientDecrease,
            0.0,
            0.0,
            "%.2e");
        ImGui::InputDouble(
            "Min step", &mSolverParams.pfb.minStep, 0.0, 0.0, "%.2e");
        break;
      case SolverType::InteriorPoint:
        ImGui::InputDouble(
            "Sigma", &mSolverParams.interior.sigma, 0.0, 0.0, "%.2f");
        ImGui::InputDouble(
            "Step scale", &mSolverParams.interior.stepScale, 0.0, 0.0, "%.2f");
        break;
      case SolverType::Mprgp:
        ImGui::InputDouble(
            "Symmetry tolerance",
            &mSolverParams.mprgp.symmetryTolerance,
            0.0,
            0.0,
            "%.2e");
        ImGui::InputDouble(
            "Division epsilon",
            &mSolverParams.mprgp.epsilonForDivision,
            0.0,
            0.0,
            "%.2e");
        ImGui::Checkbox(
            "Check positive definite",
            &mSolverParams.mprgp.checkPositiveDefinite);
        break;
      case SolverType::ShockPropagation:
        RenderShockOptions(example);
        break;
      case SolverType::Staggering:
      case SolverType::Dantzig:
      case SolverType::Lemke:
      case SolverType::Baraff:
      case SolverType::Direct:
        ImGui::TextDisabled("No solver-specific parameters.");
        break;
    }
  }

  void RenderBlockOptions(
      const char* label,
      bool& manualBlocks,
      int& blockSize,
      const ExampleCase& example)
  {
    const bool allowManual = !example.hasFindex;
    if (!allowManual) {
      ImGui::TextDisabled("Manual block sizes disabled for findex problems.");
    }

    if (!allowManual)
      ImGui::BeginDisabled();
    ImGui::Checkbox(
        (std::string("Manual blocks (") + label + ")").c_str(), &manualBlocks);
    if (manualBlocks) {
      ImGui::InputInt(
          (std::string("Block size (") + label + ")").c_str(), &blockSize);
      if (blockSize < 1)
        blockSize = 1;
    }
    if (!allowManual)
      ImGui::EndDisabled();

    const int n = static_cast<int>(example.problem.b.size());
    if (manualBlocks && allowManual) {
      const int count
          = static_cast<int>(BuildUniformBlockSizes(n, blockSize).size());
      ImGui::Text("Manual blocks: %d", count);
    } else if (example.hasFindex) {
      const int count = static_cast<int>(
          ComputeFindexBlockSizes(example.problem.findex).size());
      ImGui::Text("Derived blocks: %d", count);
    } else {
      ImGui::Text("Derived blocks: 1");
    }
  }

  void RenderShockOptions(const ExampleCase& example)
  {
    ImGui::Checkbox("Manual block size", &mShockManualBlocks);
    if (mShockManualBlocks) {
      ImGui::InputInt("Shock block size", &mShockBlockSize);
      if (mShockBlockSize < 1)
        mShockBlockSize = 1;
      if (example.hasFindex) {
        ImGui::TextDisabled("Manual blocks ignored for findex problems.");
      }
    }

    ImGui::Checkbox("Use layer ordering", &mShockUseLayers);
    if (mShockUseLayers) {
      ImGui::InputInt("Blocks per layer", &mShockBlocksPerLayer);
      if (mShockBlocksPerLayer < 1)
        mShockBlocksPerLayer = 1;
    }

    const int blockCount = BlockCountForExample(example, mShockManualBlocks);
    ImGui::Text("Blocks: %d", blockCount);
  }

  void RenderRunPanel()
  {
    if (!ImGui::CollapsingHeader("Run", ImGuiTreeNodeFlags_DefaultOpen))
      return;

    ImGui::InputInt("Runs per solver", &mRunCount);
    if (mRunCount < 1)
      mRunCount = 1;

    if (ImGui::Button("Run selected"))
      RunSolverIndex(mSolverIndex);
    ImGui::SameLine();
    if (ImGui::Button("Run all"))
      RunAllSolvers();
    ImGui::SameLine();
    if (ImGui::Button("Clear results"))
      ClearResults();
  }

  void RenderResultsPanel()
  {
    if (!ImGui::CollapsingHeader("Results", ImGuiTreeNodeFlags_DefaultOpen))
      return;

    ImGui::Checkbox("Show all solvers", &mShowAllResults);

    ImGuiTableFlags flags = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg
                            | ImGuiTableFlags_Resizable
                            | ImGuiTableFlags_SizingStretchProp;
    if (!ImGui::BeginTable("lcp_results", 9, flags))
      return;

    ImGui::TableSetupColumn("Solver");
    ImGui::TableSetupColumn("Status");
    ImGui::TableSetupColumn("Iters");
    ImGui::TableSetupColumn("Residual");
    ImGui::TableSetupColumn("Complementarity");
    ImGui::TableSetupColumn("Bound violation");
    ImGui::TableSetupColumn("Contract ok");
    ImGui::TableSetupColumn("Avg ms");
    ImGui::TableSetupColumn("Min/Max ms");
    ImGui::TableHeadersRow();

    auto renderRow = [&](int index) {
      const auto& solver = mSolvers[index];
      const auto& stats = mSolverStats[index];
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      ImGui::TextUnformatted(solver.name.c_str());

      ImGui::TableSetColumnIndex(1);
      if (!stats.hasResult) {
        ImGui::TextUnformatted("Not run");
        return;
      }
      ImGui::TextUnformatted(dart::math::toString(stats.result.status).c_str());

      ImGui::TableSetColumnIndex(2);
      ImGui::Text("%d", stats.result.iterations);

      ImGui::TableSetColumnIndex(3);
      ImGui::Text("%.3e", stats.check.residual);

      ImGui::TableSetColumnIndex(4);
      ImGui::Text("%.3e", stats.check.complementarity);

      ImGui::TableSetColumnIndex(5);
      ImGui::Text("%.3e", stats.check.boundViolation);

      ImGui::TableSetColumnIndex(6);
      if (stats.check.ok) {
        ImGui::TextColored(ImVec4(0.2f, 0.8f, 0.3f, 1.0f), "yes");
      } else {
        ImGui::TextColored(ImVec4(0.9f, 0.3f, 0.3f, 1.0f), "no");
        if (ImGui::IsItemHovered() && !stats.check.message.empty()) {
          ImGui::SetTooltip("%s", stats.check.message.c_str());
        }
      }

      ImGui::TableSetColumnIndex(7);
      ImGui::Text("%.3f", stats.avgMs);

      ImGui::TableSetColumnIndex(8);
      ImGui::Text("%.3f / %.3f", stats.minMs, stats.maxMs);
    };

    if (mShowAllResults) {
      for (int i = 0; i < static_cast<int>(mSolvers.size()); ++i)
        renderRow(i);
    } else {
      renderRow(mSolverIndex);
    }

    ImGui::EndTable();
  }

  void RenderPerformancePanel()
  {
    if (!ImGui::CollapsingHeader(
            "Performance history", ImGuiTreeNodeFlags_DefaultOpen))
      return;

    const auto& stats = mSolverStats[mSolverIndex];
    if (stats.historyMs.empty()) {
      ImGui::TextDisabled("No samples yet.");
      return;
    }

    const auto [minIt, maxIt]
        = std::minmax_element(stats.historyMs.begin(), stats.historyMs.end());
    float minVal = *minIt;
    float maxVal = *maxIt;
    if (minVal == maxVal) {
      minVal = std::max(0.0f, minVal - 0.1f);
      maxVal = maxVal + 0.1f;
    }

    ImGui::PlotLines(
        "Solve time (ms)",
        stats.historyMs.data(),
        static_cast<int>(stats.historyMs.size()),
        0,
        nullptr,
        minVal,
        maxVal,
        ImVec2(0, 80));
    ImGui::PlotHistogram(
        "Distribution (ms)",
        stats.historyMs.data(),
        static_cast<int>(stats.historyMs.size()),
        0,
        nullptr,
        minVal,
        maxVal,
        ImVec2(0, 80));
  }

  void RenderReferenceTags(const std::vector<std::string>& refs)
  {
    if (refs.empty())
      return;

    ImGui::Text("References:");
    const auto& refMap = ReferenceMap();
    for (const auto& tag : refs) {
      ImGui::SameLine();
      ImGui::SmallButton(tag.c_str());
      if (ImGui::IsItemHovered()) {
        auto it = refMap.find(tag);
        if (it != refMap.end())
          ImGui::SetTooltip("%s", it->second.c_str());
      }
    }
  }

  void ClearResults()
  {
    for (auto& stats : mSolverStats) {
      stats = RunStats{};
    }
  }

  void RunAllSolvers()
  {
    for (int i = 0; i < static_cast<int>(mSolvers.size()); ++i)
      RunSolverIndex(i);
  }

  void RunSolverIndex(int index)
  {
    const auto& solverInfo = mSolvers[index];
    auto solver = CreateSolver(solverInfo.type);
    if (!solver)
      return;

    const auto& example = mExamples[mExampleIndex];
    const auto& problem = example.problem;
    auto& stats = mSolverStats[index];

    dart::math::LcpOptions options = mOptions;
    options.customOptions = GetCustomOptionsPointer(solverInfo, example);

    const int n = static_cast<int>(problem.b.size());
    Eigen::VectorXd x = Eigen::VectorXd::Zero(n);

    double minMs = std::numeric_limits<double>::infinity();
    double maxMs = 0.0;
    double totalMs = 0.0;

    for (int i = 0; i < mRunCount; ++i) {
      if (!mOptions.warmStart || i == 0)
        x = Eigen::VectorXd::Zero(n);

      const auto start = std::chrono::steady_clock::now();
      stats.result = solver->solve(problem, x, options);
      const auto end = std::chrono::steady_clock::now();
      const double ms
          = std::chrono::duration<double, std::milli>(end - start).count();

      totalMs += ms;
      minMs = std::min(minMs, ms);
      maxMs = std::max(maxMs, ms);
      AppendHistory(stats.historyMs, static_cast<float>(ms));
    }

    stats.check = CheckSolution(problem, x, mOptions);
    stats.avgMs = totalMs / static_cast<double>(mRunCount);
    stats.minMs = std::isfinite(minMs) ? minMs : 0.0;
    stats.maxMs = maxMs;
    stats.hasResult = true;
  }

  void* GetCustomOptionsPointer(
      const SolverInfo& solver, const ExampleCase& example)
  {
    switch (solver.type) {
      case SolverType::Pgs:
        return &mSolverParams.pgs;
      case SolverType::SymmetricPsor:
        return &mSolverParams.symPsor;
      case SolverType::Jacobi:
        return &mSolverParams.jacobi;
      case SolverType::RedBlack:
        return &mSolverParams.redBlack;
      case SolverType::BlockedJacobi:
        ConfigureBlockSizes(
            mSolverParams.blockedJacobi.blockSizes,
            mBlockedJacobiManualBlocks,
            mBlockedJacobiBlockSize,
            example);
        return &mSolverParams.blockedJacobi;
      case SolverType::Bgs:
        ConfigureBlockSizes(
            mSolverParams.bgs.blockSizes,
            mBgsManualBlocks,
            mBgsBlockSize,
            example);
        return &mSolverParams.bgs;
      case SolverType::Nncg:
        return &mSolverParams.nncg;
      case SolverType::SubspaceMinimization:
        return &mSolverParams.subspace;
      case SolverType::MinimumMapNewton:
        return &mSolverParams.minMap;
      case SolverType::FischerBurmeisterNewton:
        return &mSolverParams.fb;
      case SolverType::PenalizedFischerBurmeisterNewton:
        return &mSolverParams.pfb;
      case SolverType::InteriorPoint:
        return &mSolverParams.interior;
      case SolverType::Mprgp:
        return &mSolverParams.mprgp;
      case SolverType::ShockPropagation:
        ConfigureShockParameters(example);
        return &mSolverParams.shock;
      case SolverType::Dantzig:
      case SolverType::Lemke:
      case SolverType::Baraff:
      case SolverType::Direct:
      case SolverType::Staggering:
        return nullptr;
    }
    return nullptr;
  }

  void ConfigureBlockSizes(
      std::vector<int>& blockSizes,
      bool manualBlocks,
      int blockSize,
      const ExampleCase& example)
  {
    blockSizes.clear();
    if (!manualBlocks)
      return;
    if (example.hasFindex)
      return;

    const int n = static_cast<int>(example.problem.b.size());
    blockSizes = BuildUniformBlockSizes(n, blockSize);
  }

  void ConfigureShockParameters(const ExampleCase& example)
  {
    mSolverParams.shock.blockSizes.clear();
    mSolverParams.shock.layers.clear();

    if (mShockManualBlocks && !example.hasFindex) {
      const int n = static_cast<int>(example.problem.b.size());
      mSolverParams.shock.blockSizes
          = BuildUniformBlockSizes(n, mShockBlockSize);
    }

    if (mShockUseLayers) {
      const int blockCount = BlockCountForExample(example, mShockManualBlocks);
      const int blocksPerLayer = std::max(1, mShockBlocksPerLayer);
      for (int i = 0; i < blockCount; i += blocksPerLayer) {
        std::vector<int> layer;
        for (int j = i; j < std::min(blockCount, i + blocksPerLayer); ++j)
          layer.push_back(j);
        mSolverParams.shock.layers.push_back(std::move(layer));
      }
    }
  }

  int BlockCountForExample(const ExampleCase& example, bool manualBlocks) const
  {
    if (manualBlocks && !example.hasFindex) {
      const int n = static_cast<int>(example.problem.b.size());
      return static_cast<int>(
          BuildUniformBlockSizes(n, mShockBlockSize).size());
    }
    if (example.hasFindex)
      return static_cast<int>(
          ComputeFindexBlockSizes(example.problem.findex).size());
    return 1;
  }

  static void AppendHistory(std::vector<float>& history, float value)
  {
    if (history.size() >= kMaxHistory)
      history.erase(history.begin());
    history.push_back(value);
  }

  dart::gui::ImGuiViewer* mViewer;
  double mGuiScale{1.0};
  bool mApplyInitialLayout{true};
  std::vector<ExampleCase> mExamples;
  std::vector<SolverInfo> mSolvers;
  std::vector<RunStats> mSolverStats;
  SolverParameters mSolverParams;

  int mExampleIndex{0};
  int mSolverIndex{0};
  int mRunCount{10};
  bool mShowAllResults{false};

  bool mBlockedJacobiManualBlocks{false};
  int mBlockedJacobiBlockSize{3};
  bool mBgsManualBlocks{false};
  int mBgsBlockSize{3};

  bool mShockManualBlocks{false};
  int mShockBlockSize{3};
  bool mShockUseLayers{false};
  int mShockBlocksPerLayer{1};

  dart::math::LcpOptions mOptions;
};

} // namespace

int main(int argc, char* argv[])
{
  CLI::App app("LCP solvers");
  double guiScale = 1.0;
  app.add_option("--gui-scale", guiScale, "Scale factor for ImGui widgets")
      ->check(CLI::PositiveNumber);
  CLI11_PARSE(app, argc, argv);

  dart::simulation::WorldPtr world
      = dart::simulation::World::create("lcp_solvers_world");
  AddReferenceScene(world);
  osg::ref_ptr<dart::gui::WorldNode> worldNode
      = new dart::gui::WorldNode(world);

  osg::ref_ptr<dart::gui::ImGuiViewer> viewer = new dart::gui::ImGuiViewer();
  viewer->setImGuiScale(static_cast<float>(guiScale));
  viewer->addWorldNode(worldNode);
  viewer->getImGuiHandler()->addWidget(
      std::make_shared<LcpDashboardWidget>(viewer, guiScale));

  const double baseWidth = 1280.0;
  const double baseHeight = 720.0;
  const int windowWidth
      = static_cast<int>(std::round(std::max(640.0, baseWidth * guiScale)));
  const int windowHeight
      = static_cast<int>(std::round(std::max(480.0, baseHeight * guiScale)));
  viewer->setUpViewInWindow(100, 100, windowWidth, windowHeight);
  viewer->getCameraManipulator()->setHomePosition(
      ::osg::Vec3(3.5f, 3.0f, 2.5f),
      ::osg::Vec3(0.0f, 0.0f, 0.0f),
      ::osg::Vec3(0.0f, 0.0f, 1.0f));
  viewer->setCameraManipulator(viewer->getCameraManipulator());
  viewer->run();

  return 0;
}
