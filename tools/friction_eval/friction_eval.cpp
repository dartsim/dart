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

// friction_eval: runs one friction-evaluation cell, or the L1 bank of frozen
// problems, and prints long-format CSV. See README.md for the options.

#include "friction_scenes.hpp"

#include <dart/constraint/BoxedLcpConstraintSolver.hpp>
#include <dart/constraint/DantzigBoxedLcpSolver.hpp>
#include <dart/constraint/PgsBoxedLcpSolver.hpp>

#include <Eigen/Dense>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace fe = friction_eval;
using dart::constraint::BoxedLcpSolver;
using dart::constraint::DantzigBoxedLcpSolver;
using dart::constraint::PgsBoxedLcpSolver;

namespace {

/// Row stride of the dense LCP matrix (Dantzig's padding()).
constexpr int stride(int n)
{
  return n > 1 ? ((n - 1) | 3) + 1 : n;
}

double secondsSince(std::chrono::steady_clock::time_point start)
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
      .count();
}

bool allFinite(const double* x, int n)
{
  return std::all_of(x, x + n, [](double v) { return std::isfinite(v); });
}

//==============================================================================
// Frozen problems: A x = b + w, lo <= x <= hi; friction rows (findex >= 0)
// bound |x_i| <= hi_i x_findex[i]. A is row-major with the padded stride.
struct Problem
{
  std::string name;
  int n = 0;
  std::vector<double> A, x, b, lo, hi;
  std::vector<int> findex;

  static Problem copy(
      int n,
      const double* A,
      const double* x,
      const double* b,
      const double* lo,
      const double* hi,
      const int* findex)
  {
    Problem p;
    p.n = n;
    p.A.assign(A, A + static_cast<std::size_t>(n) * stride(n));
    p.x.assign(x, x + n);
    p.b.assign(b, b + n);
    p.lo.assign(lo, lo + n);
    p.hi.assign(hi, hi + n);
    p.findex.assign(findex, findex + n);
    return p;
  }

  // Text header line, then A (n x n, unpadded), x, b, lo, hi as doubles and
  // findex as int32, native endianness.
  void save(const std::string& path) const
  {
    std::ofstream out(path, std::ios::binary);
    out << "friction_eval-lcp 1 " << n << "\n";
    for (int i = 0; i < n; ++i) {
      out.write(
          reinterpret_cast<const char*>(
              &A[static_cast<std::size_t>(i) * stride(n)]),
          n * sizeof(double));
    }
    for (const auto* v : {&x, &b, &lo, &hi})
      out.write(reinterpret_cast<const char*>(v->data()), n * sizeof(double));
    std::vector<std::int32_t> f(findex.begin(), findex.end());
    out.write(
        reinterpret_cast<const char*>(f.data()), n * sizeof(std::int32_t));
  }

  static Problem load(const std::string& path)
  {
    std::ifstream in(path, std::ios::binary);
    std::string magic;
    int version = 0;
    Problem p;
    in >> magic >> version >> p.n;
    in.get();
    if (!in || magic != "friction_eval-lcp" || version != 1 || p.n <= 0)
      throw std::runtime_error("not a friction_eval problem: " + path);
    p.name = std::filesystem::path(path).stem().string();
    p.A.assign(static_cast<std::size_t>(p.n) * stride(p.n), 0.0);
    for (int i = 0; i < p.n; ++i) {
      in.read(
          reinterpret_cast<char*>(
              &p.A[static_cast<std::size_t>(i) * stride(p.n)]),
          p.n * sizeof(double));
    }
    for (auto* v : {&p.x, &p.b, &p.lo, &p.hi}) {
      v->resize(p.n);
      in.read(reinterpret_cast<char*>(v->data()), p.n * sizeof(double));
    }
    std::vector<std::int32_t> f(p.n);
    in.read(reinterpret_cast<char*>(f.data()), p.n * sizeof(std::int32_t));
    p.findex.assign(f.begin(), f.end());
    if (!in)
      throw std::runtime_error("truncated problem: " + path);
    return p;
  }
};

/// Box-law residual of x on pristine terms, with the friction bounds taken
/// from the final normal impulses (PGS semantics), w = A x - b.
struct Residual
{
  double natural = 0.0;      // max A_ii |x_i - clamp(x_i - w_i/A_ii)| [m/s]
  double boxViolation = 0.0; // max (|x_t| - mu x_n)+ / (mu x_n + floor)
  double cfmFloor = 0.0;     // max cfm/(1+cfm) A_ii x_n on normal rows [m/s]
  bool finite = true;
};

Residual boxResidual(const Problem& p, const double* x, double cfm)
{
  Residual r;
  r.finite = allFinite(x, p.n);
  if (!r.finite)
    return r;
  const int s = stride(p.n);
  // Relative violations are floored at 1e-3 of the group's largest impulse,
  // so a nearly unloaded contact does not dominate.
  double scale = 0.0;
  std::vector<char> normal(p.n, 0);
  for (int i = 0; i < p.n; ++i) {
    scale = std::max(scale, std::abs(x[i]));
    if (p.findex[i] >= 0)
      normal[p.findex[i]] = 1;
  }
  const double eps = 1e-12 + 1e-3 * scale;
  for (int i = 0; i < p.n; ++i) {
    const double* row = &p.A[static_cast<std::size_t>(i) * s];
    double w = -p.b[i];
    for (int j = 0; j < p.n; ++j)
      w += row[j] * x[j];
    double lo = p.lo[i];
    double hi = p.hi[i];
    if (p.findex[i] >= 0) {
      hi = p.hi[i] * x[p.findex[i]];
      lo = -hi;
      r.boxViolation = std::max(
          r.boxViolation,
          std::max(0.0, std::abs(x[i]) - hi) / (std::abs(hi) + eps));
    }
    const double aii = row[i];
    if (aii > 0.0) {
      const double projected = std::min(std::max(x[i] - w / aii, lo), hi);
      r.natural = std::max(r.natural, aii * std::abs(x[i] - projected));
    }
    if (normal[i])
      r.cfmFloor = std::max(r.cfmFloor, cfm / (1.0 + cfm) * aii * x[i]);
  }
  return r;
}

//==============================================================================
// Per-solve telemetry, box-law audit and problem dumps (D §10).
struct Telemetry
{
  double cfm = 1e-5;
  long solves = 0, failures = 0, fallbacks = 0, fallbackFailures = 0;
  long rows = 0, maxRows = 0, nonFinite = 0, audited = 0;
  double natural = 0.0, naturalSum = 0.0, boxViolation = 0.0, cfmFloor = 0.0;
  double seconds = 0.0;
  std::string dumpDir, dumpTag;
  std::set<int> dumpSteps;
  int step = 0, dumped = 0;
};

/// Wraps the backend under test (D's CountingBoxedLcpSolver). It is a custom
/// type, so the 6.20 line solves islands serially with it: use it only in
/// single-thread audit rows, never in performance rows.
class CountingBoxedLcpSolver final : public BoxedLcpSolver
{
public:
  CountingBoxedLcpSolver(
      std::shared_ptr<BoxedLcpSolver> inner,
      Telemetry& telemetry,
      bool fallback)
    : mInner(std::move(inner)), mTelemetry(telemetry), mFallback(fallback)
  {
  }

  const std::string& getType() const override
  {
    static const std::string type = "CountingBoxedLcpSolver";
    return type;
  }

  bool solve(
      int n,
      double* A,
      double* x,
      double* b,
      int nub,
      double* lo,
      double* hi,
      int* findex,
      bool earlyTermination) override
  {
    auto& t = mTelemetry;
    // Backends mutate their terms (Dantzig: A, b, lo, hi; PGS: A, b).
    const Problem pristine = Problem::copy(n, A, x, b, lo, hi, findex);
    if (!mFallback && !t.dumpDir.empty() && t.dumpSteps.count(t.step)) {
      pristine.save(
          t.dumpDir + "/" + t.dumpTag + "_s" + std::to_string(t.step) + "_g"
          + std::to_string(t.dumped++) + ".lcp");
    }
    const auto start = std::chrono::steady_clock::now();
    const bool ok
        = mInner->solve(n, A, x, b, nub, lo, hi, findex, earlyTermination);
    t.seconds += secondsSince(start);
    if (mFallback) {
      ++t.fallbacks;
      t.fallbackFailures += !ok;
    } else {
      ++t.solves;
      t.failures += !ok;
      t.rows += n;
      t.maxRows = std::max<long>(t.maxRows, n);
    }
    // Audit the impulses that get applied: a finite primary success, or the
    // fallback's result (any finite fallback result is accepted).
    const Residual r = boxResidual(pristine, x, t.cfm);
    t.nonFinite += !r.finite;
    if (r.finite && (ok || mFallback)) {
      ++t.audited;
      t.natural = std::max(t.natural, r.natural);
      t.naturalSum += r.natural;
      t.boxViolation = std::max(t.boxViolation, r.boxViolation);
      t.cfmFloor = std::max(t.cfmFloor, r.cfmFloor);
    }
    return ok;
  }

#if DART_BUILD_MODE_DEBUG
  bool canSolve(int n, const double* A) override
  {
    return mInner->canSolve(n, A);
  }
#endif

private:
  std::shared_ptr<BoxedLcpSolver> mInner;
  Telemetry& mTelemetry;
  bool mFallback;
};

/// PGS-tight (D's C7): the box law solved by PgsBoxedLcpSolver in 10-sweep
/// chunks until its natural-map residual is at most 1e-6 m/s, keeping the best
/// iterate and accepting it at the 1000-sweep cap (counted) instead of falling
/// back. PGS's own relative-change test is not used: it stops early on slow
/// load propagation and never passes on rows that hover near zero.
/// With dantzigSeed it is DZ+R (O4): Dantzig on a copy of the terms, then the
/// same solve from Dantzig's x on the pristine terms, which refreshes the
/// friction bounds Dantzig froze at the frictionless normal impulses (F-b).
/// Dantzig's x is kept when it already satisfies the box law.
class TightBoxSolver final : public BoxedLcpSolver
{
public:
  static constexpr double kTolerance = 1e-6; // [m/s]
  static constexpr int kChunk = 10;
  static constexpr int kMaxSweeps = 1000;

  struct Stats
  {
    long solves = 0, capped = 0, refreshed = 0, sweeps = 0;
    double maxChange = 0.0;
  };

  explicit TightBoxSolver(bool dantzigSeed) : mDantzigSeed(dantzigSeed)
  {
    mPgs.setOption(PgsBoxedLcpSolver::Option(kChunk, 0.0, 0.0));
  }

  const std::string& getType() const override
  {
    static const std::string refresh = "DantzigRefreshSolver";
    static const std::string tight = "PgsTightSolver";
    return mDantzigSeed ? refresh : tight;
  }

  bool solve(
      int n,
      double* A,
      double* x,
      double* b,
      int nub,
      double* lo,
      double* hi,
      int* findex,
      bool earlyTermination) override
  {
    const Problem pristine = Problem::copy(n, A, x, b, lo, hi, findex);
    if (mDantzigSeed) {
      Problem terms = pristine; // Dantzig mutates A, b, lo, hi and findex.
      if (!mDantzig.solve(
              n,
              terms.A.data(),
              x,
              terms.b.data(),
              nub,
              terms.lo.data(),
              terms.hi.data(),
              terms.findex.data(),
              earlyTermination)) {
        return false;
      }
    }
    ++mStats.solves;
    const std::vector<double> seed(x, x + n);
    std::vector<double> best = seed;
    double bestResidual = boxResidual(pristine, x, 0.0).natural;
    if (mDantzigSeed && bestResidual > kTolerance)
      ++mStats.refreshed;
    int sweeps = 0;
    while (bestResidual > kTolerance && sweeps < kMaxSweeps) {
      Problem terms = pristine; // PGS normalizes A and b in place.
      mPgs.solve(
          n,
          terms.A.data(),
          x,
          terms.b.data(),
          nub,
          terms.lo.data(),
          terms.hi.data(),
          terms.findex.data(),
          false);
      sweeps += kChunk;
      const Residual r = boxResidual(pristine, x, 0.0);
      if (!r.finite)
        break;
      if (r.natural < bestResidual) {
        bestResidual = r.natural;
        best.assign(x, x + n);
      }
    }
    mStats.sweeps += sweeps;
    mStats.capped += bestResidual > kTolerance;
    double change = 0.0;
    double scale = 0.0;
    for (int i = 0; i < n; ++i) {
      change = std::max(change, std::abs(best[i] - seed[i]));
      scale = std::max(scale, std::abs(seed[i]));
    }
    mStats.maxChange = std::max(mStats.maxChange, change / (1.0 + scale));
    std::copy(best.begin(), best.end(), x);
    return true;
  }

#if DART_BUILD_MODE_DEBUG
  bool canSolve(int n, const double* A) override
  {
    return mPgs.canSolve(n, A);
  }
#endif

  const Stats& getStats() const
  {
    return mStats;
  }

private:
  bool mDantzigSeed;
  DantzigBoxedLcpSolver mDantzig;
  PgsBoxedLcpSolver mPgs;
  Stats mStats;
};

/// VA (O7): aligns the first friction axis with the step's free relative
/// tangential velocity at isotropic contacts (mu1 = mu2, slip1 = slip2, default
/// fdir, no tangential surface velocity); other contacts keep the parent's
/// parameters. A user handler disables the default handler's fast paths, so VA
/// rows are physics-only.
class VelocityAlignedHandler final
  : public dart::constraint::ContactSurfaceHandler
{
public:
  /// Free tangential speed [m/s] below which the default basis is kept.
  static constexpr double kMinSlip = 1e-5;

  dart::constraint::ContactSurfaceParams createParams(
      const dart::collision::Contact& contact,
      std::size_t numContacts) const override
  {
    auto params = ContactSurfaceHandler::createParams(contact, numContacts);
    ++mCalls;
    const bool isotropic
        = params.mPrimaryFrictionCoeff == params.mSecondaryFrictionCoeff
          && params.mPrimarySlipCompliance == params.mSecondarySlipCompliance
          && (params.mFirstFrictionalDirection
              - dart::constraint::DART_DEFAULT_FRICTION_DIR)
                     .squaredNorm()
                 < 1e-12
          && params.mContactSurfaceMotionVelocity.tail<2>().isZero(0.0);
    if (!isotropic)
      return params;
    const auto velocity = [&](const dart::dynamics::ConstBodyNodePtr& body) {
      return body ? body->getLinearVelocity(
                 body->getWorldTransform().inverse() * contact.point)
                  : Eigen::Vector3d::Zero();
    };
    const Eigen::Vector3d v = velocity(contact.getBodyNodePtr1())
                              - velocity(contact.getBodyNodePtr2());
    const Eigen::Vector3d vt = v - v.dot(contact.normal) * contact.normal;
    if (vt.norm() > kMinSlip) {
      params.mFirstFrictionalDirection = vt.normalized();
      ++mAligned;
    }
    return params;
  }

  dart::constraint::ContactConstraintPtr createConstraint(
      dart::collision::Contact& contact,
      std::size_t numContacts,
      double timeStep) const override
  {
    return fe::makeContactConstraint(*this, contact, numContacts, timeStep);
  }

  mutable long mCalls = 0;
  mutable long mAligned = 0;
};

//==============================================================================
struct Options
{
  std::string scene, solver = "dantzig", detector = "ode", label = "-";
  std::string split = "off", deactivation = "off";
  fe::Params params;
  double dt = 1e-3;
  bool va = false, perf = false;
  double erp = -1.0, cfm = -1.0, maxErv = -1.0;
  int threads = 1;
  long maxContacts = -1, maxContactsPerPair = -1;
  std::string dumpDir;
  std::set<int> dumpSteps;
  std::string bisectKey, bisectMetric;
  double bisectLo = 0.0, bisectHi = 0.0;
};

struct Probes
{
  std::shared_ptr<TightBoxSolver> tight;
  std::shared_ptr<VelocityAlignedHandler> va;
};

/// Backends that run on a bare LCP (also used by the L1 bank).
std::shared_ptr<BoxedLcpSolver> makeBackend(
    const std::string& name, Probes& probes)
{
  if (name == "dantzig")
    return std::make_shared<DantzigBoxedLcpSolver>();
  if (name == "pgs" || name == "pgs100") {
    auto pgs = std::make_shared<PgsBoxedLcpSolver>();
    pgs->setOption(PgsBoxedLcpSolver::Option(name == "pgs" ? 30 : 100));
    return pgs;
  }
  if (name == "pgs-tight" || name == "dzr") {
    probes.tight = std::make_shared<TightBoxSolver>(name == "dzr");
    return probes.tight;
  }
  return nullptr;
}

/// Applies the configuration under test. Only this factory depends on the
/// DART line (FRICTION_EVAL_DART620).
void configure(
    dart::simulation::World& world,
    const Options& o,
    Telemetry& telemetry,
    Probes& probes)
{
  using dart::constraint::ContactConstraint;
  if (o.erp >= 0.0)
    ContactConstraint::setErrorReductionParameter(o.erp);
  if (o.cfm >= 0.0)
    ContactConstraint::setConstraintForceMixing(o.cfm);
  if (o.maxErv >= 0.0)
    ContactConstraint::setMaxErrorReductionVelocity(o.maxErv);
  telemetry.cfm = ContactConstraint::getConstraintForceMixing();

  auto* solver = dynamic_cast<dart::constraint::BoxedLcpConstraintSolver*>(
      world.getConstraintSolver());
  auto detector = fe::makeDetector(o.detector);
  if (!solver || !detector)
    throw std::runtime_error("unknown detector: " + o.detector);
  solver->setCollisionDetector(detector);
  if (o.maxContacts > 0)
    solver->getCollisionOption().maxNumContacts = o.maxContacts;
  auto deactivation = world.getDeactivationOptions();
  deactivation.mEnabled = o.deactivation == "on";
  world.setDeactivationOptions(deactivation);

  auto primary = makeBackend(o.solver, probes);
#if FRICTION_EVAL_DART620
  if (o.solver == "mf-pgs") {
    primary = makeBackend("dantzig", probes);
    auto options = solver->getMatrixFreeContactSolverOptions();
    options.mEnabled = true;
    solver->setMatrixFreeContactSolverOptions(options);
  }
#endif
  if (!primary)
    throw std::runtime_error("unknown solver: " + o.solver);
  std::shared_ptr<BoxedLcpSolver> secondary
      = std::make_shared<PgsBoxedLcpSolver>();
  if (!o.perf) {
    primary
        = std::make_shared<CountingBoxedLcpSolver>(primary, telemetry, false);
    secondary
        = std::make_shared<CountingBoxedLcpSolver>(secondary, telemetry, true);
  }
  solver->setBoxedLcpSolver(primary);
  solver->setSecondaryBoxedLcpSolver(secondary);
#if FRICTION_EVAL_DART620
  // Split-impulse cells are pending PR-0: until it merges, the position pass
  // discards the velocity-phase impulses.
  solver->setSplitImpulseEnabled(o.split == "on");
  world.setNumSimulationThreads(static_cast<std::size_t>(o.threads));
  if (o.maxContactsPerPair > 0)
    solver->getCollisionOption().maxNumContactsPerPair = o.maxContactsPerPair;
#else
  if (o.split == "on" || o.threads != 1 || o.maxContactsPerPair > 0)
    throw std::runtime_error(
        "split impulse, threads and per-pair caps need the 6.20 line");
#endif
  if (o.va) {
    probes.va = std::make_shared<VelocityAlignedHandler>();
    solver->addContactSurfaceHandler(probes.va);
  }
}

//==============================================================================
/// Law audit from public API only (D §8.1): impulses from Contact::force * h
/// and contact velocities u = J(q_k) v_{k+1}: the post-step world velocities
/// (FreeJoint's generalized velocities) with lever arms from the pre-step
/// origins. Exact for free bodies; articulated bodies use their post-step
/// world velocities. Friction coefficients and surface motion come from the
/// contact-surface handler chain.
class Audit
{
public:
  void snapshot(const dart::simulation::World& world)
  {
    mOrigins.clear();
    for (std::size_t i = 0; i < world.getNumSkeletons(); ++i) {
      const auto skeleton = world.getSkeleton(i);
      for (std::size_t j = 0; j < skeleton->getNumBodyNodes(); ++j) {
        const auto* body = skeleton->getBodyNode(j);
        mOrigins[body] = fe::position(body);
      }
    }
  }

  void observe(
      const dart::simulation::World& world,
      const dart::constraint::ContactSurfaceHandler& handler)
  {
    const double h = world.getTimeStep();
    const Eigen::Vector3d up = -world.getGravity().normalized();
    const auto& result = world.getLastCollisionResult();
    // Relative violations are floored at 1e-3 of the step's largest normal
    // impulse, so a nearly unloaded contact does not dominate.
    double floor = 1e-12;
    for (std::size_t i = 0; i < result.getNumContacts(); ++i) {
      const auto& c = result.getContact(i);
      floor = std::max(floor, 1e-3 * h * c.force.dot(c.normal));
    }
    std::set<const dart::dynamics::BodyNode*> eligible;
    for (std::size_t i = 0; i < result.getNumContacts(); ++i) {
      const auto& c = result.getContact(i);
      const auto body1 = c.getBodyNodePtr1();
      const auto body2 = c.getBodyNodePtr2();
      markDriftSuppression(body1.get(), body2.get(), c, c.normal, up, eligible);
      markDriftSuppression(
          body2.get(), body1.get(), c, -c.normal, up, eligible);
      const Eigen::Vector3d lambda = c.force * h;
      const double ln = lambda.dot(c.normal);
      if (ln <= 1e-12)
        continue;
      const Eigen::Vector3d lt = lambda - ln * c.normal;
      const Eigen::Vector3d u
          = velocity(body1.get(), c.point) - velocity(body2.get(), c.point);
      const double un = u.dot(c.normal);
      const Eigen::Vector3d ut = u - un * c.normal;
      const auto params = handler.createParams(c, 1);
      const double mu1 = params.mPrimaryFrictionCoeff;
      const double mu2 = params.mSecondaryFrictionCoeff;
      const bool isotropic = mu1 == mu2 && mu1 > DART_FRICTION_COEFF_THRESHOLD;
      mMinNormalVelocity = std::min(mMinNormalVelocity, un);
      if (isotropic) {
        mConeViolation = std::max(
            mConeViolation,
            std::max(0.0, lt.norm() - mu1 * ln) / (mu1 * std::max(ln, floor)));
      }
      // Slip is relative to the surface motion, which these metrics skip.
      if (!params.mContactSurfaceMotionVelocity.tail<2>().isZero(0.0))
        continue;
      if (ut.norm() > fe::kSlideSpeed && lt.norm() > 1e-12) {
        const double err = fe::angleDeg(lt, -ut);
        mSlipDirSum += err;
        mSlipDirMax = std::max(mSlipDirMax, err);
        mDilatancySum += un / (std::max(mu1, mu2) * ut.norm());
        ++mSliding;
      } else if (isotropic && lt.norm() < 0.99 * mu1 * ln) {
        mStickSlip = std::max(mStickSlip, ut.norm());
      }
    }
    mEligible += static_cast<long>(eligible.size());
    for (std::size_t i = 0; i < world.getNumSkeletons(); ++i)
      mRootSteps += world.getSkeleton(i)->isMobile();
  }

  void report(fe::Metrics& m) const
  {
    m["cone_viol_max"] = mConeViolation;
    if (mSliding) {
      m["slip_dir_err_mean_deg"] = mSlipDirSum / mSliding;
      m["slip_dir_err_max_deg"] = mSlipDirMax;
      m["dilatancy_mean"] = mDilatancySum / mSliding;
    }
    m["stick_slip_max"] = mStickSlip;
    m["un_min"] = mMinNormalVelocity;
    m["fl_eligible_frac"]
        = mRootSteps ? static_cast<double>(mEligible) / mRootSteps : 0.0;
  }

private:
  Eigen::Vector3d velocity(
      const dart::dynamics::BodyNode* body, const Eigen::Vector3d& point) const
  {
    const auto it = mOrigins.find(body);
    if (!body || it == mOrigins.end())
      return Eigen::Vector3d::Zero();
    return body->getLinearVelocity()
           + body->getAngularVelocity().cross(point - it->second);
  }

  // The 6.20 World edits the velocities of shallow-supported free roots
  // (World.cpp findShallowSupportedFreeRoots); count where that could mask
  // creep (plan §6.3).
  static void markDriftSuppression(
      const dart::dynamics::BodyNode* body,
      const dart::dynamics::BodyNode* support,
      const dart::collision::Contact& c,
      const Eigen::Vector3d& normal,
      const Eigen::Vector3d& up,
      std::set<const dart::dynamics::BodyNode*>& eligible)
  {
    if (!body || !support || c.penetrationDepth < 0.0
        || c.penetrationDepth > 1e-4)
      return;
    const auto* skeleton = body->getSkeleton().get();
    const auto* supportSkeleton = support->getSkeleton().get();
    if (!skeleton->isMobile() || body != skeleton->getRootBodyNode()
        || !dynamic_cast<const dart::dynamics::FreeJoint*>(
            body->getParentJoint()))
      return;
    if (supportSkeleton->isMobile()
        && !(
            supportSkeleton->isResting()
            && !supportSkeleton->isImpulseApplied()))
      return;
    if (normal.dot(up) < 0.5
        || (fe::position(body) - fe::position(support)).dot(up) < -1e-4)
      return;
    eligible.insert(body);
  }

  std::map<const dart::dynamics::BodyNode*, Eigen::Vector3d> mOrigins;
  double mConeViolation = 0.0, mSlipDirSum = 0.0, mSlipDirMax = 0.0;
  double mDilatancySum = 0.0, mStickSlip = 0.0, mMinNormalVelocity = 0.0;
  long mSliding = 0, mEligible = 0, mRootSteps = 0;
};

double totalEnergy(const dart::simulation::World& world)
{
  double energy = 0.0;
  for (std::size_t i = 0; i < world.getNumSkeletons(); ++i) {
    const auto skeleton = world.getSkeleton(i);
    if (skeleton->isMobile())
      energy += skeleton->computeKineticEnergy()
                + skeleton->computePotentialEnergy();
  }
  return energy;
}

/// FNV-1a over every skeleton's positions and velocities (D1 parity).
std::uint64_t stateHash(const dart::simulation::World& world, bool& finite)
{
  std::uint64_t hash = 1469598103934665603ULL;
  finite = true;
  for (std::size_t i = 0; i < world.getNumSkeletons(); ++i) {
    const auto skeleton = world.getSkeleton(i);
    for (const auto& values :
         {skeleton->getPositions(), skeleton->getVelocities()}) {
      for (int j = 0; j < values.size(); ++j) {
        finite = finite && std::isfinite(values[j]);
        std::uint64_t bits = 0;
        std::memcpy(&bits, &values[j], sizeof(bits));
        hash = (hash ^ bits) * 1099511628211ULL;
      }
    }
  }
  return hash;
}

std::string paramString(const fe::Params& params)
{
  std::ostringstream out;
  for (const auto& [key, value] : params)
    out << (out.tellp() > 0 ? ";" : "") << key << "=" << value;
  return out.str().empty() ? "-" : out.str();
}

struct CellResult
{
  fe::Metrics metrics;
  std::string hash;
};

CellResult runCell(const Options& o, const fe::Params& params)
{
  const auto factory = fe::scenes().find(o.scene);
  if (factory == fe::scenes().end())
    throw std::runtime_error("unknown scene: " + o.scene);
  fe::Scene scene = factory->second(params, o.dt);
  auto& world = *scene.world;
  Telemetry telemetry;
  telemetry.dumpDir = o.dumpDir;
  telemetry.dumpSteps = o.dumpSteps;
  telemetry.dumpTag = o.scene + "_" + o.detector;
  Probes probes;
  configure(world, o, telemetry, probes);

  Audit audit;
  auto* solver = world.getConstraintSolver();
  // VA only rotates the basis; its parent has the contact parameters.
  const auto handler = probes.va ? probes.va->getParent()
                                 : solver->getLastContactSurfaceHandler();
  const double energy0 = totalEnergy(world);
  double energyRise = 0.0, contactSum = 0.0, contactMax = 0.0;
  int done = 0;
  const auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < scene.steps; ++i) {
    telemetry.step = i;
    if (!o.perf)
      audit.snapshot(world);
    if (scene.preStep)
      scene.preStep(i);
    world.step();
    ++done;
    const double contacts
        = static_cast<double>(world.getLastCollisionResult().getNumContacts());
    contactSum += contacts;
    contactMax = std::max(contactMax, contacts);
    if (!o.perf) {
      audit.observe(world, *handler);
      energyRise = std::max(energyRise, totalEnergy(world) - energy0);
    }
    if (scene.postStep && !scene.postStep(i))
      break;
  }
  const double seconds = secondsSince(start);

  CellResult result;
  auto& m = result.metrics;
  if (scene.finish)
    scene.finish(m);
  bool finite = true;
  std::ostringstream hash;
  hash << "0x" << std::hex << stateHash(world, finite);
  result.hash = hash.str();
  m["finite"] = finite;
  m["steps"] = done;
  m["wall_ms_per_step"] = 1e3 * seconds / done;
  m["contacts_mean"] = contactSum / done;
  m["contacts_max"] = contactMax;
  m["cap_hit"] = contactMax >= static_cast<double>(
                     solver->getCollisionOption().maxNumContacts);
  if (!o.perf) {
    audit.report(m);
    m["energy_rise_max"] = energyRise;
    m["solves"] = telemetry.solves;
    m["rows_mean"] = telemetry.solves ? static_cast<double>(telemetry.rows)
                                            / telemetry.solves
                                      : 0.0;
    m["rows_max"] = telemetry.maxRows;
    m["primary_failures"] = telemetry.failures;
    m["fallbacks"] = telemetry.fallbacks;
    m["fallback_failures"] = telemetry.fallbackFailures;
    m["non_finite_solves"] = telemetry.nonFinite;
    m["box_viol_max"] = telemetry.boxViolation;
    m["nat_res_max"] = telemetry.natural;
    m["nat_res_mean"]
        = telemetry.audited ? telemetry.naturalSum / telemetry.audited : 0.0;
    m["cfm_floor_max"] = telemetry.cfmFloor;
    m["solve_us_mean"]
        = telemetry.solves ? 1e6 * telemetry.seconds / telemetry.solves : 0.0;
    m["dumped"] = telemetry.dumped;
  }
  if (probes.tight) {
    const auto& stats = probes.tight->getStats();
    m["tight_solves"] = stats.solves;
    m["tight_capped"] = stats.capped;
    m["tight_sweeps_mean"]
        = stats.solves ? static_cast<double>(stats.sweeps) / stats.solves : 0.0;
    m["tight_change_max"] = stats.maxChange;
    if (o.solver == "dzr")
      m["dzr_refreshed"] = stats.refreshed;
  }
  if (probes.va) {
    m["va_aligned_frac"]
        = probes.va->mCalls
              ? static_cast<double>(probes.va->mAligned) / probes.va->mCalls
              : 0.0;
  }
  return result;
}

void printRow(
    const Options& o,
    const std::string& scene,
    const std::string& params,
    const std::string& metric,
    const std::string& value)
{
  std::printf(
      "%s,%s,%s,%s,%s%s,%s,%g,%s,%s,%s,%s\n",
      o.label.c_str(),
      FRICTION_EVAL_DART620 ? "6.20-line" : DART_VERSION,
      scene.c_str(),
      params.c_str(),
      o.solver.c_str(),
      o.va ? "+va" : "",
      o.detector.c_str(),
      o.dt,
      o.split.c_str(),
      o.deactivation.c_str(),
      metric.c_str(),
      value.c_str());
}

std::string number(double value)
{
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.10g", value);
  return buffer;
}

void printHeader()
{
  std::printf(
      "label,dart,scene,params,solver,detector,dt,split,deactivation,metric,"
      "value\n");
}

/// One cell, or --bisect key=lo:hi:metric: the threshold of a boolean scene
/// metric in six halvings of the bracket (1/64 resolution, D §8.2). A bisection
/// prints the metric at both ends, omits the threshold when they agree, and
/// reports finite = 0 if any of its runs ended with a non-finite state.
int runCellOrBisect(const Options& o)
{
  printHeader();
  if (o.bisectKey.empty()) {
    const auto result = runCell(o, o.params);
    const auto params = paramString(o.params);
    for (const auto& [metric, value] : result.metrics)
      printRow(o, o.scene, params, metric, number(value));
    printRow(o, o.scene, params, "state_hash", result.hash);
    return 0;
  }
  fe::Params params = o.params;
  bool finite = true;
  const auto at = [&](double value) {
    params[o.bisectKey] = value;
    const auto result = runCell(o, params);
    finite = finite && result.metrics.at("finite") != 0.0;
    const auto it = result.metrics.find(o.bisectMetric);
    if (it == result.metrics.end())
      throw std::runtime_error("no metric " + o.bisectMetric);
    return std::make_pair(it->second != 0.0, result.metrics);
  };
  double lo = o.bisectLo, hi = o.bisectHi;
  const auto [atLo, metrics] = at(lo);
  const bool atHi = at(hi).first;
  params.erase(o.bisectKey);
  const auto label = paramString(params) + ";bisect=" + o.bisectKey;
  if (atLo != atHi) {
    for (int i = 0; i < 6; ++i) {
      const double mid = 0.5 * (lo + hi);
      (at(mid).first == atLo ? lo : hi) = mid;
    }
    printRow(o, o.scene, label, "threshold", number(0.5 * (lo + hi)));
  }
  printRow(o, o.scene, label, "at_lo", number(atLo));
  printRow(o, o.scene, label, "at_hi", number(atHi));
  printRow(o, o.scene, label, "finite", number(finite));
  for (const auto& [metric, value] : metrics) {
    if (metric.rfind("pred_", 0) == 0)
      printRow(o, o.scene, label, metric, number(value));
  }
  return 0;
}

//==============================================================================
// L1 bank: frozen problems from dumps plus built-in families (plan §6.2).

/// Contacts as rows (normal, t1, t2) with W the Delassus operator.
Problem contactProblem(
    const std::string& name,
    const Eigen::MatrixXd& W,
    const Eigen::VectorXd& b,
    const std::vector<std::pair<double, double>>& mu)
{
  Problem p;
  p.name = name;
  p.n = static_cast<int>(W.rows());
  const int s = stride(p.n);
  p.A.assign(static_cast<std::size_t>(p.n) * s, 0.0);
  for (int i = 0; i < p.n; ++i)
    for (int j = 0; j < p.n; ++j)
      p.A[static_cast<std::size_t>(i) * s + j] = W(i, j);
  p.b.assign(b.data(), b.data() + p.n);
  p.x.assign(p.n, 0.0);
  p.lo.assign(p.n, 0.0);
  p.hi.assign(p.n, std::numeric_limits<double>::infinity());
  p.findex.assign(p.n, -1);
  for (int c = 0; 3 * c + 2 < p.n; ++c) {
    for (int k = 1; k <= 2; ++k) {
      const double m = k == 1 ? mu[c].first : mu[c].second;
      p.lo[3 * c + k] = -m;
      p.hi[3 * c + k] = m;
      p.findex[3 * c + k] = 3 * c;
    }
  }
  return p;
}

/// critique_checks.box_stack: boxes of side 0.2 stacked on the ground, four
/// corner contacts per interface, resting under one step of gravity (h = 1 ms).
Problem boxStack(const std::string& name, const std::vector<double>& masses)
{
  const int nb = static_cast<int>(masses.size());
  const double size = 0.2;
  Eigen::VectorXd minv(6 * nb);
  for (int k = 0; k < nb; ++k) {
    minv.segment<3>(6 * k).setConstant(1.0 / masses[k]);
    minv.segment<3>(6 * k + 3).setConstant(6.0 / (masses[k] * size * size));
  }
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(12 * nb, 6 * nb);
  int row = 0;
  for (int k = 0; k < nb; ++k) {
    const double zc = size * k;
    for (const double sx : {-1.0, 1.0}) {
      for (const double sy : {-1.0, 1.0}) {
        const Eigen::Vector3d p(sx * size / 2, sy * size / 2, zc);
        for (const auto& d :
             {Eigen::Vector3d::UnitZ(),
              Eigen::Vector3d::UnitX(),
              Eigen::Vector3d::UnitY()}) {
          const Eigen::Vector3d above
              = p - Eigen::Vector3d(0, 0, zc + size / 2);
          J.block<1, 3>(row, 6 * k) = d.transpose();
          J.block<1, 3>(row, 6 * k + 3) = above.cross(d).transpose();
          if (k > 0) {
            const Eigen::Vector3d below
                = p - Eigen::Vector3d(0, 0, zc - size / 2);
            J.block<1, 3>(row, 6 * k - 6) = -d.transpose();
            J.block<1, 3>(row, 6 * k - 3) = -below.cross(d).transpose();
          }
          ++row;
        }
      }
    }
  }
  Eigen::MatrixXd W = J * minv.asDiagonal() * J.transpose();
  W.diagonal() *= 1.0 + 1e-5;
  Eigen::VectorXd b = Eigen::VectorXd::Zero(12 * nb);
  for (int c = 0; c < 4; ++c)
    b(3 * c) = fe::kGravity * 1e-3;
  return contactProblem(
      name, W, b, std::vector<std::pair<double, double>>(4 * nb, {0.5, 0.5}));
}

/// Random rigid contacts between free bodies (every fourth on the ground).
/// With bodies < contacts/2, W is rank-deficient before DART's CFM.
Problem randomContacts(
    const std::string& name, int contacts, int bodies, unsigned seed)
{
  std::mt19937_64 rng(seed);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  std::uniform_real_distribution<double> sym(-1.0, 1.0);
  std::normal_distribution<double> gauss(0.0, 1.0);
  // Every draw comes from rng in a fixed order, so the seed alone fixes the
  // problem (function arguments would leave the order to the compiler).
  const auto draw3 = [&rng](auto& dist) {
    const double x = dist(rng), y = dist(rng), z = dist(rng);
    return Eigen::Vector3d(x, y, z);
  };
  const int n = 3 * contacts;
  Eigen::VectorXd minv(6 * bodies);
  for (int k = 0; k < bodies; ++k) {
    const double mass = 0.5 + 1.5 * unit(rng);
    minv.segment<3>(6 * k).setConstant(1.0 / mass);
    minv.segment<3>(6 * k + 3).setConstant(10.0 / mass);
  }
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(n, 6 * bodies);
  Eigen::VectorXd b(n);
  std::vector<std::pair<double, double>> mu;
  for (int c = 0; c < contacts; ++c) {
    const int a = c % bodies;
    const int other = c % 4 == 0 ? -1 : (a + 1 + c / bodies) % bodies;
    const Eigen::Vector3d normal = draw3(gauss).normalized();
    const Eigen::Vector3d t1 = normal.unitOrthogonal();
    const Eigen::Vector3d t2 = normal.cross(t1);
    const Eigen::Vector3d ra = 0.3 * draw3(sym);
    const Eigen::Vector3d rb = 0.3 * draw3(sym);
    for (int k = 0; k < 3; ++k) {
      const Eigen::Vector3d d = k == 0 ? normal : (k == 1 ? t1 : t2);
      J.block<1, 3>(3 * c + k, 6 * a) = d.transpose();
      J.block<1, 3>(3 * c + k, 6 * a + 3) = ra.cross(d).transpose();
      if (other >= 0 && other != a) {
        J.block<1, 3>(3 * c + k, 6 * other) = -d.transpose();
        J.block<1, 3>(3 * c + k, 6 * other + 3) = -rb.cross(d).transpose();
      }
    }
    b.segment<3>(3 * c) << 0.05 + 0.95 * unit(rng), 0.5 * gauss(rng),
        0.5 * gauss(rng);
    const double m = 0.2 + 0.8 * unit(rng);
    mu.emplace_back(m, m);
  }
  Eigen::MatrixXd W = J * minv.asDiagonal() * J.transpose();
  W.diagonal() *= 1.0 + 1e-5;
  return contactProblem(name, W, b, mu);
}

/// Built-in families: the Codex regressions (D §5), the design-check banks
/// (check_design_math single contacts, critique_checks stacks) and synthetic
/// SPD/PSD contact networks up to 3000 rows.
std::vector<Problem> builtinProblems()
{
  std::vector<Problem> problems;
  Eigen::Matrix3d W;
  W << 50.5, -49.5, 0, -49.5, 50.5, 0, 0, 0, 1;
  problems.push_back(contactProblem(
      "codex_orthogonal_seed", W, Eigen::Vector3d(1, 0.5, -0.5), {{0.5, 0.5}}));
  W << 2, 0.3, -0.2, 0.3, 1.5, 0.1, -0.2, 0.1, 1.2;
  problems.push_back(contactProblem(
      "codex_mu_1e-8", W, Eigen::Vector3d(1, 2, -1), {{1e-8, 1e-8}}));
  problems.push_back(contactProblem(
      "codex_finite_normal_cap",
      Eigen::Matrix3d::Identity(),
      Eigen::Vector3d(1, 0.3, 0),
      {{0.5, 0.5}}));
  problems.back().hi[0] = 0.5;
  problems.push_back(contactProblem(
      "codex_diag_1_100_1",
      Eigen::Vector3d(1, 100, 1).asDiagonal(),
      Eigen::Vector3d(1, 30, 0.2),
      {{0.5, 0.5}}));

  std::mt19937_64 rng(7);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  std::normal_distribution<double> gauss(0.0, 1.0);
  for (int i = 0; i < 300; ++i) {
    Eigen::Matrix3d jt;
    for (int k = 0; k < 9; ++k)
      jt(k / 3, k % 3) = gauss(rng);
    const double mu1 = 0.2 + unit(rng), mu2 = 0.2 + unit(rng);
    // Drawn z, y, x: the order GCC used for these as constructor arguments, so
    // E1's bank stays the same while the order no longer depends on compilers.
    const double qz = 3 * gauss(rng), qy = 3 * gauss(rng);
    const Eigen::Vector3d q(-0.5 - 1.5 * unit(rng), qy, qz);
    problems.push_back(contactProblem(
        "check_single_" + std::to_string(i),
        jt * jt.transpose() + 0.05 * Eigen::Matrix3d::Identity(),
        -q,
        {{mu1, mu2}}));
  }
  problems.push_back(boxStack("check_stack_1", {1.0}));
  problems.push_back(boxStack("check_stack_5", std::vector<double>(5, 1.0)));
  problems.push_back(boxStack("check_stack_10", std::vector<double>(10, 1.0)));
  problems.push_back(boxStack("check_stack_100to1", {1.0, 100.0}));

  for (const int contacts : {10, 100, 1000}) {
    problems.push_back(randomContacts(
        "synthetic_spd_" + std::to_string(3 * contacts),
        contacts,
        contacts,
        contacts));
    problems.push_back(randomContacts(
        "synthetic_psd_" + std::to_string(3 * contacts),
        contacts,
        contacts / 4,
        contacts + 1));
  }
  return problems;
}

/// Solves every problem with each backend on a copy and audits the result on
/// the pristine terms. As in the default World, which has a fallback, solves
/// use early termination, so "ok" = 0 marks a fallback in production.
int runL1(const Options& o, const std::vector<std::string>& files)
{
  std::vector<Problem> problems;
  if (files.empty())
    problems = builtinProblems();
  for (const auto& file : files)
    problems.push_back(Problem::load(file));
  printHeader();
  for (const auto& problem : problems) {
    for (const std::string name :
         {"dantzig", "pgs", "pgs100", "pgs-tight", "dzr"}) {
      Probes probes;
      auto solver = makeBackend(name, probes);
      Problem terms = problem;
      const auto start = std::chrono::steady_clock::now();
      const bool ok = solver->solve(
          terms.n,
          terms.A.data(),
          terms.x.data(),
          terms.b.data(),
          0,
          terms.lo.data(),
          terms.hi.data(),
          terms.findex.data(),
          true);
      const double ms = 1e3 * secondsSince(start);
      const Residual r = boxResidual(problem, terms.x.data(), 1e-5);
      Options row = o;
      row.solver = name;
      row.detector = "-";
      const auto params = "n=" + std::to_string(problem.n);
      printRow(row, problem.name, params, "ok", number(ok));
      printRow(row, problem.name, params, "finite", number(r.finite));
      printRow(row, problem.name, params, "nat_res", number(r.natural));
      printRow(row, problem.name, params, "box_viol", number(r.boxViolation));
      printRow(row, problem.name, params, "cfm_floor", number(r.cfmFloor));
      printRow(row, problem.name, params, "solve_ms", number(ms));
      if (probes.tight)
        printRow(
            row,
            problem.name,
            params,
            "tight_capped",
            number(probes.tight->getStats().capped));
    }
  }
  return 0;
}

//==============================================================================
int selfTest()
{
  int failures = 0;
  const auto check = [&](bool ok, const std::string& what) {
    std::printf("%s %s\n", ok ? "ok  " : "FAIL", what.c_str());
    failures += !ok;
  };
  const auto solveWith = [](const std::string& name, const Problem& p) {
    Probes probes;
    Problem terms = p;
    makeBackend(name, probes)
        ->solve(
            p.n,
            terms.A.data(),
            terms.x.data(),
            terms.b.data(),
            0,
            terms.lo.data(),
            terms.hi.data(),
            terms.findex.data(),
            false);
    return terms.x;
  };

  // The discrete slide reference matches semi-implicit Euler with the last
  // friction impulse capped, with and without a stop inside the horizon.
  bool slideOk = true;
  for (const auto& [a, n] :
       {std::pair{4.905, 300}, std::pair{4.905, 100}, std::pair{0.0, 300}}) {
    double v = 1.0, x = 0.0;
    for (int k = 0; k < n; ++k) {
      v = std::max(0.0, v - a * 1e-3);
      x += 1e-3 * v;
    }
    slideOk
        = slideOk
          && std::abs(x - fe::discreteSlideDistance(1.0, a, 1e-3, n)) < 1e-12;
  }
  check(slideOk, "discrete slide reference with and without a stop");

  // W = I, b = (1, 2, 0), mu 0.5: x = (1, 0.5, 0). Perturbing x_t to 0.7 gives
  // a natural-map residual of 0.2 m/s and a box-law violation of 0.4.
  const Problem unit = contactProblem(
      "unit",
      Eigen::Matrix3d::Identity(),
      Eigen::Vector3d(1, 2, 0),
      {{0.5, 0.5}});
  const double perturbed[3] = {1.0, 0.7, 0.0};
  const Residual r = boxResidual(unit, perturbed, 0.0);
  check(
      std::abs(r.natural - 0.2) < 1e-12
          && std::abs(r.boxViolation - 0.4) < 1e-3,
      "box-law residual of a hand-perturbed solution");
  for (const std::string name : {"dantzig", "pgs", "pgs-tight", "dzr"}) {
    const auto x = solveWith(name, unit);
    check(
        std::abs(x[0] - 1.0) + std::abs(x[1] - 0.5) + std::abs(x[2]) < 1e-6
            && boxResidual(unit, x.data(), 0.0).natural < 1e-6,
        name + " solves the unit contact");
  }

  // F-b: the frictionless normal impulse is 1, so Dantzig freezes the bound at
  // 0.5, and the coupling then lowers the final normal impulse to 0.75 (box-law
  // violation 1/3). The box law's solution is (0.8, 0.4, 0).
  Eigen::Matrix3d coupled;
  coupled << 1, 0.5, 0, 0.5, 1, 0, 0, 0, 1;
  const Problem frozen = contactProblem(
      "frozen", coupled, Eigen::Vector3d(1, 2, 0), {{0.5, 0.5}});
  const auto dantzig = solveWith("dantzig", frozen);
  const auto refreshed = solveWith("dzr", frozen);
  check(
      std::abs(boxResidual(frozen, dantzig.data(), 0.0).boxViolation - 1.0 / 3)
          < 1e-3,
      "Dantzig keeps the friction bound it froze (F-b)");
  check(
      std::abs(refreshed[0] - 0.8) + std::abs(refreshed[1] - 0.4) < 1e-6,
      "DZ+R re-solves the box law with refreshed bounds");

  const auto path
      = (std::filesystem::temp_directory_path() / "friction_eval_self_test.lcp")
            .string();
  frozen.save(path);
  const Problem loaded = Problem::load(path);
  std::filesystem::remove(path);
  check(
      loaded.n == frozen.n && loaded.A == frozen.A && loaded.b == frozen.b
          && loaded.lo == frozen.lo && loaded.hi == frozen.hi
          && loaded.findex == frozen.findex,
      "problem dump round trip");

  // Eigen's Random() and std::rand() share a global state; the bank must not.
  std::srand(1);
  const Problem seeded = randomContacts("seeded", 8, 3, 5);
  std::srand(2);
  const Problem again = randomContacts("seeded", 8, 3, 5);
  check(
      seeded.A == again.A && seeded.b == again.b && seeded.hi == again.hi,
      "random contact networks depend only on their seed");

  // A slide 30 deg off the basis: both box rows saturate, so the default
  // friction force points 45 deg off the basis (about 15 deg off the slip);
  // VA aligns the basis with the slip.
  for (const bool va : {false, true}) {
    Options o;
    o.scene = "A5";
    o.detector = "dart";
    o.va = va;
    o.perf = true;
    fe::Scene scene = fe::scenes().at("A5")({{"phi", 30.0}}, o.dt);
    Telemetry telemetry;
    Probes probes;
    configure(*scene.world, o, telemetry, probes);
    for (int i = 0; i < 20; ++i)
      scene.world->step();
    auto* box = scene.world->getSkeleton("box")->getBodyNode(0);
    const double err = fe::angleDeg(
        fe::planar(fe::contactForce(*scene.world, box)),
        -fe::planar(box->getLinearVelocity()));
    check(
        va ? err < 0.5 : std::abs(err - 15.0) < 2.0,
        va ? "VA aligns the friction force with the slip"
           : "the default box pulls 45 deg off a 30 deg slip");
  }
  std::printf("%s\n", failures ? "self-test FAILED" : "self-test passed");
  return failures ? 1 : 0;
}

void parseParams(std::string text, fe::Params& params)
{
  std::replace(text.begin(), text.end(), ';', ',');
  std::stringstream in(text);
  std::string item;
  while (std::getline(in, item, ',')) {
    const auto eq = item.find('=');
    if (eq == std::string::npos)
      throw std::runtime_error("bad parameter: " + item);
    params[item.substr(0, eq)] = std::stod(item.substr(eq + 1));
  }
}

} // namespace

int main(int argc, char** argv)
{
  Options o;
  std::vector<std::string> files;
  std::string mode = "cell";
  try {
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      const auto value = [&]() -> std::string {
        if (i + 1 >= argc)
          throw std::runtime_error("missing value for " + arg);
        return argv[++i];
      };
      if (arg == "--scene") {
        o.scene = value();
      } else if (arg == "--param") {
        parseParams(value(), o.params);
      } else if (arg == "--solver") {
        o.solver = value();
      } else if (arg == "--detector") {
        o.detector = value();
      } else if (arg == "--dt") {
        o.dt = std::stod(value());
      } else if (arg == "--va") {
        o.va = true;
      } else if (arg == "--perf") {
        o.perf = true;
      } else if (arg == "--erp") {
        o.erp = std::stod(value());
      } else if (arg == "--cfm") {
        o.cfm = std::stod(value());
      } else if (arg == "--max-erv") {
        o.maxErv = std::stod(value());
      } else if (arg == "--split") {
        o.split = value();
      } else if (arg == "--threads") {
        o.threads = std::stoi(value());
      } else if (arg == "--deactivation") {
        o.deactivation = value();
      } else if (arg == "--max-contacts") {
        o.maxContacts = std::stol(value());
      } else if (arg == "--max-contacts-per-pair") {
        o.maxContactsPerPair = std::stol(value());
      } else if (arg == "--dump") {
        o.dumpDir = value();
        std::filesystem::create_directories(o.dumpDir);
      } else if (arg == "--dump-steps") {
        std::stringstream in(value());
        std::string step;
        while (std::getline(in, step, ','))
          o.dumpSteps.insert(std::stoi(step));
      } else if (arg == "--bisect") {
        // key=lo:hi:metric
        std::string spec = value();
        std::replace(spec.begin(), spec.end(), ':', ' ');
        std::replace(spec.begin(), spec.end(), '=', ' ');
        std::stringstream in(spec);
        if (!(in >> o.bisectKey >> o.bisectLo >> o.bisectHi >> o.bisectMetric))
          throw std::runtime_error("bad --bisect, use key=lo:hi:metric");
      } else if (arg == "--label") {
        o.label = value();
      } else if (arg == "--l1" || arg == "--self-test" || arg == "--list") {
        mode = arg;
      } else if (mode == "--l1" && arg[0] != '-') {
        files.push_back(arg);
      } else {
        throw std::runtime_error("unknown option: " + arg);
      }
    }
    if (mode == "--self-test")
      return selfTest();
    if (mode == "--list") {
      for (const auto& [id, factory] : fe::scenes())
        std::printf("%s\n", id.c_str());
      return 0;
    }
    if (mode == "--l1")
      return runL1(o, files);
    return runCellOrBisect(o);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "friction_eval: %s\n", e.what());
    return 2;
  }
}
