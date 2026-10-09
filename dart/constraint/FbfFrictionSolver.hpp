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

#ifndef DART_CONSTRAINT_FBFFRICTIONSOLVER_HPP_
#define DART_CONSTRAINT_FBFFRICTIONSOLVER_HPP_

#include <dart/constraint/BoxedLcpSolver.hpp>
#include <dart/constraint/FrictionSolveStats.hpp>

#include <atomic>

#include <cstddef>
#include <cstdint>

namespace dart {
namespace constraint {

/// Opt-in forward-backward-forward (Tseng) friction solver for exact Coulomb
/// friction. Each outer iteration freezes every contact's De Saxce normal shift
/// (the ellipse term for circular or elliptic contacts, the box term for box
/// contacts), solves the resulting proximal problem by block Gauss-Seidel with
/// certified local cone QPs, and then corrects the shift explicitly. The step
/// size is certified from the absolute row and column sums of the contacts'
/// tangent rows of A. Input terms remain read-only; the returned impulse
/// follows the selected law, including at the iteration cap. An unsuccessful
/// iteration requests the secondary when available, or returns the best
/// completed finite iterate without one. Independent groups may solve
/// concurrently; options must not change during solve(), and reserve()
/// reserves scratch only for the calling thread. Install this backend before
/// the World is prepared (before enterSimulationMode() or the first step).
/// After switching backends in an already prepared World, each solving thread
/// grows its scratch on its first solve; later groups exceeding that capacity
/// may require further growth.
class FbfFrictionSolver : public BoxedLcpSolver
{
public:
  struct Options
  {
    /// Unequal mu1/mu2 use the sdformat friction pyramid (box law). The
    /// relative switch tolerance is 1e-9; capacity can jump by sqrt(2) at 45
    /// degrees. Disable this option for an ellipse continuous in the two
    /// coefficients.
    bool boxForAnisotropic = true;
    /// Outer iterations. Groups without a frictional cone contact run plain
    /// Gauss-Seidel sweeps instead, one per outer iteration. Zero accepts the
    /// projected warm start at the cap.
    int maxOuterIterations = 100;
    double tolerance = 1e-5; ///< Per-contact/scalar law violation [m/s].
    /// Step size gamma = stepScale / (mu_eff * L_T), where mu_eff is the
    /// largest Lipschitz factor of the contacts' shifts and L_T bounds the norm
    /// of the contacts' tangent rows of A; the step-size ratio then stays below
    /// stepScale up to rounding. Values above 0.9 use the step-size search
    /// (ratio 0.9, factor 0.7, at most 20 reductions per iteration).
    double stepScale = 0.5;
    /// Gauss-Seidel sweeps per inner solve (at least 1).
    int maxInnerSweeps = 20;
    /// Each inner solve runs at least one sweep and stops at a residual of
    /// max(1e-12, factor * min(outer violation, 100 * tolerance)) [m/s].
    double innerToleranceFactor = 0.1;
  };

  FbfFrictionSolver();
  explicit FbfFrictionSolver(const Options& options);

  const std::string& getType() const override;
  static const std::string& getStaticType();

  /// Warm starts are projected to the friction cones and scalar/PGS bounds
  /// before comparing residuals. With earlyTermination true, divergence (no
  /// iterate at least as good as the start), non-finite iterates, an
  /// uncertified local solve, or an exhausted step-size search return false
  /// to request the secondary. With it false, the best completed finite
  /// projected iterate is accepted and counted in numAcceptedAtCap. Zero
  /// maxOuterIterations also accepts that projected iterate at the cap.
  /// Invalid options (negative maxOuterIterations, maxInnerSweeps below 1,
  /// negative or non-finite tolerance or innerToleranceFactor, non-positive or
  /// non-finite stepScale), invalid indices, non-finite inputs, an
  /// unrepresentable step size (gamma or 1 / gamma not finite and positive),
  /// or failure to form a projected starting iterate with a finite law
  /// violation return false in both modes. Self indices (findex[k] == k) are
  /// rejected before touching x so the secondary can apply the built-in
  /// coupling semantics.
  bool solve(
      int n,
      double* A,
      double* x,
      double* b,
      int nub,
      double* lo,
      double* hi,
      int* findex,
      bool earlyTermination = false) override;

#if DART_BUILD_MODE_DEBUG
  // Remove this override when the base-class canSolve() is removed.
  bool canSolve(int n, const double* A) override;
#endif

  void setOptions(const Options& options);
  const Options& getOptions() const;
  void reserve(std::size_t numRows);
  /// Snapshots are consistent after concurrent group solves have completed.
  FrictionSolveStats getStats() const;
  void resetStats();

private:
  void accumulateStats(const FrictionSolveStats& stats);

  Options mOptions;
  std::atomic<std::uint64_t> mNumSolves{0};
  std::atomic<std::uint64_t> mNumConverged{0};
  std::atomic<std::uint64_t> mNumAcceptedAtCap{0};
  std::atomic<std::uint64_t> mNumFailed{0};
  std::atomic<std::uint64_t> mNumContacts{0};
  std::atomic<std::uint64_t> mNumBoxContacts{0};
  std::atomic<std::uint64_t> mNumLocalFallbacks{0};
  std::atomic<std::uint64_t> mNumIterations{0};
  std::atomic<std::uint64_t> mNumInnerIterations{0};
  std::atomic<std::uint64_t> mNumStepShrinks{0};
  std::atomic<std::uint64_t> mNumInnerCaps{0};
  std::atomic<double> mMaxViolation{0.0};
};

} // namespace constraint
} // namespace dart

#endif // DART_CONSTRAINT_FBFFRICTIONSOLVER_HPP_
