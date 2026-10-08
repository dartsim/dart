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

#ifndef DART_CONSTRAINT_NSGSFRICTIONSOLVER_HPP_
#define DART_CONSTRAINT_NSGSFRICTIONSOLVER_HPP_

#include <dart/constraint/BoxedLcpSolver.hpp>
#include <dart/constraint/FrictionSolveStats.hpp>

#include <atomic>

#include <cstddef>
#include <cstdint>

namespace dart {
namespace constraint {

/// Opt-in block nonsmooth Gauss-Seidel friction solver. Input terms remain
/// read-only; the returned impulse follows the selected law, including at the
/// iteration cap. An unsuccessful iteration requests the secondary when
/// available, or returns the best completed finite iterate without one.
/// Independent groups may solve concurrently; options must not change during
/// solve(), and reserve() reserves scratch only for the calling thread.
/// Install this backend before the World is prepared (before
/// enterSimulationMode() or the first step). After switching backends in an
/// already prepared World, each solving thread grows its scratch on its first
/// solve; later groups exceeding that capacity may require further growth.
class NsgsFrictionSolver : public BoxedLcpSolver
{
public:
  enum class Law
  {
    Coulomb,    ///< Exact non-associated Coulomb with maximal dissipation.
    Associated, ///< Convex circle/ellipse or box relaxation; contacts may
                ///< glide.
    Box         ///< DART/ODE box friction in each tangent direction.
  };

  struct Options
  {
    Law law = Law::Coulomb;
    /// Unequal mu1/mu2 use the sdformat friction pyramid. The relative switch
    /// tolerance is 1e-9; capacity can jump by sqrt(2) at 45 degrees. Disable
    /// this option for an ellipse continuous in the two coefficients.
    bool boxForAnisotropic = true;
    int maxSweeps = 100;
    double tolerance = 1e-5; ///< Per-contact/scalar law violation [m/s].
  };

  NsgsFrictionSolver();
  explicit NsgsFrictionSolver(const Options& options);

  const std::string& getType() const override;
  static const std::string& getStaticType();

  /// Warm starts are projected to the selected cone or scalar/PGS bounds
  /// before comparing residuals. With earlyTermination true, divergence,
  /// non-finite iterates, or an uncertified local solve return false to request
  /// the secondary. With it false, the best completed finite projected iterate
  /// is accepted and counted in numAcceptedAtCap. Zero maxSweeps also accepts
  /// that projected iterate at the cap. Invalid options (negative cap or
  /// tolerance, non-finite tolerance, unknown law), invalid indices, non-finite
  /// inputs, or failure to form a projected starting iterate with a finite law
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
  std::atomic<double> mMaxViolation{0.0};
};

} // namespace constraint
} // namespace dart

#endif // DART_CONSTRAINT_NSGSFRICTIONSOLVER_HPP_
