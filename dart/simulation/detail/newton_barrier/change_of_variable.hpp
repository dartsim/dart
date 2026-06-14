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
 *     copyright notice and this list of conditions in the documentation
 *     and/or other materials provided with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <dart/common/memory_allocator.hpp>
#include <dart/common/stl_allocator.hpp>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/QR>
#include <Eigen/SparseCore>

#include <algorithm>
#include <limits>
#include <span>
#include <vector>

#include <cmath>

namespace dart::simulation::detail::newton_barrier {

using EqualityIndexAllocator = dart::common::StlAllocator<int>;
using EqualityIndexVector = std::vector<int, EqualityIndexAllocator>;

struct EqualityChangeOfVariableResult
{
  EqualityChangeOfVariableResult() = default;

  explicit EqualityChangeOfVariableResult(
      dart::common::MemoryAllocator& allocator)
    : independentRows(EqualityIndexAllocator{allocator}),
      constrainedColumns(EqualityIndexAllocator{allocator}),
      freeColumns(EqualityIndexAllocator{allocator})
  {
  }

  bool valid = false;
  bool fullRowRank = false;
  int rank = 0;
  double residualNorm = std::numeric_limits<double>::infinity();
  EqualityIndexVector independentRows;
  EqualityIndexVector constrainedColumns;
  EqualityIndexVector freeColumns;
  Eigen::VectorXd particularStep;
  Eigen::MatrixXd nullspaceBasis;
};

namespace detail {

//==============================================================================
[[nodiscard]] inline bool allFinite(const Eigen::MatrixXd& matrix)
{
  return matrix.allFinite();
}

//==============================================================================
[[nodiscard]] inline bool allFinite(const Eigen::VectorXd& vector)
{
  return vector.allFinite();
}

//==============================================================================
[[nodiscard]] inline Eigen::MatrixXd selectRows(
    const Eigen::MatrixXd& matrix, std::span<const int> rows)
{
  Eigen::MatrixXd selected(rows.size(), matrix.cols());
  for (int row = 0; row < static_cast<int>(rows.size()); ++row) {
    selected.row(row) = matrix.row(rows[row]);
  }
  return selected;
}

//==============================================================================
[[nodiscard]] inline Eigen::VectorXd selectEntries(
    const Eigen::VectorXd& vector, std::span<const int> indices)
{
  Eigen::VectorXd selected(indices.size());
  for (int row = 0; row < static_cast<int>(indices.size()); ++row) {
    selected[row] = vector[indices[row]];
  }
  return selected;
}

//==============================================================================
[[nodiscard]] inline Eigen::MatrixXd selectColumns(
    const Eigen::MatrixXd& matrix, std::span<const int> columns)
{
  Eigen::MatrixXd selected(matrix.rows(), columns.size());
  for (int col = 0; col < static_cast<int>(columns.size()); ++col) {
    selected.col(col) = matrix.col(columns[col]);
  }
  return selected;
}

//==============================================================================
[[nodiscard]] inline EqualityIndexVector firstPivots(
    const Eigen::VectorXi& pivots,
    const int count,
    dart::common::MemoryAllocator& allocator
    = dart::common::MemoryAllocator::GetDefault())
{
  EqualityIndexVector selected(EqualityIndexAllocator{allocator});
  selected.reserve(count);
  for (int i = 0;
       i < pivots.size() && static_cast<int>(selected.size()) < count;
       ++i) {
    const int index = pivots[i];
    if (std::find(selected.begin(), selected.end(), index) == selected.end()) {
      selected.push_back(index);
    }
  }
  return selected;
}

//==============================================================================
[[nodiscard]] inline EqualityIndexVector complementIndices(
    const int size,
    std::span<const int> selected,
    dart::common::MemoryAllocator& allocator
    = dart::common::MemoryAllocator::GetDefault())
{
  EqualityIndexVector complement(EqualityIndexAllocator{allocator});
  complement.reserve(size - static_cast<int>(selected.size()));
  for (int index = 0; index < size; ++index) {
    if (std::find(selected.begin(), selected.end(), index) == selected.end()) {
      complement.push_back(index);
    }
  }
  return complement;
}

} // namespace detail

//==============================================================================
[[nodiscard]] inline EqualityChangeOfVariableResult
makeEqualityChangeOfVariable(
    const Eigen::SparseMatrix<double>& equalityJacobian,
    const Eigen::VectorXd& equalityResidual,
    dart::common::MemoryAllocator& allocator,
    const double tolerance = 1e-12)
{
  EqualityChangeOfVariableResult result(allocator);
  const int numRows = static_cast<int>(equalityJacobian.rows());
  const int numCols = static_cast<int>(equalityJacobian.cols());
  result.particularStep = Eigen::VectorXd::Zero(numCols);
  result.nullspaceBasis = Eigen::MatrixXd::Identity(numCols, numCols);

  if (equalityResidual.size() != numRows || numCols < 0 || numRows < 0) {
    return result;
  }

  const Eigen::MatrixXd denseJacobian(equalityJacobian);
  if (!detail::allFinite(denseJacobian)
      || !detail::allFinite(equalityResidual)) {
    return result;
  }

  if (numRows == 0) {
    result.valid = true;
    result.fullRowRank = true;
    result.rank = 0;
    result.residualNorm = 0.0;
    result.freeColumns
        = detail::complementIndices(numCols, std::span<const int>{}, allocator);
    return result;
  }

  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> rowQr(denseJacobian.transpose());
  rowQr.setThreshold(tolerance);
  result.rank = rowQr.rank();
  result.fullRowRank = result.rank == numRows;

  if (result.rank == 0) {
    result.valid = true;
    result.residualNorm = equalityResidual.norm();
    result.freeColumns
        = detail::complementIndices(numCols, std::span<const int>{}, allocator);
    return result;
  }

  result.independentRows = detail::firstPivots(
      rowQr.colsPermutation().indices(), result.rank, allocator);
  if (static_cast<int>(result.independentRows.size()) != result.rank) {
    return result;
  }

  const Eigen::MatrixXd independentJacobian
      = detail::selectRows(denseJacobian, result.independentRows);
  const Eigen::VectorXd independentResidual
      = detail::selectEntries(equalityResidual, result.independentRows);

  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> columnQr(independentJacobian);
  columnQr.setThreshold(tolerance);
  if (columnQr.rank() != result.rank) {
    return result;
  }

  result.constrainedColumns = detail::firstPivots(
      columnQr.colsPermutation().indices(), result.rank, allocator);
  if (static_cast<int>(result.constrainedColumns.size()) != result.rank) {
    return result;
  }
  result.freeColumns = detail::complementIndices(
      numCols, result.constrainedColumns, allocator);

  const Eigen::MatrixXd constrainedBlock
      = detail::selectColumns(independentJacobian, result.constrainedColumns);
  const Eigen::MatrixXd freeBlock
      = detail::selectColumns(independentJacobian, result.freeColumns);

  Eigen::FullPivLU<Eigen::MatrixXd> constrainedLu(constrainedBlock);
  constrainedLu.setThreshold(tolerance);
  if (constrainedLu.rank() != result.rank) {
    return result;
  }

  const Eigen::VectorXd constrainedParticular
      = constrainedLu.solve(-independentResidual);
  const Eigen::MatrixXd constrainedCoupling = constrainedLu.solve(freeBlock);

  result.nullspaceBasis
      = Eigen::MatrixXd::Zero(numCols, result.freeColumns.size());
  for (int row = 0; row < result.rank; ++row) {
    result.particularStep[result.constrainedColumns[row]]
        = constrainedParticular[row];
    if (result.freeColumns.empty()) {
      continue;
    }
    result.nullspaceBasis.row(result.constrainedColumns[row])
        = -constrainedCoupling.row(row);
  }
  for (int col = 0; col < static_cast<int>(result.freeColumns.size()); ++col) {
    result.nullspaceBasis(result.freeColumns[col], col) = 1.0;
  }

  result.residualNorm
      = (denseJacobian * result.particularStep + equalityResidual).norm();
  result.valid = std::isfinite(result.residualNorm)
                 && detail::allFinite(result.particularStep)
                 && detail::allFinite(result.nullspaceBasis);
  return result;
}

//==============================================================================
[[nodiscard]] inline EqualityChangeOfVariableResult
makeEqualityChangeOfVariable(
    const Eigen::SparseMatrix<double>& equalityJacobian,
    const Eigen::VectorXd& equalityResidual,
    const double tolerance = 1e-12)
{
  return makeEqualityChangeOfVariable(
      equalityJacobian,
      equalityResidual,
      dart::common::MemoryAllocator::GetDefault(),
      tolerance);
}

//==============================================================================
[[nodiscard]] inline Eigen::VectorXd solveEqualityConstrainedQuadraticReduced(
    const Eigen::MatrixXd& hessian,
    const Eigen::VectorXd& gradient,
    const EqualityChangeOfVariableResult& changeOfVariable,
    const double tolerance = 1e-12)
{
  if (!changeOfVariable.valid || hessian.rows() != hessian.cols()
      || hessian.rows() != gradient.size()
      || hessian.rows() != changeOfVariable.particularStep.size()) {
    return Eigen::VectorXd::Zero(gradient.size());
  }

  const Eigen::MatrixXd& basis = changeOfVariable.nullspaceBasis;
  if (basis.cols() == 0) {
    return changeOfVariable.particularStep;
  }

  const Eigen::MatrixXd reducedHessian = basis.transpose() * hessian * basis;
  const Eigen::VectorXd reducedGradient
      = basis.transpose()
        * (hessian * changeOfVariable.particularStep + gradient);
  const Eigen::VectorXd rhs = -reducedGradient;

  Eigen::LDLT<Eigen::MatrixXd> ldlt(reducedHessian);
  if (ldlt.info() == Eigen::Success
      && ldlt.vectorD().array().abs().minCoeff() > tolerance) {
    return changeOfVariable.particularStep + basis * ldlt.solve(rhs);
  }

  Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod(reducedHessian);
  cod.setThreshold(tolerance);
  return changeOfVariable.particularStep + basis * cod.solve(rhs);
}

} // namespace dart::simulation::detail::newton_barrier
