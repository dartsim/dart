/*
  RTQL8, Copyright (c) 2011 Georgia Tech Graphics Lab
  All rights reserved.

  Author	Sehoon Ha
  Date		06/12/2011
*/

#ifndef UTILS_EIGEN_HELPER_H
#define UTILS_EIGEN_HELPER_H

#include <Eigen/Dense>

// Important Note: I assume column-major (Sehoon Ha)

namespace eigenhelper {
    template <typename Derived>
        Eigen::Block<Derived> sub(const Eigen::DenseBase<Derived>& b, int rows, int cols) {
        return const_cast< Eigen::DenseBase<Derived>& >(b).topLeftCorner(rows, cols);
    }

    template <typename Derived>
        Eigen::Block<Derived> sub(const Eigen::DenseBase<Derived>& b,
                           int top, int left, int height, int width) {
        return const_cast< Eigen::DenseBase<Derived>& >(b).block(top, left, height, width);
    }


    template <typename Derived>
        Eigen::Block<Derived, Eigen::internal::traits<Derived>::RowsAtCompileTime, 1, true> col(
            const Eigen::DenseBase<Derived>& b, int col_index) {
        return const_cast< Eigen::DenseBase<Derived>& >(b).col(col_index);
    }
    
    template <typename Derived>
        Eigen::Block<Derived, 1, Eigen::internal::traits<Derived>::ColsAtCompileTime, false> row(
            const Eigen::DenseBase<Derived>& b, int col_index) {
        return const_cast< Eigen::DenseBase<Derived>& >(b).col(col_index);
    }


    template <typename Derived>
        Eigen::Transpose<Derived> trans(const Eigen::DenseBase<Derived>& b) {
        return const_cast< Eigen::DenseBase<Derived>& >(b).transpose();
    }

    template <typename Derived>
        Eigen::VectorBlock<Derived> first(const Eigen::DenseBase<Derived>& v, int n_elts) {
        return const_cast< Eigen::DenseBase<Derived>& >(v).head(n_elts);
    }

    template <typename Derived>
        Eigen::VectorBlock<Derived> last(const Eigen::DenseBase<Derived>& v, int n_elts) {
        return const_cast< Eigen::DenseBase<Derived>& >(v).tail(n_elts);
    }

    Eigen::Vector3d xform(const Eigen::Matrix4d& m, const Eigen::Vector3d& v);
  
} // namespace eigenhelper

#endif // #ifndef UTILS_EIGEN_HELPER_H

