#ifndef SRC_UTILS_EIGEN_HELPER_H
#define SRC_UTILS_EIGEN_HELPER_H

#include <Eigen/Dense>
using namespace Eigen;

// Important Note: I assume column-major

namespace eigenhelper {
  template <typename Derived>
    Block<Derived> sub(const DenseBase<Derived>& b, int rows, int cols) {
    return const_cast< DenseBase<Derived>& >(b).topLeftCorner(rows, cols);
  }

  template <typename Derived>
    Block<Derived> sub(const DenseBase<Derived>& b,
                       int top, int left, int height, int width) {
    return const_cast< DenseBase<Derived>& >(b).block(top, left, height, width);
  }


  template <typename Derived>
    Block<Derived, internal::traits<Derived>::RowsAtCompileTime, 1, true> col(
      const DenseBase<Derived>& b, int col_index) {
    return const_cast< DenseBase<Derived>& >(b).col(col_index);
  }
    
  template <typename Derived>
    Block<Derived, 1, internal::traits<Derived>::ColsAtCompileTime, false> row(
      const DenseBase<Derived>& b, int col_index) {
    return const_cast< DenseBase<Derived>& >(b).col(col_index);
  }


  template <typename Derived>
    Transpose<Derived> trans(const DenseBase<Derived>& b) {
    return const_cast< DenseBase<Derived>& >(b).transpose();
  }

  template <typename Derived>
    VectorBlock<Derived> first(const DenseBase<Derived>& v, int n_elts) {
    return const_cast< DenseBase<Derived>& >(v).head(n_elts);
  }

  template <typename Derived>
    VectorBlock<Derived> last(const DenseBase<Derived>& v, int n_elts) {
    return const_cast< DenseBase<Derived>& >(v).tail(n_elts);
  }

  Vector3d xform(const Matrix4d& m, const Vector3d& v);
  
} // namespace eigenhelper

#endif
