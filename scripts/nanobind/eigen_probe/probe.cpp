// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on
#include "detail/eigen.hpp"

NB_MODULE(numpy_export_probe, m)
{
  m.def("fast_export", [] { return dart_numpy_capi::numpy2_export() == 1; });
  m.def("vector", [](int size) {
    if (size < 0)
      throw nb::value_error("size must be non-negative");
    Eigen::VectorXd result(size);
    for (int i = 0; i < size; ++i)
      result[i] = i + 1;
    return result;
  });
  m.def("matrix", [](int rows, int cols) {
    if (rows < 0 || cols < 0)
      throw nb::value_error("dimensions must be non-negative");
    Eigen::MatrixXd result(rows, cols);
    for (int row = 0; row < rows; ++row)
      for (int col = 0; col < cols; ++col)
        result(row, col) = row * cols + col + 1;
    return result;
  });
  m.def(
      "mutate", [](Eigen::Ref<Eigen::VectorXd> value) { value.array() += 10; });
}
