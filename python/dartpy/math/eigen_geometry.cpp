#include "math/eigen_geometry.hpp"

#include "common/eigen_utils.hpp"

#include <Eigen/Dense>
#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>

namespace nb = nanobind;

namespace dart::python_nb {

namespace {

constexpr double kCheckTolerance = 1e-5;

template <typename T>
void checkRotation(const Eigen::Matrix<T, 3, 3>& R)
{
  const T identity_error
      = (R * R.transpose() - Eigen::Matrix<T, 3, 3>::Identity())
            .array()
            .abs()
            .maxCoeff();
  if (identity_error >= kCheckTolerance) {
    throw std::runtime_error("Rotation matrix is not orthonormal");
  }
  const T det_error = std::abs(R.determinant() - 1);
  if (det_error >= kCheckTolerance) {
    throw std::runtime_error("Rotation matrix violates right-hand rule");
  }
}

template <typename T>
void checkIsometry(const Eigen::Transform<T, 3, Eigen::Isometry>& X)
{
  Eigen::Matrix<T, 3, 3> rotation = X.linear();
  checkRotation(rotation);
  Eigen::Matrix<T, 1, 4> bottom_expected;
  bottom_expected << 0, 0, 0, 1;
  const T bottom_error
      = (X.matrix().bottomRows(1) - bottom_expected).array().abs().maxCoeff();
  if (bottom_error >= kCheckTolerance) {
    throw std::runtime_error("Homogeneous matrix is improperly scaled.");
  }
}

template <typename T>
void checkQuaternion(const Eigen::Quaternion<T>& q)
{
  const T norm_error = std::abs(q.coeffs().norm() - 1);
  if (norm_error >= kCheckTolerance) {
    throw std::runtime_error("Quaternion is not normalized");
  }
}

template <typename T>
void checkAngleAxis(const Eigen::AngleAxis<T>& v)
{
  const T norm_error = std::abs(v.axis().norm() - 1);
  if (norm_error >= kCheckTolerance) {
    throw std::runtime_error("Axis is not normalized");
  }
}

} // namespace

void defEigenGeometry(nb::module_& m)
{
  using Isometry = Eigen::Transform<double, 3, Eigen::Isometry>;
  nb::class_<Isometry>(m, "Isometry3")
      .def(nb::new_([]() { return Isometry::Identity(); }))
      .def_static("Identity", []() { return Isometry::Identity(); })
      .def(
          nb::new_([](const Eigen::Matrix4d& matrix) {
            Isometry out(matrix);
            checkIsometry(out);
            return out;
          }),
          nb::arg("matrix"))
      .def(
          nb::new_([](const Eigen::Matrix3d& rotation,
                      const Eigen::Vector3d& translation) {
            checkRotation(rotation);
            Isometry out = Isometry::Identity();
            out.linear() = rotation;
            out.translation() = translation;
            return out;
          }),
          nb::arg("rotation"),
          nb::arg("translation"))
      .def(
          nb::new_([](const Eigen::Quaterniond& q,
                      const Eigen::Vector3d& translation) {
            checkQuaternion(q);
            Isometry out = Isometry::Identity();
            out.linear() = q.toRotationMatrix();
            out.translation() = translation;
            return out;
          }),
          nb::arg("rotation"),
          nb::arg("translation"))
      .def("matrix", [](const Isometry& self) { return self.matrix(); })
      .def("set_identity", [](Isometry& self) { self.setIdentity(); })
      .def(
          "set_matrix",
          [](Isometry& self, const Eigen::Matrix4d& matrix) {
            Isometry update(matrix);
            checkIsometry(update);
            self = update;
          },
          nb::arg("matrix"))
      .def(
          "translation",
          [](const Isometry& self) {
            return Eigen::Vector3d(self.translation());
          })
      .def(
          "set_translation",
          [](Isometry& self, const nb::handle& translation) {
            self.translation() = toVector3(translation);
          },
          nb::arg("translation"))
      .def(
          "rotation",
          [](const Isometry& self) { return Eigen::Matrix3d(self.linear()); })
      .def(
          "set_rotation",
          [](Isometry& self, const Eigen::Matrix3d& rotation) {
            checkRotation(rotation);
            self.linear() = rotation;
          },
          nb::arg("rotation"))
      .def(
          "quaternion",
          [](const Isometry& self) {
            return Eigen::Quaterniond(self.linear());
          })
      .def(
          "set_quaternion",
          [](Isometry& self, const Eigen::Quaterniond& q) {
            checkQuaternion(q);
            self.linear() = q.toRotationMatrix();
          },
          nb::arg("quaternion"))
      .def(
          "__str__",
          [](nb::handle self) { return nb::str(self.attr("matrix")()); })
      .def(
          "multiply",
          [](const Isometry& self, const Isometry& other) {
            return self * other;
          },
          nb::arg("other"))
      .def(
          "multiply",
          [](const Isometry& self, const nb::handle& position) {
            return self * toVector3(position);
          },
          nb::arg("position"))
      .def("inverse", [](const Isometry& self) { return self.inverse(); })
      .def(
          "translate",
          [](Isometry& self, const nb::handle& shift) {
            self.translate(toVector3(shift));
          },
          nb::arg("shift"))
      .def(
          "pretranslate",
          [](Isometry& self, const nb::handle& shift) {
            self.pretranslate(toVector3(shift));
          },
          nb::arg("shift"));

  nb::implicitly_convertible<Eigen::Matrix4d, Isometry>();

  using Quaternion = Eigen::Quaterniond;
  nb::class_<Quaternion>(m, "Quaternion")
      .def(nb::new_([]() { return Quaternion::Identity(); }))
      .def_static("Identity", []() { return Quaternion::Identity(); })
      .def(
          nb::new_([](const Eigen::Vector4d& wxyz) {
            Quaternion out(wxyz(0), wxyz(1), wxyz(2), wxyz(3));
            checkQuaternion(out);
            return out;
          }),
          nb::arg("wxyz"))
      .def(
          nb::new_([](double w, double x, double y, double z) {
            Quaternion out(w, x, y, z);
            checkQuaternion(out);
            return out;
          }),
          nb::arg("w"),
          nb::arg("x"),
          nb::arg("y"),
          nb::arg("z"))
      .def(
          nb::new_([](const Eigen::Matrix3d& rotation) {
            Quaternion out(rotation);
            checkQuaternion(out);
            return out;
          }),
          nb::arg("rotation"))
      .def("w", [](const Quaternion& self) { return self.w(); })
      .def("x", [](const Quaternion& self) { return self.x(); })
      .def("y", [](const Quaternion& self) { return self.y(); })
      .def("z", [](const Quaternion& self) { return self.z(); })
      .def("xyz", [](const Quaternion& self) { return self.vec(); })
      .def(
          "wxyz",
          [](const Quaternion& self) {
            Eigen::Vector4d out;
            out << self.w(), self.vec();
            return out;
          })
      .def(
          "set_wxyz",
          [](Quaternion& self, const Eigen::Vector4d& wxyz) {
            Quaternion update(wxyz(0), wxyz(1), wxyz(2), wxyz(3));
            checkQuaternion(update);
            self = update;
          },
          nb::arg("wxyz"))
      .def(
          "rotation",
          [](const Quaternion& self) { return self.toRotationMatrix(); })
      .def(
          "set_rotation",
          [](Quaternion& self, const Eigen::Matrix3d& rotation) {
            Quaternion update(rotation);
            checkQuaternion(update);
            self = update;
          },
          nb::arg("rotation"))
      .def(
          "__str__",
          [](const Quaternion& self) {
            return nb::str("Quaternion(w={}, x={}, y={}, z={})")
                .format(self.w(), self.x(), self.y(), self.z());
          })
      .def(
          "multiply",
          [](const Quaternion& self, const Quaternion& other) {
            return self * other;
          },
          nb::arg("other"))
      .def(
          "multiply",
          [](const Quaternion& self, const Eigen::Vector3d& position) {
            return self * position;
          },
          nb::arg("position"))
      .def("inverse", [](const Quaternion& self) { return self.inverse(); })
      .def("conjugate", [](const Quaternion& self) { return self.conjugate(); })
      .def("to_rotation_matrix", [](const Quaternion& self) {
        return self.toRotationMatrix();
      });

  using AngleAxis = Eigen::AngleAxisd;
  nb::class_<AngleAxis>(m, "AngleAxis")
      .def(nb::new_([]() { return AngleAxis::Identity(); }))
      .def_static("Identity", []() { return AngleAxis::Identity(); })
      .def(
          nb::new_([](double angle, const Eigen::Vector3d& axis) {
            AngleAxis out(angle, axis);
            checkAngleAxis(out);
            return out;
          }),
          nb::arg("angle"),
          nb::arg("axis"))
      .def(
          nb::new_([](double angle, const nb::handle& axis) {
            Eigen::VectorXd vec = toVector(axis);
            if (vec.size() != 3) {
              throw nb::value_error("AngleAxis axis must have length 3");
            }
            AngleAxis out(angle, Eigen::Vector3d(vec));
            checkAngleAxis(out);
            return out;
          }),
          nb::arg("angle"),
          nb::arg("axis"))
      .def(
          nb::new_([](const Quaternion& q) {
            AngleAxis out(q);
            checkAngleAxis(out);
            return out;
          }),
          nb::arg("quaternion"))
      .def("angle", [](const AngleAxis& self) { return self.angle(); })
      .def("axis", [](const AngleAxis& self) { return self.axis(); })
      .def("to_rotation_matrix", [](const AngleAxis& self) {
        return self.toRotationMatrix();
      });
}

} // namespace dart::python_nb
