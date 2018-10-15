#ifndef DARTPY_SKELETON_H_
#define DARTPY_SKELETON_H_
#include <boost/python.hpp>
#include <dart/dynamics/dynamics.hpp>

namespace dart {
namespace python {

boost::python::object Skeleton_createJointAndBodyNodePair(
  dart::dynamics::Skeleton* _skeleton,
  boost::python::object _jointType,
  boost::python::object _bodyType,
  dart::dynamics::BodyNode* _parent,
  boost::python::object _jointPropertiesPython,
  boost::python::object _bodyPropertiesPython);

} // namespace python
} // namespace dart

#endif // ifndef DARTPY_SKELETON_H_
