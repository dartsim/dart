#ifndef DARTPY_BODYNODE_H_
#define DARTPY_BODYNODE_H_
#include <boost/python.hpp>
#include <dart/dynamics/dynamics.hpp>

namespace dart {
namespace python {

dart::dynamics::Joint* BodyNode_moveTo2(
  dart::dynamics::BodyNode* _bodyNode,
  boost::python::object _jointType,
  dart::dynamics::BodyNode* _newParent,
  boost::python::object _jointProperties);

} // namespace python
} // namespace dart

#endif // ifndef DARTPY_BODYNODE_H_
