#ifndef DARTPY_DETAIL_JOINTANDNODEREGISTRY_H_
#define DARTPY_DETAIL_JOINTANDNODEREGISTRY_H_
#include <dartpy/pointers.h>
#include <dart/dynamics/dynamics.hpp>

using dart::dynamics::BodyNode;
using dart::dynamics::BodyNodePtr;
using dart::dynamics::Joint;
using dart::dynamics::JointPtr;
using dart::dynamics::Skeleton;
using dart::dynamics::TemplateBodyNodePtr;


namespace dart {
namespace python {
namespace detail {

//==============================================================================
template <class JointType, class NodeType>
struct Skeleton_createJointAndBodyNodePair
{
  static boost::python::object execute(
    Skeleton* _skeleton, BodyNode* _parent,
    boost::python::object _jointPropertiesPython,
    boost::python::object _bodyPropertiesPython)
  {
    typename JointType::Properties jointProperties;
    if (!_jointPropertiesPython.is_none())
    {
      jointProperties = boost::python::extract<typename JointType::Properties>(
        _jointPropertiesPython);
    }

    typename NodeType::Properties bodyProperties;
    if (!_bodyPropertiesPython.is_none())
    {
      // Workaround for the ambiguous overload error of operator=
      const typename NodeType::Properties tmp
        = boost::python::extract<typename NodeType::Properties>(
          _bodyPropertiesPython);
      bodyProperties = std::move(tmp);
    }

    auto ret = _skeleton->createJointAndBodyNodePair<JointType, NodeType>(
      _parent, jointProperties, bodyProperties);

    return boost::python::make_tuple(
      JointPtr(ret.first), TemplateBodyNodePtr<NodeType>(ret.second));
  }
};

} // namespace detail
} // namespace python
} // namespace dart

#endif // ifndef DARTPY_DETAIL_JOINTANDNODEREGISTRY_H_
