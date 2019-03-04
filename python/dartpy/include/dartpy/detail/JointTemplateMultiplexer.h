#ifndef DARTPY_DETAIL_JOINTTEMPLATEMULTIPLEXER_H_
#define DARTPY_DETAIL_JOINTTEMPLATEMULTIPLEXER_H_
#include <dartpy/pointers.h>
#include <dart/dynamics/dynamics.hpp>

namespace dart {
namespace python {
namespace detail {

//==============================================================================
template <class JointType>
struct BodyNode_moveTo2_factory
{
  static dart::dynamics::Joint* execute(
    dart::dynamics::BodyNode* _bodyNode,
    dart::dynamics::BodyNode* _newParent,
    boost::python::object _jointPropertiesPython)
  {
    typename JointType::Properties jointProperties;

    if (!_jointPropertiesPython.is_none())
    {
      jointProperties = boost::python::extract<typename JointType::Properties>(
        _jointPropertiesPython);
    }

    return _bodyNode->moveTo<JointType>(_newParent);
  }
};

//==============================================================================
template <class JointType>
struct BodyNode_moveTo3_factory
{
  static dart::dynamics::Joint* execute(
    dart::dynamics::BodyNode* _bodyNode,
    const dart::dynamics::SkeletonPtr &_newSkeleton,
    dart::dynamics::BodyNode* _newParent,
    boost::python::object _jointPropertiesPython)
  {
    typename JointType::Properties jointProperties;

    if (!_jointPropertiesPython.is_none())
    {
      jointProperties = boost::python::extract<typename JointType::Properties>(
        _jointPropertiesPython);
    }

    return _bodyNode->moveTo<JointType>(
      _newSkeleton, _newParent, jointProperties);
  }
};

//==============================================================================
template <class JointType>
struct BodyNode_copyTo3_factory
{
  static boost::python::object execute(
    dart::dynamics::BodyNode* _bodyNode, 
    dart::dynamics::BodyNode* _newParent,
    boost::python::object _jointPropertiesPython,
    bool _recursive)
  {
    typename JointType::Properties jointProperties;

    if (!_jointPropertiesPython.is_none())
    {
      jointProperties = boost::python::extract<typename JointType::Properties>(
        _jointPropertiesPython);
    }

    const auto ret = _bodyNode->copyTo<JointType>(_newParent, jointProperties);
    return boost::python::make_tuple(ret.first, ret.second);
  }
};

//==============================================================================
template <class JointType>
struct BodyNode_split_factory
{
  static dart::dynamics::SkeletonPtr execute(
    dart::dynamics::BodyNode* _bodyNode, 
    const std::string& _skeletonName,
    boost::python::object _jointPropertiesPython)
  {
    typename JointType::Properties jointProperties;

    if (!_jointPropertiesPython.is_none())
    {
      jointProperties = boost::python::extract<typename JointType::Properties>(
        _jointPropertiesPython);
    }

    return _bodyNode->split<JointType>(_skeletonName, jointProperties);
  }
};

//==============================================================================
template <class JointType>
struct BodyNode_changeParentJointType_factory
{
  static dart::dynamics::JointPtr execute(
    dart::dynamics::BodyNode* _bodyNode, 
    boost::python::object _jointPropertiesPython)
  {
    typename JointType::Properties jointProperties;

    if (!_jointPropertiesPython.is_none())
    {
      jointProperties = boost::python::extract<typename JointType::Properties>(
        _jointPropertiesPython);
    }

    return _bodyNode->changeParentJointType<JointType>(jointProperties);
  }
};

} // namespace detail
} // namespace python
} // namespace dart

#endif // ifndef DARTPY_DETAIL_JOINTTEMPLATEMULTIPLEXER_H_
