#include <dartpy/pointers.h>
#include <dartpy/template_registry.h>
#include <dartpy/BodyNode.h>

namespace dart {
namespace python {

//==============================================================================
dart::dynamics::Joint* BodyNode_moveTo2(
  dart::dynamics::BodyNode* _bodyNode,
  boost::python::object _jointType,
  dart::dynamics::BodyNode* _newParent,
  boost::python::object _jointProperties)
{
  return JointTemplateRegistry::mBodyNode_moveTo2_registry
    [make_array(_jointType)]
    (_bodyNode, _newParent, _jointProperties);
}

//==============================================================================
dart::dynamics::Joint* BodyNode_moveTo3(
  dart::dynamics::BodyNode* _bodyNode,
  boost::python::object _jointType,
  const dart::dynamics::SkeletonPtr& _newSkeleton,
  dart::dynamics::BodyNode* _newParent,
  boost::python::object _jointProperties)
{
  return JointTemplateRegistry::mBodyNode_moveTo3_registry
    [make_array(_jointType)]
    (_bodyNode, _newSkeleton, _newParent, _jointProperties);
}


//==============================================================================
boost::python::object BodyNode_copyTo3(
  boost::python::object _jointType,
  dart::dynamics::BodyNode* _bodyNode, 
  dart::dynamics::BodyNode* _newParent,
  boost::python::object _jointProperties,
  bool _recursive)
{
  return JointTemplateRegistry::mBodyNode_copyTo3_registry
    [make_array(_jointType)]
    (_bodyNode, _newParent, _jointProperties, _recursive);
}

} // namespace python
} // namespace dart
