#include <dartpy/template_registry.h>
#include <dartpy/Skeleton.h>

namespace dart {
namespace python {

//==============================================================================
boost::python::object Skeleton_createJointAndBodyNodePair(
  dart::dynamics::Skeleton* _skeleton,
  boost::python::object _jointType,
  boost::python::object _bodyType,
  dart::dynamics::BodyNode* _parent,
  boost::python::object _jointPropertiesPython,
  boost::python::object _bodyPropertiesPython)
{
  return JointAndNodeTemplateRegistry
    ::mSkeleton_createJointAndBodyNodePair_registry
    [make_array(_jointType, _bodyType)]
    (_skeleton, _parent, _jointPropertiesPython, _bodyPropertiesPython);
}

} // namespace python
} // namespace dart
