#include <dartpy/template_registry.h>

namespace dart {
namespace python {

//==============================================================================
JointTemplateRegistry::RegistryType<
    detail::BodyNode_moveTo2_factory>
  JointTemplateRegistry::mBodyNode_moveTo2_registry;

//==============================================================================
JointTemplateRegistry::RegistryType<
    detail::BodyNode_moveTo3_factory>
  JointTemplateRegistry::mBodyNode_moveTo3_registry;

//==============================================================================
JointTemplateRegistry::RegistryType<
    detail::BodyNode_copyTo3_factory>
  JointTemplateRegistry::mBodyNode_copyTo3_registry;

//==============================================================================
JointAndNodeTemplateRegistry::RegistryType<
    detail::Skeleton_createJointAndBodyNodePair>
  JointAndNodeTemplateRegistry::mSkeleton_createJointAndBodyNodePair_registry;

} // namespace python
} // namespace dart
