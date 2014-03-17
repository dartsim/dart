## DART 3.0 to 4.0

### Additions

1. **dart/dynamics/BodyNode.h**
  * enum InverseKinematicsPolicy
  * class TransformObjFunc
  * class VelocityObjFunc
  * void fitWorldTransform(const Eigen::Isometry3d& _target, InverseKinematicsPolicy _policy = IKP_PARENT_JOINT, bool _jointLimit = true)
  * void fitWorldLinearVel(const Eigen::Vector3d& _targetLinVel, InverseKinematicsPolicy _policy = IKP_PARENT_JOINT, bool _jointVelLimit = true)
  * void fitWorldAngularVel(const Eigen::Vector3d& _targetAngVel, InverseKinematicsPolicy _policy = IKP_PARENT_JOINT, bool _jointVelLimit = true)
  * void fitWorldTransformParentJointImpl(BodyNode* _body, const Eigen::Isometry3d& _target, bool _jointLimit = true)
  * void fitWorldTransformAncestorJointsImpl(BodyNode* _body, const Eigen::Isometry3d& _target, bool _jointLimit = true)
  * void fitWorldTransformAllJointsImpl(BodyNode* _body, const Eigen::Isometry3d& _target, bool _jointLimit = true)

### Deletions
### Modifications
### New Deprecations
