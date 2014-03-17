## DART 3.0 to 4.0

### New Deprecations

### Modifications

### Additions

1. **dart/dynamics/BodyNode.h**
    + enum InverseKinematicsPolicy
    + class TransformObjFunc
    + class VelocityObjFunc
    + void fitWorldTransform(const Eigen::Isometry3d& _target, InverseKinematicsPolicy _policy = IKP_PARENT_JOINT, bool _jointLimit = true)
    + void fitWorldLinearVel(const Eigen::Vector3d& _targetLinVel, InverseKinematicsPolicy _policy = IKP_PARENT_JOINT, bool _jointVelLimit = true)
    + void fitWorldAngularVel(const Eigen::Vector3d& _targetAngVel, InverseKinematicsPolicy _policy = IKP_PARENT_JOINT, bool _jointVelLimit = true)
    + void fitWorldTransformParentJointImpl(BodyNode* _body, const Eigen::Isometry3d& _target, bool _jointLimit = true);
    + void fitWorldTransformAncestorJointsImpl(BodyNode* _body, const Eigen::Isometry3d& _target, bool _jointLimit = true); 
    + void fitWorldTransformAllJointsImpl(BodyNode* _body, const Eigen::Isometry3d& _target, bool _jointLimit = true);

1. **dart/optimizer/Function.h**

1. **dart/optimizer/Problem.h**

1. **dart/optimizer/Solver.h**

1. **dart/optimizer/nlopt/NloptSolver.h**

### Deletions


