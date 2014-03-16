## DART 3.0 to 4.0

### New Deprecations

### Modifications

### Additions

1. **dart/dynamics/Skeleton.h**
    + enum InverseKinematicsPolicy
    + void Skeleton::solveInvKinematics(BodyNode* _body, const Eigen::Isometry3d& _target, InverseKinematicsPolicy _policy, bool _jointLimit)
    + virtual void solveInvKinematicsParentJointImpl(BodyNode* _body, const Eigen::Isometry3d& _target, bool _jointLimit = true);
    + virtual void solveInvKinematicsAncestorJointsImpl(BodyNode* _body, const Eigen::Isometry3d& _target, bool _jointLimit = true); 
    + virtual void solveInvKinematicsAllJointsImpl(BodyNode* _body, const Eigen::Isometry3d& _target, bool _jointLimit = true);

1. **dart/optimizer/Function.h**

1. **dart/optimizer/Problem.h**

1. **dart/optimizer/Solver.h**

1. **dart/optimizer/nlopt/NloptSolver.h**

1. **dart/optimizer/ipopt/IpoptSolver.h**

### Deletions


