## DART 3.0 to 4.0

### Additions

1. **dart/math/Helpers.h**
  + bool isNan(double _v)
  + bool isNan(const Eigen::MatrixXd& _m)
  + bool isInf(double _v)
  + bool isInf(const Eigen::MatrixXd& _m)

1. **dart/dynamics/GenCoord.h**
  + void setConstraintForce(double _constForce)
  + double getConstraintForce() const
  + void setVelChange(double _velChange)
  + double getVelChange() const
  + void setImpulse(double _impulse)
  + double getImpulse() const
  + void integrateConfig(double _dt)
  + void integrateVel(double _dt)

1. **dart/dynamics/GenCoordSystem**
  + void setConstraintForces(const Eigen::VectorXd& _constForces)
  + Eigen::VectorXd getConstraintForces() const
  + virtual void setVelsChange(const Eigen::VectorXd& _velsChange)
  + virtual Eigen::VectorXd getVelsChange() const
  + virtual void setImpulses(const Eigen::VectorXd& _impulses)
  + virtual Eigen::VectorXd getImpulses() const
  + virtual void integrateConfigs(double _dt)
  + virtual void integrateGenVels(double _dt)

1. **dart/dynamics/BodyNode.h**
  + enum InverseKinematicsPolicy
  + class TransformObjFunc
  + class VelocityObjFunc
  + void setFrictionCoeff(double _coeff)
  + double getFrictionCoeff() const
  + void fitWorldTransform(const Eigen::Isometry3d& _target, InverseKinematicsPolicy _policy = IKP_PARENT_JOINT, bool _jointLimit = true)
  + void fitWorldLinearVel(const Eigen::Vector3d& _targetLinVel, InverseKinematicsPolicy _policy = IKP_PARENT_JOINT, bool _jointVelLimit = true)
  + void fitWorldAngularVel(const Eigen::Vector3d& _targetAngVel, InverseKinematicsPolicy _policy = IKP_PARENT_JOINT, bool _jointVelLimit = true)
  + void fitWorldTransformParentJointImpl(BodyNode* _body, const Eigen::Isometry3d& _target, bool _jointLimit = true)
  + void fitWorldTransformAncestorJointsImpl(BodyNode* _body, const Eigen::Isometry3d& _target, bool _jointLimit = true)
  + void fitWorldTransformAllJointsImpl(BodyNode* _body, const Eigen::Isometry3d& _target, bool _jointLimit = true)
  + bool isImpulseReponsible() const
  + void updateImpBiasForce()
  + void updateJointVelocityChange()
  + void updateBodyVelocityChange()
  + void updateBodyImpForceFwdDyn()
  + void setConstraintImpulse(const Eigen::Vector6d& _constImp)
  + void addConstraintImpulse(const Eigen::Vector6d& _constImp)
  + void clearConstraintImpulse()
  + const Eigen::Vector6d& getConstraintImpulse()

1. **dart/dynamics/Joint.h**
  + virtual void setConfig(size_t _idx, double _config, bool _updateTransforms = true, bool _updateVels = true, bool _updateAccs = true)
  + virtual void setConfigs(const Eigen::VectorXd& _configs, bool _updateTransforms = true, bool _updateVels = true, bool _updateAccs = true)
  + virtual void setGenVel(size_t _idx, double _genVel, bool _updateVels = true, bool _updateAccs = true)
  + virtual void setGenVels(const Eigen::VectorXd& _genVels, bool _updateVels = true, bool _updateAccs = true)
  + virtual void setGenAcc(size_t _idx, double _genAcc, bool _updateAccs = true)
  + virtual void setGenAccs(const Eigen::VectorXd& _genAccs, bool _updateAccs = true)

1. **dart/dynamics/BallJoint.h**
  + virtual void setTransformFromChildBodyNode(const Eigen::Isometry3d& _T)
  + virtual void integrateConfigs(double _dt)

1. **dart/dynamics/FreeJoint.h**
  + virtual void setTransformFromChildBodyNode(const Eigen::Isometry3d& _T)
  + virtual void integrateConfigs(double _dt)

1. **dart/dynamics/Skeleton.h**
  + virtual void setGenVels(const Eigen::VectorXd& _genVels, bool _updateVels = true, bool _updateAccs = true)
  + virtual void setGenAccs(const Eigen::VectorXd& _genAccs, bool _updateAccs = true)
  + virtual void integrateConfigs(double _dt)
  + virtual void integrateGenVels(double _dt)
  + void computeForwardKinematics(bool _updateTransforms = true, bool _updateVels = true, bool _updateAccs = true)
  + void clearImpulseTest()
  + void updateImpBiasForce(BodyNode* _bodyNode, const Eigen::Vector6d& _imp)
  + void updateVelocityChange()
  + void setImpulseApplied(bool _val)
  + bool isImpulseApplied() const
  + void computeImpulseForwardDynamics()
  + void clearConstraintImpulses()

1. **dart/collision/CollisionDetector**
  + virtual void addSkeleton(dynamics::Skeleton* _skeleton)
  + virtual void removeSkeleton(dynamics::Skeleton* _skeleton)
  + virtual void removeAllSkeletons()

1. **dart/simulation/World.h**
  + virtual void integrateConfigs(const Eigen::VectorXd& _genVels, double _dt)
  + virtual void integrateGenVels(const Eigen::VectorXd& _genAccs, double _dt)

1. **dart/integration/Integrator.h**
  + virtual void integrateConfigs(const Eigen::VectorXd& _genVels, double _dt) = 0
  + virtual void integrateGenVels(const Eigen::VectorXd& _genAccs, double _dt) = 0

1. **dart/utils/SkelParser.h**
  + static dynamics::Skeleton* readSkeleton(const std::string& _filename)

1. **dart/utils/SoftSkelParser.h**
  + static dynamics::SoftSkeleton* readSoftSkeleton(const std::string& _filename)

### Deletions

1. **dart/dynamics/BodyNode.h**
  + virtual void updateTransform_Issue122(double _timeStep)
  + virtual void updateEta_Issue122()

1. **dart/dynamics/Joint.h**
  + virtual void updateTransform_Issue122(double _timeStep)
  + virtual void updateEta_Issue122()
  + virtual void updateJacobianTimeDeriv_Issue122()

1. **dart/dynamics/Skeleton.h**
  + Eigen::VectorXd getConfig() const

1. **dart/math/Geometry.h**
  + bool isNan(const Eigen::MatrixXd& _m)
    + Note: moved to dart/math/Helpers.h

### Modifications

1. **dart/dynamics/BodyNode.h**
  + ***From:*** int getNumContactForces() const
    + ***To:*** int getNumContacts() const

1. **dart/dynamics/Joint.h**
  + ***From:*** void setTransformFromParentBodyNode(const Eigen::Isometry3d& _T)
    + ***To:*** virtual void setTransformFromParentBodyNode(const Eigen::Isometry3d& _T)

1. **dart/dynamics/Skeleton.h**
  + ***From:*** void setConfig(const std::vector<int>& _id, const Eigen::VectorXd& _config)
    + ***To:*** void setConfigSegs(const std::vector<int>& _id, const Eigen::VectorXd& _configs, bool _updateTransforms = true, bool _updateVels = true, bool _updateAccs = true)
  + ***From:*** Eigen::VectorXd getConfig(const std::vector<int>& _id) const
    + ***To:*** Eigen::VectorXd getConfigSegs(const std::vector<int>& _id) const
  + ***From:*** void setConfig(const Eigen::VectorXd& _config)
    + ***To:*** virtual void setConfigs(const Eigen::VectorXd& _configs, bool _updateTransforms = true, bool _updateVels = true, bool _updateAccs = true)
  + ***From:*** void setState(const Eigen::VectorXd& _state)
    + ***To:*** void setState(const Eigen::VectorXd& _state, bool _updateTransforms = true, bool _updateVels = true, bool _updateAccs = true)
  + ***From:*** Eigen::VectorXd getState()
    + ***To:*** Eigen::VectorXd getState() const

1. **dart/simulation/World.h**
  + ***From:*** virtual void setState(const Eigen::VectorXd &_newState)
    + ***To:*** virtual void setConfigs(const Eigen::VectorXd& _configs)
    + ***To:*** virtual void setGenVels(const Eigen::VectorXd& _genVels)
  + ***From:*** virtual Eigen::VectorXd getState() const
    + ***To:*** virtual Eigen::VectorXd getConfigs() const
    + ***To:*** virtual Eigen::VectorXd getGenVels() const
  + ***From:*** virtual Eigen::VectorXd evalDeriv()
    + ***To:*** virtual Eigen::VectorXd evalGenAccs()

1. **dart/integration/Integrator.h**
  + ***From:*** virtual void setState(const Eigen::VectorXd &_newState) = 0
    + ***To:*** virtual void setConfigs(const Eigen::VectorXd& _configs) = 0
    + ***To:*** virtual void setGenVels(const Eigen::VectorXd& _genVels) = 0
  + ***From:*** virtual Eigen::VectorXd getState() const = 0
    + ***To:*** virtual Eigen::VectorXd getConfigs() const = 0
    + ***To:*** virtual Eigen::VectorXd getGenVels() const = 0
  + ***From:*** virtual Eigen::VectorXd evalDeriv() = 0
    + ***To:*** virtual Eigen::VectorXd evalGenAccs() = 0
  + ***From:*** virtual void integrate(IntegrableSystem* system, double dt) const = 0
    + ***To:*** virtual void integrate(IntegrableSystem* system, double dt) = 0

1. **dart/integration/EulerIntegrator.h**
  + ***From:*** virtual void integrate(IntegrableSystem* system, double dt) const = 0
    + ***To:*** virtual void integrate(IntegrableSystem* system, double dt) = 0

1. **dart/integration/RK4Integrator.h**
  + ***From:*** virtual void integrate(IntegrableSystem* system, double dt) const = 0
    + ***To:*** virtual void integrate(IntegrableSystem* system, double dt) = 0

### New Deprecations


