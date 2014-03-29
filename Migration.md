## DART 3.0 to 4.0

### Additions

1. **dart/math/Helpers.h**
  + bool isNan(double _v)
  + bool isNan(const Eigen::MatrixXd& _m)
  + bool isInf(double _v)
  + bool isInf(const Eigen::MatrixXd& _m)

1. **dart/dynamics/BodyNode.h**
  + enum InverseKinematicsPolicy
  + class TransformObjFunc
  + class VelocityObjFunc
  + void fitWorldTransform(const Eigen::Isometry3d& _target, InverseKinematicsPolicy _policy = IKP_PARENT_JOINT, bool _jointLimit = true)
  + void fitWorldLinearVel(const Eigen::Vector3d& _targetLinVel, InverseKinematicsPolicy _policy = IKP_PARENT_JOINT, bool _jointVelLimit = true)
  + void fitWorldAngularVel(const Eigen::Vector3d& _targetAngVel, InverseKinematicsPolicy _policy = IKP_PARENT_JOINT, bool _jointVelLimit = true)
  + void fitWorldTransformParentJointImpl(BodyNode* _body, const Eigen::Isometry3d& _target, bool _jointLimit = true)
  + void fitWorldTransformAncestorJointsImpl(BodyNode* _body, const Eigen::Isometry3d& _target, bool _jointLimit = true)
  + void fitWorldTransformAllJointsImpl(BodyNode* _body, const Eigen::Isometry3d& _target, bool _jointLimit = true)

1. **dart/dynamics/Joint.h**
  + virtual void setConfig(size_t _idx, double _config, bool _updateTransforms = true, bool _updateVels = true, bool _updateAccs = true)
  + virtual void setConfigs(const Eigen::VectorXd& _configs, bool _updateTransforms = true, bool _updateVels = true, bool _updateAccs = true)
  + virtual void setGenVel(size_t _idx, double _genVel, bool _updateVels = true, bool _updateAccs = true)
  + virtual void setGenVels(const Eigen::VectorXd& _genVels, bool _updateVels = true, bool _updateAccs = true)
  + virtual void setGenAcc(size_t _idx, double _genAcc, bool _updateAccs = true)
  + virtual void setGenAccs(const Eigen::VectorXd& _genAccs, bool _updateAccs = true)

1. **dart/dynamics/Skeleton.h**
  + virtual void setGenVels(const Eigen::VectorXd& _genVels, bool _updateVels = true, bool _updateAccs = true)
  + virtual void setGenAccs(const Eigen::VectorXd& _genAccs, bool _updateAccs = true)
  + void computeForwardKinematics(bool _updateTransforms = true, bool _updateVels = true, bool _updateAccs = true)

1. **dart/utils/SkelParser.h**
  + static dynamics::Skeleton* readSkeleton(const std::string& _filename)

1. **dart/utils/SoftSkelParser.h**
  + static dynamics::SoftSkeleton* readSoftSkeleton(const std::string& _filename)

### Deletions

1. **dart/dynamics/Skeleton.h**
  + Eigen::VectorXd getConfig() const

1. **dart/math/Geometry.h**
  + bool isNan(const Eigen::MatrixXd& _m)
    + Note: moved to dart/math/Helpers.h

### Modifications

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

1. **dart/dynamics/BodyNode.h**

  + ***From:*** int getNumContactForces() const
  + ***To:*** int getNumContacts() const

### New Deprecations


