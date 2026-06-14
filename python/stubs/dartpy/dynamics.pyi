from __future__ import annotations

__all__: list[str] = [
    "ACCELERATION",
    "ActuatorType",
    "BallJoint",
    "BodyNode",
    "BoxShape",
    "CapsuleShape",
    "Chain",
    "ChainCriteria",
    "CollisionAspect",
    "ConeShape",
    "Coupler",
    "CylinderShape",
    "DefaultActuatorType",
    "DegreeOfFreedom",
    "Detachable",
    "DynamicsAspect",
    "EllipsoidShape",
    "EndEffector",
    "Entity",
    "EulerJoint",
    "FORCE",
    "Frame",
    "FreeJoint",
    "FreeJointProperties",
    "HeightmapShape",
    "HierarchicalIK",
    "Inertia",
    "InverseKinematics",
    "InverseKinematicsErrorMethod",
    "InverseKinematicsGradientMethod",
    "JacobianNode",
    "Joint",
    "LOCKED",
    "LineSegmentShape",
    "Linkage",
    "LinkageCriteria",
    "MIMIC",
    "MeshShape",
    "MetaSkeleton",
    "MimicConstraintType",
    "MimicDofProperties",
    "Motor",
    "PASSIVE",
    "PlanarJoint",
    "PlaneShape",
    "PointCloudShape",
    "PrismaticJoint",
    "PrismaticJointProperties",
    "PyramidShape",
    "RevoluteJoint",
    "RevoluteJointProperties",
    "SERVO",
    "ScrewJoint",
    "ScrewJointProperties",
    "Shape",
    "ShapeFrame",
    "ShapeNode",
    "SimpleFrame",
    "Skeleton",
    "SphereShape",
    "Support",
    "TranslationalJoint",
    "TranslationalJoint2D",
    "TranslationalJoint2DProperties",
    "TranslationalJointProperties",
    "UniversalJoint",
    "UniversalJointProperties",
    "VELOCITY",
    "VisualAspect",
    "WeldJoint",
    "WholeBodyIK",
    "ZeroDofJoint",
    "ZeroDofJointProperties",
    "createSimpleFrame",
    "create_simple_frame",
]


from collections.abc import Sequence
import enum
from typing import Annotated, Any, overload

import numpy
from numpy.typing import NDArray

import dartpy.common
import dartpy.math


class Entity:
    def setName(*args, **kwargs) -> Any: ...

    def getName(*args, **kwargs) -> Any: ...

    def getParentFrame(*args, **kwargs) -> Any: ...

    def descendsFrom(*args, **kwargs) -> Any: ...

    def isFrame(*args, **kwargs) -> Any: ...

    def isQuiet(*args, **kwargs) -> Any: ...

    descends_from = descendsFrom

    get_name = getName

    get_parent_frame = getParentFrame

    is_frame = isFrame

    is_quiet = isQuiet

    set_name = setName

class Detachable(Entity):
    def setParentFrame(*args, **kwargs) -> Any: ...

    descends_from = descendsFrom

    def descendsFrom(*args, **kwargs) -> Any: ...

    get_name = getName

    def getName(*args, **kwargs) -> Any: ...

    get_parent_frame = getParentFrame

    def getParentFrame(*args, **kwargs) -> Any: ...

    is_frame = isFrame

    def isFrame(*args, **kwargs) -> Any: ...

    is_quiet = isQuiet

    def isQuiet(*args, **kwargs) -> Any: ...

    set_name = setName

    def setName(*args, **kwargs) -> Any: ...

    set_parent_frame = setParentFrame

class ActuatorType(enum.Enum):
    FORCE = 0

    PASSIVE = 1

    SERVO = 2

    MIMIC = 3

    ACCELERATION = 4

    VELOCITY = 5

    LOCKED = 6

FORCE: ActuatorType = ActuatorType.FORCE

PASSIVE: ActuatorType = ActuatorType.PASSIVE

SERVO: ActuatorType = ActuatorType.SERVO

MIMIC: ActuatorType = ActuatorType.MIMIC

ACCELERATION: ActuatorType = ActuatorType.ACCELERATION

VELOCITY: ActuatorType = ActuatorType.VELOCITY

LOCKED: ActuatorType = ActuatorType.LOCKED

DefaultActuatorType: ActuatorType = ActuatorType.FORCE

class MimicConstraintType(enum.Enum):
    Motor = 0

    Coupler = 1

Motor: MimicConstraintType = MimicConstraintType.Motor

Coupler: MimicConstraintType = MimicConstraintType.Coupler

class MimicDofProperties:
    def __init__(self) -> None: ...

    @property
    def mReferenceJoint(self) -> Joint: ...

    @mReferenceJoint.setter
    def mReferenceJoint(self, arg: Joint, /) -> None: ...

    @property
    def mReferenceDofIndex(self) -> int: ...

    @mReferenceDofIndex.setter
    def mReferenceDofIndex(self, arg: int, /) -> None: ...

    @property
    def mMultiplier(self) -> float: ...

    @mMultiplier.setter
    def mMultiplier(self, arg: float, /) -> None: ...

    @property
    def mOffset(self) -> float: ...

    @mOffset.setter
    def mOffset(self, arg: float, /) -> None: ...

    @property
    def mConstraintType(self) -> MimicConstraintType: ...

    @mConstraintType.setter
    def mConstraintType(self, arg: MimicConstraintType, /) -> None: ...

    @property
    def m_constraint_type(self) -> MimicConstraintType: ...

    @m_constraint_type.setter
    def m_constraint_type(self, arg: MimicConstraintType, /) -> None: ...

    @property
    def m_multiplier(self) -> float: ...

    @m_multiplier.setter
    def m_multiplier(self, arg: float, /) -> None: ...

    @property
    def m_offset(self) -> float: ...

    @m_offset.setter
    def m_offset(self, arg: float, /) -> None: ...

    @property
    def m_reference_dof_index(self) -> int: ...

    @m_reference_dof_index.setter
    def m_reference_dof_index(self, arg: int, /) -> None: ...

    @property
    def m_reference_joint(self) -> Joint: ...

    @m_reference_joint.setter
    def m_reference_joint(self, arg: Joint, /) -> None: ...

class Joint:
    def getName(*args, **kwargs) -> Any: ...

    def setName(*args, **kwargs) -> Any: ...

    def getType(*args, **kwargs) -> Any: ...

    def __repr__(self) -> str: ...

    def setActuatorType(*args, **kwargs) -> Any: ...

    def setActuatorTypeForDof(*args, **kwargs) -> Any: ...

    def setActuatorTypes(*args, **kwargs) -> Any: ...

    def getActuatorType(*args, **kwargs) -> Any: ...

    def getActuatorTypeForDof(*args, **kwargs) -> Any: ...

    def getActuatorTypes(*args, **kwargs) -> Any: ...

    def hasActuatorType(*args, **kwargs) -> Any: ...

    def setUseCouplerConstraint(*args, **kwargs) -> Any: ...

    def isUsingCouplerConstraint(*args, **kwargs) -> Any: ...

    def isKinematic(*args, **kwargs) -> Any: ...

    def isDynamic(*args, **kwargs) -> Any: ...

    def getNumDofs(*args, **kwargs) -> Any: ...

    def getDof(*args, **kwargs) -> Any: ...

    def setDofName(*args, **kwargs) -> Any: ...

    def getDofName(*args, **kwargs) -> Any: ...

    def preserveDofName(*args, **kwargs) -> Any: ...

    def isDofNamePreserved(*args, **kwargs) -> Any: ...

    def setCommand(*args, **kwargs) -> Any: ...

    def getCommand(*args, **kwargs) -> Any: ...

    def setCommands(*args, **kwargs) -> Any: ...

    def getCommands(*args, **kwargs) -> Any: ...

    def resetCommands(*args, **kwargs) -> Any: ...

    def setPosition(*args, **kwargs) -> Any: ...

    def setPositions(*args, **kwargs) -> Any: ...

    def getPosition(*args, **kwargs) -> Any: ...

    def getPositions(*args, **kwargs) -> Any: ...

    def resetPositions(*args, **kwargs) -> Any: ...

    def setVelocity(*args, **kwargs) -> Any: ...

    def setVelocities(*args, **kwargs) -> Any: ...

    def getVelocity(*args, **kwargs) -> Any: ...

    def getVelocities(*args, **kwargs) -> Any: ...

    def resetVelocities(*args, **kwargs) -> Any: ...

    def getParentBodyNode(*args, **kwargs) -> Any: ...

    def getChildBodyNode(*args, **kwargs) -> Any: ...

    def getRelativeTransform(*args, **kwargs) -> Any: ...

    def getRelativeJacobian(*args, **kwargs) -> Any: ...

    def getRelativeJacobianTimeDeriv(*args, **kwargs) -> Any: ...

    def getTransformFromChildBodyNode(*args, **kwargs) -> Any: ...

    def getTransformFromParentBodyNode(*args, **kwargs) -> Any: ...

    def setTransformFromChildBodyNode(*args, **kwargs) -> Any: ...

    def setTransformFromParentBodyNode(*args, **kwargs) -> Any: ...

    def getWrenchToParentBodyNode(*args, **kwargs) -> Any: ...

    def getWrenchToChildBodyNode(*args, **kwargs) -> Any: ...

    def setLimitEnforcement(*args, **kwargs) -> Any: ...

    def setAcceleration(*args, **kwargs) -> Any: ...

    def getAcceleration(*args, **kwargs) -> Any: ...

    def setAccelerations(*args, **kwargs) -> Any: ...

    def getAccelerations(*args, **kwargs) -> Any: ...

    def resetAccelerations(*args, **kwargs) -> Any: ...

    def setForce(*args, **kwargs) -> Any: ...

    def getForce(*args, **kwargs) -> Any: ...

    def setForces(*args, **kwargs) -> Any: ...

    def getForces(*args, **kwargs) -> Any: ...

    def resetForces(*args, **kwargs) -> Any: ...

    def setDampingCoefficient(*args, **kwargs) -> Any: ...

    def getDampingCoefficient(*args, **kwargs) -> Any: ...

    get_acceleration = getAcceleration

    get_accelerations = getAccelerations

    get_actuator_type = getActuatorType

    get_actuator_type_for_dof = getActuatorTypeForDof

    get_actuator_types = getActuatorTypes

    get_child_body_node = getChildBodyNode

    get_command = getCommand

    get_commands = getCommands

    get_damping_coefficient = getDampingCoefficient

    get_dof = getDof

    get_dof_name = getDofName

    get_force = getForce

    get_forces = getForces

    get_name = getName

    get_num_dofs = getNumDofs

    get_parent_body_node = getParentBodyNode

    get_position = getPosition

    get_positions = getPositions

    get_relative_jacobian = getRelativeJacobian

    get_relative_jacobian_time_deriv = getRelativeJacobianTimeDeriv

    get_relative_transform = getRelativeTransform

    get_transform_from_child_body_node = getTransformFromChildBodyNode

    get_transform_from_parent_body_node = getTransformFromParentBodyNode

    get_type = getType

    get_velocities = getVelocities

    get_velocity = getVelocity

    get_wrench_to_child_body_node = getWrenchToChildBodyNode

    get_wrench_to_parent_body_node = getWrenchToParentBodyNode

    has_actuator_type = hasActuatorType

    is_dof_name_preserved = isDofNamePreserved

    is_dynamic = isDynamic

    is_kinematic = isKinematic

    is_using_coupler_constraint = isUsingCouplerConstraint

    preserve_dof_name = preserveDofName

    reset_accelerations = resetAccelerations

    reset_commands = resetCommands

    reset_forces = resetForces

    reset_positions = resetPositions

    reset_velocities = resetVelocities

    set_acceleration = setAcceleration

    set_accelerations = setAccelerations

    set_actuator_type = setActuatorType

    set_actuator_type_for_dof = setActuatorTypeForDof

    set_actuator_types = setActuatorTypes

    set_command = setCommand

    set_commands = setCommands

    set_damping_coefficient = setDampingCoefficient

    set_dof_name = setDofName

    set_force = setForce

    set_forces = setForces

    set_limit_enforcement = setLimitEnforcement

    set_name = setName

    set_position = setPosition

    set_positions = setPositions

    set_transform_from_child_body_node = setTransformFromChildBodyNode

    set_transform_from_parent_body_node = setTransformFromParentBodyNode

    set_use_coupler_constraint = setUseCouplerConstraint

    set_velocities = setVelocities

    set_velocity = setVelocity

class ZeroDofJointProperties:
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, properties: "dart::dynamics::detail::JointProperties") -> None: ...

class ZeroDofJoint(Joint):
    def getZeroDofJointProperties(*args, **kwargs) -> Any: ...

    get_zero_dof_joint_properties = getZeroDofJointProperties

class DegreeOfFreedom(dartpy.common.Subject):
    def setName(*args, **kwargs) -> Any: ...

    def getName(*args, **kwargs) -> Any: ...

    def __repr__(self) -> str: ...

    def preserveName(*args, **kwargs) -> Any: ...

    def isNamePreserved(*args, **kwargs) -> Any: ...

    def getIndexInSkeleton(*args, **kwargs) -> Any: ...

    def getIndexInTree(*args, **kwargs) -> Any: ...

    def getIndexInJoint(*args, **kwargs) -> Any: ...

    def getTreeIndex(*args, **kwargs) -> Any: ...

    def setCommand(*args, **kwargs) -> Any: ...

    def getCommand(*args, **kwargs) -> Any: ...

    def resetCommand(*args, **kwargs) -> Any: ...

    def setPosition(*args, **kwargs) -> Any: ...

    def getPosition(*args, **kwargs) -> Any: ...

    def setPositionLimits(*args, **kwargs) -> Any: ...

    def getPositionLimits(*args, **kwargs) -> Any: ...

    def setPositionLowerLimit(*args, **kwargs) -> Any: ...

    def getPositionLowerLimit(*args, **kwargs) -> Any: ...

    def setPositionUpperLimit(*args, **kwargs) -> Any: ...

    def getPositionUpperLimit(*args, **kwargs) -> Any: ...

    def isCyclic(*args, **kwargs) -> Any: ...

    def hasPositionLimit(*args, **kwargs) -> Any: ...

    def resetPosition(*args, **kwargs) -> Any: ...

    def setInitialPosition(*args, **kwargs) -> Any: ...

    def getInitialPosition(*args, **kwargs) -> Any: ...

    def setVelocity(*args, **kwargs) -> Any: ...

    def getVelocity(*args, **kwargs) -> Any: ...

    def setVelocityLimits(*args, **kwargs) -> Any: ...

    def getVelocityLimits(*args, **kwargs) -> Any: ...

    def setVelocityLowerLimit(*args, **kwargs) -> Any: ...

    def getVelocityLowerLimit(*args, **kwargs) -> Any: ...

    def setVelocityUpperLimit(*args, **kwargs) -> Any: ...

    def getVelocityUpperLimit(*args, **kwargs) -> Any: ...

    def resetVelocity(*args, **kwargs) -> Any: ...

    def setInitialVelocity(*args, **kwargs) -> Any: ...

    def getInitialVelocity(*args, **kwargs) -> Any: ...

    def setAcceleration(*args, **kwargs) -> Any: ...

    def getAcceleration(*args, **kwargs) -> Any: ...

    def resetAcceleration(*args, **kwargs) -> Any: ...

    def setAccelerationLimits(*args, **kwargs) -> Any: ...

    def getAccelerationLimits(*args, **kwargs) -> Any: ...

    def setAccelerationLowerLimit(*args, **kwargs) -> Any: ...

    def getAccelerationLowerLimit(*args, **kwargs) -> Any: ...

    def setAccelerationUpperLimit(*args, **kwargs) -> Any: ...

    def getAccelerationUpperLimit(*args, **kwargs) -> Any: ...

    def setForce(*args, **kwargs) -> Any: ...

    def getForce(*args, **kwargs) -> Any: ...

    def resetForce(*args, **kwargs) -> Any: ...

    def setForceLimits(*args, **kwargs) -> Any: ...

    def getForceLimits(*args, **kwargs) -> Any: ...

    def setForceLowerLimit(*args, **kwargs) -> Any: ...

    def getForceLowerLimit(*args, **kwargs) -> Any: ...

    def setForceUpperLimit(*args, **kwargs) -> Any: ...

    def getForceUpperLimit(*args, **kwargs) -> Any: ...

    def setVelocityChange(*args, **kwargs) -> Any: ...

    def getVelocityChange(*args, **kwargs) -> Any: ...

    def resetVelocityChange(*args, **kwargs) -> Any: ...

    def setConstraintImpulse(*args, **kwargs) -> Any: ...

    def getConstraintImpulse(*args, **kwargs) -> Any: ...

    def resetConstraintImpulse(*args, **kwargs) -> Any: ...

    def setSpringStiffness(*args, **kwargs) -> Any: ...

    def getSpringStiffness(*args, **kwargs) -> Any: ...

    def setRestPosition(*args, **kwargs) -> Any: ...

    def getRestPosition(*args, **kwargs) -> Any: ...

    def setDampingCoefficient(*args, **kwargs) -> Any: ...

    def getDampingCoefficient(*args, **kwargs) -> Any: ...

    def setCoulombFriction(*args, **kwargs) -> Any: ...

    def getCoulombFriction(*args, **kwargs) -> Any: ...

    def getSkeleton(*args, **kwargs) -> Any: ...

    get_acceleration = getAcceleration

    get_acceleration_limits = getAccelerationLimits

    get_acceleration_lower_limit = getAccelerationLowerLimit

    get_acceleration_upper_limit = getAccelerationUpperLimit

    get_command = getCommand

    get_constraint_impulse = getConstraintImpulse

    get_coulomb_friction = getCoulombFriction

    get_damping_coefficient = getDampingCoefficient

    get_force = getForce

    get_force_limits = getForceLimits

    get_force_lower_limit = getForceLowerLimit

    get_force_upper_limit = getForceUpperLimit

    get_index_in_joint = getIndexInJoint

    get_index_in_skeleton = getIndexInSkeleton

    get_index_in_tree = getIndexInTree

    get_initial_position = getInitialPosition

    get_initial_velocity = getInitialVelocity

    get_name = getName

    get_position = getPosition

    get_position_limits = getPositionLimits

    get_position_lower_limit = getPositionLowerLimit

    get_position_upper_limit = getPositionUpperLimit

    get_rest_position = getRestPosition

    get_skeleton = getSkeleton

    get_spring_stiffness = getSpringStiffness

    get_tree_index = getTreeIndex

    get_velocity = getVelocity

    get_velocity_change = getVelocityChange

    get_velocity_limits = getVelocityLimits

    get_velocity_lower_limit = getVelocityLowerLimit

    get_velocity_upper_limit = getVelocityUpperLimit

    has_position_limit = hasPositionLimit

    is_cyclic = isCyclic

    is_name_preserved = isNamePreserved

    preserve_name = preserveName

    reset_acceleration = resetAcceleration

    reset_command = resetCommand

    reset_constraint_impulse = resetConstraintImpulse

    reset_force = resetForce

    reset_position = resetPosition

    reset_velocity = resetVelocity

    reset_velocity_change = resetVelocityChange

    set_acceleration = setAcceleration

    set_acceleration_limits = setAccelerationLimits

    set_acceleration_lower_limit = setAccelerationLowerLimit

    set_acceleration_upper_limit = setAccelerationUpperLimit

    set_command = setCommand

    set_constraint_impulse = setConstraintImpulse

    set_coulomb_friction = setCoulombFriction

    set_damping_coefficient = setDampingCoefficient

    set_force = setForce

    set_force_limits = setForceLimits

    set_force_lower_limit = setForceLowerLimit

    set_force_upper_limit = setForceUpperLimit

    set_initial_position = setInitialPosition

    set_initial_velocity = setInitialVelocity

    set_name = setName

    set_position = setPosition

    set_position_limits = setPositionLimits

    set_position_lower_limit = setPositionLowerLimit

    set_position_upper_limit = setPositionUpperLimit

    set_rest_position = setRestPosition

    set_spring_stiffness = setSpringStiffness

    set_velocity = setVelocity

    set_velocity_change = setVelocityChange

    set_velocity_limits = setVelocityLimits

    set_velocity_lower_limit = setVelocityLowerLimit

    set_velocity_upper_limit = setVelocityUpperLimit

class FreeJointProperties:
    def __init__(self) -> None: ...

    @property
    def mName(self) -> str: ...

    @mName.setter
    def mName(self, arg: str, /) -> None: ...

    @property
    def mT_ParentBodyToJoint(self) -> dartpy.math.Isometry3: ...

    @mT_ParentBodyToJoint.setter
    def mT_ParentBodyToJoint(self, arg: dartpy.math.Isometry3, /) -> None: ...

    @property
    def mT_ChildBodyToJoint(self) -> dartpy.math.Isometry3: ...

    @mT_ChildBodyToJoint.setter
    def mT_ChildBodyToJoint(self, arg: dartpy.math.Isometry3, /) -> None: ...

    @property
    def m_name(self) -> str: ...

    @m_name.setter
    def m_name(self, arg: str, /) -> None: ...

class FreeJoint(Joint):
    def setTransform(*args, **kwargs) -> Any: ...

    def getRelativeTransform(*args, **kwargs) -> Any: ...

    def setPositions(*args, **kwargs) -> Any: ...

    get_acceleration = getAcceleration

    def getAcceleration(*args, **kwargs) -> Any: ...

    get_accelerations = getAccelerations

    def getAccelerations(*args, **kwargs) -> Any: ...

    get_actuator_type = getActuatorType

    def getActuatorType(*args, **kwargs) -> Any: ...

    get_actuator_type_for_dof = getActuatorTypeForDof

    def getActuatorTypeForDof(*args, **kwargs) -> Any: ...

    get_actuator_types = getActuatorTypes

    def getActuatorTypes(*args, **kwargs) -> Any: ...

    get_child_body_node = getChildBodyNode

    def getChildBodyNode(*args, **kwargs) -> Any: ...

    get_command = getCommand

    def getCommand(*args, **kwargs) -> Any: ...

    get_commands = getCommands

    def getCommands(*args, **kwargs) -> Any: ...

    get_damping_coefficient = getDampingCoefficient

    def getDampingCoefficient(*args, **kwargs) -> Any: ...

    get_dof = getDof

    def getDof(*args, **kwargs) -> Any: ...

    get_dof_name = getDofName

    def getDofName(*args, **kwargs) -> Any: ...

    get_force = getForce

    def getForce(*args, **kwargs) -> Any: ...

    get_forces = getForces

    def getForces(*args, **kwargs) -> Any: ...

    get_name = getName

    def getName(*args, **kwargs) -> Any: ...

    get_num_dofs = getNumDofs

    def getNumDofs(*args, **kwargs) -> Any: ...

    get_parent_body_node = getParentBodyNode

    def getParentBodyNode(*args, **kwargs) -> Any: ...

    get_position = getPosition

    def getPosition(*args, **kwargs) -> Any: ...

    get_positions = getPositions

    def getPositions(*args, **kwargs) -> Any: ...

    get_relative_jacobian = getRelativeJacobian

    def getRelativeJacobian(*args, **kwargs) -> Any: ...

    get_relative_jacobian_time_deriv = getRelativeJacobianTimeDeriv

    def getRelativeJacobianTimeDeriv(*args, **kwargs) -> Any: ...

    get_relative_transform = getRelativeTransform

    get_transform_from_child_body_node = getTransformFromChildBodyNode

    def getTransformFromChildBodyNode(*args, **kwargs) -> Any: ...

    get_transform_from_parent_body_node = getTransformFromParentBodyNode

    def getTransformFromParentBodyNode(*args, **kwargs) -> Any: ...

    get_type = getType

    def getType(*args, **kwargs) -> Any: ...

    get_velocities = getVelocities

    def getVelocities(*args, **kwargs) -> Any: ...

    get_velocity = getVelocity

    def getVelocity(*args, **kwargs) -> Any: ...

    get_wrench_to_child_body_node = getWrenchToChildBodyNode

    def getWrenchToChildBodyNode(*args, **kwargs) -> Any: ...

    get_wrench_to_parent_body_node = getWrenchToParentBodyNode

    def getWrenchToParentBodyNode(*args, **kwargs) -> Any: ...

    has_actuator_type = hasActuatorType

    def hasActuatorType(*args, **kwargs) -> Any: ...

    is_dof_name_preserved = isDofNamePreserved

    def isDofNamePreserved(*args, **kwargs) -> Any: ...

    is_dynamic = isDynamic

    def isDynamic(*args, **kwargs) -> Any: ...

    is_kinematic = isKinematic

    def isKinematic(*args, **kwargs) -> Any: ...

    is_using_coupler_constraint = isUsingCouplerConstraint

    def isUsingCouplerConstraint(*args, **kwargs) -> Any: ...

    preserve_dof_name = preserveDofName

    def preserveDofName(*args, **kwargs) -> Any: ...

    reset_accelerations = resetAccelerations

    def resetAccelerations(*args, **kwargs) -> Any: ...

    reset_commands = resetCommands

    def resetCommands(*args, **kwargs) -> Any: ...

    reset_forces = resetForces

    def resetForces(*args, **kwargs) -> Any: ...

    reset_positions = resetPositions

    def resetPositions(*args, **kwargs) -> Any: ...

    reset_velocities = resetVelocities

    def resetVelocities(*args, **kwargs) -> Any: ...

    set_acceleration = setAcceleration

    def setAcceleration(*args, **kwargs) -> Any: ...

    set_accelerations = setAccelerations

    def setAccelerations(*args, **kwargs) -> Any: ...

    set_actuator_type = setActuatorType

    def setActuatorType(*args, **kwargs) -> Any: ...

    set_actuator_type_for_dof = setActuatorTypeForDof

    def setActuatorTypeForDof(*args, **kwargs) -> Any: ...

    set_actuator_types = setActuatorTypes

    def setActuatorTypes(*args, **kwargs) -> Any: ...

    set_command = setCommand

    def setCommand(*args, **kwargs) -> Any: ...

    set_commands = setCommands

    def setCommands(*args, **kwargs) -> Any: ...

    set_damping_coefficient = setDampingCoefficient

    def setDampingCoefficient(*args, **kwargs) -> Any: ...

    set_dof_name = setDofName

    def setDofName(*args, **kwargs) -> Any: ...

    set_force = setForce

    def setForce(*args, **kwargs) -> Any: ...

    set_forces = setForces

    def setForces(*args, **kwargs) -> Any: ...

    set_limit_enforcement = setLimitEnforcement

    def setLimitEnforcement(*args, **kwargs) -> Any: ...

    set_name = setName

    def setName(*args, **kwargs) -> Any: ...

    set_position = setPosition

    def setPosition(*args, **kwargs) -> Any: ...

    set_positions = setPositions

    set_transform = setTransform

    set_transform_from_child_body_node = setTransformFromChildBodyNode

    def setTransformFromChildBodyNode(*args, **kwargs) -> Any: ...

    set_transform_from_parent_body_node = setTransformFromParentBodyNode

    def setTransformFromParentBodyNode(*args, **kwargs) -> Any: ...

    set_use_coupler_constraint = setUseCouplerConstraint

    def setUseCouplerConstraint(*args, **kwargs) -> Any: ...

    set_velocities = setVelocities

    def setVelocities(*args, **kwargs) -> Any: ...

    set_velocity = setVelocity

    def setVelocity(*args, **kwargs) -> Any: ...

class RevoluteJointProperties:
    def __init__(self) -> None: ...

    @property
    def mName(self) -> "std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >": ...

    @mName.setter
    def mName(self, arg: "std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >", /) -> None: ...

    @property
    def mAxis(self) -> Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]: ...

    @mAxis.setter
    def mAxis(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')], /) -> None: ...

    @property
    def mT_ParentBodyToJoint(self) -> dartpy.math.Isometry3: ...

    @mT_ParentBodyToJoint.setter
    def mT_ParentBodyToJoint(self, arg: dartpy.math.Isometry3, /) -> None: ...

    @property
    def mT_ChildBodyToJoint(self) -> dartpy.math.Isometry3: ...

    @mT_ChildBodyToJoint.setter
    def mT_ChildBodyToJoint(self, arg: dartpy.math.Isometry3, /) -> None: ...

    @property
    def m_axis(self) -> Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]: ...

    @m_axis.setter
    def m_axis(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')], /) -> None: ...

    @property
    def m_name(self) -> "std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >": ...

    @m_name.setter
    def m_name(self, arg: "std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >", /) -> None: ...

class RevoluteJoint(Joint):
    def setAxis(*args, **kwargs) -> Any: ...

    def getAxis(*args, **kwargs) -> Any: ...

    get_axis = getAxis

    set_axis = setAxis

class TranslationalJoint2DProperties:
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, properties: "dart::dynamics::detail::GenericJointProperties<dart::math::RealVectorSpace<2ul> >") -> None: ...

class TranslationalJoint2D(Joint):
    def getTranslationalJoint2DProperties(*args, **kwargs) -> Any: ...

    def getType(self) -> "std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >": ...

    def isCyclic(*args, **kwargs) -> Any: ...

    def setXYPlane(*args, **kwargs) -> Any: ...

    def setYZPlane(*args, **kwargs) -> Any: ...

    def setZXPlane(*args, **kwargs) -> Any: ...

    def setArbitraryPlane(*args, **kwargs) -> Any: ...

    def getStaticType(*args, **kwargs): ...

    get_static_type = getStaticType

    get_translational_joint2_d_properties = getTranslationalJoint2DProperties

    is_cyclic = isCyclic

    set_arbitrary_plane = setArbitraryPlane

    set_xy_plane = setXYPlane

    set_yz_plane = setYZPlane

    set_zx_plane = setZXPlane

class PrismaticJointProperties:
    def __init__(self) -> None: ...

    @property
    def mAxis(self) -> Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]: ...

    @mAxis.setter
    def mAxis(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')], /) -> None: ...

    @property
    def m_axis(self) -> Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]: ...

    @m_axis.setter
    def m_axis(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')], /) -> None: ...

class PrismaticJoint(Joint):
    def setAxis(*args, **kwargs) -> Any: ...

    def getAxis(*args, **kwargs) -> Any: ...

    get_axis = getAxis

    set_axis = setAxis

class PlanarJoint(Joint):
    pass

class TranslationalJointProperties:
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, properties: "dart::dynamics::detail::GenericJointProperties<dart::math::RealVectorSpace<3ul> >") -> None: ...

class TranslationalJoint(Joint):
    def getTranslationalJointProperties(*args, **kwargs) -> Any: ...

    def getType(self) -> "std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >": ...

    def isCyclic(*args, **kwargs) -> Any: ...

    def getRelativeJacobianStatic(*args, **kwargs) -> Any: ...

    def getStaticType(*args, **kwargs): ...

    get_relative_jacobian_static = getRelativeJacobianStatic

    get_static_type = getStaticType

    get_translational_joint_properties = getTranslationalJointProperties

    is_cyclic = isCyclic

class EulerJoint(Joint):
    get_acceleration = getAcceleration

    def getAcceleration(*args, **kwargs) -> Any: ...

    get_accelerations = getAccelerations

    def getAccelerations(*args, **kwargs) -> Any: ...

    get_actuator_type = getActuatorType

    def getActuatorType(*args, **kwargs) -> Any: ...

    get_actuator_type_for_dof = getActuatorTypeForDof

    def getActuatorTypeForDof(*args, **kwargs) -> Any: ...

    get_actuator_types = getActuatorTypes

    def getActuatorTypes(*args, **kwargs) -> Any: ...

    get_child_body_node = getChildBodyNode

    def getChildBodyNode(*args, **kwargs) -> Any: ...

    get_command = getCommand

    def getCommand(*args, **kwargs) -> Any: ...

    get_commands = getCommands

    def getCommands(*args, **kwargs) -> Any: ...

    get_damping_coefficient = getDampingCoefficient

    def getDampingCoefficient(*args, **kwargs) -> Any: ...

    get_dof = getDof

    def getDof(*args, **kwargs) -> Any: ...

    get_dof_name = getDofName

    def getDofName(*args, **kwargs) -> Any: ...

    get_force = getForce

    def getForce(*args, **kwargs) -> Any: ...

    get_forces = getForces

    def getForces(*args, **kwargs) -> Any: ...

    get_name = getName

    def getName(*args, **kwargs) -> Any: ...

    get_num_dofs = getNumDofs

    def getNumDofs(*args, **kwargs) -> Any: ...

    get_parent_body_node = getParentBodyNode

    def getParentBodyNode(*args, **kwargs) -> Any: ...

    get_position = getPosition

    def getPosition(*args, **kwargs) -> Any: ...

    get_positions = getPositions

    def getPositions(*args, **kwargs) -> Any: ...

    get_relative_jacobian = getRelativeJacobian

    def getRelativeJacobian(*args, **kwargs) -> Any: ...

    get_relative_jacobian_time_deriv = getRelativeJacobianTimeDeriv

    def getRelativeJacobianTimeDeriv(*args, **kwargs) -> Any: ...

    get_relative_transform = getRelativeTransform

    def getRelativeTransform(*args, **kwargs) -> Any: ...

    get_transform_from_child_body_node = getTransformFromChildBodyNode

    def getTransformFromChildBodyNode(*args, **kwargs) -> Any: ...

    get_transform_from_parent_body_node = getTransformFromParentBodyNode

    def getTransformFromParentBodyNode(*args, **kwargs) -> Any: ...

    get_type = getType

    def getType(*args, **kwargs) -> Any: ...

    get_velocities = getVelocities

    def getVelocities(*args, **kwargs) -> Any: ...

    get_velocity = getVelocity

    def getVelocity(*args, **kwargs) -> Any: ...

    get_wrench_to_child_body_node = getWrenchToChildBodyNode

    def getWrenchToChildBodyNode(*args, **kwargs) -> Any: ...

    get_wrench_to_parent_body_node = getWrenchToParentBodyNode

    def getWrenchToParentBodyNode(*args, **kwargs) -> Any: ...

    has_actuator_type = hasActuatorType

    def hasActuatorType(*args, **kwargs) -> Any: ...

    is_dof_name_preserved = isDofNamePreserved

    def isDofNamePreserved(*args, **kwargs) -> Any: ...

    is_dynamic = isDynamic

    def isDynamic(*args, **kwargs) -> Any: ...

    is_kinematic = isKinematic

    def isKinematic(*args, **kwargs) -> Any: ...

    is_using_coupler_constraint = isUsingCouplerConstraint

    def isUsingCouplerConstraint(*args, **kwargs) -> Any: ...

    preserve_dof_name = preserveDofName

    def preserveDofName(*args, **kwargs) -> Any: ...

    reset_accelerations = resetAccelerations

    def resetAccelerations(*args, **kwargs) -> Any: ...

    reset_commands = resetCommands

    def resetCommands(*args, **kwargs) -> Any: ...

    reset_forces = resetForces

    def resetForces(*args, **kwargs) -> Any: ...

    reset_positions = resetPositions

    def resetPositions(*args, **kwargs) -> Any: ...

    reset_velocities = resetVelocities

    def resetVelocities(*args, **kwargs) -> Any: ...

    set_acceleration = setAcceleration

    def setAcceleration(*args, **kwargs) -> Any: ...

    set_accelerations = setAccelerations

    def setAccelerations(*args, **kwargs) -> Any: ...

    set_actuator_type = setActuatorType

    def setActuatorType(*args, **kwargs) -> Any: ...

    set_actuator_type_for_dof = setActuatorTypeForDof

    def setActuatorTypeForDof(*args, **kwargs) -> Any: ...

    set_actuator_types = setActuatorTypes

    def setActuatorTypes(*args, **kwargs) -> Any: ...

    set_command = setCommand

    def setCommand(*args, **kwargs) -> Any: ...

    set_commands = setCommands

    def setCommands(*args, **kwargs) -> Any: ...

    set_damping_coefficient = setDampingCoefficient

    def setDampingCoefficient(*args, **kwargs) -> Any: ...

    set_dof_name = setDofName

    def setDofName(*args, **kwargs) -> Any: ...

    set_force = setForce

    def setForce(*args, **kwargs) -> Any: ...

    set_forces = setForces

    def setForces(*args, **kwargs) -> Any: ...

    set_limit_enforcement = setLimitEnforcement

    def setLimitEnforcement(*args, **kwargs) -> Any: ...

    set_name = setName

    def setName(*args, **kwargs) -> Any: ...

    set_position = setPosition

    def setPosition(*args, **kwargs) -> Any: ...

    set_positions = setPositions

    def setPositions(*args, **kwargs) -> Any: ...

    set_transform_from_child_body_node = setTransformFromChildBodyNode

    def setTransformFromChildBodyNode(*args, **kwargs) -> Any: ...

    set_transform_from_parent_body_node = setTransformFromParentBodyNode

    def setTransformFromParentBodyNode(*args, **kwargs) -> Any: ...

    set_use_coupler_constraint = setUseCouplerConstraint

    def setUseCouplerConstraint(*args, **kwargs) -> Any: ...

    set_velocities = setVelocities

    def setVelocities(*args, **kwargs) -> Any: ...

    set_velocity = setVelocity

    def setVelocity(*args, **kwargs) -> Any: ...

class ScrewJointProperties:
    def __init__(self) -> None: ...

    @property
    def mAxis(self) -> Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]: ...

    @mAxis.setter
    def mAxis(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')], /) -> None: ...

    @property
    def mPitch(self) -> float: ...

    @mPitch.setter
    def mPitch(self, arg: float, /) -> None: ...

    @property
    def m_axis(self) -> Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]: ...

    @m_axis.setter
    def m_axis(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')], /) -> None: ...

    @property
    def m_pitch(self) -> float: ...

    @m_pitch.setter
    def m_pitch(self, arg: float, /) -> None: ...

class ScrewJoint(Joint):
    def getType(self) -> "std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >": ...

    def isCyclic(*args, **kwargs) -> Any: ...

    def setAxis(*args, **kwargs) -> Any: ...

    def getAxis(*args, **kwargs) -> Any: ...

    def setPitch(*args, **kwargs) -> Any: ...

    def getPitch(*args, **kwargs) -> Any: ...

    def getStaticType(*args, **kwargs): ...

    get_axis = getAxis

    get_pitch = getPitch

    get_static_type = getStaticType

    is_cyclic = isCyclic

    set_axis = setAxis

    set_pitch = setPitch

class UniversalJointProperties:
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, properties: "dart::dynamics::detail::GenericJointProperties<dart::math::RealVectorSpace<2ul> >") -> None: ...

class UniversalJoint(Joint):
    def getUniversalJointProperties(*args, **kwargs) -> Any: ...

    def getType(self) -> "std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >": ...

    def isCyclic(*args, **kwargs) -> Any: ...

    def setAxis1(*args, **kwargs) -> Any: ...

    def setAxis2(*args, **kwargs) -> Any: ...

    def getAxis1(*args, **kwargs) -> Any: ...

    def getAxis2(*args, **kwargs) -> Any: ...

    def getRelativeJacobianStatic(*args, **kwargs) -> Any: ...

    def getStaticType(*args, **kwargs): ...

    get_axis1 = getAxis1

    get_axis2 = getAxis2

    get_relative_jacobian_static = getRelativeJacobianStatic

    get_static_type = getStaticType

    get_universal_joint_properties = getUniversalJointProperties

    is_cyclic = isCyclic

    set_axis1 = setAxis1

    set_axis2 = setAxis2

class BallJoint(Joint):
    def convertToPositions(*args, **kwargs): ...

    def convertToRotation(*args, **kwargs): ...

    def convertToTransform(*args, **kwargs): ...

    convert_to_positions = convertToPositions

    convert_to_rotation = convertToRotation

    convert_to_transform = convertToTransform

    get_acceleration = getAcceleration

    def getAcceleration(*args, **kwargs) -> Any: ...

    get_accelerations = getAccelerations

    def getAccelerations(*args, **kwargs) -> Any: ...

    get_actuator_type = getActuatorType

    def getActuatorType(*args, **kwargs) -> Any: ...

    get_actuator_type_for_dof = getActuatorTypeForDof

    def getActuatorTypeForDof(*args, **kwargs) -> Any: ...

    get_actuator_types = getActuatorTypes

    def getActuatorTypes(*args, **kwargs) -> Any: ...

    get_child_body_node = getChildBodyNode

    def getChildBodyNode(*args, **kwargs) -> Any: ...

    get_command = getCommand

    def getCommand(*args, **kwargs) -> Any: ...

    get_commands = getCommands

    def getCommands(*args, **kwargs) -> Any: ...

    get_damping_coefficient = getDampingCoefficient

    def getDampingCoefficient(*args, **kwargs) -> Any: ...

    get_dof = getDof

    def getDof(*args, **kwargs) -> Any: ...

    get_dof_name = getDofName

    def getDofName(*args, **kwargs) -> Any: ...

    get_force = getForce

    def getForce(*args, **kwargs) -> Any: ...

    get_forces = getForces

    def getForces(*args, **kwargs) -> Any: ...

    get_name = getName

    def getName(*args, **kwargs) -> Any: ...

    get_num_dofs = getNumDofs

    def getNumDofs(*args, **kwargs) -> Any: ...

    get_parent_body_node = getParentBodyNode

    def getParentBodyNode(*args, **kwargs) -> Any: ...

    get_position = getPosition

    def getPosition(*args, **kwargs) -> Any: ...

    get_positions = getPositions

    def getPositions(*args, **kwargs) -> Any: ...

    get_relative_jacobian = getRelativeJacobian

    def getRelativeJacobian(*args, **kwargs) -> Any: ...

    get_relative_jacobian_time_deriv = getRelativeJacobianTimeDeriv

    def getRelativeJacobianTimeDeriv(*args, **kwargs) -> Any: ...

    get_relative_transform = getRelativeTransform

    def getRelativeTransform(*args, **kwargs) -> Any: ...

    get_transform_from_child_body_node = getTransformFromChildBodyNode

    def getTransformFromChildBodyNode(*args, **kwargs) -> Any: ...

    get_transform_from_parent_body_node = getTransformFromParentBodyNode

    def getTransformFromParentBodyNode(*args, **kwargs) -> Any: ...

    get_type = getType

    def getType(*args, **kwargs) -> Any: ...

    get_velocities = getVelocities

    def getVelocities(*args, **kwargs) -> Any: ...

    get_velocity = getVelocity

    def getVelocity(*args, **kwargs) -> Any: ...

    get_wrench_to_child_body_node = getWrenchToChildBodyNode

    def getWrenchToChildBodyNode(*args, **kwargs) -> Any: ...

    get_wrench_to_parent_body_node = getWrenchToParentBodyNode

    def getWrenchToParentBodyNode(*args, **kwargs) -> Any: ...

    has_actuator_type = hasActuatorType

    def hasActuatorType(*args, **kwargs) -> Any: ...

    is_dof_name_preserved = isDofNamePreserved

    def isDofNamePreserved(*args, **kwargs) -> Any: ...

    is_dynamic = isDynamic

    def isDynamic(*args, **kwargs) -> Any: ...

    is_kinematic = isKinematic

    def isKinematic(*args, **kwargs) -> Any: ...

    is_using_coupler_constraint = isUsingCouplerConstraint

    def isUsingCouplerConstraint(*args, **kwargs) -> Any: ...

    preserve_dof_name = preserveDofName

    def preserveDofName(*args, **kwargs) -> Any: ...

    reset_accelerations = resetAccelerations

    def resetAccelerations(*args, **kwargs) -> Any: ...

    reset_commands = resetCommands

    def resetCommands(*args, **kwargs) -> Any: ...

    reset_forces = resetForces

    def resetForces(*args, **kwargs) -> Any: ...

    reset_positions = resetPositions

    def resetPositions(*args, **kwargs) -> Any: ...

    reset_velocities = resetVelocities

    def resetVelocities(*args, **kwargs) -> Any: ...

    set_acceleration = setAcceleration

    def setAcceleration(*args, **kwargs) -> Any: ...

    set_accelerations = setAccelerations

    def setAccelerations(*args, **kwargs) -> Any: ...

    set_actuator_type = setActuatorType

    def setActuatorType(*args, **kwargs) -> Any: ...

    set_actuator_type_for_dof = setActuatorTypeForDof

    def setActuatorTypeForDof(*args, **kwargs) -> Any: ...

    set_actuator_types = setActuatorTypes

    def setActuatorTypes(*args, **kwargs) -> Any: ...

    set_command = setCommand

    def setCommand(*args, **kwargs) -> Any: ...

    set_commands = setCommands

    def setCommands(*args, **kwargs) -> Any: ...

    set_damping_coefficient = setDampingCoefficient

    def setDampingCoefficient(*args, **kwargs) -> Any: ...

    set_dof_name = setDofName

    def setDofName(*args, **kwargs) -> Any: ...

    set_force = setForce

    def setForce(*args, **kwargs) -> Any: ...

    set_forces = setForces

    def setForces(*args, **kwargs) -> Any: ...

    set_limit_enforcement = setLimitEnforcement

    def setLimitEnforcement(*args, **kwargs) -> Any: ...

    set_name = setName

    def setName(*args, **kwargs) -> Any: ...

    set_position = setPosition

    def setPosition(*args, **kwargs) -> Any: ...

    set_positions = setPositions

    def setPositions(*args, **kwargs) -> Any: ...

    set_transform_from_child_body_node = setTransformFromChildBodyNode

    def setTransformFromChildBodyNode(*args, **kwargs) -> Any: ...

    set_transform_from_parent_body_node = setTransformFromParentBodyNode

    def setTransformFromParentBodyNode(*args, **kwargs) -> Any: ...

    set_use_coupler_constraint = setUseCouplerConstraint

    def setUseCouplerConstraint(*args, **kwargs) -> Any: ...

    set_velocities = setVelocities

    def setVelocities(*args, **kwargs) -> Any: ...

    set_velocity = setVelocity

    def setVelocity(*args, **kwargs) -> Any: ...

class WeldJoint(ZeroDofJoint):
    def getWeldJointProperties(*args, **kwargs) -> Any: ...

    def getType(self) -> str: ...

    def isCyclic(*args, **kwargs) -> Any: ...

    def setTransformFromParentBodyNode(self, transform: dartpy.math.Isometry3) -> None: ...

    def setTransformFromChildBodyNode(self, transform: dartpy.math.Isometry3) -> None: ...

    def getStaticType(*args, **kwargs): ...

    get_static_type = getStaticType

    get_weld_joint_properties = getWeldJointProperties

    get_zero_dof_joint_properties = getZeroDofJointProperties

    def getZeroDofJointProperties(*args, **kwargs) -> Any: ...

    is_cyclic = isCyclic

class Shape:
    def getType(*args, **kwargs) -> Any: ...

    def computeInertia(*args, **kwargs) -> Any: ...

    def __repr__(self) -> str: ...

    compute_inertia = computeInertia

    get_type = getType

class SphereShape(Shape):
    def __init__(self, radius: float) -> None: ...

    def setRadius(*args, **kwargs) -> Any: ...

    def getRadius(*args, **kwargs) -> Any: ...

    def computeInertia(self, mass: float) -> Annotated[NDArray[numpy.float64], dict(shape=(3, 3), order='F')]: ...

    def getStaticType(*args, **kwargs): ...

    def computeVolumeOf(*args, **kwargs): ...

    def computeInertiaOf(*args, **kwargs): ...

    def __repr__(self) -> str: ...

    compute_inertia_of = computeInertiaOf

    compute_volume_of = computeVolumeOf

    get_radius = getRadius

    get_static_type = getStaticType

    set_radius = setRadius

class BoxShape(Shape):
    @overload
    def __init__(self, size: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]) -> None: ...

    @overload
    def __init__(self, size: object) -> None: ...

    def setSize(*args, **kwargs) -> Any: ...

    def getSize(*args, **kwargs) -> Any: ...

    def computeInertia(*args, **kwargs) -> Any: ...

    def computeVolumeOf(*args, **kwargs): ...

    def computeInertiaOf(*args, **kwargs): ...

    def __repr__(self) -> str: ...

    compute_inertia = computeInertia

    compute_inertia_of = computeInertiaOf

    compute_volume_of = computeVolumeOf

    get_size = getSize

    get_type = getType

    def getType(*args, **kwargs) -> Any: ...

    set_size = setSize

class CapsuleShape(Shape):
    def __init__(self, radius: float, height: float) -> None: ...

    def getRadius(*args, **kwargs) -> Any: ...

    def setRadius(*args, **kwargs) -> Any: ...

    def getHeight(*args, **kwargs) -> Any: ...

    def setHeight(*args, **kwargs) -> Any: ...

    def computeInertia(*args, **kwargs) -> Any: ...

    compute_inertia = computeInertia

    get_height = getHeight

    get_radius = getRadius

    get_type = getType

    def getType(*args, **kwargs) -> Any: ...

    set_height = setHeight

    set_radius = setRadius

class CylinderShape(Shape):
    def __init__(self, radius: float, height: float) -> None: ...

    def getRadius(*args, **kwargs) -> Any: ...

    def setRadius(*args, **kwargs) -> Any: ...

    def getHeight(*args, **kwargs) -> Any: ...

    def setHeight(*args, **kwargs) -> Any: ...

    def computeInertia(*args, **kwargs) -> Any: ...

    compute_inertia = computeInertia

    get_height = getHeight

    get_radius = getRadius

    get_type = getType

    def getType(*args, **kwargs) -> Any: ...

    set_height = setHeight

    set_radius = setRadius

class EllipsoidShape(Shape):
    def __init__(self, diameters: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')]) -> None: ...

    def getDiameters(*args, **kwargs) -> Any: ...

    def setDiameters(*args, **kwargs) -> Any: ...

    def computeInertia(*args, **kwargs) -> Any: ...

    compute_inertia = computeInertia

    get_diameters = getDiameters

    get_type = getType

    def getType(*args, **kwargs) -> Any: ...

    set_diameters = setDiameters

class ConeShape(Shape):
    def __init__(self, radius: float, height: float) -> None: ...

    def getRadius(*args, **kwargs) -> Any: ...

    def setRadius(*args, **kwargs) -> Any: ...

    def getHeight(*args, **kwargs) -> Any: ...

    def setHeight(*args, **kwargs) -> Any: ...

    def computeInertia(*args, **kwargs) -> Any: ...

    compute_inertia = computeInertia

    get_height = getHeight

    get_radius = getRadius

    get_type = getType

    def getType(*args, **kwargs) -> Any: ...

    set_height = setHeight

    set_radius = setRadius

class PyramidShape(Shape):
    def __init__(self, base_width: float, base_depth: float, height: float) -> None: ...

    def getBaseWidth(*args, **kwargs) -> Any: ...

    def setBaseWidth(*args, **kwargs) -> Any: ...

    def getBaseDepth(*args, **kwargs) -> Any: ...

    def setBaseDepth(*args, **kwargs) -> Any: ...

    def getHeight(*args, **kwargs) -> Any: ...

    def setHeight(*args, **kwargs) -> Any: ...

    def computeInertia(*args, **kwargs) -> Any: ...

    compute_inertia = computeInertia

    get_base_depth = getBaseDepth

    get_base_width = getBaseWidth

    get_height = getHeight

    get_type = getType

    def getType(*args, **kwargs) -> Any: ...

    set_base_depth = setBaseDepth

    set_base_width = setBaseWidth

    set_height = setHeight

class PlaneShape(Shape):
    def __init__(self, normal: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')], offset: float) -> None: ...

    def setNormal(*args, **kwargs) -> Any: ...

    def getNormal(*args, **kwargs) -> Any: ...

    def setOffset(*args, **kwargs) -> Any: ...

    def getOffset(*args, **kwargs) -> Any: ...

    compute_inertia = computeInertia

    def computeInertia(*args, **kwargs) -> Any: ...

    get_normal = getNormal

    get_offset = getOffset

    get_type = getType

    def getType(*args, **kwargs) -> Any: ...

    set_normal = setNormal

    set_offset = setOffset

class LineSegmentShape(Shape):
    def __init__(self, thickness: float = ...) -> None: ...

    def addVertex(*args, **kwargs) -> Any: ...

    def addConnection(*args, **kwargs) -> Any: ...

    def setVertex(*args, **kwargs) -> Any: ...

    def getThickness(*args, **kwargs) -> Any: ...

    def setThickness(*args, **kwargs) -> Any: ...

    add_connection = addConnection

    add_vertex = addVertex

    compute_inertia = computeInertia

    def computeInertia(*args, **kwargs) -> Any: ...

    get_thickness = getThickness

    get_type = getType

    def getType(*args, **kwargs) -> Any: ...

    set_thickness = setThickness

    set_vertex = setVertex

class MeshShape(Shape):
    def __init__(self, scale: object, mesh: dartpy.math.TriMesh) -> None: ...

    def getTriMesh(*args, **kwargs) -> Any: ...

    def getScale(*args, **kwargs) -> Any: ...

    def getStaticType(*args, **kwargs): ...

    compute_inertia = computeInertia

    def computeInertia(*args, **kwargs) -> Any: ...

    get_scale = getScale

    get_static_type = getStaticType

    get_tri_mesh = getTriMesh

    get_type = getType

    def getType(*args, **kwargs) -> Any: ...

class HeightmapShape(Shape):
    def __init__(self) -> None: ...

    def setHeightField(*args, **kwargs) -> Any: ...

    def getHeightField(*args, **kwargs) -> Any: ...

    def setScale(*args, **kwargs) -> Any: ...

    def getScale(*args, **kwargs) -> Any: ...

    def getWidth(*args, **kwargs) -> Any: ...

    def getDepth(*args, **kwargs) -> Any: ...

    def getMinHeight(*args, **kwargs) -> Any: ...

    def getMaxHeight(*args, **kwargs) -> Any: ...

    def flipY(*args, **kwargs) -> Any: ...

    def getStaticType(*args, **kwargs): ...

    compute_inertia = computeInertia

    def computeInertia(*args, **kwargs) -> Any: ...

    flip_y = flipY

    get_depth = getDepth

    get_height_field = getHeightField

    get_max_height = getMaxHeight

    get_min_height = getMinHeight

    get_scale = getScale

    get_static_type = getStaticType

    get_type = getType

    def getType(*args, **kwargs) -> Any: ...

    get_width = getWidth

    set_height_field = setHeightField

    set_scale = setScale

class PointCloudShape(Shape):
    def __init__(self, visual_size: float = ...) -> None: ...

    def reserve(self, size: int) -> None: ...

    def addPoint(*args, **kwargs) -> Any: ...

    def addPoints(*args, **kwargs) -> Any: ...

    def setPoints(*args, **kwargs) -> Any: ...

    def getPoints(*args, **kwargs) -> Any: ...

    def getNumPoints(*args, **kwargs) -> Any: ...

    def removeAllPoints(*args, **kwargs) -> Any: ...

    def setPointShapeType(*args, **kwargs) -> Any: ...

    def getPointShapeType(*args, **kwargs) -> Any: ...

    def setColorMode(*args, **kwargs) -> Any: ...

    def getColorMode(*args, **kwargs) -> Any: ...

    def setOverallColor(*args, **kwargs) -> Any: ...

    def getOverallColor(*args, **kwargs) -> Any: ...

    def setVisualSize(*args, **kwargs) -> Any: ...

    def getVisualSize(*args, **kwargs) -> Any: ...

    def getStaticType(*args, **kwargs): ...

    class ColorMode(enum.Enum):
        USE_SHAPE_COLOR = 0

        BIND_OVERALL = 1

        BIND_PER_POINT = 2

    class PointShapeType(enum.Enum):
        BOX = 0

        BILLBOARD_SQUARE = 1

        BILLBOARD_CIRCLE = 2

        POINT = 3

    add_point = addPoint

    add_points = addPoints

    compute_inertia = computeInertia

    def computeInertia(*args, **kwargs) -> Any: ...

    get_color_mode = getColorMode

    get_num_points = getNumPoints

    get_overall_color = getOverallColor

    get_point_shape_type = getPointShapeType

    get_points = getPoints

    get_static_type = getStaticType

    get_type = getType

    def getType(*args, **kwargs) -> Any: ...

    get_visual_size = getVisualSize

    remove_all_points = removeAllPoints

    set_color_mode = setColorMode

    set_overall_color = setOverallColor

    set_point_shape_type = setPointShapeType

    set_points = setPoints

    set_visual_size = setVisualSize

class Frame:
    def getRelativeTransform(*args, **kwargs) -> Any: ...

    def getWorldTransform(*args, **kwargs) -> Any: ...

    def getTransform(*args, **kwargs) -> Any: ...

    def getParentFrame(*args, **kwargs) -> Any: ...

    def descendsFrom(*args, **kwargs) -> Any: ...

    def isShapeFrame(*args, **kwargs) -> Any: ...

    def isWorld(*args, **kwargs) -> Any: ...

    def World(*args, **kwargs): ...

    world = World

    descends_from = descendsFrom

    get_parent_frame = getParentFrame

    get_relative_transform = getRelativeTransform

    get_transform = getTransform

    get_world_transform = getWorldTransform

    is_shape_frame = isShapeFrame

    is_world = isWorld

class JacobianNode(Frame):
    def getIK(*args, **kwargs) -> Any: ...

    def getOrCreateIK(*args, **kwargs) -> Any: ...

    def clearIK(*args, **kwargs) -> Any: ...

    def dependsOn(*args, **kwargs) -> Any: ...

    def getNumDependentGenCoords(*args, **kwargs) -> Any: ...

    def getDependentGenCoordIndex(*args, **kwargs) -> Any: ...

    def getNumDependentDofs(*args, **kwargs) -> Any: ...

    def getChainDofs(*args, **kwargs) -> Any: ...

    def getJacobian(*args, **kwargs) -> Any: ...

    def getWorldJacobian(*args, **kwargs) -> Any: ...

    def getLinearJacobian(*args, **kwargs) -> Any: ...

    def getAngularJacobian(*args, **kwargs) -> Any: ...

    def getJacobianSpatialDeriv(*args, **kwargs) -> Any: ...

    def getJacobianClassicDeriv(*args, **kwargs) -> Any: ...

    def getLinearJacobianDeriv(*args, **kwargs) -> Any: ...

    def getAngularJacobianDeriv(*args, **kwargs) -> Any: ...

    def dirtyJacobian(*args, **kwargs) -> Any: ...

    def dirtyJacobianDeriv(*args, **kwargs) -> Any: ...

    clear_ik = clearIK

    depends_on = dependsOn

    dirty_jacobian = dirtyJacobian

    dirty_jacobian_deriv = dirtyJacobianDeriv

    get_angular_jacobian = getAngularJacobian

    get_angular_jacobian_deriv = getAngularJacobianDeriv

    get_chain_dofs = getChainDofs

    get_dependent_gen_coord_index = getDependentGenCoordIndex

    get_ik = getIK

    get_jacobian = getJacobian

    get_jacobian_classic_deriv = getJacobianClassicDeriv

    get_jacobian_spatial_deriv = getJacobianSpatialDeriv

    get_linear_jacobian = getLinearJacobian

    get_linear_jacobian_deriv = getLinearJacobianDeriv

    get_num_dependent_dofs = getNumDependentDofs

    get_num_dependent_gen_coords = getNumDependentGenCoords

    get_or_create_ik = getOrCreateIK

    get_world_jacobian = getWorldJacobian

class Support:
    def setGeometry(*args, **kwargs) -> Any: ...

    def getGeometry(*args, **kwargs) -> Any: ...

    def setActive(*args, **kwargs) -> Any: ...

    def isActive(*args, **kwargs) -> Any: ...

    get_geometry = getGeometry

    is_active = isActive

    set_active = setActive

    set_geometry = setGeometry

class EndEffector(JacobianNode):
    def setDefaultRelativeTransform(*args, **kwargs) -> Any: ...

    def getName(*args, **kwargs) -> Any: ...

    def resetRelativeTransform(*args, **kwargs) -> Any: ...

    def setRelativeTransform(*args, **kwargs) -> Any: ...

    def createSupport(*args, **kwargs) -> Any: ...

    def getSupport(*args, **kwargs) -> Any: ...

    def hasSupport(*args, **kwargs) -> Any: ...

    def removeSupport(*args, **kwargs) -> Any: ...

    def getWorldTransform(*args, **kwargs) -> Any: ...

    def getIK(*args, **kwargs) -> Any: ...

    world = World

    def World(*args, **kwargs): ...

    clear_ik = clearIK

    def clearIK(*args, **kwargs) -> Any: ...

    create_support = createSupport

    depends_on = dependsOn

    def dependsOn(*args, **kwargs) -> Any: ...

    descends_from = descendsFrom

    def descendsFrom(*args, **kwargs) -> Any: ...

    dirty_jacobian = dirtyJacobian

    def dirtyJacobian(*args, **kwargs) -> Any: ...

    dirty_jacobian_deriv = dirtyJacobianDeriv

    def dirtyJacobianDeriv(*args, **kwargs) -> Any: ...

    get_angular_jacobian = getAngularJacobian

    def getAngularJacobian(*args, **kwargs) -> Any: ...

    get_angular_jacobian_deriv = getAngularJacobianDeriv

    def getAngularJacobianDeriv(*args, **kwargs) -> Any: ...

    get_chain_dofs = getChainDofs

    def getChainDofs(*args, **kwargs) -> Any: ...

    get_dependent_gen_coord_index = getDependentGenCoordIndex

    def getDependentGenCoordIndex(*args, **kwargs) -> Any: ...

    get_ik = getIK

    get_jacobian = getJacobian

    def getJacobian(*args, **kwargs) -> Any: ...

    get_jacobian_classic_deriv = getJacobianClassicDeriv

    def getJacobianClassicDeriv(*args, **kwargs) -> Any: ...

    get_jacobian_spatial_deriv = getJacobianSpatialDeriv

    def getJacobianSpatialDeriv(*args, **kwargs) -> Any: ...

    get_linear_jacobian = getLinearJacobian

    def getLinearJacobian(*args, **kwargs) -> Any: ...

    get_linear_jacobian_deriv = getLinearJacobianDeriv

    def getLinearJacobianDeriv(*args, **kwargs) -> Any: ...

    get_name = getName

    get_num_dependent_dofs = getNumDependentDofs

    def getNumDependentDofs(*args, **kwargs) -> Any: ...

    get_num_dependent_gen_coords = getNumDependentGenCoords

    def getNumDependentGenCoords(*args, **kwargs) -> Any: ...

    get_or_create_ik = getOrCreateIK

    def getOrCreateIK(*args, **kwargs) -> Any: ...

    get_parent_frame = getParentFrame

    def getParentFrame(*args, **kwargs) -> Any: ...

    get_relative_transform = getRelativeTransform

    def getRelativeTransform(*args, **kwargs) -> Any: ...

    get_support = getSupport

    get_transform = getTransform

    def getTransform(*args, **kwargs) -> Any: ...

    get_world_jacobian = getWorldJacobian

    def getWorldJacobian(*args, **kwargs) -> Any: ...

    get_world_transform = getWorldTransform

    has_support = hasSupport

    is_shape_frame = isShapeFrame

    def isShapeFrame(*args, **kwargs) -> Any: ...

    is_world = isWorld

    def isWorld(*args, **kwargs) -> Any: ...

    remove_support = removeSupport

    reset_relative_transform = resetRelativeTransform

    set_default_relative_transform = setDefaultRelativeTransform

    set_relative_transform = setRelativeTransform

class BodyNode(JacobianNode):
    def getName(*args, **kwargs) -> Any: ...

    def createShapeNode(*args, **kwargs) -> Any: ...

    def getShapeNodes(*args, **kwargs) -> Any: ...

    def getNumShapeNodes(*args, **kwargs) -> Any: ...

    def getBodyForce(*args, **kwargs) -> Any: ...

    def getSpatialVelocity(*args, **kwargs) -> Any: ...

    def getLinearVelocity(*args, **kwargs) -> Any: ...

    def getBodyNodePtr(*args, **kwargs) -> Any: ...

    def getChildBodyNode(*args, **kwargs) -> Any: ...

    def getChildJoint(*args, **kwargs) -> Any: ...

    def getParentBodyNode(*args, **kwargs) -> Any: ...

    def getParentJoint(*args, **kwargs) -> Any: ...

    def getNumEndEffectors(*args, **kwargs) -> Any: ...

    def getEndEffector(*args, **kwargs) -> Any: ...

    def getInertia(*args, **kwargs) -> Any: ...

    def setInertia(*args, **kwargs) -> Any: ...

    def addExtForce(*args, **kwargs) -> Any: ...

    def setExtForce(*args, **kwargs) -> Any: ...

    def clearExternalForces(*args, **kwargs) -> Any: ...

    def getOrCreateIK(*args, **kwargs) -> Any: ...

    def getIK(*args, **kwargs) -> Any: ...

    def __repr__(self) -> str: ...

    def createEndEffector(*args, **kwargs) -> Any: ...

    def createSimpleFrame(*args, **kwargs) -> Any: ...

    world = World

    def World(*args, **kwargs): ...

    add_ext_force = addExtForce

    clear_external_forces = clearExternalForces

    clear_ik = clearIK

    def clearIK(*args, **kwargs) -> Any: ...

    create_end_effector = createEndEffector

    create_shape_node = createShapeNode

    create_simple_frame = createSimpleFrame

    depends_on = dependsOn

    def dependsOn(*args, **kwargs) -> Any: ...

    descends_from = descendsFrom

    def descendsFrom(*args, **kwargs) -> Any: ...

    dirty_jacobian = dirtyJacobian

    def dirtyJacobian(*args, **kwargs) -> Any: ...

    dirty_jacobian_deriv = dirtyJacobianDeriv

    def dirtyJacobianDeriv(*args, **kwargs) -> Any: ...

    get_angular_jacobian = getAngularJacobian

    def getAngularJacobian(*args, **kwargs) -> Any: ...

    get_angular_jacobian_deriv = getAngularJacobianDeriv

    def getAngularJacobianDeriv(*args, **kwargs) -> Any: ...

    get_body_force = getBodyForce

    get_body_node_ptr = getBodyNodePtr

    get_chain_dofs = getChainDofs

    def getChainDofs(*args, **kwargs) -> Any: ...

    get_child_body_node = getChildBodyNode

    get_child_joint = getChildJoint

    get_dependent_gen_coord_index = getDependentGenCoordIndex

    def getDependentGenCoordIndex(*args, **kwargs) -> Any: ...

    get_end_effector = getEndEffector

    get_ik = getIK

    get_inertia = getInertia

    get_jacobian = getJacobian

    def getJacobian(*args, **kwargs) -> Any: ...

    get_jacobian_classic_deriv = getJacobianClassicDeriv

    def getJacobianClassicDeriv(*args, **kwargs) -> Any: ...

    get_jacobian_spatial_deriv = getJacobianSpatialDeriv

    def getJacobianSpatialDeriv(*args, **kwargs) -> Any: ...

    get_linear_jacobian = getLinearJacobian

    def getLinearJacobian(*args, **kwargs) -> Any: ...

    get_linear_jacobian_deriv = getLinearJacobianDeriv

    def getLinearJacobianDeriv(*args, **kwargs) -> Any: ...

    get_linear_velocity = getLinearVelocity

    get_name = getName

    get_num_dependent_dofs = getNumDependentDofs

    def getNumDependentDofs(*args, **kwargs) -> Any: ...

    get_num_dependent_gen_coords = getNumDependentGenCoords

    def getNumDependentGenCoords(*args, **kwargs) -> Any: ...

    get_num_end_effectors = getNumEndEffectors

    get_num_shape_nodes = getNumShapeNodes

    get_or_create_ik = getOrCreateIK

    get_parent_body_node = getParentBodyNode

    get_parent_frame = getParentFrame

    def getParentFrame(*args, **kwargs) -> Any: ...

    get_parent_joint = getParentJoint

    get_relative_transform = getRelativeTransform

    def getRelativeTransform(*args, **kwargs) -> Any: ...

    get_shape_nodes = getShapeNodes

    get_spatial_velocity = getSpatialVelocity

    get_transform = getTransform

    def getTransform(*args, **kwargs) -> Any: ...

    get_world_jacobian = getWorldJacobian

    def getWorldJacobian(*args, **kwargs) -> Any: ...

    get_world_transform = getWorldTransform

    def getWorldTransform(*args, **kwargs) -> Any: ...

    is_shape_frame = isShapeFrame

    def isShapeFrame(*args, **kwargs) -> Any: ...

    is_world = isWorld

    def isWorld(*args, **kwargs) -> Any: ...

    set_ext_force = setExtForce

    set_inertia = setInertia

class MetaSkeleton:
    def cloneMetaSkeleton(*args, **kwargs) -> Any: ...

    def setName(*args, **kwargs) -> Any: ...

    def getName(*args, **kwargs) -> Any: ...

    def getNumBodyNodes(*args, **kwargs) -> Any: ...

    def getBodyNode(*args, **kwargs) -> Any: ...

    def getBodyNodes(*args, **kwargs) -> Any: ...

    def getNumJoints(*args, **kwargs) -> Any: ...

    def getJoint(*args, **kwargs) -> Any: ...

    def getNumDofs(*args, **kwargs) -> Any: ...

    def getDof(*args, **kwargs) -> Any: ...

    def getDofs(*args, **kwargs) -> Any: ...

    clone_meta_skeleton = cloneMetaSkeleton

    get_body_node = getBodyNode

    get_body_nodes = getBodyNodes

    get_dof = getDof

    get_dofs = getDofs

    get_joint = getJoint

    get_name = getName

    get_num_body_nodes = getNumBodyNodes

    get_num_dofs = getNumDofs

    get_num_joints = getNumJoints

    set_name = setName

class Linkage(MetaSkeleton):
    @overload
    def __init__(self, criteria: LinkageCriteria) -> None: ...

    @overload
    def __init__(self, criteria: LinkageCriteria, name: str) -> None: ...

    def cloneLinkage(*args, **kwargs) -> Any: ...

    def cloneMetaSkeleton(*args, **kwargs) -> Any: ...

    def __repr__(self) -> str: ...

    def isAssembled(*args, **kwargs) -> Any: ...

    def reassemble(self) -> None: ...

    def satisfyCriteria(*args, **kwargs) -> Any: ...

    clone_linkage = cloneLinkage

    clone_meta_skeleton = cloneMetaSkeleton

    get_body_node = getBodyNode

    def getBodyNode(*args, **kwargs) -> Any: ...

    get_body_nodes = getBodyNodes

    def getBodyNodes(*args, **kwargs) -> Any: ...

    get_dof = getDof

    def getDof(*args, **kwargs) -> Any: ...

    get_dofs = getDofs

    def getDofs(*args, **kwargs) -> Any: ...

    get_joint = getJoint

    def getJoint(*args, **kwargs) -> Any: ...

    get_name = getName

    def getName(*args, **kwargs) -> Any: ...

    get_num_body_nodes = getNumBodyNodes

    def getNumBodyNodes(*args, **kwargs) -> Any: ...

    get_num_dofs = getNumDofs

    def getNumDofs(*args, **kwargs) -> Any: ...

    get_num_joints = getNumJoints

    def getNumJoints(*args, **kwargs) -> Any: ...

    is_assembled = isAssembled

    satisfy_criteria = satisfyCriteria

    set_name = setName

    def setName(*args, **kwargs) -> Any: ...

class LinkageCriteria:
    def __init__(self) -> None: ...

    def satisfy(self) -> list[BodyNode]: ...

    @property
    def mStart(self) -> LinkageCriteria.Target: ...

    @mStart.setter
    def mStart(self, arg: LinkageCriteria.Target, /) -> None: ...

    @property
    def mTargets(self) -> list[LinkageCriteria.Target]: ...

    @mTargets.setter
    def mTargets(self, arg: Sequence[LinkageCriteria.Target], /) -> None: ...

    @property
    def mTerminals(self) -> list[LinkageCriteria.Terminal]: ...

    @mTerminals.setter
    def mTerminals(self, arg: Sequence[LinkageCriteria.Terminal], /) -> None: ...

    class ExpansionPolicy(enum.Enum):
        INCLUDE = 0

        EXCLUDE = 1

        DOWNSTREAM = 2

        UPSTREAM = 3

    INCLUDE: LinkageCriteria.ExpansionPolicy = LinkageCriteria.ExpansionPolicy.INCLUDE

    EXCLUDE: LinkageCriteria.ExpansionPolicy = LinkageCriteria.ExpansionPolicy.EXCLUDE

    DOWNSTREAM: LinkageCriteria.ExpansionPolicy = LinkageCriteria.ExpansionPolicy.DOWNSTREAM

    UPSTREAM: LinkageCriteria.ExpansionPolicy = LinkageCriteria.ExpansionPolicy.UPSTREAM

    class Terminal:
        @overload
        def __init__(self) -> None: ...

        @overload
        def __init__(self, terminal: BodyNode) -> None: ...

        @overload
        def __init__(self, terminal: BodyNode, inclusive: bool) -> None: ...

        @property
        def mTerminal(self) -> BodyNode: ...

        @mTerminal.setter
        def mTerminal(self, terminal: object | None) -> None: ...

        @property
        def mInclusive(self) -> bool: ...

        @mInclusive.setter
        def mInclusive(self, arg: bool, /) -> None: ...

        @property
        def m_inclusive(self) -> bool: ...

        @m_inclusive.setter
        def m_inclusive(self, arg: bool, /) -> None: ...

        @property
        def m_terminal(self) -> BodyNode: ...

        @m_terminal.setter
        def m_terminal(self, terminal: object | None) -> None: ...

    class Target:
        @overload
        def __init__(self) -> None: ...

        @overload
        def __init__(self, target: BodyNode) -> None: ...

        @overload
        def __init__(self, target: BodyNode, policy: LinkageCriteria.ExpansionPolicy) -> None: ...

        @overload
        def __init__(self, target: BodyNode, policy: LinkageCriteria.ExpansionPolicy, chain: bool) -> None: ...

        @property
        def mNode(self) -> BodyNode: ...

        @mNode.setter
        def mNode(self, node: object | None) -> None: ...

        @property
        def mPolicy(self) -> LinkageCriteria.ExpansionPolicy: ...

        @mPolicy.setter
        def mPolicy(self, arg: LinkageCriteria.ExpansionPolicy, /) -> None: ...

        @property
        def mChain(self) -> bool: ...

        @mChain.setter
        def mChain(self, arg: bool, /) -> None: ...

        @property
        def m_chain(self) -> bool: ...

        @m_chain.setter
        def m_chain(self, arg: bool, /) -> None: ...

        @property
        def m_node(self) -> BodyNode: ...

        @m_node.setter
        def m_node(self, node: object | None) -> None: ...

        @property
        def m_policy(self) -> LinkageCriteria.ExpansionPolicy: ...

        @m_policy.setter
        def m_policy(self, arg: LinkageCriteria.ExpansionPolicy, /) -> None: ...

    @property
    def m_start(self) -> LinkageCriteria.Target: ...

    @m_start.setter
    def m_start(self, arg: LinkageCriteria.Target, /) -> None: ...

    @property
    def m_targets(self) -> list[LinkageCriteria.Target]: ...

    @m_targets.setter
    def m_targets(self, arg: Sequence[LinkageCriteria.Target], /) -> None: ...

    @property
    def m_terminals(self) -> list[LinkageCriteria.Terminal]: ...

    @m_terminals.setter
    def m_terminals(self, arg: Sequence[LinkageCriteria.Terminal], /) -> None: ...

class ChainCriteria:
    @overload
    def __init__(self, start: BodyNode, target: BodyNode) -> None: ...

    @overload
    def __init__(self, start: BodyNode, target: BodyNode, include_upstream_parent_joint: bool) -> None: ...

    def satisfy(self) -> list[BodyNode]: ...

    def convert(self) -> LinkageCriteria: ...

    @staticmethod
    def static_convert(criteria: LinkageCriteria) -> ChainCriteria: ...

    @property
    def mStart(self) -> BodyNode: ...

    @mStart.setter
    def mStart(self, start: object | None) -> None: ...

    @property
    def mTarget(self) -> BodyNode: ...

    @mTarget.setter
    def mTarget(self, target: object | None) -> None: ...

    @property
    def mIncludeUpstreamParentJoint(self) -> bool: ...

    @mIncludeUpstreamParentJoint.setter
    def mIncludeUpstreamParentJoint(self, arg: bool, /) -> None: ...

    @property
    def m_include_upstream_parent_joint(self) -> bool: ...

    @m_include_upstream_parent_joint.setter
    def m_include_upstream_parent_joint(self, arg: bool, /) -> None: ...

    @property
    def m_start(self) -> BodyNode: ...

    @m_start.setter
    def m_start(self, start: object | None) -> None: ...

    @property
    def m_target(self) -> BodyNode: ...

    @m_target.setter
    def m_target(self, target: object | None) -> None: ...

class Chain(Linkage):
    @overload
    def __init__(self, criteria: ChainCriteria) -> None: ...

    @overload
    def __init__(self, criteria: ChainCriteria, name: str) -> None: ...

    @overload
    def __init__(self, start: BodyNode, target: BodyNode, include_upstream_parent_joint: bool = ..., name: str = ...) -> None: ...

    def getNumBodyNodes(*args, **kwargs) -> Any: ...

    def cloneChain(*args, **kwargs) -> Any: ...

    def cloneMetaSkeleton(*args, **kwargs) -> Any: ...

    def isStillChain(*args, **kwargs) -> Any: ...

    def __repr__(self) -> str: ...

    clone_chain = cloneChain

    clone_linkage = cloneLinkage

    def cloneLinkage(*args, **kwargs) -> Any: ...

    clone_meta_skeleton = cloneMetaSkeleton

    get_body_node = getBodyNode

    def getBodyNode(*args, **kwargs) -> Any: ...

    get_body_nodes = getBodyNodes

    def getBodyNodes(*args, **kwargs) -> Any: ...

    get_dof = getDof

    def getDof(*args, **kwargs) -> Any: ...

    get_dofs = getDofs

    def getDofs(*args, **kwargs) -> Any: ...

    get_joint = getJoint

    def getJoint(*args, **kwargs) -> Any: ...

    get_name = getName

    def getName(*args, **kwargs) -> Any: ...

    get_num_body_nodes = getNumBodyNodes

    get_num_dofs = getNumDofs

    def getNumDofs(*args, **kwargs) -> Any: ...

    get_num_joints = getNumJoints

    def getNumJoints(*args, **kwargs) -> Any: ...

    is_assembled = isAssembled

    def isAssembled(*args, **kwargs) -> Any: ...

    is_still_chain = isStillChain

    satisfy_criteria = satisfyCriteria

    def satisfyCriteria(*args, **kwargs) -> Any: ...

    set_name = setName

    def setName(*args, **kwargs) -> Any: ...

class VisualAspect:
    def __init__(self) -> None: ...

    def setColor(*args, **kwargs) -> Any: ...

    def setRGBA(*args, **kwargs) -> Any: ...

    def getRGBA(*args, **kwargs) -> Any: ...

    def setHidden(*args, **kwargs) -> Any: ...

    def getHidden(*args, **kwargs) -> Any: ...

    def setShadowed(*args, **kwargs) -> Any: ...

    def getShadowed(*args, **kwargs) -> Any: ...

    def setMetallic(*args, **kwargs) -> Any: ...

    def getMetallic(*args, **kwargs) -> Any: ...

    def setRoughness(*args, **kwargs) -> Any: ...

    def getRoughness(*args, **kwargs) -> Any: ...

    def setReflectance(*args, **kwargs) -> Any: ...

    def getReflectance(*args, **kwargs) -> Any: ...

    get_hidden = getHidden

    get_metallic = getMetallic

    get_rgba = getRGBA

    get_reflectance = getReflectance

    get_roughness = getRoughness

    get_shadowed = getShadowed

    set_color = setColor

    set_hidden = setHidden

    set_metallic = setMetallic

    set_rgba = setRGBA

    set_reflectance = setReflectance

    set_roughness = setRoughness

    set_shadowed = setShadowed

class CollisionAspect:
    def __init__(self) -> None: ...

    def setCollidable(*args, **kwargs) -> Any: ...

    def getCollidable(*args, **kwargs) -> Any: ...

    get_collidable = getCollidable

    set_collidable = setCollidable

class DynamicsAspect:
    def __init__(self) -> None: ...

    def setFrictionCoeff(*args, **kwargs) -> Any: ...

    def getFrictionCoeff(*args, **kwargs) -> Any: ...

    get_friction_coeff = getFrictionCoeff

    set_friction_coeff = setFrictionCoeff

class ShapeFrame(Frame):
    def setShape(*args, **kwargs) -> Any: ...

    def getShape(*args, **kwargs) -> Any: ...

    def hasVisualAspect(*args, **kwargs) -> Any: ...

    def getVisualAspect(*args, **kwargs) -> Any: ...

    def createVisualAspect(*args, **kwargs) -> Any: ...

    def hasCollisionAspect(*args, **kwargs) -> Any: ...

    def getCollisionAspect(*args, **kwargs) -> Any: ...

    def createCollisionAspect(*args, **kwargs) -> Any: ...

    def hasDynamicsAspect(*args, **kwargs) -> Any: ...

    def getDynamicsAspect(*args, **kwargs) -> Any: ...

    def createDynamicsAspect(*args, **kwargs) -> Any: ...

    def isShapeNode(*args, **kwargs) -> Any: ...

    def __repr__(self) -> str: ...

    def asShapeNode(*args, **kwargs) -> Any: ...

    as_shape_node = asShapeNode

    create_collision_aspect = createCollisionAspect

    create_dynamics_aspect = createDynamicsAspect

    create_visual_aspect = createVisualAspect

    get_collision_aspect = getCollisionAspect

    get_dynamics_aspect = getDynamicsAspect

    get_shape = getShape

    get_visual_aspect = getVisualAspect

    has_collision_aspect = hasCollisionAspect

    has_dynamics_aspect = hasDynamicsAspect

    has_visual_aspect = hasVisualAspect

    is_shape_node = isShapeNode

    set_shape = setShape

class ShapeNode(ShapeFrame):
    def setRelativeTransform(*args, **kwargs) -> Any: ...

    def setRelativeTranslation(*args, **kwargs) -> Any: ...

    def getRelativeTranslation(*args, **kwargs) -> Any: ...

    def setRelativeRotation(*args, **kwargs) -> Any: ...

    def getRelativeRotation(*args, **kwargs) -> Any: ...

    def setOffset(*args, **kwargs) -> Any: ...

    def getOffset(*args, **kwargs) -> Any: ...

    def __repr__(self) -> str: ...

    get_offset = getOffset

    get_relative_rotation = getRelativeRotation

    get_relative_translation = getRelativeTranslation

    set_offset = setOffset

    set_relative_rotation = setRelativeRotation

    set_relative_transform = setRelativeTransform

    set_relative_translation = setRelativeTranslation

class SimpleFrame(ShapeFrame):
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, ref_frame: Frame) -> None: ...

    @overload
    def __init__(self, ref_frame: Frame, name: str) -> None: ...

    @overload
    def __init__(self, ref_frame: Frame, name: str, relative_transform: dartpy.math.Isometry3 = ...) -> None: ...

    def isShapeFrame(self) -> bool: ...

    def isShapeNode(self) -> bool: ...

    def isWorld(self) -> bool: ...

    def setName(*args, **kwargs) -> Any: ...

    def __repr__(self) -> str: ...

    def getName(*args, **kwargs) -> Any: ...

    def spawnChildSimpleFrame(*args, **kwargs) -> Any: ...

    def setShape(self, shape: Shape) -> None: ...

    @overload
    def getShape(self) -> Shape: ...

    @overload
    def getShape(self) -> Shape: ...

    def hasVisualAspect(self) -> bool: ...

    def getVisualAspect(self, create_if_null: bool = ...) -> VisualAspect: ...

    def createVisualAspect(self) -> VisualAspect: ...

    def hasCollisionAspect(self) -> bool: ...

    def getCollisionAspect(self, create_if_null: bool = ...) -> CollisionAspect: ...

    def createCollisionAspect(self) -> CollisionAspect: ...

    def hasDynamicsAspect(self) -> bool: ...

    def getDynamicsAspect(self, create_if_null: bool = ...) -> DynamicsAspect: ...

    def createDynamicsAspect(self) -> DynamicsAspect: ...

    def setRelativeTranslation(*args, **kwargs) -> Any: ...

    def setRelativeRotation(*args, **kwargs) -> Any: ...

    def setRelativeTransform(*args, **kwargs) -> Any: ...

    def setTranslation(*args, **kwargs) -> Any: ...

    def setTransform(*args, **kwargs) -> Any: ...

    @overload
    def getTransform(self) -> dartpy.math.Isometry3: ...

    @overload
    def getTransform(self, with_respect_to: Frame) -> dartpy.math.Isometry3: ...

    def getParentFrame(self) -> Frame: ...

    def descendsFrom(self, some_frame: Frame | None = ...) -> bool: ...

    get_name = getName

    set_name = setName

    set_relative_rotation = setRelativeRotation

    set_relative_transform = setRelativeTransform

    set_relative_translation = setRelativeTranslation

    set_transform = setTransform

    set_translation = setTranslation

    spawn_child_simple_frame = spawnChildSimpleFrame

class InverseKinematicsErrorMethod:
    def getMethodName(*args, **kwargs) -> Any: ...

    def getBounds(*args, **kwargs) -> Any: ...

    def setLinearBounds(*args, **kwargs) -> Any: ...

    def setAngularBounds(*args, **kwargs) -> Any: ...

    def setBounds(*args, **kwargs) -> Any: ...

    get_bounds = getBounds

    get_method_name = getMethodName

    set_angular_bounds = setAngularBounds

    set_bounds = setBounds

    set_linear_bounds = setLinearBounds

class InverseKinematicsGradientMethod:
    def setComponentWeights(*args, **kwargs) -> Any: ...

    def getComponentWeights(*args, **kwargs) -> Any: ...

    get_component_weights = getComponentWeights

    set_component_weights = setComponentWeights

class InverseKinematics:
    def isActive(*args, **kwargs) -> Any: ...

    def setActive(*args, **kwargs) -> Any: ...

    def getTarget(*args, **kwargs) -> Any: ...

    def useWholeBody(*args, **kwargs) -> Any: ...

    def setTarget(*args, **kwargs) -> Any: ...

    def getErrorMethod(*args, **kwargs) -> Any: ...

    def getSolver(*args, **kwargs) -> Any: ...

    def setSolver(*args, **kwargs) -> Any: ...

    def getProblem(*args, **kwargs) -> Any: ...

    def getGradientMethod(*args, **kwargs) -> Any: ...

    def solveAndApply(*args, **kwargs) -> Any: ...

    def __repr__(self) -> str: ...

    get_error_method = getErrorMethod

    get_gradient_method = getGradientMethod

    get_problem = getProblem

    get_solver = getSolver

    get_target = getTarget

    is_active = isActive

    set_active = setActive

    set_solver = setSolver

    set_target = setTarget

    solve_and_apply = solveAndApply

    use_whole_body = useWholeBody

def createSimpleFrame(*args, **kwargs): ...

class HierarchicalIK:
    def solveAndApply(*args, **kwargs) -> Any: ...

    solve_and_apply = solveAndApply

class WholeBodyIK(HierarchicalIK):
    def refreshIKHierarchy(*args, **kwargs) -> Any: ...

    refresh_ik_hierarchy = refreshIKHierarchy

class Skeleton(MetaSkeleton):
    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, name: str) -> None: ...

    def getNumBodyNodes(self) -> int: ...

    def getNumJoints(self) -> int: ...

    def getNumEndEffectors(*args, **kwargs) -> Any: ...

    def getNumDofs(self) -> int: ...

    @overload
    def getBodyNode(self, arg: int, /) -> BodyNode: ...

    @overload
    def getBodyNode(self, arg: str, /) -> BodyNode: ...

    def getRootBodyNode(*args, **kwargs) -> Any: ...

    @overload
    def getJoint(self, arg: int, /) -> Joint: ...

    @overload
    def getJoint(self, arg: str, /) -> Joint: ...

    def getRootJoint(*args, **kwargs) -> Any: ...

    @overload
    def getDof(self, index: int) -> DegreeOfFreedom: ...

    @overload
    def getDof(self, name: str) -> DegreeOfFreedom: ...

    def getPositions(*args, **kwargs) -> Any: ...

    def setPositions(*args, **kwargs) -> Any: ...

    def getVelocities(*args, **kwargs) -> Any: ...

    def setVelocities(*args, **kwargs) -> Any: ...

    def getForces(*args, **kwargs) -> Any: ...

    def resetPositions(*args, **kwargs) -> Any: ...

    def getMassMatrix(*args, **kwargs) -> Any: ...

    def getCoriolisAndGravityForces(*args, **kwargs) -> Any: ...

    def setForces(*args, **kwargs) -> Any: ...

    def getConstraintForces(*args, **kwargs) -> Any: ...

    def getCOM(*args, **kwargs) -> Any: ...

    def clearExternalForces(*args, **kwargs) -> Any: ...

    def enableSelfCollisionCheck(*args, **kwargs) -> Any: ...

    def disableSelfCollisionCheck(*args, **kwargs) -> Any: ...

    def isEnabledSelfCollisionCheck(*args, **kwargs) -> Any: ...

    def enableAdjacentBodyCheck(*args, **kwargs) -> Any: ...

    def disableAdjacentBodyCheck(*args, **kwargs) -> Any: ...

    def isEnabledAdjacentBodyCheck(*args, **kwargs) -> Any: ...

    def getIK(*args, **kwargs) -> Any: ...

    def __repr__(self) -> str: ...

    def createFreeJointAndBodyNodePair(*args, **kwargs) -> Any: ...

    def createBallJointAndBodyNodePair(*args, **kwargs) -> Any: ...

    def createEulerJointAndBodyNodePair(*args, **kwargs) -> Any: ...

    def createRevoluteJointAndBodyNodePair(*args, **kwargs) -> Any: ...

    def createPrismaticJointAndBodyNodePair(*args, **kwargs) -> Any: ...

    def createScrewJointAndBodyNodePair(*args, **kwargs) -> Any: ...

    def createPlanarJointAndBodyNodePair(*args, **kwargs) -> Any: ...

    def createUniversalJointAndBodyNodePair(*args, **kwargs) -> Any: ...

    def createTranslationalJointAndBodyNodePair(*args, **kwargs) -> Any: ...

    def createTranslationalJoint2DAndBodyNodePair(*args, **kwargs) -> Any: ...

    def createWeldJointAndBodyNodePair(*args, **kwargs) -> Any: ...

    def getEndEffector(*args, **kwargs) -> Any: ...

    clear_external_forces = clearExternalForces

    create_ball_joint_and_body_node_pair = createBallJointAndBodyNodePair

    create_euler_joint_and_body_node_pair = createEulerJointAndBodyNodePair

    create_free_joint_and_body_node_pair = createFreeJointAndBodyNodePair

    create_planar_joint_and_body_node_pair = createPlanarJointAndBodyNodePair

    create_prismatic_joint_and_body_node_pair = createPrismaticJointAndBodyNodePair

    create_revolute_joint_and_body_node_pair = createRevoluteJointAndBodyNodePair

    create_screw_joint_and_body_node_pair = createScrewJointAndBodyNodePair

    create_translational_joint2_d_and_body_node_pair = createTranslationalJoint2DAndBodyNodePair

    create_translational_joint_and_body_node_pair = createTranslationalJointAndBodyNodePair

    create_universal_joint_and_body_node_pair = createUniversalJointAndBodyNodePair

    create_weld_joint_and_body_node_pair = createWeldJointAndBodyNodePair

    disable_adjacent_body_check = disableAdjacentBodyCheck

    disable_self_collision_check = disableSelfCollisionCheck

    enable_adjacent_body_check = enableAdjacentBodyCheck

    enable_self_collision_check = enableSelfCollisionCheck

    get_com = getCOM

    get_constraint_forces = getConstraintForces

    get_coriolis_and_gravity_forces = getCoriolisAndGravityForces

    get_end_effector = getEndEffector

    get_forces = getForces

    get_ik = getIK

    get_mass_matrix = getMassMatrix

    get_num_end_effectors = getNumEndEffectors

    get_positions = getPositions

    get_root_body_node = getRootBodyNode

    get_root_joint = getRootJoint

    get_velocities = getVelocities

    is_enabled_adjacent_body_check = isEnabledAdjacentBodyCheck

    is_enabled_self_collision_check = isEnabledSelfCollisionCheck

    reset_positions = resetPositions

    set_forces = setForces

    set_positions = setPositions

    set_velocities = setVelocities

class Inertia:
    @overload
    def __init__(self, mass: float = ..., com: object | None = ..., moment_of_inertia: object | None = ...) -> None: ...

    @overload
    def __init__(self, spatial_inertia_tensor: Annotated[NDArray[numpy.float64], dict(shape=(6, 6), order='F')]) -> None: ...

    def setMass(*args, **kwargs) -> Any: ...

    def getMass(*args, **kwargs) -> Any: ...

    def setLocalCOM(*args, **kwargs) -> Any: ...

    def getLocalCOM(*args, **kwargs) -> Any: ...

    def setMoment(*args, **kwargs) -> Any: ...

    def getMoment(*args, **kwargs) -> Any: ...

    def setSpatialTensor(*args, **kwargs) -> Any: ...

    def getSpatialTensor(*args, **kwargs) -> Any: ...

    def transformed(self, transform: dartpy.math.Isometry3) -> Inertia: ...

    def transform(self, transform: dartpy.math.Isometry3) -> Inertia: ...

    def verify(self, print_warnings: bool = ..., tolerance: float = ...) -> bool: ...

    def __eq__(self, arg: Inertia, /) -> bool: ...

    def verifyMoment(*args, **kwargs): ...

    def verifySpatialTensor(*args, **kwargs): ...

    get_local_com = getLocalCOM

    get_mass = getMass

    get_moment = getMoment

    get_spatial_tensor = getSpatialTensor

    set_local_com = setLocalCOM

    set_mass = setMass

    set_moment = setMoment

    set_spatial_tensor = setSpatialTensor

    verify_moment = verifyMoment

    verify_spatial_tensor = verifySpatialTensor

create_simple_frame = createSimpleFrame
