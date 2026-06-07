from __future__ import annotations

__all__: list[str] = [
    "BallJointConstraint",
    "ConstraintBase",
    "CylindricalJointConstraint",
    "DynamicJointConstraint",
    "JointConstraint",
    "JointCoulombFrictionConstraint",
    "RevoluteJointConstraint",
    "WeldJointConstraint",
]


from typing import Any, overload

import dartpy.dynamics


class ConstraintBase:
    def getType(*args, **kwargs) -> Any: ...

    def getDimension(*args, **kwargs) -> Any: ...

    def update(self) -> None: ...

    get_dimension = getDimension

    get_type = getType

class DynamicJointConstraint(ConstraintBase):
    def setErrorAllowance(*args, **kwargs): ...

    def getErrorAllowance(*args, **kwargs): ...

    def setErrorReductionParameter(*args, **kwargs): ...

    def getErrorReductionParameter(*args, **kwargs): ...

    def setMaxErrorReductionVelocity(*args, **kwargs): ...

    def getMaxErrorReductionVelocity(*args, **kwargs): ...

    def setConstraintForceMixing(*args, **kwargs): ...

    def getConstraintForceMixing(*args, **kwargs): ...

    get_constraint_force_mixing = getConstraintForceMixing

    get_error_allowance = getErrorAllowance

    get_error_reduction_parameter = getErrorReductionParameter

    get_max_error_reduction_velocity = getMaxErrorReductionVelocity

    set_constraint_force_mixing = setConstraintForceMixing

    set_error_allowance = setErrorAllowance

    set_error_reduction_parameter = setErrorReductionParameter

    set_max_error_reduction_velocity = setMaxErrorReductionVelocity

class BallJointConstraint(DynamicJointConstraint):
    @overload
    def __init__(self, body_node: dartpy.dynamics.BodyNode, joint_position: object) -> None: ...

    @overload
    def __init__(self, body_node1: object, body_node2: object, joint_position: object) -> None: ...

    def getType(*args, **kwargs) -> Any: ...

    def getStaticType(*args, **kwargs): ...

    get_constraint_force_mixing = getConstraintForceMixing

    def getConstraintForceMixing(*args, **kwargs): ...

    get_dimension = getDimension

    def getDimension(*args, **kwargs) -> Any: ...

    get_error_allowance = getErrorAllowance

    def getErrorAllowance(*args, **kwargs): ...

    get_error_reduction_parameter = getErrorReductionParameter

    def getErrorReductionParameter(*args, **kwargs): ...

    get_max_error_reduction_velocity = getMaxErrorReductionVelocity

    def getMaxErrorReductionVelocity(*args, **kwargs): ...

    get_static_type = getStaticType

    get_type = getType

    set_constraint_force_mixing = setConstraintForceMixing

    def setConstraintForceMixing(*args, **kwargs): ...

    set_error_allowance = setErrorAllowance

    def setErrorAllowance(*args, **kwargs): ...

    set_error_reduction_parameter = setErrorReductionParameter

    def setErrorReductionParameter(*args, **kwargs): ...

    set_max_error_reduction_velocity = setMaxErrorReductionVelocity

    def setMaxErrorReductionVelocity(*args, **kwargs): ...

class CylindricalJointConstraint(DynamicJointConstraint):
    @overload
    def __init__(self, body_node: dartpy.dynamics.BodyNode, joint_position: object, axis: object) -> None: ...

    @overload
    def __init__(self, body_node1: object, body_node2: object, joint_position: object, axis1: object, axis2: object) -> None: ...

    def getType(self) -> str: ...

    def getStaticType(*args, **kwargs): ...

    get_static_type = getStaticType

class RevoluteJointConstraint(DynamicJointConstraint):
    @overload
    def __init__(self, body_node: dartpy.dynamics.BodyNode, joint_position: object, axis: object) -> None: ...

    @overload
    def __init__(self, body_node1: object, body_node2: object, joint_position: object, axis1: object, axis2: object) -> None: ...

    def getType(self) -> str: ...

    def getStaticType(*args, **kwargs): ...

    get_static_type = getStaticType

class WeldJointConstraint(DynamicJointConstraint):
    @overload
    def __init__(self, body_node: dartpy.dynamics.BodyNode) -> None: ...

    @overload
    def __init__(self, body_node1: dartpy.dynamics.BodyNode, body_node2: dartpy.dynamics.BodyNode) -> None: ...

    def setRelativeTransform(*args, **kwargs) -> Any: ...

    def getRelativeTransform(*args, **kwargs) -> Any: ...

    def getType(self) -> str: ...

    def getStaticType(*args, **kwargs): ...

    get_relative_transform = getRelativeTransform

    get_static_type = getStaticType

    set_relative_transform = setRelativeTransform

class JointConstraint(ConstraintBase):
    def __init__(self, joint: dartpy.dynamics.Joint) -> None: ...

    def setErrorAllowance(*args, **kwargs): ...

    def getErrorAllowance(*args, **kwargs): ...

    def setErrorReductionParameter(*args, **kwargs): ...

    def getErrorReductionParameter(*args, **kwargs): ...

    def setMaxErrorReductionVelocity(*args, **kwargs): ...

    def getMaxErrorReductionVelocity(*args, **kwargs): ...

    def setConstraintForceMixing(*args, **kwargs): ...

    def getConstraintForceMixing(*args, **kwargs): ...

    get_constraint_force_mixing = getConstraintForceMixing

    get_error_allowance = getErrorAllowance

    get_error_reduction_parameter = getErrorReductionParameter

    get_max_error_reduction_velocity = getMaxErrorReductionVelocity

    set_constraint_force_mixing = setConstraintForceMixing

    set_error_allowance = setErrorAllowance

    set_error_reduction_parameter = setErrorReductionParameter

    set_max_error_reduction_velocity = setMaxErrorReductionVelocity

class JointCoulombFrictionConstraint(ConstraintBase):
    def __init__(self, joint: dartpy.dynamics.Joint) -> None: ...

    def setConstraintForceMixing(*args, **kwargs): ...

    def getConstraintForceMixing(*args, **kwargs): ...

    get_constraint_force_mixing = getConstraintForceMixing

    set_constraint_force_mixing = setConstraintForceMixing
