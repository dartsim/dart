/*
 * Copyright (c) 2011-2025, The DART development contributors
 * All rights reserved.
 *
 * This file provides forward declarations and smart pointer aliases for
 * constraint classes.
 */

#pragma once

#include <memory>

namespace dart {
namespace constraint {

class BoxedLcpConstraintSolver;
class BoxedLcpSolver;
class ConstraintSolver;
class ConstrainedGroup;
class ConstraintBase;
class ContactSurfaceHandler;
class ContactConstraint;
class SoftContactConstraint;
class JointConstraint;
class MimicMotorConstraint;
class CouplerConstraint;
class JointCoulombFrictionConstraint;
class DynamicJointConstraint;
class BallJointConstraint;
class RevoluteJointConstraint;
class WeldJointConstraint;
class BalanceConstraint;

using BoxedLcpConstraintSolverPtr = std::shared_ptr<BoxedLcpConstraintSolver>;
using ConstBoxedLcpConstraintSolverPtr
    = std::shared_ptr<const BoxedLcpConstraintSolver>;
using WeakBoxedLcpConstraintSolverPtr = std::weak_ptr<BoxedLcpConstraintSolver>;
using WeakConstBoxedLcpConstraintSolverPtr
    = std::weak_ptr<const BoxedLcpConstraintSolver>;
using UniqueBoxedLcpConstraintSolverPtr
    = std::unique_ptr<BoxedLcpConstraintSolver>;

using BoxedLcpSolverPtr = std::shared_ptr<BoxedLcpSolver>;
using ConstBoxedLcpSolverPtr = std::shared_ptr<const BoxedLcpSolver>;
using WeakBoxedLcpSolverPtr = std::weak_ptr<BoxedLcpSolver>;
using WeakConstBoxedLcpSolverPtr = std::weak_ptr<const BoxedLcpSolver>;

using ConstraintSolverPtr = std::shared_ptr<ConstraintSolver>;
using ConstConstraintSolverPtr = std::shared_ptr<const ConstraintSolver>;
using WeakConstraintSolverPtr = std::weak_ptr<ConstraintSolver>;
using WeakConstConstraintSolverPtr = std::weak_ptr<const ConstraintSolver>;
using UniqueConstraintSolverPtr = std::unique_ptr<ConstraintSolver>;

using ConstrainedGroupPtr = std::shared_ptr<ConstrainedGroup>;
using ConstConstrainedGroupPtr = std::shared_ptr<const ConstrainedGroup>;
using WeakConstrainedGroupPtr = std::weak_ptr<ConstrainedGroup>;
using WeakConstConstrainedGroupPtr = std::weak_ptr<const ConstrainedGroup>;

using ConstraintBasePtr = std::shared_ptr<ConstraintBase>;
using ConstConstraintBasePtr = std::shared_ptr<const ConstraintBase>;
using WeakConstraintBasePtr = std::weak_ptr<ConstraintBase>;
using WeakConstConstraintBasePtr = std::weak_ptr<const ConstraintBase>;

using ContactSurfaceHandlerPtr = std::shared_ptr<ContactSurfaceHandler>;
using ContactSurfaceHandlerConstPtr
    = std::shared_ptr<const ContactSurfaceHandler>;

using ContactConstraintPtr = std::shared_ptr<ContactConstraint>;
using SoftContactConstraintPtr = std::shared_ptr<SoftContactConstraint>;
using JointConstraintPtr = std::shared_ptr<JointConstraint>;
using MimicMotorConstraintPtr = std::shared_ptr<MimicMotorConstraint>;
using CouplerConstraintPtr = std::shared_ptr<CouplerConstraint>;
using JointCoulombFrictionConstraintPtr
    = std::shared_ptr<JointCoulombFrictionConstraint>;

using DynamicJointConstraintPtr = std::shared_ptr<DynamicJointConstraint>;
using BallJointConstraintPtr = std::shared_ptr<BallJointConstraint>;
using RevoluteJointConstraintPtr = std::shared_ptr<RevoluteJointConstraint>;
using WeldJointConstraintPtr = std::shared_ptr<WeldJointConstraint>;
using BalanceConstraintPtr = std::shared_ptr<BalanceConstraint>;

} // namespace constraint
} // namespace dart
