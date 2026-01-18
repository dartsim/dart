# dart/dynamics/

Agent guidelines for the dynamics module.

## Overview

Core articulated body system: Skeleton, BodyNode, Joint, and Shape classes.

## Key Concepts

- **Skeleton**: Tree of BodyNodes connected by Joints; O(n) algorithms (ABA, RNEA, CRBA)
- **BodyNode**: Rigid body with mass, inertia, and shapes
- **Joint**: 10+ types (Revolute, Prismatic, Free, Ball, Weld, etc.)
- **Shape**: Geometry for collision/visual (Box, Sphere, Mesh, etc.)
- **Aspect System**: Runtime extensibility via State/Properties pattern

## Code Patterns

- Use `Skeleton::create()` factory, not constructors
- Use `createJointAndBodyNodePair<JointType>()` to add bodies
- Configuration space: generalized positions (q) and velocities (q̇)
- Lie group representations for joint transforms

## Testing

Unit tests: `tests/unit/dynamics/`
Integration tests: `tests/integration/test_Dynamics*.cpp`

## See Also

- @docs/onboarding/dynamics.md - Detailed dynamics documentation
- @docs/onboarding/aspect-system.md - Aspect/State/Properties pattern
- @docs/onboarding/architecture.md - Overall architecture
