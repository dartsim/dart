# dart/constraint/

Agent guidelines for the constraint module.

## Overview

LCP-based constraint solving for contacts, joint limits, and motors.

## Constraint Types

| Type        | Class                  | Purpose                                 |
| ----------- | ---------------------- | --------------------------------------- |
| Contact     | `ContactConstraint`    | Contact forces between bodies           |
| Joint Limit | `JointLimitConstraint` | Position/velocity limits                |
| Servo Motor | `ServoMotorConstraint` | Joint servo control                     |
| Mimic Motor | `MimicMotorConstraint` | Mimic joint behavior                    |
| Coupler     | `CouplerConstraint`    | Bilateral coupling (equal-and-opposite) |

## Key Concepts

- **ConstraintSolver**: Manages all active constraints
- **LcpSolver**: Dantzig (primary) or PGS (fallback)
- **Constraint equation**: Compute impulses to satisfy constraints

## Code Patterns

- Constraints auto-created from contacts and joint limits
- Custom constraints: inherit `ConstraintBase`
- Skeleton grouping for independent solving

## Testing

Unit tests: `tests/unit/constraint/`
Integration tests: `tests/integration/test_Constraint*.cpp`

## See Also

- @docs/onboarding/constraints.md - Detailed constraint documentation
- @docs/background/lcp/ - LCP solver theory and methods
- @dart/collision/AGENTS.md - Contact generation
