---
description: DART physics simulation specialist
---

# DART Physics Simulation Skill

## Purpose

Specialized skill for DART (Dynamic Animation and Robotics Toolkit) physics simulation tasks including dynamics optimization, collision detection, and integration with Gazebo.

## MCP Configuration

```yaml
mcp:
  dart-docs:
    command: python3
    args: ["-m", "http.server", "8000"]
    working_dir: "/home/js/dev/dartsim/dart/refactor/docs/_build/html"
    description: "Local DART documentation server"
```

## When to Use This Skill

### Performance Optimization

- Dynamics algorithm optimization (O(n) algorithms)
- Collision detection performance tuning
- Memory management for large simulations
- Multi-threading for physics computations

### Gazebo Integration

- Physics validation with `test-gz`
- Gazebo plugin development
- URDF/SDF compatibility issues
- Physics breaking change detection

### Advanced Dynamics

- Custom constraint implementation
- Contact force modeling
- Integration scheme modification
- Featherstone algorithm extensions

## Key Commands

```bash
# Performance profiling
pixi run -e debug build --config Release
pixi run benchmark --filter dynamics

# Physics validation
pixi run -e gazebo test-gz
pixi run test --filter integration/physics

# Documentation access
pixi run docs --open
```

## DART Physics Components

### Core Dynamics (`dart/dynamics/`)

- **Skeleton**: Articulated body representation
- **DegreeOfFreedom**: Joint state management
- **BodyNode**: Rigid body dynamics
- **Joint**: Various joint types (revolute, prismatic, etc.)

### Collision Detection (`dart/collision/`)

- **CollisionDetector**: Broad-phase and narrow-phase
- **CollisionObject**: Geometric collision shapes
- **Contact**: Contact point and force information

### Constraints (`dart/constraint/`)

- **ConstraintSolver**: Constraint resolution
- **JointConstraint**: Joint limit enforcement
- **ContactConstraint**: Contact friction and collision

## Success Criteria

- Physics benchmarks maintain or improve performance
- Gazebo integration tests pass
- Memory usage stays within acceptable bounds
- No regression in simulation accuracy

## Limitations

- Cannot modify core Featherstone algorithms without deep expertise
- Template metaprogramming requires careful validation
- Performance changes require comprehensive benchmarking

## Related Documentation

- `docs/onboarding/architecture/dynamics.md`
- `docs/onboarding/performance/optimization.md`
- `docs/onboarding/gazebo-integration.md`
