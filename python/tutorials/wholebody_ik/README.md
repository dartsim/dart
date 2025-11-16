# Whole-Body Inverse Kinematics Tutorial (Python)

This tutorial demonstrates how to use whole-body inverse kinematics (IK) to control a humanoid robot's posture using DART's Python bindings (dartpy).

## Learning Objectives

1. Load and configure a complex humanoid robot (Atlas) in Python
2. Create end effectors for robot limbs (hands, feet)
3. Set up IK targets and constraints
4. Use whole-body IK to achieve desired poses
5. Understand the importance of proper error method bounds

## Key Concepts

### What is Whole-Body IK?

Whole-body IK allows you to control the pose of a complex robot by specifying desired positions for its end effectors (hands, feet, etc.). The IK solver automatically computes the joint angles needed to achieve those positions while considering:

- Joint limits
- Collision avoidance
- Multiple simultaneous constraints
- Full body coordination

### Critical: Error Method Bounds

The tutorial highlights an important detail about DART's IK system:

**You must set tight bounds on the error method for position tracking!**

```python
ik.getErrorMethod().setLinearBounds(
    np.array([-1e-8, -1e-8, -1e-8]),
    np.array([1e-8, 1e-8, 1e-8])
)
ik.getErrorMethod().setAngularBounds(
    np.array([-1e-8, -1e-8, -1e-8]),
    np.array([1e-8, 1e-8, 1e-8])
)
```

Why? The `TaskSpaceRegion` error method only computes non-zero error when the displacement is *outside* the specified bounds. With infinite bounds (the default), any displacement is "within bounds", producing zero error and zero gradient. This prevents the optimizer from finding the solution.

## Interactive Features

- **Drag and drop**: Click and drag the colored spheres at the end effectors to move them
- **Real-time IK**: Watch the robot adjust its whole-body posture to reach the targets

## Running the Tutorial

```bash
# Make sure dartpy is installed
pixi run build-py

# Run the tutorial
python python/tutorials/wholebody_ik/main.py
```

## What You'll See

When you run the tutorial:

1. The Atlas robot loads in a standing pose
2. Colored spheres appear at the hand positions (end effectors)
3. You can drag these spheres to new positions
4. The robot's entire body adjusts to reach the new hand positions
5. Joint limits and constraints are automatically respected

## Code Structure

The tutorial is organized into clear functions:

- `load_atlas_robot()` - Loads the Atlas URDF and sets initial pose
- `create_hand_end_effector()` - Creates end effector with proper offset
- `setup_hand_ik()` - Configures IK solver with tight bounds
- `WholeBodyIKWorldNode` - Custom world node for simulation updates

## Extending the Tutorial

Try adding:

- Foot end effectors for balancing
- COM (Center of Mass) constraints
- Joint angle preferences
- Obstacle avoidance
- Multiple priority levels (hierarchical IK)
- Custom target movements (e.g., circular motion)

## Differences from C++ Version

The Python version uses dartpy bindings which have slightly different APIs:

- `dart.math.Isometry3()` instead of `Eigen::Isometry3d`
- `set_translation()` instead of `.translation() =`
- NumPy arrays instead of Eigen vectors
- Similar structure but Pythonic naming conventions

## Related Examples

- C++ version: `tutorials/tutorial_wholebody_ik/`
- Interactive example: `examples/atlas_puppet`
- Test suite: `python/tests/integration/test_atlas_ik.py`

## References

- dartpy Documentation: https://dartsim.github.io/
- DART IK Documentation: https://dartsim.github.io/
- Atlas Robot: https://www.bostondynamics.com/atlas
