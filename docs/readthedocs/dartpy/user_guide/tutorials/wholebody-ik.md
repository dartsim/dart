# Whole-Body Inverse Kinematics (Python)

## Overview

This tutorial demonstrates how to use whole-body inverse kinematics (IK) to control a humanoid robot's posture using dartpy (DART's Python bindings). The tutorial consists of four Lessons covering:

- Loading and configuring a humanoid robot (Atlas)
- Creating end effectors with proper offsets
- Configuring IK with correct error method bounds
- Enabling interactive drag-and-drop for IK targets

Please reference the source code in [**python/tutorials/wholebody_ik/main.py**](https://github.com/dartsim/dart/blob/main/python/tutorials/wholebody_ik/main.py) and [**python/tutorials/wholebody_ik/main_finished.py**](https://github.com/dartsim/dart/blob/main/python/tutorials/wholebody_ik/main_finished.py).

## What is Whole-Body IK?

Whole-body inverse kinematics allows you to control the pose of a complex robot by specifying desired positions for its end effectors (hands, feet, etc.). The IK solver automatically computes the joint angles needed to achieve those positions while considering:

- **Joint limits**: Respects physical constraints of the robot
- **Collision avoidance**: Prevents self-intersections
- **Multiple simultaneous constraints**: Handles multiple end effectors at once
- **Full body coordination**: Uses all relevant DOFs to achieve the goal

## Lesson 1: Load Robot and Set Standing Pose

Let's start by locating the `load_atlas_robot` function. We load the Atlas humanoid robot from a URDF file and configure it into a standing pose.

```python
def load_atlas_robot():
    loader = dart.utils.DartLoader()
    atlas = loader.parseSkeleton("dart://sample/sdf/atlas/atlas_v3_no_head.urdf")

    if not atlas:
        raise RuntimeError("Failed to load Atlas robot!")

    # Set up initial standing pose
    atlas.getDof("r_leg_hpy").setPosition(-45.0 * np.pi / 180.0)
    atlas.getDof("r_leg_kny").setPosition(90.0 * np.pi / 180.0)
    atlas.getDof("r_leg_aky").setPosition(-45.0 * np.pi / 180.0)
    atlas.getDof("l_leg_hpy").setPosition(-45.0 * np.pi / 180.0)
    atlas.getDof("l_leg_kny").setPosition(90.0 * np.pi / 180.0)
    atlas.getDof("l_leg_aky").setPosition(-45.0 * np.pi / 180.0)

    # Prevent knees from bending backwards
    atlas.getDof("r_leg_kny").setPositionLowerLimit(10.0 * np.pi / 180.0)
    atlas.getDof("l_leg_kny").setPositionLowerLimit(10.0 * np.pi / 180.0)

    return atlas
```

The standing pose bends the hips and knees to create a natural standing configuration. We also set joint limits on the knees to prevent them from bending backward, which would be physically unrealistic.

### Key dartpy Concepts

- **`dart.utils.DartLoader()`**: Creates a loader for robot models
- **`loader.parseSkeleton(uri)`**: Loads a robot from a URDF/SDF file
- **`atlas.getDof(name)`**: Gets a degree of freedom by name
- **`setPosition(angle)`**: Sets the joint angle in radians
- **`setPositionLowerLimit(angle)`**: Sets joint lower limit

## Lesson 2: Create End Effectors with Offsets

An end effector represents a point of interest on the robot that we want to control. For the hands, we create end effectors with an offset that moves the control point to the palm center rather than the wrist.

```python
def create_hand_end_effector(hand_body_node, name):
    # Create transformation for the end effector
    hand_offset = dart.math.Isometry3()
    y_offset = 0.12 if name == "l_hand" else -0.12
    hand_offset.set_translation([0.0, y_offset, 0.0])

    hand = hand_body_node.createEndEffector(name)
    hand.setDefaultRelativeTransform(hand_offset, True)

    return hand
```

The offset is critical because it determines where the IK target will be. Without the offset, the target would be at the wrist joint, which is less intuitive than the palm center.

### Key dartpy Concepts

- **`dart.math.Isometry3()`**: Creates a 3D transformation (rotation + translation)
- **`set_translation([x, y, z])`**: Sets the translation component (NumPy array or list)
- **`createEndEffector(name)`**: Creates an end effector on a body node
- **`setDefaultRelativeTransform(transform, relative)`**: Sets the offset from the body node

## Lesson 3: Configure IK with Proper Bounds

This is the **most critical** lesson. DART's `TaskSpaceRegion` error method uses a constraint-based formulation that only produces non-zero error when the displacement is **outside** the specified bounds.

### The Problem with Infinite Bounds

```python
# ❌ WRONG: This produces ZERO error!
ik.getErrorMethod().setLinearBounds(
    np.array([-np.inf, -np.inf, -np.inf]),
    np.array([np.inf, np.inf, np.inf])
)
ik.getErrorMethod().setAngularBounds(
    np.array([-np.inf, -np.inf, -np.inf]),
    np.array([np.inf, np.inf, np.inf])
)
```

With infinite bounds, any displacement is "within bounds", so the error is always zero. This means:
- Zero error → Zero gradient
- Zero gradient → Optimizer can't find direction to move
- No movement → IK fails!

### The Solution: Tight Bounds

```python
# ✅ CORRECT: This produces non-zero error!
ik.getErrorMethod().setLinearBounds(
    np.array([-1e-8, -1e-8, -1e-8]),
    np.array([1e-8, 1e-8, 1e-8])
)
ik.getErrorMethod().setAngularBounds(
    np.array([-1e-8, -1e-8, -1e-8]),
    np.array([1e-8, 1e-8, 1e-8])
)
```

With tight bounds (±1e-8), any displacement larger than 1e-8 is "outside bounds", producing a non-zero error that the optimizer can minimize.

### Complete Setup Function

```python
def setup_hand_ik(hand):
    ik = hand.getIK(True)

    # CRITICAL: Set tight bounds
    ik.getErrorMethod().setLinearBounds(
        np.array([-1e-8, -1e-8, -1e-8]),
        np.array([1e-8, 1e-8, 1e-8])
    )
    ik.getErrorMethod().setAngularBounds(
        np.array([-1e-8, -1e-8, -1e-8]),
        np.array([1e-8, 1e-8, 1e-8])
    )

    # Use whole-body IK
    ik.useWholeBody()

    # Configure solver
    solver = ik.getSolver()
    solver.setNumMaxIterations(100)
    solver.setTolerance(1e-4)

    # Activate IK
    ik.setActive(True)
```

### Key dartpy Concepts

- **`hand.getIK(create_if_null)`**: Gets or creates IK module for end effector
- **`getErrorMethod()`**: Gets the error computation method
- **`setLinearBounds(lower, upper)`**: Sets bounds for position error (NumPy arrays)
- **`setAngularBounds(lower, upper)`**: Sets bounds for orientation error (NumPy arrays)
- **`useWholeBody()`**: Enables using all dependent DOFs
- **`getSolver()`**: Gets the optimization solver
- **`setActive(True)`**: Activates the IK module

## Lesson 4: Enable Interactive Drag-and-Drop

dartpy provides a convenient drag-and-drop feature that allows you to interactively move end effectors. When you drag an end effector, the IK solver automatically adjusts the robot's posture to reach the new position.

```python
def main():
    # ... previous setup code ...

    viewer = dart.gui.osg.Viewer()
    viewer.addWorldNode(world_node)

    # Enable drag-and-drop for both hands
    viewer.enableDragAndDrop(left_hand)
    viewer.enableDragAndDrop(right_hand)

    viewer.run()
```

When you run the tutorial, you'll see colored spheres at the hand positions. Click and drag these spheres to move the hands, and watch the entire robot adjust its posture to reach the new positions!

### Key dartpy Concepts

- **`dart.gui.osg.Viewer()`**: Creates an OpenSceneGraph viewer
- **`addWorldNode(world_node)`**: Adds the simulation world to the viewer
- **`enableDragAndDrop(end_effector)`**: Enables interactive dragging for an end effector
- **`run()`**: Starts the interactive viewer loop

## Running the Tutorial

```bash
# Make sure dartpy is built and installed
pixi run build-py

# Run the unfinished version (won't work until you complete the TODOs)
python python/tutorials/wholebody_ik/main.py

# Run the finished version
python python/tutorials/wholebody_ik/main_finished.py
```

## Complete Example

Here's the complete main function that ties everything together:

```python
def main():
    # Load the Atlas humanoid robot
    atlas = load_atlas_robot()

    print("=" * 50)
    print("  Whole-Body IK Tutorial")
    print("=" * 50)
    print(f"Loaded robot: {atlas.getName()}")
    print(f"Number of DOFs: {atlas.getNumDofs()}")

    # Create end effectors for both hands
    left_hand_body = atlas.getBodyNode("l_hand")
    right_hand_body = atlas.getBodyNode("r_hand")

    left_hand = create_hand_end_effector(left_hand_body, "l_hand")
    right_hand = create_hand_end_effector(right_hand_body, "r_hand")

    # Set up IK for both hands
    setup_hand_ik(left_hand)
    setup_hand_ik(right_hand)

    # Create the world and add the robot
    world = dart.simulation.World()
    world.setGravity([0.0, -9.81, 0.0])
    world.addSkeleton(atlas)

    # Create the viewer
    world_node = WholeBodyIKWorldNode(world, atlas)
    viewer = dart.gui.osg.Viewer()
    viewer.addWorldNode(world_node)

    # Enable drag-and-drop
    viewer.enableDragAndDrop(left_hand)
    viewer.enableDragAndDrop(right_hand)

    # Set up viewer
    viewer.setUpViewInWindow(0, 0, 1280, 960)
    viewer.setCameraHomePosition(
        [3.0, 2.0, 2.0], [0.0, 0.5, 0.0], [0.0, 0.0, 1.0]
    )

    # Run!
    viewer.run()
```

## Key Takeaways

1. **End effectors** represent points of interest that you want to control with IK
2. **Offsets** allow you to place end effectors at convenient locations (like palm center vs. wrist)
3. **Tight error bounds** (±1e-8) are **CRITICAL** for position tracking with `TaskSpaceRegion`
4. **Whole-body IK** uses all relevant DOFs to achieve the goal
5. **Drag-and-drop** provides intuitive interactive control

## Extending the Tutorial

Try adding:

- **Foot end effectors**: Control foot positions for balancing
- **COM constraints**: Keep the center of mass over the support polygon
- **Joint angle preferences**: Bias the solution toward more natural configurations
- **Obstacle avoidance**: Add collision objects and constraints
- **Hierarchical priorities**: Use multiple priority levels for different objectives
- **Programmatic target movement**: Move targets in code (e.g., circular motion)

## Related Examples and Tests

- **Python Example**: `python/examples/atlas_puppet/` - Interactive humanoid control
- **Python Tests**: `python/tests/integration/test_atlas_ik.py` - Integration tests for Atlas IK
- **C++ Version**: See the C++ tutorial for parallel implementation

## Further Reading

- [dartpy Installation](/dartpy/user_guide/installation)
- [dartpy Examples](/dartpy/user_guide/examples)
- [DART IK Documentation](https://dartsim.github.io/)
- [C++ Tutorials](/dart/user_guide/tutorials) - For understanding DART concepts
