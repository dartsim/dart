# Whole-Body Inverse Kinematics

## Overview

This tutorial demonstrates how to use whole-body inverse kinematics (IK) to control a humanoid robot's posture. The tutorial consists of four Lessons covering:

- Loading and configuring a humanoid robot (Atlas)
- Creating end effectors with proper offsets
- Configuring IK with correct error method bounds
- Enabling interactive drag-and-drop for IK targets

Please reference the source code in [**tutorial_wholebody_ik/main.cpp**](https://github.com/dartsim/dart/blob/main/tutorials/tutorial_wholebody_ik/main.cpp) and [**tutorial_wholebody_ik_finished/main.cpp**](https://github.com/dartsim/dart/blob/main/tutorials/tutorial_wholebody_ik_finished/main.cpp) for C++, or [**python/tutorials/wholebody_ik/main.py**](https://github.com/dartsim/dart/blob/main/python/tutorials/wholebody_ik/main.py) and [**python/tutorials/wholebody_ik/main_finished.py**](https://github.com/dartsim/dart/blob/main/python/tutorials/wholebody_ik/main_finished.py) for Python.

## What is Whole-Body IK?

Whole-body inverse kinematics allows you to control the pose of a complex robot by specifying desired positions for its end effectors (hands, feet, etc.). The IK solver automatically computes the joint angles needed to achieve those positions while considering:

- **Joint limits**: Respects physical constraints of the robot
- **Collision avoidance**: Prevents self-intersections
- **Multiple simultaneous constraints**: Handles multiple end effectors at once
- **Full body coordination**: Uses all relevant DOFs to achieve the goal

## Lesson 1: Load Robot and Set Standing Pose

Let's start by locating the `loadAtlasRobot` function. We load the Atlas humanoid robot from a URDF file and configure it into a standing pose.

### C++ Version

```cpp
SkeletonPtr loadAtlasRobot()
{
  DartLoader loader;
  SkeletonPtr atlas = loader.parseSkeleton(
      "dart://sample/sdf/atlas/atlas_v3_no_head.urdf");

  if (!atlas) {
    std::cerr << "Failed to load Atlas robot!" << std::endl;
    return nullptr;
  }

  // Set up initial standing pose
  atlas->getDof("r_leg_hpy")->setPosition(-45.0 * constantsd::pi() / 180.0);
  atlas->getDof("r_leg_kny")->setPosition(90.0 * constantsd::pi() / 180.0);
  atlas->getDof("r_leg_aky")->setPosition(-45.0 * constantsd::pi() / 180.0);
  atlas->getDof("l_leg_hpy")->setPosition(-45.0 * constantsd::pi() / 180.0);
  atlas->getDof("l_leg_kny")->setPosition(90.0 * constantsd::pi() / 180.0);
  atlas->getDof("l_leg_aky")->setPosition(-45.0 * constantsd::pi() / 180.0);

  // Prevent knees from bending backwards
  atlas->getDof("r_leg_kny")->setPositionLowerLimit(10.0 * constantsd::pi() / 180.0);
  atlas->getDof("l_leg_kny")->setPositionLowerLimit(10.0 * constantsd::pi() / 180.0);

  return atlas;
}
```

### Python Version

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

## Lesson 2: Create End Effectors with Offsets

An end effector represents a point of interest on the robot that we want to control. For the hands, we create end effectors with an offset that moves the control point to the palm center rather than the wrist.

### C++ Version

```cpp
EndEffector* createHandEndEffector(
    BodyNode* handBodyNode,
    const std::string& name)
{
  // Create an offset transformation
  Eigen::Isometry3d handOffset = Eigen::Isometry3d::Identity();
  handOffset.translation() = Eigen::Vector3d(
      0.0,
      (name == "l_hand" ? 0.12 : -0.12),
      0.0);

  EndEffector* hand = handBodyNode->createEndEffector(name);
  hand->setDefaultRelativeTransform(handOffset, true);

  return hand;
}
```

### Python Version

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

## Lesson 3: Configure IK with Proper Bounds

This is the **most critical** lesson. DART's `TaskSpaceRegion` error method uses a constraint-based formulation that only produces non-zero error when the displacement is **outside** the specified bounds.

### The Problem with Infinite Bounds

```cpp
// ❌ WRONG: This produces ZERO error!
ik->getErrorMethod().setBounds(
    Eigen::Vector6d::Constant(-std::numeric_limits<double>::infinity()),
    Eigen::Vector6d::Constant(std::numeric_limits<double>::infinity()));
```

With infinite bounds, any displacement is "within bounds", so the error is always zero. This means:
- Zero error → Zero gradient
- Zero gradient → Optimizer can't find direction to move
- No movement → IK fails!

### The Solution: Tight Bounds

```cpp
// ✅ CORRECT: This produces non-zero error!
ik->getErrorMethod().setBounds(
    Eigen::Vector6d::Constant(-1e-8),
    Eigen::Vector6d::Constant(1e-8));
```

With tight bounds (±1e-8), any displacement larger than 1e-8 is "outside bounds", producing a non-zero error that the optimizer can minimize.

### Complete Setup Function

**C++ Version:**

```cpp
void setupHandIK(EndEffector* hand)
{
  auto ik = hand->getIK(true);

  // CRITICAL: Set tight bounds
  ik->getErrorMethod().setBounds(
      Eigen::Vector6d::Constant(-1e-8),
      Eigen::Vector6d::Constant(1e-8));

  // Use whole-body IK
  ik->useWholeBody();

  // Configure solver
  ik->getSolver()->setNumMaxIterations(100);
  ik->getSolver()->setTolerance(1e-4);

  // Activate IK
  ik->setActive(true);
}
```

**Python Version:**

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

## Lesson 4: Enable Interactive Drag-and-Drop

DART provides a convenient drag-and-drop feature that allows you to interactively move end effectors. When you drag an end effector, the IK solver automatically adjusts the robot's posture to reach the new position.

### C++ Version

```cpp
int main()
{
  // ... previous setup code ...

  Viewer viewer;
  viewer.addWorldNode(worldNode);

  // Enable drag-and-drop for both hands
  viewer.enableDragAndDrop(leftHand);
  viewer.enableDragAndDrop(rightHand);

  viewer.run();
}
```

### Python Version

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

## Running the Tutorial

### C++ Version

```bash
# Build the tutorial
pixi run build

# Run the unfinished version (won't work until you complete the TODOs)
./build/default/cpp/Release/bin/tutorial_wholebody_ik

# Run the finished version
./build/default/cpp/Release/bin/tutorial_wholebody_ik_finished
```

### Python Version

```bash
# Run the unfinished version (won't work until you complete the TODOs)
python python/tutorials/wholebody_ik/main.py

# Run the finished version
python python/tutorials/wholebody_ik/main_finished.py
```

## Key Takeaways

1. **End effectors** represent points of interest that you want to control with IK
2. **Offsets** allow you to place end effectors at convenient locations (like palm center vs. wrist)
3. **Tight error bounds** (±1e-8) are **critical** for position tracking with `TaskSpaceRegion`
4. **Whole-body IK** uses all relevant DOFs to achieve the goal
5. **Drag-and-drop** provides intuitive interactive control

## Extending the Tutorial

Try adding:

- **Foot end effectors**: Control foot positions for balancing
- **COM constraints**: Keep the center of mass over the support polygon
- **Joint angle preferences**: Bias the solution toward more natural configurations
- **Obstacle avoidance**: Add collision objects and constraints
- **Hierarchical priorities**: Use multiple priority levels for different objectives

## Related Examples and Tests

- **C++ Example**: `examples/atlas_puppet/` - Interactive humanoid control
- **C++ Tests**: `tests/integration/dynamics/test_AtlasIK.cpp` - Unit tests demonstrating the bug and fix
- **Python Tests**: `python/tests/integration/test_atlas_ik.py` - Integration tests for Atlas IK

## Further Reading

- [DART IK Documentation](https://dartsim.github.io/)
- [TaskSpaceRegion API](https://dartsim.github.io/dart/classdart_1_1dynamics_1_1_task_space_region.html)
- [EndEffector API](https://dartsim.github.io/dart/classdart_1_1dynamics_1_1_end_effector.html)
