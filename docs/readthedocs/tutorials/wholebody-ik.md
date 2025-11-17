# Whole-Body Inverse Kinematics

## C++ Walkthrough

### Overview

This tutorial shows how to build a whole-body inverse kinematics (IK) controller
for Atlas using the C++ API. You will:

- Load Atlas and configure a natural standing pose
- Create end effectors with offsets for intuitive targeting
- Configure the IK module with correct error bounds and solver settings
- Interact with the robot via the viewer or run a scripted headless loop
- Inspect joint-space solutions, pose errors, and solver diagnostics

Source files:

- Exercise: [`tutorials/tutorial_wholebody_ik/main.cpp`](https://github.com/dartsim/dart/blob/main/tutorials/tutorial_wholebody_ik/main.cpp)
- Finished: [`tutorials/tutorial_wholebody_ik_finished/main.cpp`](https://github.com/dartsim/dart/blob/main/tutorials/tutorial_wholebody_ik_finished/main.cpp)

### Running the Tutorial

```bash
# Build the tutorial targets
pixi run build

# Interactive viewer (unfinished/finished)
./build/default/cpp/Release/bin/tutorial_wholebody_ik
./build/default/cpp/Release/bin/tutorial_wholebody_ik_finished

# Headless scripted mode with diagnostics
./build/default/cpp/Release/bin/tutorial_wholebody_ik_finished \
    --headless --steps=200 --radius=0.10
```

### Lesson 1 – Load Atlas and Set the Pose

```cpp
SkeletonPtr loadAtlasRobot()
{
  DartLoader loader;
  SkeletonPtr atlas
      = loader.parseSkeleton("dart://sample/sdf/atlas/atlas_v3_no_head.urdf");
  if (!atlas)
    return nullptr;

  atlas->getDof("r_leg_hpy")->setPosition(-45.0 * constantsd::pi() / 180.0);
  atlas->getDof("r_leg_kny")->setPosition(90.0 * constantsd::pi() / 180.0);
  atlas->getDof("r_leg_aky")->setPosition(-45.0 * constantsd::pi() / 180.0);
  atlas->getDof("l_leg_hpy")->setPosition(-45.0 * constantsd::pi() / 180.0);
  atlas->getDof("l_leg_kny")->setPosition(90.0 * constantsd::pi() / 180.0);
  atlas->getDof("l_leg_aky")->setPosition(-45.0 * constantsd::pi() / 180.0);

  atlas->getDof("r_leg_kny")->setPositionLowerLimit(10.0 * constantsd::pi() / 180.0);
  atlas->getDof("l_leg_kny")->setPositionLowerLimit(10.0 * constantsd::pi() / 180.0);
  return atlas;
}
```

### Lesson 2 – Create End Effectors With Offsets

```cpp
EndEffector* createHandEndEffector(BodyNode* handBodyNode, const std::string& name)
{
  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  offset.translation() = Eigen::Vector3d(0.0, name == "l_hand" ? 0.12 : -0.12, 0.0);

  EndEffector* hand = handBodyNode->createEndEffector(name);
  hand->setDefaultRelativeTransform(offset, true);
  return hand;
}
```

### Lesson 3 – Configure IK With Proper Bounds

`TaskSpaceRegion` only produces non-zero error when the pose leaves the bounds,
so tight bounds are essential:

```cpp
void setupHandIK(EndEffector* hand)
{
  auto ik = hand->getIK(true);
  ik->getErrorMethod().setBounds(
      Eigen::Vector6d::Constant(-1e-8), Eigen::Vector6d::Constant(1e-8));
  ik->useWholeBody();
  ik->getSolver()->setNumMaxIterations(100);
  ik->getSolver()->setTolerance(1e-4);
  ik->setActive(true);
}
```

### Lesson 4 – Interactive Drag-and-Drop

```cpp
Viewer viewer;
viewer.addWorldNode(worldNode);
viewer.enableDragAndDrop(leftHand);
viewer.enableDragAndDrop(rightHand);
viewer.setUpViewInWindow(0, 0, 1280, 960);
viewer.run();
```

### Headless IK Control Loop

The finished tutorial accepts `--headless`, `--steps`, and `--radius` flags. It
creates `SimpleFrame` targets and repeatedly drives them through a trajectory:

```cpp
auto leftTarget = createHandTarget(leftHand);
auto rightTarget = createHandTarget(rightHand);
world->addSimpleFrame(leftTarget.mFrame);
world->addSimpleFrame(rightTarget.mFrame);

auto atlasIK = atlas->getIK(true);
atlasIK->setActive(true);
configureSmoothMotion(atlasIK.get());

for (std::size_t i = 0; i < steps; ++i) {
  updateTargetPose(leftTarget, leftOffset(i));
  updateTargetPose(rightTarget, rightOffset(i));
  const bool solved = atlasIK->solveAndApply(true);
  world->step();
  std::cout << "[headless] step " << i << " solved=" << solved << std::endl;
}
```

### Inspecting IK Solutions Programmatically

Capture joint vectors, pose errors, iteration counts, and solve times:

```cpp
Eigen::VectorXd q = atlas->getPositions();
Eigen::Vector3d leftError
    = leftTarget.mFrame->getTransform().translation()
      - leftHand->getWorldTransform().translation();

auto solver = atlasIK->getSolver();
auto* gradientSolver
    = dynamic_cast<dart::optimizer::GradientDescentSolver*>(solver.get());
std::size_t iterations
    = gradientSolver ? gradientSolver->getLastNumIterations() : 0;

const auto start = std::chrono::steady_clock::now();
bool solved = atlasIK->solveAndApply(true);
double solveMs
    = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - start)
          .count();
```

The finished headless loop prints these values each step so you can confirm that
the IK converged before feeding the joint vector to other systems.

### Improving IK Solution Quality

- **Gradient weights** – `configureSmoothMotion()` down-weights the floating
  base so the torso remains steady.
- **Component clamps** – `setComponentWiseClamp(0.15)` keeps large gradients
  from flipping joints when the target jumps.
- **Damping** – when using `InverseKinematics::JacobianDLS`, raising the damping
  coefficient smooths behavior near singularities.
- **Warm starts** – `solveAndApply(true)` updates the skeleton, so each solve
  starts from the previous pose; feeding smooth target trajectories yields smooth
  joint motion.

### Complete Example (GUI)

```cpp
int main()
{
  SkeletonPtr atlas = loadAtlasRobot();
  EndEffector* leftHand = createHandEndEffector(atlas->getBodyNode("l_hand"), "l_hand");
  EndEffector* rightHand = createHandEndEffector(atlas->getBodyNode("r_hand"), "r_hand");
  setupHandIK(leftHand);
  setupHandIK(rightHand);

  WorldPtr world = World::create();
  world->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));
  world->addSkeleton(atlas);

  Viewer viewer;
  viewer.addWorldNode(new WholeBodyIKWorldNode(world, atlas));
  viewer.enableDragAndDrop(leftHand);
  viewer.enableDragAndDrop(rightHand);
  viewer.run();
}
```

### Extending the Tutorial

- Add foot end effectors and support polygons for balancing
- Constrain the center of mass over the support polygon
- Bias the solution toward preferred joint angles
- Introduce obstacle avoidance and collision constraints
- Stack multiple IK objectives with hierarchical priorities

### Related Resources

- Example: `examples/atlas_puppet/`
- Tests: `tests/integration/dynamics/test_AtlasIK.cpp`
- Python walkthrough: [Jump to the dartpy section](#python-walkthrough)
- [DART IK Documentation](https://dartsim.github.io/)

## Python Walkthrough

### Overview

This tutorial mirrors the C++ whole-body IK walkthrough, but uses dartpy. You
will:

- Load Atlas and configure a natural standing pose
- Create end effectors for intuitive palm targets
- Configure the IK module with tight error bounds and solver settings
- Interact through the viewer or run a scripted headless loop
- Inspect joint-space results, pose errors, and solver diagnostics

Source files:

- Exercise: [`python/tutorials/wholebody_ik/main.py`](https://github.com/dartsim/dart/blob/main/python/tutorials/wholebody_ik/main.py)
- Finished: [`python/tutorials/wholebody_ik/main_finished.py`](https://github.com/dartsim/dart/blob/main/python/tutorials/wholebody_ik/main_finished.py)

### Running the Tutorial

```bash
# Build dartpy
pixi run build-py

# Guided version (contains TODOs)
python python/tutorials/wholebody_ik/main.py

# Finished version, GUI mode (default)
python python/tutorials/wholebody_ik/main_finished.py

# Finished version, headless diagnostics
python python/tutorials/wholebody_ik/main_finished.py --mode headless \
    --headless-steps 200 --trajectory-radius 0.10
```

### Lesson 1 – Load Atlas and Set the Pose

```python
def load_atlas_robot():
    loader = dart.utils.DartLoader()
    atlas = loader.parseSkeleton("dart://sample/sdf/atlas/atlas_v3_no_head.urdf")
    if not atlas:
        raise RuntimeError("Failed to load Atlas robot!")

    atlas.getDof("r_leg_hpy").setPosition(-45.0 * np.pi / 180.0)
    atlas.getDof("r_leg_kny").setPosition(90.0 * np.pi / 180.0)
    atlas.getDof("r_leg_aky").setPosition(-45.0 * np.pi / 180.0)
    atlas.getDof("l_leg_hpy").setPosition(-45.0 * np.pi / 180.0)
    atlas.getDof("l_leg_kny").setPosition(90.0 * np.pi / 180.0)
    atlas.getDof("l_leg_aky").setPosition(-45.0 * np.pi / 180.0)

    atlas.getDof("r_leg_kny").setPositionLowerLimit(10.0 * np.pi / 180.0)
    atlas.getDof("l_leg_kny").setPositionLowerLimit(10.0 * np.pi / 180.0)
    return atlas
```

### Lesson 2 – Create End Effectors With Offsets

```python
def create_hand_end_effector(hand_body_node, name):
    hand_offset = dart.math.Isometry3()
    y_offset = 0.12 if name == "l_hand" else -0.12
    hand_offset.set_translation([0.0, y_offset, 0.0])

    hand = hand_body_node.createEndEffector(name)
    hand.setDefaultRelativeTransform(hand_offset, True)
    return hand
```

Offsets move the IK target to the palm center rather than the wrist.

### Lesson 3 – Configure IK With Proper Bounds

`TaskSpaceRegion` only produces non-zero error when the pose escapes its bounds.
Using `±1e-8` bounds ensures every displacement yields a gradient.

```python
def setup_hand_ik(hand):
    ik = hand.getIK(True)
    ik.getErrorMethod().setLinearBounds(np.full(3, -1e-8), np.full(3, 1e-8))
    ik.getErrorMethod().setAngularBounds(np.full(3, -1e-8), np.full(3, 1e-8))
    ik.useWholeBody()

    solver = ik.getSolver()
    solver.setNumMaxIterations(100)
    solver.setTolerance(1e-4)
    ik.setActive(True)
```

### Lesson 4 – Interactive Drag-and-Drop

```python
viewer = dart.gui.osg.Viewer()
viewer.addWorldNode(world_node)
viewer.enableDragAndDrop(left_hand)
viewer.enableDragAndDrop(right_hand)
viewer.setUpViewInWindow(0, 0, 1280, 960)
viewer.setCameraHomePosition([3.0, 2.0, 2.0], [0.0, 0.5, 0.0], [0.0, 0.0, 1.0])
viewer.run()
```

Dragging either target updates the IK goal and the solver adjusts the whole body
in real time.

### Headless IK Control Loop

The `--mode headless` flag drives both hands through a scripted trajectory
without rendering:

```python
left_target = create_hand_target(left_hand)
right_target = create_hand_target(right_hand)
atlas_ik = atlas.getIK(True)
atlas_ik.setActive(True)

for idx in range(steps):
    phase = 2.0 * np.pi * idx / steps
    left_offset = np.array([radius * np.cos(phase), 0.0, radius * np.sin(phase)])
    right_offset = np.array([0.5 * radius * np.cos(phase),
                             0.5 * radius * np.sin(phase), 0.0])
    update_target_pose(left_target, left_offset)
    update_target_pose(right_target, right_offset)

    success = atlas_ik.solveAndApply(True)
    world.step()
    print("left", left_hand.getWorldTransform().translation(), "success", success)
```

Because `solveAndApply(True)` writes the solution into the skeleton, the next
iteration automatically warm-starts from the prior pose.

### Inspecting IK Solutions Programmatically

When feeding the solution into other systems, capture joint positions, pose
errors, and solver diagnostics:

```python
joint_positions = atlas.getPositions().copy()
left_error = np.linalg.norm(
    np.array(left_target.frame.getTransform().translation()).reshape(3)
    - np.array(left_hand.getWorldTransform().translation()).reshape(3)
)

solver = atlas_ik.getSolver()
iterations = solver.getLastNumIterations()
```

Wrap `solveAndApply(True)` with `time.perf_counter()` to obtain solve time. The
finished headless loop prints `q[0:6]`, error norms, iteration counts, and time
per step so you can confirm convergence before consuming the pose.

### Improving IK Solution Quality

- **Gradient weights** – down-weight the first six DOFs to keep the floating
  base steady (`gradient.setComponentWeights(...)`).
- **Component clamps** – `setComponentWiseClamp(0.15)` limits per-joint updates
  and avoids elbow flips when the target jumps.
- **Optional damping** – if the gradient method exposes
  `setDampingCoefficient`, increase it to smooth motion near singularities.
- **Target filtering** – feed the targets with splines or filtered data instead
  of discontinuous steps.

### Complete Example (GUI)

```python
def main():
    atlas = load_atlas_robot()
    left_hand = create_hand_end_effector(atlas.getBodyNode("l_hand"), "l_hand")
    right_hand = create_hand_end_effector(atlas.getBodyNode("r_hand"), "r_hand")
    setup_hand_ik(left_hand)
    setup_hand_ik(right_hand)

    world = dart.simulation.World()
    world.setGravity([0.0, -9.81, 0.0])
    world.addSkeleton(atlas)

    node = WholeBodyIKWorldNode(world, atlas)
    viewer = dart.gui.osg.Viewer()
    viewer.addWorldNode(node)
    viewer.enableDragAndDrop(left_hand)
    viewer.enableDragAndDrop(right_hand)
    viewer.run()
```

### Extending the Tutorial

- Add foot end effectors and support polygons for balance control
- Constrain the center of mass over the support polygon
- Bias the solution toward preferred joint angles
- Introduce obstacle avoidance or collision constraints
- Stack multiple IK objectives with hierarchical priorities

### Related Resources

- Example: `python/examples/atlas_puppet/`
- Tests: `python/tests/integration/test_atlas_ik.py`
- C++ walkthrough: [Jump to the native section](#c-walkthrough)
- [dartpy Installation Guide](/dartpy/user_guide/installation)
- [DART IK Documentation](https://dartsim.github.io/)
