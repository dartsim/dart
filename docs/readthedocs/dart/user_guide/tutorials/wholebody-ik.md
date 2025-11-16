# Whole-Body Inverse Kinematics (C++)

## Overview

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

## Running the Tutorial

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

## Lesson 1 – Load Atlas and Set the Pose

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

## Lesson 2 – Create End Effectors With Offsets

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

## Lesson 3 – Configure IK With Proper Bounds

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

## Lesson 4 – Interactive Drag-and-Drop

```cpp
Viewer viewer;
viewer.addWorldNode(worldNode);
viewer.enableDragAndDrop(leftHand);
viewer.enableDragAndDrop(rightHand);
viewer.setUpViewInWindow(0, 0, 1280, 960);
viewer.run();
```

## Headless IK Control Loop

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

## Inspecting IK Solutions Programmatically

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

## Improving IK Solution Quality

- **Gradient weights** – `configureSmoothMotion()` down-weights the floating
  base so the torso remains steady.
- **Component clamps** – `setComponentWiseClamp(0.15)` keeps large gradients
  from flipping joints when the target jumps.
- **Damping** – when using `InverseKinematics::JacobianDLS`, raising the damping
  coefficient smooths behavior near singularities.
- **Warm starts** – `solveAndApply(true)` updates the skeleton, so each solve
  starts from the previous pose; feeding smooth target trajectories yields smooth
  joint motion.

## Complete Example (GUI)

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

## Extending the Tutorial

- Add foot end effectors and support polygons for balancing
- Constrain the center of mass over the support polygon
- Bias the solution toward preferred joint angles
- Introduce obstacle avoidance and collision constraints
- Stack multiple IK objectives with hierarchical priorities

## Related Resources

- Example: `examples/atlas_puppet/`
- Tests: `tests/integration/dynamics/test_AtlasIK.cpp`
- Python port: :doc:`dartpy/user_guide/tutorials/wholebody-ik`
- [DART IK Documentation](https://dartsim.github.io/)
