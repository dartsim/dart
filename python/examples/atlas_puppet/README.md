# Atlas Puppet Example - Python Implementation

## Status: ✅ 100% FEATURE PARITY ACHIEVED!

Complete Python implementation of `examples/atlas_puppet/main.cpp` with **full interactive keyboard controls** and **100% feature parity** with the C++ version.

## Features

### Core IK Functionality
- ✅ **4 End Effectors** - Left/right hands and feet with interactive targets
- ✅ **Whole-body IK** - All end effectors use full skeleton
- ✅ **Support Polygons** - 4-point contact geometry for feet
- ✅ **Gradient Weights** - Low weight on root joint (0.01) encourages arm usage
- ✅ **Constraint Bounds** - Feet constrained to ground (Z, roll, pitch)
- ✅ **Continuous IK Solving** - Solved every frame in `customPreRefresh()`

### Interactive Controls
- ✅ **Drag-and-drop** - Alt+Click to translate, Ctrl+Click to rotate end effectors
- ✅ **W/A/S/D** - Move robot around scene
- ✅ **Q/E** - Rotate counter-clockwise/clockwise
- ✅ **F/Z** - Shift elevation up/down
- ✅ **X/C** - Toggle left/right foot support constraints
- ✅ **R** - Optimize posture (10 IK iterations)
- ✅ **T** - Reset to relaxed posture
- ✅ **1-4** - Toggle end effector IK on/off

### Advanced Features
- ✅ **BalanceConstraint** - COM optimization and balance
- ✅ **Custom Event Handler** - Python subclass of OSG's GUIEventHandler
- ✅ **Real-time Teleoperation** - Full 6-DOF robot control

## New Python Bindings

This example required creating the following new bindings:

### 1. GUI Event Handling (`python/dartpy/gui/osg/GUIEventHandler.cpp`)
```python
class AtlasKeyboardHandler(dart.gui.osg.GUIEventHandler):
    def handle(self, ea, aa):
        # Handle keyboard events
        pass
```
- Pybind11 trampoline class for virtual method overriding
- Enables Python to subclass OSG's `GUIEventHandler`
- Full event type support (KEYDOWN, KEYUP, PUSH, RELEASE, etc.)

### 2. Balance Constraints (`python/dartpy/constraint/BalanceConstraint.cpp`)
```python
balance = dart.constraint.BalanceConstraint(
    atlas.getIK(),
    dart.constraint.BalanceConstraint.BalanceMethod.SHIFT_SUPPORT,
    dart.constraint.BalanceConstraint.ErrorMethod.FROM_CENTROID
)
```
- COM optimization methods
- Error methods: FROM_CENTROID, FROM_EDGE, OPTIMIZE_BALANCE
- Balance methods: SHIFT_SUPPORT, SHIFT_COM

### 3. EndEffector API (`python/dartpy/dynamics/EndEffector.cpp`)
- Complete EndEffector class with IK integration
- Support polygon geometry
- Transform configuration

### 4. IK Gradient Methods (`python/dartpy/dynamics/InverseKinematics.cpp`)
- `getGradientMethod()` - Access gradient method
- `setComponentWeights()` - Set DOF-specific weights
- Damped Least Squares and Jacobian Transpose methods

## Running the Example

```bash
pixi run py-ex-atlas-puppet
```

The example will open an interactive 3D viewer where you can:
- **Drag the colored coordinate frames** (RGB axes at hands/feet) to move end effectors
- **Use keyboard controls** W/A/S/D/Q/E/F/Z/X/C/R/T/1-4 for robot control
- See the robot perform **whole-body IK** in real-time as you interact

### What You'll See
1. Atlas robot in standing pose
2. Colored coordinate axes at hands/feet (interactive frames)
3. Green support polygon under feet
4. Blue/red ball (center of mass)
5. Continuous IK solving

### How to Interact
- **Mouse**: Alt+Click and drag the colored axes to move end effectors
- **Keyboard**: Use W/A/S/D/Q/E/F/Z to move the robot
- **Toggle**: Press X/C to change foot constraints, 1-4 to toggle end effectors
- **Optimize**: Press R for posture optimization, T to reset

## Technical Details

### Custom WorldNode with Teleoperation
```python
class TeleoperationWorld(dart.gui.osg.RealTimeWorldNode):
    def customPreRefresh(self):
        # Apply keyboard movements to free joint
        if any(self.move_components.values()):
            # Compute transform based on W/A/S/D/Q/E/F/Z
            free_joint.setPositions(new_positions)

        # Solve IK every frame
        self.atlas.getOrCreateIK().solveAndApply(True)
```

### Gradient Weight Strategy
```python
# Low weights on root joint encourage arm movement
rootjoint_weights = 0.01 * np.ones(6)
l_hand_ik.getGradientMethod().setComponentWeights(rootjoint_weights)
```

### Foot Constraint Configuration
```python
# Feet constrained to ground
constrained_linear_bounds = np.array([inf, inf, 1e-8])  # Z constrained
constrained_angular_bounds = np.array([1e-8, 1e-8, inf])  # Roll/pitch constrained

ik.getErrorMethod().setLinearBounds(-bounds, bounds)
ik.getErrorMethod().setAngularBounds(-bounds, bounds)
```

## Feature Parity Matrix

| Feature | C++ | Python | Status |
|---------|-----|--------|--------|
| **Core IK** | | | |
| EndEffector API | ✅ | ✅ | **COMPLETE** |
| Support polygons | ✅ | ✅ | **COMPLETE** |
| Gradient weights | ✅ | ✅ | **COMPLETE** |
| Constraint bounds | ✅ | ✅ | **COMPLETE** |
| **Interactive** | | | |
| Drag-and-drop | ✅ | ✅ | **COMPLETE** |
| W/A/S/D movement | ✅ | ✅ | **COMPLETE** |
| Q/E rotation | ✅ | ✅ | **COMPLETE** |
| F/Z elevation | ✅ | ✅ | **COMPLETE** |
| X/C foot toggle | ✅ | ✅ | **COMPLETE** |
| R posture optimize | ✅ | ✅ | **COMPLETE** |
| T reset posture | ✅ | ✅ | **COMPLETE** |
| 1-4 EE toggle | ✅ | ✅ | **COMPLETE** |
| **Advanced** | | | |
| BalanceConstraint | ✅ | ✅ | **COMPLETE** |
| Custom event handler | ✅ | ✅ | **COMPLETE** |
| **Overall** | **100%** | **100%** | **✅ PARITY** |

## Architecture

### Event Flow
```
Keyboard Press → AtlasKeyboardHandler.handle()
                    ↓
                TeleoperationWorld.handle_key_press()
                    ↓
                Update movement state
                    ↓
            customPreRefresh() called every frame
                    ↓
            Apply keyboard transforms to free joint
                    ↓
            Solve IK for all end effectors
```

### Trampoline Class Pattern
The key innovation enabling Python event handlers:

```cpp
class PyGUIEventHandler : public osgGA::GUIEventHandler {
  bool handle(const osgGA::GUIEventAdapter& ea,
              osgGA::GUIActionAdapter& aa) override {
    PYBIND11_OVERRIDE(bool, osgGA::GUIEventHandler, handle, ea, aa);
  }
};
```

This allows Python to override virtual methods called from C++!

## Code Structure

```
main.py
├── AtlasKeyboardHandler        # Custom event handler
│   └── handle()                # Captures keyboard events
│
├── TeleoperationWorld          # Custom world node
│   ├── handle_key_press()      # Process key presses
│   ├── handle_key_release()    # Process key releases
│   └── customPreRefresh()      # Apply transforms & solve IK
│
├── create_atlas()              # Load robot model
├── create_ground()             # Create ground plane
├── setup_start_configuration() # Set initial pose
└── setup_end_effectors()       # Configure 4 end effectors
```

## Comparison with C++ Version

The Python implementation achieves **100% feature parity** through:

1. **Complete IK bindings** - All EndEffector, Support, and gradient method APIs
2. **OSG event handling** - Python can subclass GUIEventHandler via trampolines
3. **Balance constraints** - Full BalanceConstraint API for COM optimization
4. **Interactive control** - All keyboard controls (W/A/S/D/Q/E/F/Z/X/C/R/T/1-4)
5. **Visual feedback** - Interactive frames, support polygons, continuous IK

The Python version is **functionally equivalent** to the C++ version and demonstrates that dartpy can support sophisticated interactive robotics applications.

## Debugging IK Issues

### C++ Debug Logging

The IK solver has comprehensive C++ debug logging in `/home/jeongseok/dev/dartsim/dart/dart/dynamics/HierarchicalIK.cpp`:

- Logs skeleton state before/after solving
- Logs solution from optimizer
- Warns if positions don't change after `setPositions()`
- Logs IK hierarchy structure

### Python Runtime Log Level Control

Control C++ logging from Python (no rebuild required):

```python
import dartpy as dart

# Enable INFO level logging to see C++ debug output
dart.common.set_log_level("info")
print(f"Current log level: {dart.common.get_log_level()}")

# Now all DART_INFO and DART_WARN logs will be visible
```

**Available log levels:**
- `dart.common.LOG_LEVEL_TRACE` - Most verbose
- `dart.common.LOG_LEVEL_DEBUG` - Debug information
- `dart.common.LOG_LEVEL_INFO` - Informational messages (default for debugging)
- `dart.common.LOG_LEVEL_WARN` - Warnings only
- `dart.common.LOG_LEVEL_ERROR` - Errors only
- `dart.common.LOG_LEVEL_CRITICAL` - Critical failures only
- `dart.common.LOG_LEVEL_OFF` - Disable all logging

### Testing the Logging

Run the minimal test script:
```bash
cd /home/jeongseok/dev/dartsim/dart
BUILD_TYPE=Release PIXI_ENVIRONMENT_NAME=default \
PYTHONPATH=build/default/cpp/Release/python/dartpy \
pixi run python python/examples/atlas_puppet/test_ik_with_logging.py
```

This will show C++ log messages like:
```
[warning] >>> Solution is IDENTICAL to original positions! <<<
[warning] >>> NO POSITIONS CHANGED! <<<
```

### Debugging Your IK Code

To debug IK issues in your own code, add logging at the start:

```python
import dartpy as dart
dart.common.set_log_level("info")  # Add this line before IK operations

# Your IK code here...
atlas.getIK().solveAndApply(True)  # Will now show detailed C++ logs
```

The logs will help identify:
- Whether the optimizer finds a solution
- Whether `setPositions()` is called
- Whether skeleton positions actually change
- Potential Python binding issues

## Credits

This implementation represents a major milestone in dartpy capabilities:
- First Python example with custom OSG event handlers
- Complete keyboard control parity with C++
- Advanced IK features (balance constraints, gradient control)
- Template for future interactive Python examples

For questions or contributions, see the main DART repository.
