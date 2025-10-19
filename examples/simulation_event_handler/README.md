# Simulation Event Handler

This example demonstrates a comprehensive event handler class that replaces the functionality typically found in `MyWindow.cpp` files in physics simulation applications. The `SimulationEventHandler` provides a complete solution for interactive rigid body physics simulation control.

## Features

### 1. Force Application on Rigid Bodies
- Apply forces to selected rigid bodies in world coordinates
- Real-time force magnitude adjustment
- Support for both translational forces and rotational torques
- Forces applied at the center of mass of selected bodies

### 2. Keyboard Interaction for Simulation Control
- **Simulation Control**: Play/pause, step-by-step execution, reset
- **Body Selection**: Navigate through all rigid bodies in the scene
- **Force Application**: Arrow keys for linear forces, QWEAZC keys for torques
- **Parameter Adjustment**: Real-time modification of force magnitudes and time steps
- **Visualization Control**: Toggle force arrow visualization

### 3. Arrow Visualization
- Red arrows showing applied forces in real-time
- Arrows scale with force magnitude and direction
- Arrows automatically clear after each application
- Toggle on/off visualization for performance

### 4. Time Stepping Logic
- Configurable simulation time step
- Real-time time step adjustment during simulation
- Step-by-step simulation for debugging
- Automatic integration with DART's physics engine

## Controls

### Simulation Control
- **Space**: Toggle simulation play/pause
- **S**: Step simulation one frame forward
- **R**: Reset simulation to initial state

### Body Selection
- **Tab**: Select next rigid body
- **Backspace**: Select previous rigid body

### Force Application (on selected body)
- **Arrow Keys**: Apply forces in X/Y directions
- **U**: Apply upward force (+Z direction)
- **D**: Apply downward force (-Z direction)

### Torque Application (on selected body)
- **Q/A**: Apply torque around X axis (+/-)
- **W/Z**: Apply torque around Y axis (+/-)
- **E/C**: Apply torque around Z axis (+/-)

### Parameter Adjustment
- **+/=**: Increase force/torque magnitude
- **-/_**: Decrease force/torque magnitude
- **>/.**: Increase simulation time step
- **</, **: Decrease simulation time step

### Visualization
- **V**: Toggle force arrow visualization

### Information
- **I**: Print current simulation state
- **H/?**: Show detailed help

## Usage

### Basic Usage

```cpp
#include "SimulationEventHandler.hpp"

// Create physics world
WorldPtr world = World::create();
world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));

// Add rigid bodies to the world
// ... (create and add skeletons)

// Create viewer
dart::gui::osg::Viewer viewer;

// Create event handler
SimulationEventHandler* handler = new SimulationEventHandler(world, &viewer);

// Set up viewer
viewer.addEventHandler(handler);
viewer.run();
```

### Advanced Usage

```cpp
// Customize event handler behavior
handler->setTimeStep(0.0005);           // Set smaller time step
handler->setForceMagnitude(200.0);      // Set higher force magnitude
handler->setShowForceArrows(false);     // Disable arrow visualization

// Update in custom render loop
void customRenderLoop() {
  handler->update();  // Call each frame for arrow updates
  // ... other rendering code
}
```

## Implementation Details

### Architecture
- **Header File**: `SimulationEventHandler.hpp` - Class declaration and interface
- **Implementation**: `SimulationEventHandler.cpp` - Full functionality implementation
- **Demo**: `main.cpp` - Example usage with sample rigid bodies

### Key Components
1. **Event Handling**: Inherits from `osgGA::GUIEventHandler` for OSG integration
2. **Force System**: Manages force/torque application with visual feedback
3. **Body Management**: Automatic detection and selection of rigid bodies
4. **Visualization**: OpenSceneGraph-based arrow rendering system
5. **State Management**: Tracks simulation state and user preferences

### Performance Considerations
- Arrow visualization updates only every 5 frames to reduce overhead
- Automatic cleanup of visualization objects
- Efficient rigid body detection and caching
- Memory-safe OSG object management

## Building

This example is integrated with the DART build system:

```bash
# From DART root directory
mkdir build && cd build
cmake ..
make simulation_event_handler

# Run the example
./bin/simulation_event_handler
```

## Extending the Event Handler

The `SimulationEventHandler` class is designed to be easily extensible:

### Adding New Controls
```cpp
// In handle() method, add new key cases:
case 'n':  // Custom action
  performCustomAction();
  return true;
```

### Custom Force Application
```cpp
// Override or extend force application methods:
void applyCustomForce(const Eigen::Vector3d& direction) {
  if (mSelectedBody) {
    Eigen::Vector3d customForce = computeCustomForce(direction);
    mSelectedBody->addExtForce(customForce);
  }
}
```

### Additional Visualizations
```cpp
// Add new visualization types:
void addCustomVisualization() {
  // Create custom OSG nodes
  // Add to scene graph
}
```

## Comparison to MyWindow.cpp

This event handler replaces typical `MyWindow.cpp` functionality by providing:

1. **Structured Event Handling**: Clean separation of concerns vs. monolithic window class
2. **Modern DART API**: Uses current DART patterns and best practices
3. **Extensible Design**: Easy to modify and extend for specific applications
4. **Better Documentation**: Comprehensive documentation and examples
5. **Integrated Visualization**: Built-in force visualization system
6. **Flexible Integration**: Works with any DART/OSG application

## Notes

- The event handler automatically detects rigid bodies (those with FreeJoint, BallJoint, etc.)
- Force arrows are temporary and clear after each update cycle
- Time step changes affect the entire world simulation
- The handler maintains compatibility with DART's constraint system
- All coordinate systems use world coordinates for consistency
