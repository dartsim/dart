# Rigid Cubes Example

This example demonstrates basic rigid body simulation with interactive force application using DART's OpenSceneGraph (OSG) viewer. The example loads a world containing multiple rigid cube objects and allows real-time interaction through keyboard controls.

## What This Example Demonstrates

- **Rigid Body Physics**: Simulation of multiple rigid body cubes with gravity
- **Force Application**: Interactive application of external forces to rigid bodies
- **OSG Viewer Pattern**: Modern DART visualization using OpenSceneGraph
- **Real-time Simulation**: Continuous physics simulation with real-time rendering
- **Event Handling**: Keyboard input handling in OSG viewer context

## Features

- Real-time rigid body simulation with gravity
- Interactive force application with keyboard controls
- Proper OSG viewer setup with shadow rendering
- Force decay mechanism (forces reduce by half each simulation step)
- Visual feedback for applied forces

## Controls

- **Space bar**: Toggle simulation on/off
- **'p'**: Toggle playback/stop mode
- **'v'**: Toggle visualization markers
- **'1'**: Apply negative X direction force (-500N)
- **'2'**: Apply positive X direction force (+500N)
- **'3'**: Apply negative Z direction force (-500N)
- **'4'**: Apply positive Z direction force (+500N)

## Implementation Details

### OSG Viewer Components

1. **RigidCubesWorldNode**: Custom world node extending `RealTimeWorldNode`
   - Handles force application and decay in `customPreStep()`
   - Manages external force state between simulation steps

2. **RigidCubesEventHandler**: Custom event handler for keyboard input
   - Processes simulation control keys (space, 'p', 'v')
   - Handles force application keys ('1'-'4')
   - Provides console feedback for user actions

3. **Main Function**: Standard OSG viewer setup pattern
   - World loading from skeleton file
   - OSG viewer configuration with shadow support
   - Camera positioning and window setup
   - Event handler registration

### Physics Simulation

- Loads world from `dart://sample/skel/cubes.skel`
- Applies gravity: (0, -9.81, 0) m/s²
- Forces are applied to the second skeleton's first body node
- Force magnitude: 500N in specified directions
- Force decay: Reduces by 50% each simulation step

## Build Instructions

This project requires DART with OSG support. From this directory:

```bash
mkdir build
cd build
cmake ..
make
```

## Execute Instructions

Launch the executable from the build directory:

```bash
./rigid_cubes
```

Follow the control instructions displayed in the console and viewer window.

## Technical Notes

- **OSG Patterns**: Uses modern OSG viewer patterns with proper reference counting
- **Real-time Simulation**: Leverages `RealTimeWorldNode` for smooth physics stepping
- **Shadow Rendering**: Includes default shadow technique for enhanced visualization
- **Force Management**: Implements the same force application and decay logic as the legacy viewer version

## Related Examples

- `boxes`: Similar rigid body simulation with different interaction
- `rigid_shapes`: More complex rigid body shapes and interactions
- `atlas_puppet`: Advanced OSG viewer usage with robot manipulation
