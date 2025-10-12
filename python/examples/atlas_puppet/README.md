# Atlas Puppet Example - Python Version

## Current Status: SIMPLIFIED IMPLEMENTATION

This example is a simplified version of the C++ atlas_puppet example located in `examples/atlas_puppet/`.

### What's Included

This Python implementation demonstrates:
- Loading the Atlas robot model
- Setting up end effector targets for hands and feet
- Basic inverse kinematics with end effectors
- Ground plane creation
- Interactive 3D visualization with OSG

### What's Different from C++

The Python version provides the core functionality but with some simplifications:
- Simplified support polygon handling
- Basic IK setup without all gradient method customizations
- Streamlined initialization

### Recent Improvements

Python bindings have been added for:
- `dart::dynamics::EndEffector` class
- `dart::dynamics::BodyNode::createEndEffector()` method
- `dart::dynamics::Support` class
- Related end effector and IK methods

### Running This Example

```bash
pixi run py-ex-atlas-puppet
```

This will launch an interactive 3D viewer where you can see the Atlas robot in a standing pose with configured end effectors.

### Future Enhancements

Additional features that could be added:
- Full drag-and-drop interaction for end effector targets
- Advanced balance constraint optimization
- Complete support polygon visualization
- Interactive frame manipulation
