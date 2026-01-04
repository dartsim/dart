---
description: Robot model format conversion and validation specialist
mcp:
  robot-models:
    command: python3
    args: ["-m", "http.server", "8001"]
    working_dir: "/tmp/robot-models"
    description: "Local robot model documentation server"
---

# Robot Models Skill

## Purpose

Specialized skill for handling robotics model formats (URDF, SDF, MJCF, SKEL) with focus on conversion, validation, and cross-platform compatibility for DART integration.

## When to Use This Skill

### Format Conversion Tasks

- Convert URDF ↔ SDF for Gazebo compatibility
- Convert URDF ↔ MJCF for MuJoCo integration
- Convert any format ↔ DART SKEL for internal use
- Validate models against format specifications

### Model Validation and Debugging

- Detect malformed URDF/SDF/MJCF files
- Identify missing properties or inconsistent data
- Validate mesh file references and paths
- Check joint limits and robot kinematic consistency

### Cross-Platform Compatibility

- Ensure models work across ROS/Gazebo/MuJoCo
- Resolve vendor-specific extensions
- Optimize models for different physics engines
- Generate model documentation and metadata

## Key Tools and Commands

```bash
# Model validation
dart-model-validator robot.urdf --format urdf --strict
dart-model-validator world.sdf --format sdf --check-meshes

# Format conversion
dart-model-converter --input robot.urdf --output robot.sdf
dart-model-converter --input robot.urdf --output robot.mjcf --normalize

# Batch processing
dart-model-batch --input-dir models/ --output-dir converted/ --format sdf

# Mesh and texture optimization
dart-model-optimize --input robot.urdf --optimize-meshes --compress-textures
```

## Model Format Specifications

### URDF (Unified Robot Description Format)

**Key Elements**:

- `<robot>` root with name attribute
- `<link>` elements for rigid bodies
- `<joint>` elements for kinematic connections
- `<visual>` and `<collision>` for geometry
- `<material>` for appearance properties

**Common Issues**:

- Missing inertial properties (defaults to zero)
- Incorrect joint axis specifications
- Broken mesh file references
- Inconsistent coordinate frames

### SDF (Simulation Description Format)

**Key Elements**:

- `<sdf>` root with version attribute
- `<model>` elements for robot models
- `<world>` elements for complete simulation scenes
- Enhanced physics parameters (gravity, friction)
- Plugin configurations for extensions

**DART-Specific Features**:

- Dart-specific collision properties
- Custom joint limit enforcement
- Soft-body integration parameters

### MJCF (MuJoCo XML Format)

**Key Elements**:

- `<mujoco>` root with model attribute
- `<worldbody>` for kinematic structure
- `<actuator>` for motor models
- `<option>` for simulation parameters

**Conversion Challenges**:

- Different joint type representations
- Actuator model mapping
- Contact and friction parameter translation

## Conversion Strategies

### 1. Kinematic Structure Preservation

```xml
<!-- URDF input -->
<link name="base_link">
  <inertial>
    <mass value="1.0"/>
    <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
  </inertial>
</link>

<!-- SDF output (preserve structure) -->
<model name="robot">
  <link name="base_link">
    <inertial>
      <mass>1.0</mass>
      <inertia>
        <ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz>
        <ixy>0.0</ixy><ixz>0.0</ixz><iyz>0.0</iyz>
      </inertia>
    </inertial>
  </link>
</model>
```

### 2. Joint Type Mapping

| URDF Type  | SDF Equivalent       | MJCF Equivalent | Notes                   |
| ---------- | -------------------- | --------------- | ----------------------- |
| revolute   | revolute             | hinge           | Axis mapping critical   |
| prismatic  | prismatic            | slide           | Limits transfer         |
| continuous | revolute (no limits) | hinge           | Special case            |
| fixed      | fixed                | weld            | No degrees of freedom   |
| floating   | **multiple joints**  | freejoint       | Decompose into 6 joints |

### 3. Mesh and Material Handling

```xml
<!-- URDF visual -->
<visual>
  <geometry>
    <mesh filename="package://robot/meshes/base.dae"/>
  </geometry>
  <material name="blue"/>
</visual>

<!-- SDF converted (handle package paths) -->
<visual name="visual">
  <geometry>
    <mesh><uri>model://robot/meshes/base.dae</uri></mesh>
  </geometry>
  <material>
    <ambient>0.0 0.0 1.0 1.0</ambient>
    <diffuse>0.0 0.0 1.0 1.0</diffuse>
  </material>
</visual>
```

## Validation Procedures

### 1. Structural Validation

- **Check XML syntax** and required elements
- **Validate kinematic tree** (no cycles, single root)
- **Verify joint-parent relationships** exist
- **Check for required properties** (mass, inertia)

### 2. Geometric Validation

- **Mesh file existence** and accessibility
- **Mesh format compatibility** (STL, DAE, OBJ)
- **Scale and orientation** consistency
- **Collision-visual correspondence**

### 3. Physical Validation

- **Inertia tensor properties** (positive definite)
- **Joint limit consistency** (upper ≥ lower)
- **Mass distribution** reasonableness
- **Coordinate frame conventions**

## Performance Considerations

### 1. Large Model Handling

- **Stream processing** for files > 100MB
- **Lazy loading** for mesh files
- **Progressive conversion** with status updates
- **Memory management** for complex scenes

### 2. Batch Processing

```bash
# Efficient batch conversion
find models/ -name "*.urdf" | parallel -j 4 dart-model-converter {}
# Or using DART's built-in batching
dart-model-batch --input-dir models/ --parallel 4
```

## Common Pitfalls and Solutions

### ❌ Coordinate Frame Mismatches

```xml
<!-- Wrong: Mixed coordinate conventions -->
<joint type="revolute">
  <axis xyz="0 0 1"/>  <!-- URDF: Z-axis forward -->
  <!-- Child link expects Y-axis forward -->
</joint>

<!-- Correct: Consistent coordinate transforms -->
<joint type="revolute">
  <axis xyz="0 1 0"/>  <!-- Adjusted for child frame -->
  <!-- Add rotation if needed -->
  <parent link="base"/>
  <child link="arm"/>
  <origin xyz="0 0 0" rpy="0 0 -1.5708"/>  <!-- 90° rotation -->
</joint>
```

### ❌ Missing Inertial Properties

```xml
<!-- Wrong: No inertia (causes simulation errors) -->
<link name="base">
  <visual><geometry><box size="1 1 1"/></geometry></visual>
</link>

<!-- Correct: Calculate or use reasonable defaults -->
<link name="base">
  <inertial>
    <mass value="1.0"/>
    <inertia ixx="0.1667" ixy="0.0" ixz="0.0" iyy="0.1667" iyz="0.0" izz="0.1667"/>
  </inertial>
  <visual><geometry><box size="1 1 1"/></geometry></visual>
</link>
```

## Integration with DART

### 1. Loading in DART

```cpp
// Use unified API for any format
auto world = dart::io::readWorld("robot.urdf");
auto skeleton = dart::io::readSkeleton("robot.sdf");

// Validate loaded model
if (!skeleton) {
    throw std::runtime_error("Failed to load robot model");
}

// Check model integrity
skeleton->validateStructure();
skeleton->computeForwardKinematics();
```

### 2. Export from DART

```cpp
// Export to different formats
dart::io::saveWorld(world, "robot_converted.sdf");
dart::io::saveSkeleton(skeleton, "robot_converted.urdf");
dart::io::saveSkeleton(skeleton, "robot_converted.mjcf");
```

## Success Criteria

### Model Quality

- [ ] Loads without errors in target format
- [ ] Preserves kinematic structure
- [ ] Maintains physical properties
- [ ] Compatible with target simulator

### Performance

- [ ] Conversion time < file_size \* 0.1 seconds
- [ ] Memory usage < 3x file size
- [ ] Batch processing scales linearly
- [ ] No memory leaks in conversion

### Usability

- [ ] Clear error messages for validation failures
- [ ] Progress indicators for long operations
- [ ] Detailed conversion logs
- [ ] Command-line help and examples

## Related Documentation

- [DART IO Module Guidelines](../../../dart/io/AGENTS.md)
- [DART Collision Module](../../../dart/collision/AGENTS.md)
- [DART Dynamics Module](../../../dart/dynamics/AGENTS.md)
- [URDF Specification](http://wiki.ros.org/urdf)
- [SDF Specification](http://sdformat.org/spec)
- [MJCF Documentation](https://mujoco.org/book/)

---

_Robot models are the gateway between DART and robotics ecosystems. Ensure accurate, efficient conversions to maximize compatibility._
