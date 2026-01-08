# SKEL Format Migration Guide

**For Users**: How to migrate from SKEL to modern formats  
**Target Audience**: DART users with existing SKEL files  
**Deprecation Timeline**: DART 7.1 warnings → DART 8.0 removal

## Quick Start: What Should I Do?

### If You're Starting New Projects
- **Use URDF** for robot models (ROS ecosystem standard)
- **Use SDF** for complete simulation worlds (Gazebo standard)
- Consider **YAML** alternatives when Phase 2 is available (more readable)

### If You Have Existing SKEL Files
1. **Immediate**: Convert SKEL to URDF/SDF using our tools
2. **Before DART 8.0**: Complete migration to avoid breaking changes
3. **Plan Ahead**: Consider YAML when it becomes available

## Migration Options

### Option 1: Automated Conversion (Recommended)
Use DART's built-in conversion utilities:

```bash
# Convert SKEL skeleton to URDF
pixi run convert-skel-to-urdf input.skel output.urdf

# Convert SKEL world to SDF  
pixi run convert-skel-to-sdf input.skel output.sdf

# Batch convert directory
pixi run convert-skel-dir --input skel_files/ --output urdf_files/ --format urdf
```

### Option 2: Manual Conversion
For complex models requiring custom handling:

```cpp
#include <dart/io/io.hpp>

int main() {
    // Load SKEL file
    auto world = dart::io::readWorld("my_model.skel");
    
    // Export as URDF (when Phase 4 export is available)
    auto skeleton = world->getSkeleton(0);
    dart::io::writeUrdf(skeleton, "my_model.urdf");
    
    // Or as SDF world
    dart::io::writeSdf(world, "my_world.sdf");
    
    return 0;
}
```

### Option 3: Gradual Migration
Keep SKEL files during transition:

```cpp
// During deprecation period
if (fileExists("model.urdf")) {
    return dart::io::readSkeleton("model.urdf");
} else {
    DART_WARN("Using deprecated SKEL format. Please convert to URDF.");
    return dart::io::readSkeleton("model.skel");
}
```

## Format Comparison: SKEL vs Modern Alternatives

### SKEL vs URDF

#### SKEL Structure (Legacy)
```xml
<skel version="1.0">
  <skeleton name="robot">
    <body name="link1">
      <inertia>
        <mass>1.0</mass>
        <offset>0 0 0</offset>
      </inertia>
      <visualization_shape>
        <geometry>
          <box>
            <size>1 1 1</size>
          </box>
        </geometry>
        <color>1 0 0</color>
      </visualization_shape>
    </body>
    <joint type="revolute" name="joint1">
      <parent>link1</parent>
      <child>link2</child>
    </joint>
  </skeleton>
</skel>
```

#### URDF Structure (Modern)
```xml
<?xml version="1.0"?>
<robot name="robot">
  <link name="link1">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
    <visual>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
  </link>
  <joint name="joint1" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
  </joint>
</robot>
```

### YAML Alternative (Future)
```yaml
robot:
  name: "robot"
  links:
    - name: "link1"
      inertial:
        mass: 1.0
        inertia: [0.1, 0, 0, 0.1, 0, 0.1]
        origin: {xyz: [0, 0, 0]}
      visual:
        geometry:
          box: {size: [1, 1, 1]}
        material:
          color: {rgba: [1, 0, 0, 1]}
  joints:
    - name: "joint1"
      type: "revolute"
      parent: "link1"
      child: "link2"
```

## Key Differences and Migration Notes

### 1. Transform Representation
- **SKEL**: `<transformation>0 -0.375 0 0 0 0</transformation>` (string)
- **URDF**: `<origin xyz="0 -0.375 0" rpy="0 0 0"/>` (attributes)
- **YAML**: `origin: {xyz: [0, -0.375, 0], rpy: [0, 0, 0]}` (structured)

### 2. Inertia Specification
- **SKEL**: Mass + offset (simplified)
- **URDF**: Full 6-element inertia matrix
- **Migration**: Auto-compute inertia matrix from SKEL data

### 3. Joint Configuration
- **SKEL**: Separate `<joint>` elements outside bodies
- **URDF**: Joint elements connect bodies
- **YAML**: Structured joint definitions

### 4. Visual vs Collision
- **SKEL**: Separate `<visualization_shape>` and `<collision_shape>`
- **URDF**: Separate `<visual>` and `<collision>` elements
- **Direct mapping**: 1:1 conversion possible

## Step-by-Step Migration Guide

### Step 1: Inventory Your SKEL Files
```bash
# Find all SKEL files in your project
find . -name "*.skel" -type f > skel_files.txt

# Analyze complexity
wc -l $(cat skel_files.txt)
```

### Step 2: Test Conversion on Sample Files
```bash
# Convert a simple test file first
pixi run convert-skel-to-urdf test_simple.skel test_simple.urdf

# Verify the conversion works
pixi run dart-examples --urdf test_simple.urdf
```

### Step 3: Batch Convert Your Files
```bash
# Convert all skeleton files
for file in *.skel; do
    echo "Converting $file to URDF..."
    pixi run convert-skel-to-urdf "$file" "${file%.skel}.urdf"
done

# Convert world files
for file in *_world.skel; do
    echo "Converting $file to SDF..."
    pixi run convert-skel-to-sdf "$file" "${file%.skel}.sdf"
done
```

### Step 4: Update Your Code
```cpp
// Old code (deprecated)
auto world = dart::io::readWorld("model.skel");

// New code (modern)
auto world = dart::io::readWorld("model.sdf");  // SDF world
// or
auto robot = dart::io::readSkeleton("model.urdf");  // URDF robot
```

### Step 5: Update Build Scripts and CI
```cmake
# CMakeLists.txt updates
find_package(dart REQUIRED COMPONENTS io-utils)

# Update file paths in your build system
set(MODEL_FILES
    models/robot1.urdf
    models/robot2.urdf
    worlds/simulation1.sdf
)
```

### Step 6: Test Thoroughly
```cpp
// Validation script
bool validateConversion(const std::string& skelPath, const std::string& urdfPath) {
    auto skelWorld = dart::io::readWorld(skelPath);
    auto urdfSkeleton = dart::io::readSkeleton(urdfPath);
    
    // Compare key properties
    return compareSkeletons(skelWorld->getSkeleton(0), urdfSkeleton);
}
```

## Conversion Tools Reference

### Command Line Tools
```bash
# Basic conversion
pixi run convert-skel-to-urdf input.skel output.urdf
pixi run convert-skel-to-sdf input.skel output.sdf

# Advanced options
pixi run convert-skel-to-urdf input.skel output.urdf \
    --pretty-print \
    --include-comments \
    --urdf-version 1.0

# Batch conversion
pixi run convert-skel-dir \
    --input skel_models/ \
    --output urdf_models/ \
    --format urdf \
    --recursive \
    --validate
```

### C++ API
```cpp
#include <dart/io/io.hpp>
#include <dart/io/conversion.hpp>

// Direct conversion
bool result = dart::io::convertSkelToUrdf("input.skel", "output.urdf");

// Advanced conversion with options
dart::io::ConversionOptions options;
options.prettyPrint = true;
options.includeComments = true;
options.preserveOriginalNames = true;

bool result = dart::io::convertSkelToUrdf(
    "input.skel", 
    "output.urdf", 
    options
);

// In-memory conversion
auto skelWorld = dart::io::readWorld("input.skel");
auto urdfString = dart::io::writeUrdfToString(skelWorld->getSkeleton(0));
```

### Python API (Future)
```python
import dartpy as dart

# Simple conversion
world = dart.io.read_world("input.skel")
dart.io.write_urdf(world.get_skeleton(0), "output.urdf")

# Batch conversion
import glob
for skel_file in glob.glob("*.skel"):
    world = dart.io.read_world(skel_file)
    urdf_file = skel_file.replace('.skel', '.urdf')
    dart.io.write_urdf(world.get_skeleton(0), urdf_file)
```

## Common Migration Issues and Solutions

### Issue 1: Missing Inertia Matrix
**Problem**: SKEL uses simplified inertia, URDF requires full matrix
```xml
<!-- SKEL -->
<inertia>
  <mass>1.0</mass>
  <offset>0 0 0</offset>
</inertia>
```

**Solution**: Auto-compute from geometric properties
```xml
<!-- URDF -->
<inertial>
  <mass value="1.0"/>
  <origin xyz="0 0 0"/>
  <inertia ixx="0.0833" ixy="0" ixz="0" iyy="0.0833" iyz="0" izz="0.0833"/>
</inertial>
```

### Issue 2: Joint Type Differences
**Problem**: SKEL joint types don't map 1:1 to URDF
```xml
<!-- SKEL -->
<joint type="free" name="joint1">
```

**Solution**: Map to appropriate URDF types
- `free` → `floating` (special handling in URDF)
- `weld` → `fixed`
- `revolute` → `revolute`
- `prismatic` → `prismatic`

### Issue 3: Material Definition
**Problem**: SKEL colors don't map directly to URDF materials
```xml
<!-- SKEL -->
<color>0.8 0.3 0.3</color>
```

**Solution**: Create URDF material structure
```xml
<!-- URDF -->
<material name="default_material">
  <color rgba="0.8 0.3 0.3 1.0"/>
</material>
```

### Issue 4: File Path References
**Problem**: Relative paths break after conversion
```xml
<!-- SKEL -->
<mesh>
  <file_name>../../models/foot.obj</file_name>
</mesh>
```

**Solution**: Update path references during conversion
```xml
<!-- URDF -->
<mesh filename="package://my_package/models/foot.obj"/>
```

## Validation and Testing

### Automated Validation Script
```python
#!/usr/bin/env python3
import sys
import dartpy as dart

def validate_conversion(skel_file, urdf_file):
    """Validate that SKEL and URDF files produce equivalent robots"""
    
    # Load both versions
    skel_world = dart.io.read_world(skel_file)
    urdf_skeleton = dart.io.read_skeleton(urdf_file)
    
    skel_robot = skel_world.get_skeleton(0)
    
    # Compare basic properties
    assert skel_robot.get_num_body_nodes() == urdf_skeleton.get_num_body_nodes()
    assert skel_robot.get_num_joints() == urdf_skeleton.get_num_joints()
    
    # Compare masses
    for i in range(skel_robot.get_num_body_nodes()):
        skel_mass = skel_robot.get_body_node(i).get_mass()
        urdf_mass = urdf_skeleton.get_body_node(i).get_mass()
        assert abs(skel_mass - urdf_mass) < 1e-6
    
    print(f"✅ {skel_file} → {urdf_file} validation passed")
    return True

if __name__ == "__main__":
    skel_file = sys.argv[1]
    urdf_file = sys.argv[2]
    validate_conversion(skel_file, urdf_file)
```

### Performance Comparison
```cpp
// Benchmark conversion performance
void benchmarkConversion(const std::string& skelFile) {
    // Load SKEL
    auto start = std::chrono::high_resolution_clock::now();
    auto skelWorld = dart::io::readWorld(skelFile);
    auto skelLoadTime = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now() - start);
    
    // Convert to URDF
    start = std::chrono::high_resolution_clock::now();
    auto urdfString = dart::io::writeUrdfToString(skelWorld->getSkeleton(0));
    auto conversionTime = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now() - start);
    
    // Load converted URDF
    start = std::chrono::high_resolution_clock::now();
    auto urdfSkeleton = dart::io::readSkeletonFromString(urdfString);
    auto urdfLoadTime = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now() - start);
    
    std::cout << "SKEL load: " << skelLoadTime.count() << "ms\n";
    std::cout << "Conversion: " << conversionTime.count() << "ms\n";
    std::cout << "URDF load: " << urdfLoadTime.count() << "ms\n";
}
```

## Best Practices for Migration

### 1. Start Small
- Test migration on simple files first
- Validate conversion accuracy before bulk conversion
- Keep original SKEL files as backup during transition

### 2. Version Control
- Commit converted files separately
- Use clear commit messages about migration
- Tag migration milestones in your repository

### 3. Documentation
- Document any manual adjustments needed
- Keep migration scripts for future reference
- Update project READMEs and build instructions

### 4. Testing
- Test converted models in your simulations
- Validate that behavior matches original SKEL files
- Include conversion validation in CI/CD pipeline

### 5. Gradual Transition
- Support both old and new formats during transition period
- Add deprecation warnings to your own code if applicable
- Plan for complete SKEL removal before DART 8.0

## Getting Help

### Resources
- **DART Documentation**: [https://dart.readthedocs.io/](https://dart.readthedocs.io/)
- **URDF Specification**: [ROS Wiki](http://wiki.ros.org/urdf)
- **SDF Specification**: [Gazebo Docs](http://sdformat.org/spec)

### Community Support
- **GitHub Issues**: [dartsim/dart](https://github.com/dartsim/dart/issues)
- **Discourse Forum**: [DART Community](https://discourse.gazebosim.org/c/dart)
- **ROS Discourse**: [Robotics Q&A](https://discourse.ros.org/)

### Migration Tools
- **DART Conversion Utilities**: Included in DART 7.1+
- **Online Converters**: Future web-based conversion tools
- **Professional Services**: Contact DART team for enterprise migration support

## Timeline Checklist

### Before DART 7.2
- [ ] Inventory all SKEL files in your project
- [ ] Test conversion on sample files
- [ ] Plan migration timeline for your project

### Before DART 7.5
- [ ] Convert all SKEL files to URDF/SDF
- [ ] Update code to use new file formats
- [ ] Validate converted models work correctly
- [ ] Update documentation and build scripts

### Before DART 8.0
- [ ] Remove all SKEL dependencies from code
- [ ] Delete SKEL files from repositories
- [ ] Ensure CI/CD uses only modern formats
- [ ] Test with DART 8.0 (SKEL support removed)

Remember: SKEL support will be **completely removed in DART 8.0**. Start your migration early to avoid disruption!