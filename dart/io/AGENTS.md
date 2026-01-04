# Agent Guidelines for DART IO Module

This module provides a unified API for loading various robotics file formats (URDF, SDF, MJCF, SKEL) through the `readWorld()` and `readSkeleton()` functions. Agents working here must understand different format specifications and maintain compatibility across robotics ecosystems.

## Read First

- **Base Guidelines**: Read `/AGENTS.md` for overall DART patterns
- **Unified API**: Always use `dart::io` functions, never direct parsers
- **Format Support**: URDF, SDF, MJCF, SKEL formats supported
- **Testing**: IO tests in `tests/unit/io/` and `tests/integration/io/`

## Module-Specific Commands

```bash
# Build IO with specific format support
pixi run build --config DART_BUILD_URDF=ON
pixi run build --config DART_BUILD_SDF=ON
pixi run build --config DART_BUILD_MJCF=ON

# Test IO functionality
pixi run test --filter io
pixi run test --filter integration/io

# Test with sample files
pixi run test --data examples/atlas_puppet/robot.urdf
pixi run test --data examples/boxes/world.sdf

# Debug parsing issues
pixi run -e debug build --config DART_VERBOSE=ON
./dart-loader --verbose --debug problematic_file.urdf
```

## Critical Architecture Patterns

### 1. Unified Loading API
```cpp
// Correct: Use unified API
auto world = dart::io::readWorld("robot.urdf");
auto skeleton = dart::io::readSkeleton("robot.sdf");

// Wrong: Use format-specific parsers directly
auto urdfLoader = URDFLoader();
auto world = urdfLoader->load("robot.urdf"); // Avoid
```

### 2. Format Auto-Detection
```cpp
// Correct: Let DART detect format
auto world = dart::io::readWorld("robot"); // Extension automatically detected

// Wrong: Manual format handling
if (endsWith("urdf")) {
    world = loadURDF("robot");
} else if (endsWith("sdf")) {
    world = loadSDF("robot");
} // Avoid
```

### 3. Error Handling and Validation
```cpp
// Correct: Robust error handling
try {
    auto world = dart::io::readWorld("robot.urdf");
    if (!world) {
        std::cerr << "Failed to load world" << std::endl;
        return nullptr;
    }
    // Validate loaded content
    if (world->getNumSkeletons() == 0) {
        std::cerr << "No skeletons found in file" << std::endl;
    }
} catch (const std::exception& e) {
    std::cerr << "Parsing error: " << e.what() << std::endl;
}
```

## Supported File Formats

### URDF (Unified Robot Description Format)
- **Use case**: ROS robotics, academic research
- **Characteristics**: XML-based, joint/kinematic hierarchy, visual/collision tags
- **Extensions**: `.urdf`
- **Special handling**: Mesh file loading, material properties

### SDF (Simulation Description Format)  
- **Use case**: Gazebo simulation, complex world models
- **Characteristics**: XML-based, multiple robots, world properties
- **Extensions**: `.sdf`, `.world`
- **Special handling**: Physics engine parameters, world-level properties

### MJCF (MuJoCo XML Format)
- **Use case**: Physics research, reinforcement learning
- **Characteristics**: XML-based, detailed physics parameters
- **Extensions**: `.xml`, `.mjcf`
- **Special handling**: Actuator models, contact parameters

### SKEL (DART Native Format)
- **Use case**: DART-specific development, debugging
- **Characteristics**: JSON-like, minimal overhead
- **Extensions**: `.skel`
- **Special handling**: DART-specific features, experimental properties

## When to Modify This Module

### Add New File Format Support
1. **Implement parser class** inheriting from appropriate base
2. **Add factory registration** in `Read.cpp`
3. **Update format detection** logic
4. **Add comprehensive tests** with sample files
5. **Update documentation** with format specification
6. **Add format examples** to `examples/` directory

### Improve Loading Performance
1. **Profile with large files**: Use complex URDF/SDF files
2. **Optimize mesh loading**: Consider async loading for large meshes
3. **Cache parsed results**: For repeated loading of same files
4. **Memory management**: Efficient handling of large scene graphs
5. **Parallel parsing**: Where format allows independent section parsing

### Fix Compatibility Issues
1. **Test across ecosystems**: ROS, Gazebo, MuJoCo workflows
2. **Validate against specifications**: Official URDF/SDF/MJCF standards
3. **Handle vendor extensions**: Common non-standard tags
4. **Maintain backward compatibility**: Existing files must continue loading
5. **Provide migration paths**: Guidelines for deprecated features

## Common Pitfalls to Avoid

### ❌ Bypassing Unified API
```cpp
// Wrong: Direct format-specific loading
if (filename.find(".urdf") != std::string::npos) {
    return loadURDF(filename);  // Inconsistent behavior
}

// Correct: Use unified loading
return dart::io::readWorld(filename);  // Consistent behavior
```

### ❌ Inadequate Error Handling
```cpp
// Wrong: Silent failures
auto world = dart::io::readWorld(filename);
// world might be nullptr, no error reported

// Correct: Comprehensive error checking
auto world = dart::io::readWorld(filename);
if (!world) {
    throw std::runtime_error("Failed to load world: " + filename);
}
```

### ❌ Resource Path Issues
```cpp
// Wrong: Relative path handling
auto mesh = loadMesh("meshes/visual.dae");  // Depends on working directory

// Correct: Path resolution relative to file
auto mesh = loadMesh(resolvePath("meshes/visual.dae", filename));
```

## Testing Requirements

### Before Submitting Changes:
1. **Unit Tests**: `pixi run test --filter unit/io`
2. **Integration Tests**: `pixi run test --filter integration/io`
3. **Format Tests**: Test with sample files from each format
4. **Error Handling**: Test with malformed files
5. **Performance Tests**: Large file loading benchmarks
6. **Cross-platform Tests**: Windows/Linux/macOS path handling

### Test Categories to Consider:
- **Valid files**: Known-good URDF/SDF/MJCF files
- **Malformed files**: Invalid XML, missing tags, type errors
- **Large files**: Performance with complex robots/worlds
- **Path handling**: Relative paths, special characters, Unicode
- **Edge cases**: Empty files, minimal files, vendor extensions

## Integration Points

### With Dynamics Module:
- **Skeleton creation**: Loaded skeletons become `Skeleton` objects
- **Joint type mapping**: Format-specific joints to DART joint types
- **Inertia computation**: From geometric properties or explicit values

### With GUI Module:
- **Mesh loading**: Visual geometry from URDF/SDF meshes
- **Material properties**: Colors, textures from file formats
- **Scene organization**: World hierarchy for visualization

### With Python Bindings:
- **API exposure**: `readWorld()`, `readSkeleton()` in Python
- **Format support**: All format options available in Python
- **Error handling**: Python exceptions for parsing errors

## File Format-Specific Notes

### URDF Specifics:
- **Coordinate frames**: Often need transformation to DART conventions
- **Joint limits**: URDF limits may need conversion
- **Mimic joints**: Special handling for coupled joints
- **Visual/collision**: Separate meshes may require unification

### SDF Specifics:
- **World-level properties**: Gravity, time_step, physics engine params
- **Multiple models**: Handle several robots in one file
- **Plugin configuration**: May need special parsing
- **Include/exclude**: Model composition directives

### MJCF Specifics:
- **Actuator models**: Different from URDF joint actuators
- **Contact parameters**: Detailed friction, softness models
- **Tendon modeling**: Special MuJoCo features
- **Custom fields**: MuJoCo-specific extensions

## Getting Help

- **Format specifications**: Use Librarian agent to research URDF/SDF/MJCF standards
- **Parser design**: Use Oracle agent for XML/JSON parsing architecture
- **Performance optimization**: Use Explore agent to find existing IO patterns
- **Cross-platform issues**: Use multi-model orchestration for compatibility

## Files to Understand First

1. `Read.hpp` - Main unified API interface
2. `Read.cpp` - Format detection and loading logic
3. `Export.hpp` - Unified saving/export functionality
4. Test files in `tests/unit/io/` - Expected behavior examples
5. Sample files in `examples/` - Real-world usage patterns

---

*Remember: IO module is the gateway to DART for most users. Maintain robust error handling and format compatibility.*