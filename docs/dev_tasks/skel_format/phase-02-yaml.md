# Phase 2: YAML Support Specification

**Duration**: DART 7.2 - 7.4 (3 releases)  
**Priority**: Medium  
**Status**: Planning

## Overview

This phase adds YAML front-ends for existing formats (URDF/SDF) rather than converting SKEL to YAML. This approach provides human-readable configuration while maintaining compatibility with the broader robotics ecosystem.

## Design Philosophy

### Why YAML Front-ends, Not YAML SKEL?
1. **Ecosystem Compatibility**: YAML files convert to standard URDF/SDF internally
2. **Tool Interoperability**: Existing URDF/SDF tools continue to work
3. **Migration Path**: Users can convert YAML to XML when needed
4. **Maintenance**: Leverages existing robust parsers

### YAML Benefits Over XML
- **Human Readability**: Cleaner syntax, less verbose
- **Array Support**: Native arrays for transforms/inertia
- **Comments**: More readable inline documentation
- **Editing**: Better editor support and validation

## Technical Specification

### 1. YAML Schema Design

#### URDF YAML Schema
```yaml
# robot.yaml - URDF equivalent in YAML format
robot:
  name: "my_robot"
  
  links:
    - name: "base_link"
      inertial:
        mass: 1.0
        inertia: [0.1, 0, 0, 0.1, 0, 0.1]  # Ixx, Ixy, Ixz, Iyy, Iyz, Izz
        origin:
          xyz: [0.0, 0.0, 0.0]
          rpy: [0.0, 0.0, 0.0]
      
      visual:
        geometry:
          box:
            size: [1.0, 1.0, 1.0]
        material:
          color:
            rgba: [1.0, 0.0, 0.0, 1.0]
      
      collision:
        geometry:
          cylinder:
            radius: 0.05
            length: 1.0
    
    - name: "link1"
      inertial:
        mass: 0.5
        inertia: [0.05, 0, 0, 0.05, 0, 0.05]
        origin:
          xyz: [0.0, 0.0, 0.5]
          rpy: [0.0, 0.0, 0.0]

  joints:
    - name: "joint1"
      type: "revolute"
      parent: "base_link"
      child: "link1"
      
      origin:
        xyz: [0.0, 0.0, 0.5]
        rpy: [0.0, 0.0, 0.0]
      
      axis:
        xyz: [0.0, 0.0, 1.0]
      
      limit:
        lower: -1.57
        upper: 1.57
        effort: 10.0
        velocity: 1.0
```

#### SDF YAML Schema
```yaml
# world.yaml - SDF equivalent in YAML format
world:
  name: "my_world"
  
  physics:
    type: "ode"
    max_step_size: 0.01
    real_time_factor: 1.0
    gravity:
      xyz: [0.0, 0.0, -9.81]
  
  models:
    - name: "ground_plane"
      static: true
      links:
        - name: "link"
          collision:
            - name: "collision"
              geometry:
                plane:
                  normal: [0.0, 0.0, 1.0]
                  size: [100.0, 100.0]
    
    - name: "robot"
      include:
        uri: "model://robot_model"  # Supports SDF includes
        pose: [0.0, 0.0, 0.5, 0.0, 0.0, 0.0]
```

### 2. Parser Architecture

#### YAML Front-end Design
```cpp
// dart/utils/yaml/YamlParser.hpp
class YamlParser {
public:
  static WorldPtr readWorld(const std::string& uri);
  static SkeletonPtr readSkeleton(const std::string& uri);
  
  // Convert YAML to URDF/SDF in memory
  static std::string convertToUrdf(const std::string& yamlContent);
  static std::string convertToSdf(const std::string& yamlContent);
  
private:
  // Internal conversion logic
  static void processRobotNode(const YAML::Node& node, std::string& urdfOutput);
  static void processWorldNode(const YAML::Node& node, std::string& sdfOutput);
};
```

#### Integration with Unified IO
```cpp
// In dart/io/Read.cpp
WorldPtr readWorld(const std::string& uri, const ReadOptions& options) {
  // Extension-based format detection
  if (extension == ".yml" || extension == ".yaml") {
    // Auto-detect format type (robot vs world)
    if (isRobotYaml(uri)) {
      std::string urdfContent = YamlParser::convertToUrdf(content);
      return UrdfParser::readWorldFromString(urdfContent, options);
    } else {
      std::string sdfContent = YamlParser::convertToSdf(content);
      return SdfParser::readWorldFromString(sdfContent, options);
    }
  }
  // ... existing format detection
}
```

### 3. Implementation Plan

#### Phase 2.1: Core YAML Support (DART 7.2)
- Add `rapidyaml` dependency to build system
- Implement basic YAML parsing utilities
- Create schema definition for URDF YAML
- Add unit tests for conversion accuracy

#### Phase 2.2: URDF YAML Front-end (DART 7.3)
- Implement YAML→URDF conversion
- Integrate with `dart::io` unified API
- Add comprehensive test coverage
- Update documentation with examples

#### Phase 2.3: SDF YAML Front-end (DART 7.4)
- Implement YAML→SDF conversion
- Support SDF-specific features (includes, models)
- Performance optimization and benchmarking
- Tool integration examples

### 4. Library Dependencies

#### rapidyaml (Recommended)
```toml
# pixi.toml addition
[dependencies]
rapidyaml = "0.5.0"  # Header-only, high-performance
```

**Advantages:**
- 10-100x faster than yaml-cpp
- Low memory allocation
- Modern C++11 interface
- Header-only, easy integration

#### Alternative: yaml-cpp
- Widely used but slower
- More complex API
- Heavier dependency

### 5. Performance Considerations

#### Conversion Overhead
- **YAML Parse**: ~1-2ms for typical robot files (rapidyaml)
- **URDF/SDF Generation**: ~0.5-1ms
- **Final Parse**: Existing parser performance
- **Total Overhead**: <5ms for most use cases

#### Optimization Strategies
- Cache converted URDF/SDF in memory
- Lazy conversion (only when needed)
- Parallel processing for multiple files
- Pre-compiled schemas for validation

### 6. Validation and Error Handling

#### Schema Validation
```cpp
class YamlSchemaValidator {
public:
  static ValidationResult validateRobotYaml(const std::string& content);
  static ValidationResult validateWorldYaml(const std::string& content);
  
  struct ValidationResult {
    bool isValid;
    std::vector<std::string> errors;
    std::vector<std::string> warnings;
  };
};
```

#### Error Reporting
- Clear, human-readable error messages
- Line/column numbers for YAML syntax errors
- Schema validation errors with context
- Suggestions for common issues

### 7. Testing Strategy

#### Unit Tests
- Schema validation accuracy
- Conversion correctness (YAML→URDF→YAML roundtrip)
- Performance benchmarks vs XML
- Edge cases and malformed input

#### Integration Tests
- End-to-end loading in DART simulations
- Compatibility with existing tools
- Real-world robot models conversion
- CI pipeline integration

#### Example Test Cases
```cpp
TEST(YamlParser, ConvertSimpleRobot) {
  std::string yaml = loadTestFile("simple_robot.yaml");
  std::string expectedUrdf = loadTestFile("simple_robot.urdf");
  
  auto converted = YamlParser::convertToUrdf(yaml);
  EXPECT_EQ(normalizeXml(converted), normalizeXml(expectedUrdf));
}

TEST(YamlParser, PerformanceBenchmark) {
  auto yaml = loadTestFile("complex_robot.yaml");
  
  auto start = std::chrono::high_resolution_clock::now();
  auto world = YamlParser::readWorld(yaml);
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::high_resolution_clock::now() - start);
  
  EXPECT_LT(duration.count(), 10); // Should load in <10ms
}
```

### 8. Documentation and Examples

#### User Documentation
- "Getting Started with YAML Robot Models"
- "YAML vs XML: When to Use Each Format"
- "Converting Existing Models to YAML"
- "YAML Schema Reference"

#### Example Files
- `examples/yaml/simple_robot.yaml`
- `examples/yaml/complex_world.yaml`
- `examples/yaml/conversion_examples/`
- Migration scripts and tools

### 9. Migration Tools

#### XML→YAML Converter
```python
# scripts/convert_xml_to_yaml.py
def convert_urdf_to_yaml(urdf_path, yaml_path):
    """Convert URDF file to equivalent YAML format"""
    # Parse URDF using existing DART utilities
    # Generate equivalent YAML structure
    # Validate and save
```

#### Batch Conversion
```bash
# Convert entire directories
pixi run convert-xml-dir --input skel_files/ --output yaml_files/ --format urdf
```

### 10. Future Extensions

#### Advanced YAML Features
- Anchors and aliases for repeated elements
- Custom tags for DART-specific extensions
- Environment variable substitution
- Conditional includes

#### Tool Integration
- VS Code YAML schema validation
- Robot model viewers with YAML support
- CI/CD pipeline YAML validation steps
- Python/ROS integration utilities