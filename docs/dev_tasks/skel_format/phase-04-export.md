# Phase 4: Export Capabilities Roadmap

**Duration**: DART 7.4 - 8.0+ (ongoing)  
**Priority**: Medium  
**Status**: Planning

## Overview

This phase completes DART's format ecosystem by adding comprehensive export capabilities, enabling round-trip editing, format conversion, and integration with external tools. This transforms DART from a format consumer to a format-agnostic platform.

## Current State: Read-Only Ecosystem

### Existing Limitations
- **No Export Functions**: Cannot write URDF/SDF/YAML files
- **No Format Conversion**: Cannot convert between formats
- **Round-trip Issues**: Load-modify-save workflows impossible
- **Tool Integration**: Limited to consuming external format files

### Impact on Users
- Manual file editing required for changes
- Cannot programmatically generate robot models
- Limited integration with external pipelines
- Dependency on external conversion tools

## Export Architecture Design

### 1. Core Export Framework

#### Writer Interface Design
```cpp
// dart/io/write/Writer.hpp
template<typename T>
class Writer {
public:
    virtual ~Writer() = default;
    virtual bool write(const T& object, const std::string& path, const WriteOptions& options = {}) = 0;
    virtual std::string writeToString(const T& object, const WriteOptions& options = {}) = 0;
};

// Specialized writers
class UrdfWriter : public Writer<SkeletonPtr> {
public:
    bool write(const SkeletonPtr& skeleton, const std::string& path, const WriteOptions& options) override;
    std::string writeToString(const SkeletonPtr& skeleton, const WriteOptions& options) override;
};

class SdfWriter : public Writer<WorldPtr> {
public:
    bool write(const WorldPtr& world, const std::string& path, const WriteOptions& options) override;
    std::string writeToString(const WorldPtr& world, const WriteOptions& options) override;
};

class YamlWriter : public Writer<SkeletonPtr> {
public:
    bool write(const SkeletonPtr& skeleton, const std::string& path, const WriteOptions& options) override;
    std::string writeToString(const SkeletonPtr& skeleton, const WriteOptions& options) override;
};
```

#### Unified Export API
```cpp
// dart/io/Write.hpp - Extension to unified IO
namespace dart::io {

// Extension to existing unified API
bool writeSkeleton(const SkeletonPtr& skeleton, const std::string& path, const WriteOptions& options = {});
bool writeWorld(const WorldPtr& world, const std::string& path, const WriteOptions& options = {});

// Format-specific convenience functions
bool writeUrdf(const SkeletonPtr& skeleton, const std::string& path, const WriteOptions& options = {});
bool writeSdf(const WorldPtr& world, const std::string& path, const WriteOptions& options = {});
bool writeYaml(const SkeletonPtr& skeleton, const std::string& path, const WriteOptions& options = {});

// Format conversion functions
bool convertSkeleton(const std::string& inputPath, const std::string& outputPath, const WriteOptions& options = {});
bool convertWorld(const std::string& inputPath, const std::string& outputPath, const WriteOptions& options = {});

} // namespace dart::io
```

### 2. WriteOptions Configuration

#### Write Options Structure
```cpp
// dart/io/WriteOptions.hpp
struct WriteOptions {
    // Formatting options
    bool prettyPrint = true;
    int indentSize = 2;
    bool includeComments = true;
    
    // Content options
    bool preserveOriginalNames = true;
    bool includeMetadata = true;
    bool minimizePrecision = false;
    
    // Format-specific options
    struct {
        bool useDartExtensions = true;  // Include DART-specific tags
        bool includeInertialData = true;
        bool includeVisualShapes = true;
        bool includeCollisionShapes = true;
    } urdf;
    
    struct {
        bool useSdf22Features = true;    // Use latest SDF features
        bool includePhysics = true;
        bool includePlugins = true;
    } sdf;
    
    struct {
        bool useArrays = true;           // Use array syntax vs objects
        bool includeAnchors = false;     // Use YAML anchors for duplicates
        bool customSchemaExtensions = true;
    } yaml;
};
```

### 3. Implementation Strategy

#### Phase 4.1: URDF Export (DART 7.4)
```cpp
// URDF Writer Implementation
class UrdfWriter {
private:
    std::string generateLinkXml(const BodyNode* link, const WriteOptions& options);
    std::string generateJointXml(const Joint* joint, const WriteOptions& options);
    std::string generateInertialXml(const InertiaPtr& inertia, const WriteOptions& options);
    std::string generateGeometryXml(const ShapePtr& shape, const WriteOptions& options);
    std::string generateMaterialXml(const ShapePtr& shape, const WriteOptions& options);
    
    // DART-specific extensions
    std::string generateTransmissionXml(const Joint* joint, const WriteOptions& options);
    std::string generateSensorXml(const BodyNode* link, const WriteOptions& options);
};
```

#### Phase 4.2: SDF Export (DART 7.5)
```cpp
// SDF Writer Implementation
class SdfWriter {
private:
    std::string generateWorldXml(const WorldPtr& world, const WriteOptions& options);
    std::string generateModelXml(const SkeletonPtr& skeleton, const WriteOptions& options);
    std::string generatePhysicsXml(const WorldPtr& world, const WriteOptions& options);
    std::string generatePluginXml(const std::string& pluginType, const WriteOptions& options);
    
    // SDF-specific features
    std::string generateIncludeXml(const std::string& uri, const std::string& pose);
    std::string generateLightXml(const LightPtr& light, const WriteOptions& options);
};
```

#### Phase 4.3: YAML Export (DART 7.6)
```cpp
// YAML Writer Implementation  
class YamlWriter {
private:
    void generateRobotYaml(const SkeletonPtr& skeleton, YAML::Emitter& emitter, const WriteOptions& options);
    void generateWorldYaml(const WorldPtr& world, YAML::Emitter& emitter, const WriteOptions& options);
    
    // YAML-specific features
    void generateAnchors(YAML::Emitter& emitter, const SkeletonPtr& skeleton);
    void generateCustomTags(YAML::Emitter& emitter, const WriteOptions& options);
    
    // Array vs object formatting
    void formatInertia(const InertiaPtr& inertia, YAML::Emitter& emitter, const WriteOptions& options);
    void formatTransform(const Eigen::Isometry3d& transform, YAML::Emitter& emitter, const WriteOptions& options);
};
```

### 4. Format Conversion Workflows

#### Conversion Pipeline
```cpp
// High-level conversion implementation
bool convertFile(const std::string& inputPath, const std::string& outputPath, const WriteOptions& options) {
    // 1. Load input file
    auto inputObject = loadFile(inputPath);
    
    // 2. Determine output format
    auto outputFormat = detectFormat(outputPath);
    
    // 3. Select appropriate writer
    auto writer = createWriter(outputFormat, inputObject->getType());
    
    // 4. Write output file
    return writer->write(inputObject, outputPath, options);
}

// Example usage
bool convertUrdfToSdf(const std::string& urdfPath, const std::string& sdfPath) {
    // Load URDF as Skeleton
    auto skeleton = dart::io::readSkeleton(urdfPath);
    
    // Create world with skeleton
    auto world = World::create();
    world->addSkeleton(skeleton);
    
    // Write as SDF
    return dart::io::writeSdf(world, sdfPath);
}
```

#### CLI Tool Integration
```bash
# Command-line conversion tools
pixi run dart-convert --input robot.urdf --output robot.sdf
pixi run dart-convert --input world.yaml --output world.urdf --format skeleton
pixi run dart-convert --input robot.urdf --output robot.yaml --pretty --include-comments

# Batch conversion
pixi run dart-convert-dir --input urdf_models/ --output sdf_models/ --format sdf
pixi run dart-convert-dir --input . --output converted/ --recursive
```

### 5. Advanced Export Features

#### Metadata and Annotations
```cpp
// Metadata preservation and export
struct ExportMetadata {
    std::string sourceFormat;
    std::string sourcePath;
    std::chrono::system_clock::time_point exportTime;
    std::string dartVersion;
    std::map<std::string, std::string> customProperties;
};

class MetadataWriter {
public:
    void addMetadata(const std::string& key, const std::string& value);
    void includeSourceInfo(const std::string& sourcePath);
    void exportWithComments(const WriteOptions& options);
};
```

#### Round-trip Fidelity
```cpp
// Ensuring round-trip accuracy
class RoundTripValidator {
public:
    ValidationResult validateRoundTrip(const std::string& originalPath);
    double compareSkeletons(const SkeletonPtr& original, const SkeletonPtr& loaded);
    bool equivalentInertialProperties(const BodyNode* original, const BodyNode* loaded);
    bool equivalentJointLimits(const Joint* original, const Joint* loaded);
    
    struct ValidationResult {
        bool isEquivalent;
        double structuralAccuracy;  // 0.0 to 1.0
        std::vector<std::string> differences;
        std::vector<std::string> warnings;
    };
};
```

#### Programmable Model Generation
```cpp
// API for generating models programmatically
class ModelBuilder {
public:
    static SkeletonPtr createHumanoid(const std::string& name, double height);
    static SkeletonPtr createManipulator(int dof, double reach);
    static SkeletonPtr createVehicle(int wheels, double mass);
    
    // Configuration-based generation
    static SkeletonPtr fromConfig(const std::string& configPath);
    static SkeletonPtr fromParameters(const ModelParameters& params);
};

// Usage examples
auto robot = ModelBuilder::createHumanoid("my_robot", 1.75);
dart::io::writeUrdf(robot, "humanoid.urdf");
dart::io::writeYaml(robot, "humanoid.yaml");
```

## Implementation Phases

### Phase 4.1: URDF Export Foundation (DART 7.4)
**Duration**: 2 releases  
**Priority**: High

**Goals:**
- Implement basic URDF writer
- Support standard URDF elements
- Add round-trip validation
- CLI conversion tool

**Tasks:**
- `UrdfWriter` class implementation
- `WriteOptions` structure
- Basic round-trip testing
- Command-line conversion tools
- Documentation and examples

**Deliverables:**
- Working URDF export capability
- Basic conversion tools
- Round-trip accuracy >95%
- User documentation

### Phase 4.2: SDF Export (DART 7.5)
**Duration**: 2 releases  
**Priority**: Medium

**Goals:**
- Implement SDF world and model writers
- Support physics and plugins
- Advanced conversion workflows

**Tasks:**
- `SdfWriter` implementation
- Physics properties export
- Plugin configuration support
- World-to-model conversion
- Performance optimization

**Deliverables:**
- Complete SDF export capability
- World and model export
- Advanced conversion workflows
- Integration testing

### Phase 4.3: YAML Export (DART 7.6)
**Duration**: 2 releases  
**Priority**: Medium

**Goals:**
- Implement YAML writers for URDF/SDF equivalents
- Advanced YAML features (anchors, custom tags)
- Integration with Phase 2 YAML support

**Tasks:**
- `YamlWriter` implementation
- Advanced YAML formatting
- Anchor and alias optimization
- Custom schema extensions
- Performance comparison

**Deliverables:**
- Full YAML export capability
- Advanced YAML features
- Integration with YAML import
- Performance benchmarks

### Phase 4.4: Advanced Features (DART 7.7+)
**Duration**: Ongoing  
**Priority**: Low

**Goals:**
- USD export integration
- Advanced conversion workflows
- Performance optimization
- Tool integration

**Tasks:**
- USD writer implementation (if Phase 3 approved)
- Batch processing tools
- Performance optimization
- IDE integration plugins
- Cloud export services

**Deliverables:**
- Complete format ecosystem
- High-performance conversion
- Tool integration
- Cloud services

## Testing Strategy

### Unit Testing
```cpp
// Round-trip accuracy testing
TEST(ExportRoundTrip, UrdfSkeleton) {
    auto original = loadTestSkeleton("complex_robot.urdf");
    auto urdfString = dart::io::writeUrdfToString(original);
    auto loaded = dart::io::readSkeletonFromString(urdfString);
    
    auto validator = RoundTripValidator();
    auto result = validator.validateRoundTrip(original, loaded);
    
    EXPECT_GT(result.structuralAccuracy, 0.95);
    EXPECT_TRUE(result.isEquivalent);
}

// Format conversion testing  
TEST(FormatConversion, UrdfToSdf) {
    auto urdfPath = getTestFile("test_robot.urdf");
    auto sdfPath = createTempFile("converted.sdf");
    
    EXPECT_TRUE(convertUrdfToSdf(urdfPath, sdfPath));
    
    auto world = dart::io::readWorld(sdfPath);
    EXPECT_NE(world, nullptr);
    EXPECT_EQ(world->getNumSkeletons(), 1);
}
```

### Performance Testing
```cpp
// Large model export performance
TEST(Performance, LargeModelExport) {
    auto largeModel = generateLargeRobotModel(1000); // 1000 DOF robot
    
    auto start = std::chrono::high_resolution_clock::now();
    auto urdfString = dart::io::writeUrdfToString(largeModel);
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now() - start);
    
    EXPECT_LT(duration.count(), 1000); // Should export in <1 second
    EXPECT_GT(urdfString.size(), 1000); // Reasonable file size
}
```

### Integration Testing
- End-to-end workflow testing
- External tool compatibility
- Real-world model conversion
- CI/CD pipeline integration

## Performance Considerations

### Memory Efficiency
- Streaming writers for large models
- Memory pooling for string operations
- Lazy evaluation where possible

### Computational Efficiency
- Efficient XML/YAML generation
- Optimized string formatting
- Parallel processing for batch operations

### File Size Optimization
- Optional pretty printing
- Compression for binary formats
- Deduplication for repeated elements

## Error Handling and Validation

### Export Validation
```cpp
struct ExportValidationResult {
    bool isValid;
    std::vector<std::string> errors;
    std::vector<std::string> warnings;
    std::map<std::string, std::string> suggestions;
};

class ExportValidator {
public:
    ExportValidationResult validateSkeleton(const SkeletonPtr& skeleton, Format format);
    ExportValidationResult validateWorld(const WorldPtr& world, Format format);
    
    // Format-specific validation
    ExportValidationResult validateUrdfCompliance(const SkeletonPtr& skeleton);
    ExportValidationResult validateSdfCompliance(const WorldPtr& world);
};
```

### Error Recovery
- Graceful degradation for unsupported features
- Alternative export strategies
- Detailed error reporting with line numbers
- Suggestions for fixing common issues

## Success Metrics

### Phase 4.1 Success
- ✅ URDF export accuracy >95% round-trip
- ✅ Export performance comparable to import
- ✅ All standard URDF features supported
- ✅ CLI conversion tools working

### Phase 4.2 Success
- ✅ Complete SDF export capability
- ✅ Advanced conversion workflows
- ✅ Integration with existing tools
- ✅ Performance meeting requirements

### Phase 4.3 Success
- ✅ YAML export with advanced features
- ✅ Integration with YAML import system
- ✅ Performance optimization complete
- ✅ Comprehensive documentation

### Phase 4.4 Success
- ✅ Complete bidirectional format support
- ✅ High-performance conversion system
- ✅ Tool integration ecosystem
- ✅ User adoption and satisfaction

## Dependencies

### Phase Dependencies
- **Phase 1**: SKEL deprecation (reduces maintenance burden)
- **Phase 2**: YAML support infrastructure
- **Phase 3**: USD support (if implemented)

### Technical Dependencies
- **XML Libraries**: TinyXML2 or pugixml for output
- **YAML Libraries**: rapidyaml for YAML output
- **Testing Framework**: Existing DART testing infrastructure
- **Build System**: Updates to CMake/pixi configuration

### Resource Dependencies
- **Development Time**: 2-3 FTE for 6+ months
- **Testing Resources**: Comprehensive test suite development
- **Documentation**: Technical writing and examples
- **Community Support**: User feedback and issue resolution

## Risk Assessment

### High Risks
- **Complexity**: Supporting all format variants and extensions
- **Performance**: Export overhead for large models
- **Compatibility**: Maintaining compatibility with external tools
- **Maintenance**: Ongoing format standard updates

### Medium Risks
- **Adoption**: User migration from external conversion tools
- **Testing**: Comprehensive test coverage development
- **Documentation**: Complete and accurate documentation

### Low Risks
- **Technical Feasibility**: Well-understood problem domain
- **Integration**: Builds on existing architecture
- **Tooling**: Good library support for output formats

## Future Extensions

### Advanced Features
- **Template System**: Parameterized model generation
- **Plugin Architecture**: Extensible format support
- **Cloud Integration**: Web-based conversion services
- **AI-Assisted Export**: Intelligent format optimization

### Ecosystem Integration
- **IDE Plugins**: VS Code, CLion integration
- **CI/CD Integration**: Automated format validation
- **Design Tools**: Integration with CAD/CAM software
- **Simulation Platforms**: Direct integration with simulators