# Phase 3: USD Support Investigation

**Duration**: DART 7.3 - 7.7 (4+ releases)  
**Priority**: Medium  
**Status**: Investigation Phase

## Overview

Universal Scene Description (USD) is emerging as a universal 3D scene description format with strong backing from NVIDIA, Apple, and Pixar. This phase investigates USD integration opportunities for DART, positioning it at the convergence of robotics, simulation, and computer graphics.

## What is USD?

### Core Concepts
- **.usd files**: Hierarchical scene description with composition
- **Layers**: Non-destructive editing and referencing
- **Prims**: Scene graph elements with attributes
- **Schemas**: Type-safe property definitions
- **Composition**: Assembly of scenes from multiple layers

### Key Features
- **Composition System**: Layer overrides, references, inherits
- **Time Sampling**: Animated properties and keyframes
- **Extensibility**: Custom schemas for domain-specific data
- **Performance**: Lazy loading, paging, streaming
- **Interoperability**: Cross-application compatibility

### Industry Adoption
- **NVIDIA**: Omniverse platform foundation
- **Apple**: ARKit/RealityKit integration
- **Pixar**: Animation production pipeline
- **Blender**: Native USD support
- **Unreal Engine**: USD workflow integration
- **Unity**: Experimental USD support

## Potential USD Applications in DART

### 1. Robot Model Representation
```usda
# robot.usda - USD representation of robot
#usda 1.0

def Xform "Robot" (
    kind = "model"
    customData = {
        string assetType = "robot"
        string framework = "dart"
    }
) {
    def DARTRobot "RobotSchema" (
        apiSchema = ["DARTRobotAPI"]
    ) {
        uniform bool selfCollision = false
        uniform string controllerType = "inverse_dynamics"
        
        def DARTLink "base_link" {
            double mass = 1.0
            double3[] inertia = [0.1, 0, 0, 0.1, 0, 0.1]
            
            def Mesh "visual_geometry" (
                purpose = "render"
            ) {
                rel material = </Robot/RobotSchema/base_link/visual_material>
                asset references = @meshes/base_link.obj@
            }
            
            def Mesh "collision_geometry" (
                purpose = "proxy"
            ) {
                asset references = @meshes/base_link.obj@
            }
        }
        
        def DARTJoint "joint1" {
            token jointType = "revolute"
            rel parent = </Robot/RobotSchema/base_link>
            rel child = </Robot/RobotSchema/link1>
            double3 axis = (0, 0, 1)
            double2 limits = (-1.57, 1.57)
        }
    }
}
```

### 2. Scene and World Composition
```usda
# world.usda - Simulation world with USD composition
#usda 1.0

def Xform "World" {
    # Reference robot model
    override Xform "MyRobot" = </Robot> (
        references = @robot.usda@
    ) {
        double3 xformOp:translate = (0, 0, 1.0)
    }
    
    # Reference environment
    override Xform "Environment" = @environment.usda@</Environment>
    
    # DART-specific physics settings
    def DARTPhysics "WorldPhysics" (
        apiSchema = ["DARTPhysicsAPI"]
    ) {
        double3 gravity = (0, 0, -9.81)
        double timeStep = 0.001
        token collisionDetector = "bullet"
    }
}
```

### 3. Simulation Results and Recording
```usda
# simulation_output.usda - Time-sampled simulation data
#usda 1.0

def Xform "Robot" (
    references = @robot.usda@
) {
    def DARTJoint "joint1" {
        double3 position.timeSamples = {
            0: (0),
            0.016: (0.1),
            0.032: (0.2),
            # ... animation data
        }
        
        double3 velocity.timeSamples = {
            0: (0),
            0.016: (6.25),
            0.032: (6.25),
        }
    }
}
```

## Technical Investigation Areas

### 1. USD SDK Integration

#### Library Dependencies
```toml
# pixi.toml potential additions
[dependencies]
usd-core = "23.5"  # Pixar USD core library
usd-materia = "23.5"  # Material schemas
# Optional specialized packages
usd-physics = "0.1"  # Physics-specific schemas (if available)
```

#### Integration Architecture
```cpp
// dart/usd/UsdParser.hpp
class UsdParser {
public:
    static WorldPtr readWorld(const std::string& usdPath);
    static SkeletonPtr readSkeleton(const std::string& usdPath);
    
    // Write USD representations
    static bool writeWorld(const WorldPtr& world, const std::string& usdPath);
    static bool writeSkeleton(const SkeletonPtr& skeleton, const std::string& usdPath);
    
private:
    // Custom schema handling
    static void registerDartSchemas();
    static SkeletonPtr parseDartRobot(const UsdStageRefPtr& stage, const SdfPath& path);
};
```

### 2. Custom DART Schemas

#### DARTRobot Schema Definition
```cpp
// dart/usd/schemas/DARTRobot.h
TF_DECLARE_PUBLIC_TOKENS(DARTRobotTokens, DART_USD_API, (
    (DARTRobot, "DARTRobot")
    (DARTLink, "DARTLink")
    (DARTJoint, "DARTJoint")
    (mass, "mass")
    (inertia, "inertia")
    (jointType, "jointType")
    (selfCollision, "selfCollision")
));

class DARTRobot : public UsdGeomXform {
public:
    static const TfTokenVector &GetSchemaAttributeNames(bool includeInherited=true);
    
    DARTRobot(const UsdStagePtr &stage, const SdfPath &path);
    
    // DART-specific attributes
    UsdAttribute GetSelfCollisionAttr() const;
    UsdAttribute GetControllerTypeAttr() const;
    
private:
    static UsdSchemaKind _GetSchemaKind() { return UsdSchemaKind::ConcreteTyped; }
};
```

### 3. Performance Considerations

#### USD Performance Characteristics
- **Memory**: Efficient lazy loading and paging
- **Disk**: Binary .usdc format for compact storage
- **Network**: Streaming and partial loading support
- **GPU**: Direct GPU buffer access through USD Hydra

#### DART Integration Challenges
- **Conversion Overhead**: USD ↔ DART object conversion
- **Time Sampling**: Efficient handling of animated data
- **Composition**: Layer complexity and evaluation time
- **Custom Schemas**: Registration and validation overhead

### 4. Use Case Analysis

#### High-Value Applications

##### Digital Twins and Simulation
- **Scenario**: Industrial robot simulation with digital twin integration
- **Benefits**: CAD integration, visualization pipelines, enterprise workflows
- **USD Value**: Composition, asset management, cross-application workflows

##### Research and Academia
- **Scenario**: Academic robotics research requiring visualization
- **Benefits**: Professional visualization, publication-ready graphics
- **USD Value**: Blender integration, rendering pipelines, animation export

##### Autonomous Systems
- **Scenario**: Self-driving simulation with sensor simulation
- **Benefits**: Sensor integration, environment modeling, scenario testing
- **USD Value**: Scene composition, material properties, sensor simulation

##### Lower-Value Applications
- Simple robot control (overhead not justified)
- Legacy projects without visualization needs
- Real-time embedded applications (memory constraints)

## Implementation Roadmap

### Phase 3.1: Investigation and Prototyping (DART 7.3)
**Goals:**
- Evaluate USD SDK integration complexity
- Develop basic USD→DART conversion prototype
- Define custom DART USD schemas
- Performance benchmarking

**Tasks:**
- USD SDK dependency investigation
- Basic USD parsing prototype
- Schema design for DART concepts
- Memory and performance profiling

**Deliverables:**
- USD integration feasibility report
- Basic prototype showing USD file loading
- Custom schema specification
- Performance comparison with existing formats

### Phase 3.2: Core Implementation (DART 7.4-7.5)
**Goals:**
- Implement USD→DART conversion
- Add USD to unified IO API
- Support basic robot model loading
- Implement custom schema validation

**Tasks:**
- Complete UsdParser implementation
- Custom schema registration
- Integration with dart::io
- Comprehensive test suite

**Deliverables:**
- Working USD robot model loading
- Documentation and examples
- Performance benchmarks
- Integration tests

### Phase 3.3: Advanced Features (DART 7.6-7.7)
**Goals:**
- Add DART→USD export capabilities
- Support scene composition
- Time sampling for simulation data
- Advanced USD features

**Tasks:**
- USD export implementation
- Composition system support
- Animation/simulation data handling
- Tool integration examples

**Deliverables:**
- Bidirectional USD support
- Advanced example workflows
- Performance optimization
- Tool integration guides

## Risk Assessment

### High Risks
- **Complexity**: USD learning curve and integration complexity
- **Performance**: Potential overhead for real-time applications
- **Dependencies**: Large USD SDK dependency footprint
- **Maintenance**: Ongoing USD version compatibility

### Medium Risks
- **Adoption**: User community resistance to new format
- **Tooling**: Limited USD tooling in robotics ecosystem
- **Schema Evolution**: Managing custom schema changes over time

### Low Risks
- **Technology Maturity**: USD is well-established and stable
- **Industry Support**: Strong backing from major companies
- **Documentation**: Extensive USD documentation and resources

## Success Criteria

### Phase 3.1 Success
- ✅ USD prototype can load simple robot models
- ✅ Performance overhead <20% compared to URDF loading
- ✅ Custom schema validation working
- ✅ Clear implementation path identified

### Phase 3.2 Success
- ✅ Complex robot models load correctly
- ✅ Full USD integration in dart::io API
- ✅ Comprehensive test coverage
- ✅ Documentation and examples available

### Phase 3.3 Success
- ✅ Round-trip USD↔DART conversion working
- ✅ Advanced USD features (composition, animation) supported
- ✅ Tool integration examples provided
- ✅ Performance meets real-time requirements for use cases

## Dependencies and Prerequisites

### External Dependencies
- **Pixar USD SDK**: Core USD functionality
- **USD Hydra**: Rendering integration (optional)
- **Build System Updates**: CMake/Conan/pixi integration

### Internal Dependencies
- **Phase 1 Completion**: SKEL deprecation (reduces maintenance burden)
- **Phase 2 Foundation**: YAML support infrastructure
- **Resource Management**: Updated asset loading system

### Team Requirements
- **USD Expertise**: Team member(s) with USD experience
- **Graphics Knowledge**: Understanding of scene graph concepts
- **Testing Resources**: Performance benchmarking infrastructure

## Alternative Approaches

### Option A: Minimal USD Support
- Focus only on basic robot model loading
- No custom schemas, use generic USD schemas
- Limited composition support
- **Pros**: Lower complexity, faster implementation
- **Cons**: Limited DART-specific features

### Option B: Full USD Integration
- Complete custom schema development
- Advanced composition and time sampling
- Deep integration with USD ecosystem
- **Pros**: Maximum capability, future-proof
- **Cons**: High complexity, long development time

### Option C: USD as Export Format Only
- Keep existing import formats (URDF/SDF)
- Add USD export for visualization/pipeline integration
- **Pros**: Leverages existing parsers, adds value
- **Cons**: Asymmetric support, limited adoption

### Recommendation
Start with **Option A** for Phase 3.1, evaluate based on user feedback and use case demand before expanding to more comprehensive support.