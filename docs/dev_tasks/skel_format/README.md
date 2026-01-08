# SKEL Format Evolution Plan

**Status**: Planning Phase  
**Target Completion**: DART 7.x lifecycle  
**Issue Reference**: [GitHub #496](https://github.com/dartsim/dart/issues/496) (2015)

## Executive Summary

This document outlines the strategic evolution of DART's native SKEL file format. Rather than directly converting SKEL XML to YAML as proposed in 2015, we recommend a phased approach that:

1. **Deprecates SKEL** (legacy format)
2. **Enhances existing standard formats** (URDF/SDF) with YAML alternatives
3. **Invests in emerging standards** (USD)
4. **Adds export capabilities** for format interchange

## Current State Analysis

### SKEL Format Status
- **Format**: Custom XML schema (`.skel` extension)
- **Status**: Legacy, read-only since DART 7
- **Parser**: `dart::utils::SkelParser` using TinyXML2
- **Issues**: Verbose, string-based data representation, no export

### Current Ecosystem
- **Strong**: URDF (`urdfdom`), SDF (`libsdformat`)
- **Experimental**: MJCF support
- **Missing**: USD support, export capabilities, format conversion

## Strategic Decision: Don't Convert SKEL → YAML

**Rationale:**
1. **SKEL is deprecated** - users should use URDF/SDF
2. **Industry standards exist** - URDF (ROS), SDF (Gazebo)
3. **YAML adoption is elsewhere** - MuJoCo, Drake already support YAML
4. **Investment ROI** - better to enhance existing formats than legacy

## Four-Phase Evolution Plan

### Phase 1: SKEL Deprecation (Immediate)
- Add deprecation warnings to `SkelParser`
- Update documentation and examples
- Remove SKEL from tutorials

### Phase 2: YAML Front-ends (Short-term)
- Add YAML alternatives for URDF/SDF parsing
- Use high-performance YAML parsers (rapidyaml)
- Maintain compatibility with existing tools

### Phase 3: USD Support (Medium-term)
- Investigate USD implementation for scene description
- Position DART for future robotics graphics convergence

### Phase 4: Export Capabilities (Long-term)
- Implement URDF/SDF/YAML writers
- Enable format conversion workflows
- Complete bidirectional format support

## Implementation Timeline

| Phase | Duration | Priority | Dependencies |
|-------|----------|----------|---------------|
| Phase 1 | 1-2 releases | High | Documentation updates |
| Phase 2 | 2-3 releases | Medium | YAML parser integration |
| Phase 3 | 3-4 releases | Medium | USD SDK evaluation |
| Phase 4 | 4+ releases | Medium | Phases 1-2 completion |

## Success Criteria

- **Phase 1**: All documentation points users to URDF/SDF
- **Phase 2**: YAML URDF/SDF files load natively in DART
- **Phase 3**: USD files can be loaded into DART worlds
- **Phase 4**: Round-trip editing possible between formats

## Impact Assessment

### Benefits
- Reduced maintenance burden (legacy format removal)
- Better ecosystem compatibility (standard formats)
- Future-proof positioning (USD adoption)
- Improved user experience (YAML readability)

### Risks
- Legacy user disruption during deprecation
- Additional parser maintenance (YAML front-ends)
- USD implementation complexity

### Mitigations
- Clear migration paths and documentation
- Use established YAML libraries
- Phased USD implementation with clear use cases

## Alternatives Considered

1. **Direct SKEL→YAML Conversion** - Rejected (investment in legacy)
2. **Complete SKEL Removal** - Too disruptive for existing users
3. **JSON Alternative** - YAML superior for human readability
4. **Custom Format** - Rejected (industry alignment important)

## Related Documents

- [Phase 1: SKEL Deprecation Plan](phase-01-deprecation.md)
- [Phase 2: YAML Support Specification](phase-02-yaml.md)
- [Phase 3: USD Support Investigation](phase-03-usd.md)
- [Phase 4: Export Capabilities Roadmap](phase-04-export.md)
- [User Migration Guide](migration-guide.md)

## References

- Original GitHub Issue #496 (2015): "Proposal: Use YAML for .skel files"
- DART 7 Migration Documentation
- URDF Specification (ROS)
- SDF Specification (Gazebo)
- USD Specification (Pixar/NVIDIA)