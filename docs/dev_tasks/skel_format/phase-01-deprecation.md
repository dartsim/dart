# Phase 1: SKEL Format Deprecation Plan

**Duration**: DART 7.1 - 7.2 (2 releases)  
**Priority**: High  
**Status**: Planning

## Overview

This phase systematically deprecates the SKEL format while maintaining backward compatibility and providing clear migration paths for existing users.

## Deprecation Timeline

### DART 7.1 - Deprecation Warnings
- Add runtime deprecation warnings to `SkelParser`
- Update all SKEL-related documentation with deprecation notices
- Begin migration of examples and tutorials to URDF/SDF

### DART 7.2 - Documentation Migration
- Remove SKEL examples from main documentation
- Update tutorials to use URDF/SDF exclusively
- Add migration guide to main documentation

### DART 8.0 - Removal (Future)
- Complete removal of `SkelParser`
- Remove SKEL test files and examples
- Final cleanup of SKEL references

## Implementation Tasks

### 1. Code Changes

#### Add Deprecation Warnings
```cpp
// In dart/utils/SkelParser.cpp
WorldPtr SkelParser::readWorld(const std::string& uri) {
  DART_DEPRECATED("SKEL format is deprecated. Please use URDF or SDF instead. "
                  "See docs/dev_tasks/skel_format/migration-guide.md");
  // ... existing implementation
}
```

#### Update Unified IO API
```cpp
// In dart/io/Read.cpp
if (extension == ".skel") {
  DART_WARN("Loading .skel file: SKEL format is deprecated. "
            "Consider converting to URDF/SDF format.");
  return SkelParser::readWorld(uri);
}
```

### 2. Documentation Updates

#### Update Onboarding Guides
- `docs/onboarding/io-parsing.md` - Add deprecation notice
- `docs/onboarding/README.md` - Update format recommendations
- All tutorial README files - Replace SKEL examples

#### Create Deprecation Notices
Add consistent deprecation banners:
```markdown
> **⚠️ Deprecation Notice**: The SKEL format is deprecated and will be removed in DART 8.0. 
> Please use [URDF](urdf.md) or [SDF](sdf.md) formats instead. 
> See our [migration guide](migration-guide.md) for converting existing SKEL files.
```

### 3. Example Migration

#### Convert Core Examples
Identify and convert high-impact examples:
- `data/skel/biped.skel` → `data/urdf/biped.urdf`
- `data/skel/test/test_shapes.skel` → `data/sdf/test_shapes.sdf`
- Tutorial examples and code snippets

#### Automated Conversion Script
Create a utility script to help users convert SKEL to URDF:
```python
# scripts/convert_skel_to_urdf.py
# - Load SKEL file using DART
# - Export to URDF structure
# - Handle transform conversions
# - Report conversion statistics
```

## Communication Strategy

### Release Notes
- DART 7.1: Announce deprecation with migration path
- DART 7.2: Emphasize upcoming removal, highlight conversion tools
- DART 8.0: Final removal announcement

### Documentation Updates
- Prominent deprecation notices in all SKEL-related docs
- Clear migration paths and conversion examples
- FAQ addressing common migration concerns

### Community Outreach
- Blog post announcing SKEL deprecation
- GitHub issue template for migration questions
- ROS Discourse post for community awareness

## Risk Assessment

### High Risk Areas
- Legacy users with large SKEL file collections
- Academic papers referencing SKEL examples
- CI/CD pipelines using SKEL test files

### Mitigation Strategies
- Provide robust conversion tools
- Maintain examples in separate repository
- Extended support period for critical cases
- Clear documentation for all migration scenarios

## Success Metrics

### Quantitative
- 90% reduction in SKEL file usage in examples
- 75% reduction in SKEL-related support issues
- 100% of documentation pointing to URDF/SDF

### Qualitative
- User feedback on migration experience
- Community understanding of deprecation timeline
- Successful transitions of high-profile projects

## Dependencies

### Required
- Conversion script development
- Documentation updates
- CI pipeline updates

### Optional
- External SKEL→URDF/SDF conversion tools
- Migration assistance for major projects
- Backward compatibility testing suite

## Rollback Plan

If critical issues arise:
- Suspend deprecation warnings for one release
- Provide extended support timeline
- Revert documentation changes temporarily
- Engage with affected users directly

## Next Phase Readiness

This phase enables Phase 2 (YAML support) by:
- Reducing legacy format maintenance burden
- Establishing clear format preferences (URDF/SDF)
- Providing conversion tools for existing content
- Setting expectations for future format evolution