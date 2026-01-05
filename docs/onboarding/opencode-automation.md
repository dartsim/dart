# OpenCode Automation for DART

This guide documents the automated code improvement and performance monitoring system using OpenCode agents integrated into DART's CI/CD pipeline.

## Overview

DART uses automated GitHub Actions workflows to continuously improve code quality and monitor performance:

- **C++20 Modernization**: Weekly automated updates to adopt modern C++ patterns
- **Performance Monitoring**: Daily benchmarking with regression detection
- **Code Quality**: Continuous integration with clang-tidy and formatting

## Automation Workflows

### 1. C++20 Modernization (`opencode_modernization.yml`)

**Schedule**: Weekly on Monday at 2 AM UTC

**Purpose**: Apply safe C++20 modernizations and code quality improvements

**Scope Options**:
- `all`: All codebase (default)
- `dart-gui`: GUI components only
- `collision`: Collision detection modules
- `dynamics`: Dynamics engine
- `simulation`: Simulation framework
- `io`: I/O and parsing modules

**Modernizations Applied**:
- `modernize-*` clang-tidy checks (conservative subset)
- Performance optimizations
- Code readability enhancements
- Safe refactorings

**What it Does**:
1. Creates `.clang-tidy` configuration for modern C++ patterns
2. Runs clang-tidy with focused check sets
3. Verifies compilation after changes
4. Runs baseline benchmarks
5. Creates pull request with improvements
6. Adds detailed comments explaining changes

**Safety Features**:
- Conservative check selection (avoid breaking changes)
- Compilation verification
- Scope limiting for targeted improvements
- Manual trigger option for specific modules

### 2. Performance Monitoring (`performance_monitoring.yml`)

**Schedule**: Daily at 3 AM UTC

**Purpose**: Detect performance regressions and track improvements

**Features**:
- Automated benchmark execution
- Comparison with baseline (main branch)
- Statistical analysis of performance changes
- Regression detection (>5% slowdown threshold)
- Improvement recognition (>10% speedup threshold)
- Automatic issue creation for regressions

**Benchmark Coverage**:
- `bm_boxes`: Collision detection performance
- `bm_kinematics`: Kinematic calculations
- `bm_lcpsolver`: Constraint solver performance

**Configuration**:
- Consistent test environment
- Multiple repetitions for statistical significance
- Results stored as JSON for analysis
- 30-day artifact retention

## Usage Guide

### Running Modernization Manually

```bash
# Trigger modernization for specific scope
gh workflow run opencode_modernization.yml \
  --field scope=dart-gui

# Run for all code
gh workflow run opencode_modernization.yml
```

### Running Performance Analysis

```bash
# Analyze all benchmarks
gh workflow run performance_monitoring.yml

# Analyze specific benchmark
gh workflow run performance_monitoring.yml \
  --field benchmark_filter=bm_boxes

# Compare against custom baseline
gh workflow run performance_monitoring.yml \
  --field baseline_branch=feature/my-feature
```

### Reviewing Automation Results

1. **Modernization PRs**: Look for `🤖 OpenCode:` prefixed pull requests
2. **Performance Issues**: Check for `📊 Performance Regression` issues
3. **Artifacts**: Download benchmark results from Actions tab
4. **Reports**: Review automated performance reports

## Configuration

### clang-tidy Configuration

The modernization workflow creates a temporary `.clang-tidy` file with:

```yaml
Checks: >
  modernize-*
  -modernize-use-trailing-return-type
  -modernize-use-auto
  readability-*
  performance-*
  -readability-identifier-naming
  -readability-magic-numbers
  bugprone-*
  -bugprone-narrowing-conversions
  concurrency-*
  -concurrency-mt-unsafe
```

### Performance Thresholds

- **Regression**: >5% slowdown
- **Improvement**: >10% speedup
- **Significance**: Based on 5 repetitions with 0.5s minimum time

## Integration with Existing Workflows

### CI Integration

- Modernization runs before daily CI to avoid conflicts
- Performance monitoring uses same build configuration as CI
- Results stored separately from CI artifacts

### Pixi Task Integration

Automation leverages existing `pixi` tasks:
- `pixi run config`: Build configuration
- `pixi run -- bash -lc`: Isolated execution
- Benchmark executables from standard build

## Best Practices

### For Maintainers

1. **Review Automation PRs**: Check modernization PRs weekly
2. **Monitor Performance**: Address regression issues promptly
3. **Update Configuration**: Adjust clang-tidy checks as needed
4. **Benchmark Maintenance**: Keep benchmarks relevant and fast

### For Contributors

1. **Avoid Conflicts**: Sync main branch before weekend work
2. **Performance Awareness**: Test benchmarks before PRs
3. **Modern Code**: Use C++20 features when appropriate
4. **Baseline Updates**: Update benchmarks when changing algorithms

## Monitoring and Troubleshooting

### Checking Automation Status

```bash
# List recent automation runs
gh run list --workflow=opencode_modernization.yml
gh run list --workflow=performance_monitoring.yml

# View specific run
gh run view <run-id>

# Check artifacts
gh run view <run-id> --json artifacts
```

### Common Issues

**Modernization Conflicts**:
- Occur when changes are in progress during Monday run
- Solution: Rebase on main and resolve conflicts

**Performance Noise**:
- CI environment variability
- Solution: Multiple repetitions and statistical analysis

**Build Failures**:
- Compilation issues after modernization
- Solution: Review clang-tidy changes, adjust configuration

## Security and Permissions

### Required Permissions

```yaml
permissions:
  contents: write      # For creating PRs and issues
  pull-requests: write # For PR management
  actions: read       # For artifact access
```

### Token Usage

- Uses `GITHUB_TOKEN` for operations
- No external service integrations
- All automation contained within repository

## Future Enhancements

### Planned Features

1. **Custom Agents**: Specialized agents for specific patterns
2. **Machine Learning**: Performance prediction and optimization
3. **Integration Testing**: Extended automated testing
4. **Documentation Updates**: Automated API documentation

### Extension Points

- Add new benchmark targets
- Extend clang-tidy configurations
- Create module-specific modernizations
- Add performance visualization

## Contributing to Automation

### Adding New Benchmarks

1. Create benchmark in `tests/benchmark/`
2. Add to `CMakeLists.txt`
3. Update performance monitoring workflow
4. Test with manual workflow run

### Modifying Modernization

1. Update `.clang-tidy` configuration in workflow
2. Test with scope-limited run
3. Review changes in test PR
4. Update documentation

### Adding New Workflows

1. Follow existing patterns in `.github/workflows/`
2. Use consistent naming and permissions
3. Include proper error handling
4. Add documentation

## Related Documentation

- [CI/CD System](ci-cd.md) - Core CI/CD infrastructure
- [Testing Guide](testing.md) - Test suite organization
- [Build System](build-system.md) - Build configuration
- [Code Style](code-style.md) - Coding standards

## Support and Issues

For automation-related issues:

1. Check workflow logs in GitHub Actions tab
2. Review recent automation PRs and issues
3. Consult this documentation
4. Create issue with `automation` label

This automation system enhances DART's code quality and performance while maintaining development velocity and reliability.