# CI/CD System

## Overview

DART uses GitHub Actions for continuous integration and deployment. The CI system validates code quality, runs tests across multiple platforms, builds documentation, and publishes Python wheels.

## Workflow Architecture

### Core CI Workflows

| Workflow             | Purpose               | Platforms      | Trigger                    |
| -------------------- | --------------------- | -------------- | -------------------------- |
| `ci_ubuntu.yml`      | Build, test, coverage | Ubuntu         | PR, push, schedule         |
| `ci_macos.yml`       | Build, test           | macOS          | PR, push, schedule         |
| `ci_windows.yml`     | Build, test           | Windows        | PR, push, schedule         |
| `ci_gz_physics.yml`  | Gazebo integration    | Ubuntu         | PR, push, schedule         |
| `api_doc.yml`        | API documentation     | Ubuntu         | Push to main, docs changes |
| `publish_dartpy.yml` | Python wheels         | Multi-platform | Push, schedule, tags       |

### Design Principles

**Optimize for fast feedback on PRs:**

- Essential validations run on every PR
- Full matrix testing runs on main branch and releases
- Debug builds run on schedule (2x per week)

**Efficient resource usage:**

- Compilation caching (sccache) reduces build time by 50-70%
- Path filtering prevents unnecessary workflow runs
- Conditional execution skips non-essential jobs on PRs

**Maintain full test coverage:**

- All tests run on at least one platform per PR
- Complete platform matrix on main branch
- Scheduled runs ensure periodic full validation

## Compilation Caching Strategy

### Why Caching Matters

DART compilation takes 15-25 minutes per build without caching. With proper caching:

- **First build**: Normal compilation time (populates cache)
- **Subsequent builds**: 50-70% faster (only changed files recompile)

### sccache (All Platforms)

We standardized on sccache everywhere because it:

- Works with Clang, GCC, and MSVC (including PDB handling)
- Supports remote cache backends (if we add them later)
- Ships an official GitHub Action with built-in metrics

**Linux/macOS setup** (see `.github/workflows/ci_ubuntu.yml` and `ci_macos.yml`):

```yaml
- name: Setup sccache
  uses: mozilla-actions/sccache-action@v0.0.9

- name: Configure environment for sccache
  run: |
    echo "SCCACHE_GHA_ENABLED=true" >> $GITHUB_ENV
    echo "CMAKE_C_COMPILER_LAUNCHER=sccache" >> $GITHUB_ENV
    echo "CMAKE_CXX_COMPILER_LAUNCHER=sccache" >> $GITHUB_ENV
```

**Windows setup** (see `.github/workflows/ci_windows.yml`):

```yaml
- name: Setup sccache
  uses: mozilla-actions/sccache-action@v0.0.9

- name: Configure environment for sccache
  shell: powershell
  run: |
    echo "SCCACHE_GHA_ENABLED=true" | Out-File -FilePath $env:GITHUB_ENV -Encoding utf8 -Append
    echo "CMAKE_C_COMPILER_LAUNCHER=sccache" | Out-File -FilePath $env:GITHUB_ENV -Encoding utf8 -Append
    echo "CMAKE_CXX_COMPILER_LAUNCHER=sccache" | Out-File -FilePath $env:GITHUB_ENV -Encoding utf8 -Append
```

**Local builds:** `pixi` auto-detects sccache. If the CLI finds `sccache` on PATH (and you have not overridden `CMAKE_*_COMPILER_LAUNCHER`), it sets the launchers before invoking CMake so your local builds benefit from the same cache.

## MSVC Multi-Core Compilation

**Critical configuration** (`CMakeLists.txt` line 282-284):

```cmake
# /MP - Multi-processor compilation (uses all available cores)
# /FS - Force synchronous PDB writes (prevents PDB conflicts in parallel builds with /MP)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc /permissive- /Zc:twoPhase- /MP /FS")
```

**Two levels of parallelization:**

1. **Intra-target parallelization** (`/MP` flag):
   - Multiple cores compile within a single target
   - MSVC automatically uses all available CPU cores
   - Significantly speeds up compilation of large source files

2. **Inter-target parallelization** (`--parallel` flag):
   - Multiple targets build simultaneously
   - Configured in `pixi.toml`: `cmake --build build/... --parallel`
   - Visual Studio generator coordinates parallel target builds

**Important:** This is already optimally configured. Do not add redundant parallelization flags.

## Conditional Execution Patterns

### Debug Builds

Debug builds are slower and primarily useful for debugging, not CI validation.

**Pattern** (see `ci_ubuntu.yml` and `ci_macos.yml`):

```yaml
build:
  name: ${{ matrix.build_type }}
  runs-on: ubuntu-latest
  if: matrix.build_type == 'Release' || github.event_name == 'schedule' || github.ref == 'refs/heads/main'
  strategy:
    matrix:
      build_type: ["Release", "Debug"]
```

**Behavior:**

- **PRs**: Only Release builds run
- **Main branch**: Both Release and Debug run
- **Scheduled runs**: Both Release and Debug run
- **Savings**: 30-40 minutes per PR

### Python Wheel Builds

Full wheel matrix is expensive (60-90 minutes) and only needed for releases.

**Pattern** (see `publish_dartpy.yml`):

```yaml
build_wheels:
  if: |
    matrix.release_only == false ||
    github.ref == 'refs/heads/main' ||
    startsWith(github.ref, 'refs/tags/') ||
    github.event_name == 'schedule'
```

**Behavior:**

- **PRs**: Only essential configurations build
- **Main branch**: Full matrix builds
- **Release tags**: Full matrix builds
- **Scheduled runs**: Full matrix builds

### Documentation Builds

API docs only need rebuilding when documentation or public headers change.

**Pattern** (see `api_doc.yml`):

```yaml
on:
  push:
    branches: ["main"]
    paths:
      - "docs/**"
      - "dart/**/*.hpp"
      - ".github/workflows/api_doc.yml"
  pull_request:
    paths:
      - "docs/**"
      - ".github/workflows/api_doc.yml"
```

**Behavior:**

- **PRs**: Only runs if docs/headers change
- **Main branch**: Only runs if docs/headers change
- **Savings**: 20-30 minutes on most PRs

## Lint Check Strategy

**Centralized approach:** Linting is deterministic and platform-independent, so running it once is sufficient.

**Implementation** (see `ci_ubuntu.yml`):

```yaml
- name: Check Lint
  if: matrix.build_type == 'Release'
  run: |
    DART_VERBOSE=ON \
    BUILD_TYPE=Release \
    pixi run check-lint
```

**Behavior:**

- Only runs on Ubuntu Release build
- Removed from macOS and Windows workflows
- Savings: 12-25 minutes per PR

## Testing Strategy

### Test Coverage

**Per PR:**

- Ubuntu Release: Full tests + coverage
- macOS Release: Full tests
- Windows Release: Full tests
- Gazebo integration: Integration tests

**Scheduled runs:**

- Add Debug builds for all platforms
- Ensure periodic full validation

### Test Execution

All tests run through `pixi run test-all`, which includes:

- Linting (check-lint)
- Unit tests (C++)
- Python tests (dartpy)
- Documentation build (optional)

**Best practice:** Don't duplicate steps already in `test-all`.

## Monitoring and Maintenance

### Expected CI Times

**Without caching (first run):**

- Ubuntu: 45-60 min
- macOS: 30-45 min
- Windows: 25-35 min
- Total: ~100-150 min

**With caching (subsequent runs):**

- Ubuntu: 20-30 min
- macOS: 15-25 min
- Windows: 15-20 min
- Total: ~30-60 min (50-70% reduction)

### Cache Health

**Monitor cache hit rates:**

- Target: >80% after first run
- Check cache statistics in workflow logs
- Adjust `max-size` if caches frequently exceed limits

**Cache invalidation:**

- Caches are specific to OS + build type
- Automatic invalidation on cache key change
- Manual cache clearing via GitHub UI if needed

### Maintenance Tasks

**Regular:**

- Review average CI times weekly
- Monitor cache hit rates
- Check for test flakiness

**Quarterly:**

- Update GitHub Actions versions
- Review and optimize cache sizes
- Evaluate new optimization opportunities

## Troubleshooting

### Slow CI Builds

**Check:**

1. Cache hit rate in workflow logs
2. Whether ccache/sccache is being used (look for "compiler launcher" in logs)
3. If Debug builds are running on PRs (should only run on schedule)

**Solutions:**

- Clear GitHub Actions cache and rebuild
- Verify `CMAKE_*_COMPILER_LAUNCHER` environment variables are set
- Check conditional execution logic

### Cache-Related Build Failures

**Symptoms:**

- Build succeeds locally but fails in CI
- Errors about missing headers or outdated objects

**Solutions:**

- Force cache bust by changing cache key
- Add dependency tracking to cache key (e.g., hash of `pixi.lock`)
- Run full clean build on schedule to catch issues

### Platform-Specific Issues

**Debug-only failures:**

- Debug builds still run on schedule
- Check scheduled workflow runs for failures
- Debug-specific issues (assertions, memory checks) caught there

**Single-platform failures:**

- Ensure test runs on at least one platform
- Review platform-specific conditionals
- Check if issue is in platform-specific code

## Best Practices

### When Adding New Tests

1. **Add to appropriate test suite** (unit, integration, Python)
2. **Verify test runs on CI** by checking workflow logs
3. **Keep tests fast** - slow tests impact developer productivity
4. **Avoid flaky tests** - use proper synchronization and timeouts

### When Modifying Workflows

1. **Test in a branch first** before merging to main
2. **Monitor impact** on CI times for next 5-10 PRs
3. **Document changes** in workflow comments
4. **Maintain backward compatibility** where possible

### When Adding Dependencies

1. **Update `pixi.toml`** with version constraints
2. **Run `pixi install`** to update lock file
3. **Consider cache impact** - large dependencies slow cache restore
4. **Test on all platforms** before merging

## Performance Optimization History

**Phase 1: Quick Wins** (Implemented)

- Centralized lint checks: 12-25 min saved
- Conditional Debug builds: 30-40 min saved
- Optimized wheel builds: 60-90 min saved
- Path filtering for docs: 20-30 min saved

**Phase 2: Caching** (Implemented)

- ccache/sccache: 30-50 min saved on subsequent builds
- MSVC multi-core verified as optimally configured

**Future opportunities:**

- Parallel test execution (CTest `--parallel`)
- Split test suites into parallel jobs
- Docker-based CI for faster dependency installation

## Related Documentation

- [Testing Guide](testing.md) - Test suite organization and running tests
- [Contributing Guide](contributing.md) - Contribution workflow including CI requirements
- [Build System](build-system.md) - CMake configuration details

## References

- GitHub Actions workflows: `.github/workflows/`
- Build configuration: `pixi.toml`, `CMakeLists.txt`
- Test scripts: `scripts/test_all.py`
