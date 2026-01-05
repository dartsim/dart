---
description: DART intelligent test orchestration and management
mcp:
  test-orchestrator:
    command: python3
    args: ["-m", "http.server", "8003"]
    working_dir: "/tmp/test-orchestrator"
    description: "Intelligent test orchestration server"
  test-runner:
    command: python3
    args: ["-m", "http.server", "8004"]
    working_dir: "/tmp/test-runner"
    description: "Test execution runner"
---

# Test Orchestration Skill

## Purpose

Specialized skill for intelligent test selection, parallel execution, flaky test detection, and performance-aware test management for the DART codebase.

## When to Use This Skill

### Intelligent Test Selection

- Automatically select relevant tests based on code changes
- Prioritize high-impact test suites for modified components
- Balance test coverage and execution time
- Adapt selection based on historical test results

### Parallel Test Execution

- Distribute test execution across available CPU cores
- Manage memory constraints during parallel testing
- Optimize test order for maximum parallelism
- Handle resource contention between test processes

### Flaky Test Management

- Detect statistical flaky test patterns
- Automatically quarantine and re-run flaky tests
- Provide detailed flaky test analysis and reports
- Suggest fixes for common flaky test causes

### Performance-Aware Testing

- Monitor test execution performance and resource usage
- Detect performance regressions during test execution
- Suggest performance optimizations based on test metrics
- Correlate code changes with test performance impacts

## Key Tools and Commands

```bash
# Intelligent test execution
dart-test-orchestrator --smart --parallel --max-cores 8
dart-test-orchestrator --component-changed dart/dynamics
dart-test-orchestrator --performance-aware --memory-limit 4GB

# Flaky test analysis
dart-test-flaky --analyze --history 50 --threshold 0.3
dart-test-flaky --quarantine --auto-fix-suggestions
dart-test-flaky --report --detailed --output flaky_report.html

# Performance-aware testing
dart-test-perf --monitor --baseline baseline.json
dart-test-perf --regression-detect --threshold 1.2
dart-test-perf --profile --hotspot-identification

# Test suite optimization
dart-test-optimize --analyze-dependencies
dart-test-optimize --parallel-structure --cache-results
dart-test-optimize --memory-layout --simulate-execution
```

## Test Selection Intelligence

### 1. Component-Based Test Mapping

```yaml
# Intelligent test selection configuration
component_test_mapping:
  dart/collision:
    unit_tests:
      - "tests/unit/collision/*"
      - "tests/unit/common/*" # Shared utilities
    integration_tests:
      - "tests/integration/collision/*"
      - "tests/integration/physics/*" # Physics integration
    benchmarks:
      - "tests/benchmark/collision/*"
    priority: "high" # Collision changes are critical

  dart/dynamics:
    unit_tests:
      - "tests/unit/dynamics/*"
      - "tests/unit/common/*"
    integration_tests:
      - "tests/integration/dynamics/*"
      - "tests/integration/physics/*"
    benchmarks:
      - "tests/benchmark/dynamics/*"
    priority: "critical" # Dynamics changes affect core algorithms

  dart/io:
    unit_tests:
      - "tests/unit/io/*"
      - "tests/unit/common/*"
    integration_tests:
      - "tests/integration/io/*"
    benchmarks:
      - "tests/benchmark/io/*"
    priority: "medium"
```

### 2. Smart Test Selection Algorithm

```python
def select_tests(changed_files, test_database):
    # Analyze file changes and impact
    component_impacts = analyze_component_impact(changed_files)

    # Select tests based on component priority
    selected_tests = []
    for component, impact in component_impacts.items():
        if impact['severity'] >= 'medium':
            selected_tests.extend(get_tests_for_component(component))

    # Optimize for parallel execution
    optimized_order = optimize_for_parallel_execution(selected_tests)

    # Balance coverage and execution time
    final_selection = balance_coverage_time(optimized_order)

    return final_selection

def analyze_component_impact(changed_files):
    impacts = {}
    for file_path in changed_files:
        component = identify_component(file_path)
        if component not in impacts:
            impacts[component] = {
                'files_changed': [],
                'test_priority': 'low',
                'estimated_tests': []
            }

        impacts[component]['files_changed'].append(file_path)
        impacts[component]['test_priority'] = calculate_priority(impacts[component])
        impacts[component]['estimated_tests'] = estimate_test_count(component)

    return impacts
```

## Parallel Execution Management

### 1. Resource-Aware Test Distribution

```python
class TestExecutor:
    def __init__(self, max_cores=8, memory_limit="4GB"):
        self.max_cores = max_cores
        self.memory_limit = memory_limit
        self.core_pool = ResourcePool(max_cores)
        self.memory_pool = ResourcePool(calculate_memory_slots(memory_limit))

    def execute_parallel(self, tests):
        execution_plan = self.create_execution_plan(tests)
        futures = []

        for test_group in execution_plan:
            # Allocate resources
            cores = self.core_pool.acquire(test_group.required_cores)
            memory = self.memory_pool.acquire(test_group.required_memory)

            # Execute test with allocated resources
            future = execute_test_with_resources(test_group, cores, memory)
            futures.append(future)

            # Release resources when test completes
            future.add_done_callback(lambda: self.release_resources(cores, memory))

        return collect_results(futures)
```

### 2. Test Dependency Resolution

```python
def optimize_test_execution(tests):
    # Build dependency graph
    dependency_graph = build_test_dependency_graph(tests)

    # Identify independent test groups
    independent_groups = find_independent_groups(dependency_graph)

    # Schedule for maximum parallelism
    execution_order = topological_sort_with_parallel_groups(dependency_graph)

    return execution_order

def build_test_dependency_graph(tests):
    graph = TestDependencyGraph()

    for test in tests:
        # Analyze test dependencies (fixtures, shared resources)
        dependencies = analyze_test_dependencies(test)

        # Add test to dependency graph
        graph.add_test(test.name, dependencies)

    return graph
```

## Flaky Test Detection

### 1. Statistical Analysis

```python
class FlakyTestDetector:
    def __init__(self, window_size=20, failure_threshold=0.3):
        self.window_size = window_size
        self.failure_threshold = failure_threshold
        self.test_history = defaultdict(list)

    def analyze_test_result(self, test_name, result):
        history = self.test_history[test_name]
        history.append(result)

        # Keep only recent results
        if len(history) > self.window_size:
            history.pop(0)

        # Calculate flaky score
        recent_results = history[-self.window_size:]
        failure_rate = sum(1 for r in recent_results if not r.passed) / len(recent_results)

        return {
            'flaky_score': failure_rate,
            'is_flaky': failure_rate > self.failure_threshold,
            'confidence': self.calculate_confidence(len(recent_results)),
            'recommendations': self.suggest_fixes(test_name, recent_results)
        }

    def suggest_fixes(self, test_name, recent_results):
        # Analyze failure patterns
        failure_patterns = extract_failure_patterns(recent_results)

        recommendations = []
        for pattern in failure_patterns:
            if pattern['type'] == 'timing':
                recommendations.append(f"Add timeout/synchronization for {test_name}")
            elif pattern['type'] == 'resource':
                recommendations.append(f"Isolate {test_name} from resource conflicts")
            elif pattern['type'] == 'order':
                recommendations.append(f"Add explicit dependency for {test_name}")

        return recommendations
```

### 2. Automatic Quarantine System

```python
class TestQuarantine:
    def __init__(self):
        self.quarantined_tests = set()
        self.quarantine_history = defaultdict(list)

    def quarantine_test(self, test_name, reason):
        self.quarantined_tests.add(test_name)
        self.quarantine_history[test_name].append({
            'timestamp': datetime.now(),
            'reason': reason,
            'estimated_fix_time': self.estimate_fix_time(reason)
        })

    def should_run_quarantined(self, test_name):
        history = self.quarantine_history[test_name]
        if not history:
            return False

        latest_quarantine = history[-1]
        time_since_quarantine = datetime.now() - latest_quarantine['timestamp']

        # Only run after estimated fix time or with explicit override
        return time_since_quarantine > latest_quarantine['estimated_fix_time']

    def auto_release_check(self, test_name):
        # Check if quarantined test appears stable in recent runs
        recent_results = self.get_recent_test_results(test_name, 10)

        if len(recent_results) >= 5 and all(r.passed for r in recent_results[-5:]):
            self.release_from_quarantine(test_name, "Stable for 5 consecutive runs")
            return True

        return False
```

## Performance-Aware Testing

### 1. Execution Monitoring

```python
class TestPerformanceMonitor:
    def __init__(self, baseline_file="test_baselines.json"):
        self.baselines = self.load_baselines(baseline_file)
        self.current_metrics = {}

    def monitor_test_execution(self, test_name, start_time, end_time, memory_usage):
        execution_time = end_time - start_time
        baseline_time = self.baselines.get(test_name, {}).get('execution_time', 0)

        performance_ratio = execution_time / baseline_time if baseline_time > 0 else 1.0

        self.current_metrics[test_name] = {
            'execution_time': execution_time,
            'performance_ratio': performance_ratio,
            'memory_usage': memory_usage,
            'is_regression': performance_ratio > 1.2,  # 20% regression threshold
            'is_improvement': performance_ratio < 0.8,  # 20% improvement threshold
        }

        return self.current_metrics[test_name]

    def generate_performance_report(self):
        report = {
            'summary': self.generate_summary_stats(),
            'regressions': self.identify_regressions(),
            'improvements': self.identify_improvements(),
            'recommendations': self.generate_optimization_suggestions()
        }

        return report
```

### 2. Adaptive Test Scheduling

```python
class AdaptiveTestScheduler:
    def __init__(self):
        self.test_performance_history = defaultdict(list)
        self.execution_costs = {}

    def schedule_tests(self, available_time, tests):
        # Estimate execution costs based on historical data
        for test in tests:
            self.execution_costs[test.name] = self.estimate_execution_cost(test)

        # Optimize schedule for maximum coverage
        schedule = self.optimize_test_schedule(available_time, tests)
        return schedule

    def optimize_test_schedule(self, time_limit, tests):
        # Use knapsack-like algorithm for optimal test selection
        optimal_set = self.select_optimal_test_set(time_limit, tests)

        # Order for maximum parallelism
        execution_order = self.optimize_for_parallel_execution(optimal_set)

        return execution_order
```

## Integration with DART Workflows

### 1. CI/CD Integration

```yaml
# Enhanced GitHub Actions workflow
name: Intelligent Test Orchestration
on: [push, pull_request]

jobs:
  analyze-changes:
    outputs:
      test-selection: ${{ steps.analyze.outputs.test-selection }}
      execution-plan: ${{ steps.analyze.outputs.execution-plan }}
    steps:
      - name: Analyze code changes
        id: analyze
        run: |
          python scripts/analyze_changes.py --output test-selection.json

      - name: Create execution plan
        run: |
          python scripts/create_execution_plan.py --input test-selection.json --output execution-plan.json

  execute-tests:
    needs: analyze-changes
    strategy:
      matrix:
        execution-group: ${{ fromJson(needs.analyze.outputs.execution-plan).groups }}
    steps:
      - name: Execute test group
        run: |
          python scripts/execute_test_group.py --group ${{ matrix.execution-group }}

      - name: Upload results
        uses: actions/upload-artifact@v2
        with:
          name: test-results-${{ matrix.execution-group }}
          path: test-results/
```

### 2. Local Development Integration

```bash
# Enhanced development workflow
# 1. Smart test selection
dart-test-orchestrator --smart --changed-files $(git diff --name-only HEAD~1 HEAD)

# 2. Execute with performance monitoring
dart-test-orchestrator --execute --performance-monitor --baseline current_baseline.json

# 3. Analyze flaky tests
dart-test-orchestrator --flaky-analysis --history 100 --quarantine-unstable

# 4. Generate optimization report
dart-test-orchestrator --performance-report --output performance_report.html
```

## Success Criteria

### Test Selection Intelligence

- [ ] 95% accuracy in identifying relevant tests for code changes
- [ ] 80% reduction in unnecessary test execution
- [ ] 50% improvement in test execution parallelism
- [ ] Maintain 100% test coverage for modified components

### Flaky Test Management

- [ ] 90% detection rate for actually flaky tests
- [ ] < 5% false positive rate for flaky test detection
- [ ] Automatic quarantine reduces CI noise by 70%
- [ ] Actionable fix suggestions for 80% of flaky test patterns

### Performance-Aware Testing

- [ ] Real-time performance regression detection during test execution
- [ ] Performance baselines maintained for 95% of tests
- [ ] Automated optimization suggestions for performance regressions
- [ ] Memory usage monitoring prevents resource exhaustion

### System Integration

- [ ] Seamless integration with existing DART CI/CD pipeline
- [ ] Compatible with local development workflows
- [ ] Scalable to test suites with 10,000+ tests
- [ ] Minimal overhead to existing test execution ( < 5%)

## Getting Help

- **Test orchestration algorithms**: Use Oracle agent for complex scheduling
- **Statistical analysis**: Use Librarian agent for flaky test detection methods
- **Performance optimization**: Use Explore agent to find performance monitoring patterns
- **CI/CD integration**: Use multi-model orchestration for workflow debugging

## Related Documentation

- [Test Module Guidelines](../../../tests/AGENTS.md)
- [Dynamics Module Guidelines](../../../dart/dynamics/AGENTS.md)
- [Performance Optimization Skill](../performance-optimization/SKILL.md)
- [DART Build System](../../../docs/onboarding/build-system.md)
- [Testing Documentation](../../../docs/onboarding/testing.md)

---

_Intelligent test orchestration transforms DART testing from a bottleneck into a competitive advantage through smart selection, parallel execution, and performance awareness._
