---
description: DART performance monitoring, regression detection, and optimization intelligence
mcp:
  performance-monitor:
    command: python3
    args: ["-m", "http.server", "8007"]
    working_dir: "/tmp/performance-monitor"
    description: "Performance monitoring and analytics server"
  regression-detector:
    command: python3
    args: ["-m", "http.server", "8008"]
    working_dir: "/tmp/regression-detector"
    description: "Performance regression detection service"
---

# Performance Intelligence Skill

## Purpose

Specialized skill for continuous performance monitoring, regression detection, and optimization intelligence for DART's critical algorithms. Provides automated analysis, baseline management, and optimization suggestions.

## When to Use This Skill

### Continuous Performance Monitoring

- Track real-time performance metrics during development
- Maintain performance baselines across code changes
- Detect performance regressions automatically
- Monitor resource usage and bottlenecks

### Regression Detection

- Identify performance degradations in critical paths
- Statistical analysis of performance trends
- Adaptive thresholds for different algorithm types
- Automatic alerts for significant regressions

### Optimization Intelligence

- Suggest performance improvements based on patterns
- Identify optimization opportunities automatically
- Track optimization effectiveness over time
- Provide data-driven optimization recommendations

## Key Tools and Commands

```bash
# Performance monitoring dashboard
dart-perf-monitor --dashboard --real-time
dart-perf-monitor --baseline-manage --update-current
dart-perf-monitor --track-metrics dynamics,collision,io

# Regression detection
dart-regression-detector --analyze --window 100  # Last 100 runs
dart-regression-detector --alert --threshold 1.2      # 20% regression
dart-regression-detector --report --trend-analysis

# Optimization intelligence
dart-perf-intelligence --analyze-patterns --historical
dart-perf-intelligence --suggest-optimizations --current-code
dart-perf-intelligence --baseline-compare --new-vs-old

# Baseline management
dart-perf-baseline --capture --scenario "standard_robot"
dart-perf-baseline --validate --against-commit abc123
dart-perf-baseline --update --from-results latest_benchmark.json
```

## Performance Monitoring Architecture

### 1. Real-Time Metrics Collection

```python
class PerformanceMonitor:
    def __init__(self, sampling_rate=10.0):  # Hz
        self.sampling_rate = sampling_rate
        self.metrics_buffer = CircularBuffer(1000)
        self.alert_thresholds = self.load_thresholds()

    def collect_metrics(self, operation, duration, memory_usage, cpu_usage):
        metric = PerformanceMetric(
            operation=operation,
            duration=duration,
            memory_usage=memory_usage,
            cpu_usage=cpu_usage,
            timestamp=time.time()
        )

        self.metrics_buffer.add(metric)

        # Check for real-time alerts
        if self.check_performance_alert(metric):
            self.send_alert(metric)

    def check_performance_alert(self, metric):
        threshold = self.alert_thresholds.get(metric.operation, {})
        return (metric.duration > threshold['critical_latency'] or
                metric.memory_usage > threshold['memory_critical'])
```

### 2. Baseline Management System

```python
class BaselineManager:
    def __init__(self, storage_file="performance_baselines.json"):
        self.storage_file = storage_file
        self.baselines = self.load_baselines()
        self.version_info = self.get_dart_version()

    def create_baseline(self, scenario_name, metrics_data):
        baseline = PerformanceBaseline(
            scenario=scenario_name,
            metrics=metrics_data,
            dart_version=self.version_info,
            created_at=datetime.now(),
            environment=self.capture_environment()
        )

        self.baselines[scenario_name] = baseline
        self.save_baselines()
        return baseline

    def compare_with_baseline(self, current_metrics, scenario_name):
        baseline = self.baselines.get(scenario_name)
        if not baseline:
            return BaselineComparison(status="NO_BASELINE")

        comparison = BaselineComparison(
            baseline=baseline,
            current=current_metrics,
            differences=self.calculate_differences(baseline.metrics, current_metrics),
            regression_score=self.calculate_regression_score(baseline.metrics, current_metrics)
        )

        return comparison
```

### 3. Regression Detection Engine

```python
class RegressionDetector:
    def __init__(self, window_size=50, regression_threshold=1.2):
        self.window_size = window_size
        self.regression_threshold = regression_threshold
        self.performance_history = defaultdict(list)

    def analyze_performance_trend(self, operation, current_performance):
        history = self.performance_history[operation]
        history.append(current_performance)

        if len(history) < self.window_size:
            return RegressionAnalysis(status="INSUFFICIENT_DATA")

        # Statistical analysis
        recent_performance = history[-self.window_size:]
        mean_performance = statistics.mean(recent_performance)
        std_performance = statistics.stdev(recent_performance)

        # Calculate regression score
        baseline_mean = statistics.mean(history[:-20])  # Exclude recent from baseline
        current_z_score = (current_performance - baseline_mean) / std_performance if std_performance > 0 else 0

        is_regression = (current_z_score > 2.0 and  # Significant degradation
                       current_performance > baseline_mean * self.regression_threshold)

        return RegressionAnalysis(
            status="REGRESSION_DETECTED" if is_regression else "NORMAL",
            regression_score=current_z_score,
            confidence=self.calculate_confidence(recent_performance),
            trend_analysis=self.analyze_trend(recent_performance)
        )

    def analyze_trend(self, performance_data):
        # Linear regression for trend analysis
        x = list(range(len(performance_data)))
        slope, intercept, r_value, p_value, std_err = stats.linregress(x, performance_data)

        return TrendAnalysis(
            slope=slope,
            is_degrading=slope > 0.1,  # Positive slope = increasing time
            trend_strength=abs(r_value),
            confidence=1 - p_value  # Higher p-value = lower confidence
        )
```

## Intelligent Optimization Suggestions

### 1. Pattern-Based Optimization

```python
class OptimizationAnalyzer:
    def __init__(self):
        self.pattern_database = self.load_optimization_patterns()
        self.success_history = defaultdict(list)

    def analyze_for_optimizations(self, code_changes, performance_data):
        opportunities = []

        # Analyze code changes for optimization opportunities
        for file_path, changes in code_changes.items():
            patterns = self.identify_performance_patterns(changes)
            opportunities.extend(patterns)

        # Analyze performance data for bottlenecks
        bottlenecks = self.identify_bottlenecks(performance_data)

        # Generate optimization suggestions
        suggestions = self.generate_suggestions(opportunities, bottlenecks)

        return OptimizationReport(
            suggestions=suggestions,
            confidence_scores=self.calculate_suggestion_confidence(suggestions),
            estimated_impact=self.estimate_optimization_impact(suggestions),
            priority_ranking=self.rank_suggestions_by_priority(suggestions)
        )

    def identify_performance_patterns(self, code_changes):
        patterns = []

        # Loop optimization patterns
        if self.contains_nested_loops(code_changes):
            patterns.append({
                'type': 'loop_optimization',
                'description': 'Replace nested loops with linear algorithms',
                'estimated_speedup': '2-5x',
                'confidence': 0.8
            })

        # Memory access patterns
        if self.contains_cache_thrashing(code_changes):
            patterns.append({
                'type': 'memory_optimization',
                'description': 'Improve data locality and cache usage',
                'estimated_speedup': '1.3-2x',
                'confidence': 0.7
            })

        # SIMD opportunities
        if self.contains_vectorizable_operations(code_changes):
            patterns.append({
                'type': 'simd_vectorization',
                'description': 'Utilize SIMD instructions for vector operations',
                'estimated_speedup': '4-8x',
                'confidence': 0.9
            })

        return patterns
```

### 2. Machine Learning Optimization

```python
class MLOptimizationEngine:
    def __init__(self):
        self.model_cache = self.load_trained_models()
        self.feature_extractor = PerformanceFeatureExtractor()

    def predict_optimizations(self, code_context, performance_history):
        # Extract features from code
        features = self.feature_extractor.extract_features(code_context)

        # Predict optimization opportunities
        predictions = []
        for model_name, model in self.model_cache.items():
            prediction = model.predict(features)
            predictions.append({
                'model': model_name,
                'optimization_type': prediction['type'],
                'confidence': prediction['confidence'],
                'estimated_impact': prediction['impact']
            })

        # Ensemble predictions
        ensemble_prediction = self.ensemble_predictions(predictions)

        return OptimizationSuggestions(
            predictions=predictions,
            ensemble_prediction=ensemble_prediction,
            context_features=features
        )

    def train_optimization_model(self, training_data):
        # Train models on historical optimization data
        features = []
        targets = []

        for example in training_data:
            code_features = self.feature_extractor.extract_features(example['code'])
            performance_gain = example['performance_improvement']

            features.append(code_features)
            targets.append(performance_gain)

        # Train multiple models for ensemble
        models = {
            'gradient_boost': self.train_gradient_boost(features, targets),
            'neural_network': self.train_neural_network(features, targets),
            'random_forest': self.train_random_forest(features, targets)
        }

        self.save_trained_models(models)
        return models
```

## Integration with Development Workflow

### 1. Continuous Integration Integration

```yaml
# Enhanced CI/CD workflow
name: Performance Intelligence Integration
on: [push, pull_request]

jobs:
  performance-analysis:
    steps:
      - name: Run Performance Benchmarks
        run: |
          pixi run benchmark --all-metrics --output perf_results.json

      - name: Analyze Performance Trends
        run: |
          dart-perf-intelligence --analyze-trends --input perf_results.json
                                --baseline current_baseline.json
                                --output trend_analysis.json

      - name: Detect Regressions
        run: |
          dart-regression-detector --analyze --input perf_results.json
                                       --baseline current_baseline.json
                                       --alert-threshold 1.2

      - name: Generate Optimization Suggestions
        run: |
          dart-perf-intelligence --suggest --input perf_results.json
                                       --history performance_history.json
                                       --output optimization_suggestions.json
```

### 2. Development Environment Integration

```bash
# Continuous performance monitoring in development
# Start monitoring in background
dart-perf-monitor --daemon --real-time --alerts critical

# Periodic performance analysis
dart-perf-intelligence --analyze-periodic --interval 3600  # Every hour

# Integration with IDE/editors
dart-perf-ide-plugin --monitor-on-save --auto-analyze
dart-perf-ide-plugin --suggest-optimizations --real-time
```

## Success Criteria

### Regression Detection

- [ ] 95% accuracy in detecting actual performance regressions
- [ ] < 5% false positive rate for regression detection
- [ ] Real-time detection within 100ms of performance drop
- [ ] Statistical confidence scoring for regression alerts

### Optimization Intelligence

- [ ] 80% accuracy in predicting optimization opportunities
- [ ] 70% success rate for suggested optimizations
- [ ] 5% false positive rate for optimization suggestions
- [ ] Measurable performance improvement from suggested optimizations

### Performance Monitoring

- [ ] < 1% overhead for real-time performance monitoring
- [ ] Continuous 99.9% uptime for monitoring services
- [ ] Sub-second response time for performance queries
- [ ] Scalable to 10,000+ concurrent monitoring operations

### Baseline Management

- [ ] Automated baseline updates for new DART versions
- [ ] Cross-platform baseline consistency
- [ ] Version-aware baseline comparisons
- [ ] Baseline validation and integrity checking

## Advanced Features

### 1. Predictive Performance Analysis

```python
class PredictiveAnalyzer:
    def predict_future_performance(self, code_changes, historical_data):
        # Use time series analysis to predict performance impact
        impact_prediction = self.analyze_impact_trend(code_changes, historical_data)

        # Predict optimal optimization strategies
        strategy_prediction = self.predict_optimal_strategy(impact_prediction)

        return PredictiveAnalysis(
            impact_prediction=impact_prediction,
            strategy_prediction=strategy_prediction,
            confidence_intervals=self.calculate_confidence_intervals(),
            risk_assessment=self.assess_optimization_risks()
        )
```

### 2. Multi-Dimensional Performance Analysis

```python
class MultiDimensionalAnalyzer:
    def analyze_performance_multi_dimensional(self, performance_data):
        dimensions = {
            'speed': self.analyze_speed_metrics(performance_data),
            'memory': self.analyze_memory_metrics(performance_data),
            'scalability': self.analyze_scalability(performance_data),
            'efficiency': self.analyze_resource_efficiency(performance_data),
            'stability': self.analyze_stability_metrics(performance_data)
        }

        # Cross-dimensional analysis
        correlations = self.analyze_cross_dimensional_correlations(dimensions)

        return MultiDimensionalReport(
            dimensions=dimensions,
            correlations=correlations,
            overall_score=self.calculate_overall_performance_score(dimensions),
            recommendations=self.generate_cross_dimensional_recommendations(dimensions)
        )
```

## Getting Help

- **Performance analysis**: Use Oracle agent for complex performance optimization
- **Machine learning**: Use Librarian agent for performance analysis techniques
- **Statistical analysis**: Use Explore agent to find performance patterns
- **System integration**: Use multi-model orchestration for complex debugging

## Related Documentation

- [Dynamics Module Guidelines](../../../dart/dynamics/AGENTS.md)
- [Performance Optimization Skill](../performance-optimization/SKILL.md)
- [Test Orchestration Skill](../test-orchestration/SKILL.md)
- [DART Build System](../../../docs/onboarding/build-system.md)
- [Performance Monitoring Guide](../../../docs/onboarding/performance.md)

---

_Performance intelligence transforms DART from reactive to predictive, enabling proactive optimization and data-driven performance improvements._
