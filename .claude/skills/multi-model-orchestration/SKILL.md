---
description: Multi-model AI agent coordination and orchestration
mcp:
  agent-coordinator:
    command: python3
    args: ["-m", "http.server", "8005"]
    working_dir: "/tmp/agent-coordinator"
    description: "Agent coordination and task distribution server"
  model-router:
    command: python3
    args: ["-m", "http.server", "8006"]
    working_dir: "/tmp/model-router"
    description: "Model selection and routing service"
---

# Multi-Model Orchestration Skill

## Purpose
Specialized skill for coordinating multiple AI agents (Oracle, Librarian, Explore, Frontend) with intelligent task distribution, context management, and result synthesis for complex DART development tasks.

## When to Use This Skill

### Complex Task Decomposition
- Break down large DART tasks into specialized sub-tasks
- Route sub-tasks to appropriate agent models
- Coordinate parallel execution across multiple agents
- Synthesize results into cohesive solutions

### Cross-Agent Collaboration
- Enable agents to share context and intermediate results
- Manage dependencies between agent tasks
- Provide conflict resolution for competing approaches
- Maintain task state and progress tracking

### Intelligent Model Selection
- Choose optimal AI model based on task characteristics
- Balance cost, performance, and quality considerations
- Adapt model selection based on historical performance
- Handle fallback scenarios when preferred models are unavailable

## Key Tools and Commands

```bash
# Multi-agent task coordination
dart-agent-coordinator --task "optimize-dynamics" --auto-assign
dart-agent-coordinator --parallel --max-agents 4 --timeout 300
dart-agent-coordinator --context-share --result-synthesis

# Model routing and selection
dart-model-router --analyze-task --recommend-model
dart-model-router --route-task --to oracle --priority high
dart-model-router --fallback-chain gpt-5.2,claude-opus,claude-sonnet

# Agent coordination
dart-agent-executor --agent oracle --task "architecture-review" --context-file context.json
dart-agent-executor --agent librarian --task "research-patterns" --parallel 2
dart-agent-executor --agent explore --task "code-analysis" --timeout 60
dart-agent-executor --agent frontend --task "documentation" --input-draft
```

## Agent Role Definitions

### 1. Oracle Agent (GPT-5.2)
**Primary Responsibilities**:
- Complex architecture decisions
- Advanced debugging and troubleshooting
- Performance optimization strategies
- Code review and quality assessment

**Task Characteristics**:
- High complexity, requires deep reasoning
- Architecture-level decisions with system-wide impact
- Performance-critical algorithm design
- Complex debugging scenarios requiring synthesis

**Activation Triggers**:
```yaml
oracle_triggers:
  keywords:
    - "architecture"
    - "design decision"
    - "complex optimization"
    - "debug complex issue"
    - "performance regression"
    - "system design"
  file_patterns:
    - "**/CMakeLists.txt"  # Build system changes
    - "**/AGENTS.md"        # Agent guidelines
    - "docs/onboarding/"    # Architecture documentation
  complexity_threshold: "high"
  max_context_tokens: 200000
```

### 2. Librarian Agent (Claude Sonnet 4.5)
**Primary Responsibilities**:
- Cross-project research and documentation
- Implementation pattern discovery
- Best practice analysis
- External API and library research

**Task Characteristics**:
- Research-heavy tasks requiring external knowledge
- Documentation and tutorial creation
- Cross-project compatibility analysis
- Pattern and example discovery
- Standards and specification research

**Activation Triggers**:
```yaml
librarian_triggers:
  keywords:
    - "research"
    - "documentation"
    - "examples"
    - "cross-project"
    - "compatibility"
    - "standards"
    - "best practices"
  sources:
    - "github"
    - "docs"
    - "academic_papers"
    - "specifications"
  max_context_tokens: 100000
```

### 3. Explore Agent (Grok Code)
**Primary Responsibilities**:
- Codebase navigation and pattern discovery
- Fast code analysis and searching
- Structural analysis of complex codebases
- Implementation pattern identification

**Task Characteristics**:
- Large-scale codebase analysis
- Pattern discovery and documentation
- Code navigation and understanding
- Structural relationship mapping
- Implementation research within codebase

**Activation Triggers**:
```yaml
explore_triggers:
  keywords:
    - "find pattern"
    - "code structure"
    - "how does X work"
    - "where is Y"
    - "implementations of"
    - "codebase analysis"
  speed_priority: "high"
  timeout: 120  # seconds
  parallel_capable: true
```

### 4. Frontend Agent (Gemini 3 Pro)
**Primary Responsibilities**:
- Documentation and tutorial creation
- User interface and example development
- Visual content generation
- API documentation and examples

**Task Characteristics**:
- User-facing documentation
- Tutorial and example creation
- Visual content and diagrams
- API documentation generation
- User interface improvements

**Activation Triggers**:
```yaml
frontend_triggers:
  keywords:
    - "documentation"
    - "tutorial"
    - "examples"
    - "user guide"
    - "API docs"
    - "visual content"
    - "UI"
  output_formats:
    - "markdown"
    - "html"
    - "pdf"
    - "interactive"
  style_consistency: "dart_brand"
```

## Task Decomposition Strategies

### 1. Hierarchical Task Breakdown
```yaml
# Example: Complex DART feature implementation
task_decomposition:
  main_task: "implement_new_constraint_solver"
  subtasks:
    - task:
        name: "architecture_review"
        agent: "oracle"
        dependencies: []
        estimated_time: 30  # minutes
        
    - task:
        name: "research_existing_solvers"
        agent: "librarian"
        dependencies: ["architecture_review.completed"]
        estimated_time: 20
        
    - task:
        name: "analyze_current_implementation"
        agent: "explore"
        dependencies: ["architecture_review.completed"]
        estimated_time: 15
        
    - task:
        name: "prototype_implementation"
        agent: "oracle"
        dependencies: [
          "research_existing_solvers.completed",
          "analyze_current_implementation.completed"
        ]
        estimated_time: 60
        
    - task:
        name: "create_documentation"
        agent: "frontend"
        dependencies: ["prototype_implementation.completed"]
        estimated_time: 25
        
    - task:
        name: "integration_testing"
        agent: "oracle"
        dependencies: [
          "prototype_implementation.completed",
          "create_documentation.completed"
        ]
        estimated_time: 30
```

### 2. Parallel Task Execution
```yaml
# Example: Performance optimization task
parallel_execution:
  main_task: "optimize_dynamics_performance"
  parallel_tasks:
    - task:
        name: "analyze_bottlenecks"
        agent: "explore"
        priority: "high"
        resources: {"cpu": 2, "memory": "1GB"}
        
    - task:
        name: "research_optimization_techniques"
        agent: "librarian"
        priority: "medium"
        resources: {"cpu": 1, "memory": "512MB"}
        
    - task:
        name: "implement_optimizations"
        agent: "oracle"
        priority: "high"
        dependencies: [
          "analyze_bottlenecks.completed",
          "research_optimization_techniques.completed"
        ]
        resources: {"cpu": 4, "memory": "2GB"}
```

## Context Management

### 1. Context Sharing Protocol
```yaml
context_sharing:
  format: "json"  # or "yaml", "protobuf"
  compression: "gzip"  # for large context
  encryption: false  # unless sensitive data
  
  sharing_rules:
    - "code_changes": "Always share full file contents"
    - "analysis_results": "Share analysis data and intermediate results"
    - "architecture_decisions": "Share decision rationale and trade-offs"
    - "performance_data": "Share profiling results and benchmarks"
    - "documentation_drafts": "Share generated content for review"
```

### 2. Context Preservation
```yaml
context_preservation:
  critical_context:
    - "user_requirements"
    - "architecture_constraints"
    - "performance_requirements"
    - "compatibility_requirements"
    - "existing_code_state"
  
  context_synthesis:
    strategy: "merge"  # or "voting", "priority_override"
    conflict_resolution: "manual_review"  # or "automatic_consensus"
    
  memory_management:
    max_context_per_agent: 150000  # tokens
    context_retention_time: 3600  # seconds (1 hour)
    compression_threshold: 10000  # compress if > 10KB
```

## Result Synthesis and Coordination

### 1. Result Aggregation
```python
class ResultAggregator:
    def __init__(self):
        self.results = {}
        self.conflicts = []
        self.synthesis_strategy = "weighted_voting"
    
    def collect_result(self, agent_id, task_id, result):
        self.results[(agent_id, task_id)] = {
            'content': result,
            'timestamp': datetime.now(),
            'confidence': self.calculate_confidence(result)
        }
        
        # Check for conflicts
        self.detect_conflicts(agent_id, task_id, result)
    
    def detect_conflicts(self, agent_id, task_id, result):
        # Check against existing results
        for other_agent, other_task in self.results:
            if other_task == task_id:
                existing_result = self.results[(other_agent, other_task)]
                conflict_score = self.calculate_conflict_score(result, existing_result)
                
                if conflict_score > 0.7:  # High conflict threshold
                    self.conflicts.append({
                        'task': task_id,
                        'agents': [agent_id, other_agent],
                        'conflict_score': conflict_score,
                        'results': [result, existing_result]
                    })
    
    def synthesize_final_result(self, task_id):
        task_results = [(agent, result) for (agent, result) in self.results.items() if task_id in result['task_id']]
        
        if not task_results:
            return None
            
        # Apply synthesis strategy
        if self.synthesis_strategy == "weighted_voting":
            return self.weighted_voting_synthesis(task_id, task_results)
        elif self.synthesis_strategy == "priority_override":
            return self.priority_override_synthesis(task_id, task_results)
        else:
            return self.consensus_synthesis(task_id, task_results)
```

### 2. Conflict Resolution
```python
class ConflictResolver:
    def __init__(self, escalation_path="human_review"):
        self.escalation_path = escalation_path
        self.resolution_strategies = [
            "merge_with_priority",
            "hybrid_approach",
            "further_analysis",
            "expert_review"
        ]
    
    def resolve_conflict(self, conflict):
        # Try automatic resolution first
        for strategy in self.resolution_strategies:
            resolution = self.attempt_resolution(conflict, strategy)
            if resolution['success']:
                return resolution
        
        # Escalate if no automatic resolution
        return self.escalate_conflict(conflict)
    
    def attempt_resolution(self, conflict, strategy):
        if strategy == "merge_with_priority":
            return self.merge_with_priority(conflict)
        elif strategy == "hybrid_approach":
            return self.create_hybrid_solution(conflict)
        # ... other strategies
```

## Performance Optimization

### 1. Intelligent Resource Allocation
```python
class ResourceManager:
    def __init__(self, total_cpu=16, total_memory="32GB"):
        self.cpu_pool = ResourcePool(total_cpu)
        self.memory_pool = ResourcePool(total_memory)
        self.allocation_history = []
    
    def allocate_for_agent(self, agent_type, task_complexity):
        # Base allocation based on agent type
        base_allocation = self.get_base_allocation(agent_type)
        
        # Adjust for task complexity
        complexity_multiplier = self.get_complexity_multiplier(task_complexity)
        
        final_allocation = {
            'cpu': min(base_allocation['cpu'] * complexity_multiplier, self.cpu_pool.available()),
            'memory': min(base_allocation['memory'] * complexity_multiplier, self.memory_pool.available())
        }
        
        # Record allocation for optimization
        self.allocation_history.append({
            'timestamp': datetime.now(),
            'agent_type': agent_type,
            'task_complexity': task_complexity,
            'allocation': final_allocation
        })
        
        return final_allocation
    
    def optimize_allocations(self):
        # Analyze allocation patterns and optimize
        # Machine learning approach to predict optimal allocations
        # Consider task completion times vs resource usage
        pass
```

### 2. Agent Performance Tracking
```python
class AgentPerformanceTracker:
    def __init__(self):
        self.agent_metrics = defaultdict(lambda: defaultdict(list))
        self.performance_baselines = {}
    
    def record_task_completion(self, agent_id, task_type, duration, quality_score):
        self.agent_metrics[agent_id][task_type].append({
            'duration': duration,
            'quality_score': quality_score,
            'timestamp': datetime.now()
        })
        
        # Update baseline performance
        self.update_baseline(agent_id, task_type, duration, quality_score)
    
    def get_agent_recommendations(self, task_type):
        recommendations = []
        
        for agent_id, metrics in self.agent_metrics.items():
            recent_metrics = metrics[task_type][-10:]  # Last 10 tasks
            avg_duration = sum(m['duration'] for m in recent_metrics) / len(recent_metrics)
            avg_quality = sum(m['quality_score'] for m in recent_metrics) / len(recent_metrics)
            
            if avg_quality > 0.8 and avg_duration < self.get_baseline_duration(agent_id, task_type) * 0.8:
                recommendations.append({
                    'agent': agent_id,
                    'confidence': self.calculate_confidence(recent_metrics),
                    'reason': f"Strong performance in {task_type} tasks"
                })
        
        return sorted(recommendations, key=lambda x: x['confidence'], reverse=True)
```

## Integration with DART Workflows

### 1. Build System Integration
```yaml
# Enhanced build workflow with multi-agent coordination
build_integration:
  agent_coordination:
    pre_build:
      - task: "analyze_dependencies"
        agent: "librarian"
        output: "dependency_analysis.json"
        
      - task: "architecture_review"
        agent: "oracle"
        input: "dependency_analysis.json"
        output: "architecture_review.json"
    
    parallel_build:
      - task: "optimize_build"
        agent: "oracle"
        dependencies: ["architecture_review.completed"]
        resources: {"cpu": 4}
        
      - task: "generate_documentation"
        agent: "frontend"
        input: "architecture_review.json"
        dependencies: ["optimize_build.completed"]
    
    post_build:
      - task: "validate_build"
        agent: "oracle"
        dependencies: ["optimize_build.completed", "generate_documentation.completed"]
```

### 2. Testing Integration
```yaml
# Enhanced testing workflow
testing_integration:
  test_planning:
    agent: "explore"
    inputs:
      - "code_changes"
      - "test_history"
      - "performance_baselines"
    outputs:
      - "test_plan"
      - "resource_allocation"
  
  test_execution:
    coordinator: "oracle"
    parallel_tasks:
      - agent: "explore"
        task: "execute_unit_tests"
        
      - agent: "librarian"
        task: "execute_integration_tests"
        
      - agent: "frontend"
        task: "generate_test_documentation"
  
  result_synthesis:
    agent: "oracle"
    inputs:
      - "unit_test_results"
      - "integration_test_results"
      - "test_documentation"
    outputs:
      - "test_summary"
      - "performance_analysis"
      - "quality_report"
```

## Success Criteria

### Coordination Effectiveness
- [ ] 90% of tasks successfully decomposed and assigned
- [ ] 85% of agent conflicts automatically resolved
- [ ] Resource utilization efficiency > 80%
- [ ] Context preservation success rate > 95%

### Model Selection Accuracy
- [ ] 95% of tasks assigned to optimal agent model
- [ ] Task completion time improvement > 30%
- [ ] Quality score improvement > 25%
- [ ] Cost-effectiveness improvement > 20%

### System Integration
- [ ] Seamless integration with existing DART workflows
- [ ] Minimal overhead to existing processes (< 10%)
- [ ] Reliability > 99% (no system failures)
- [ ] Scalability to 10+ parallel agents

## Getting Help

- **Complex coordination**: Use Oracle agent for multi-agent orchestration
- **Model selection**: Use Librarian agent for AI model research
- **Performance optimization**: Use Explore agent for pattern analysis
- **Integration issues**: Use multi-model orchestration for complex debugging

## Related Documentation

- [Test Orchestration Skill](../test-orchestration/SKILL.md)
- [DART Guidelines](../../../AGENTS.md)
- [Multi-Model Coordination](../../../docs/dev_tasks/ai-agent-optimization/phase3-planning.md)
- [Agent Configuration](../../../.opencode/oh-my-opencode.json)

---

*Multi-model orchestration enables complex DART development tasks to be decomposed, distributed, and synthesized across specialized AI agents for optimal results.*