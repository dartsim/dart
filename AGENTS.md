# Agent Guidelines for DART

This file is a pointer board for agents working in this repository. Keep it concise and expand other documents instead.

## Read First

- Architectural, build, and workflow expectations live in `docs/onboarding` (start with `docs/onboarding/ci-cd.md` and `docs/onboarding/build-system.md`).
- The day-to-day pixi workflow (install, config, build, lint, test) is documented in `docs/onboarding/building.md` and `docs/onboarding/testing.md`.
- Coding standards, formatting, and contribution flow are in `CONTRIBUTING.md`.
- Feature‑specific notes belong beside the code (e.g., README in the component directory) or in `docs/`.
- Unified model loading API (`dart::io`) is documented in `docs/onboarding/io-parsing.md`.
- Python bindings (`dartpy`) guidance is in `docs/onboarding/python-bindings.md`.

## Daily Reminders

- Use the existing tooling (`pixi run …`) described in the onboarding docs; do not invent new entry points.
- When you learn something new, update the relevant document (usually under `docs/onboarding/` or the component README) and then add a short pointer here only if discoverability is still lacking.
- Treat AGENTS.md as a TODO list for missing documentation: if this file grows beyond pointers, migrate the details to the appropriate doc and shrink this file again before you finish your task.
- Gazebo / gz-physics integration notes (including patching policy and deprecated `collision-*` compatibility components) live in `docs/onboarding/build-system.md` under "Gazebo Integration Feature".

## AI Agent Workflow Commands

For AI agents working on DART (OpenCode, Claude Code, etc.):

```bash
# Environment setup (first command in any session)
pixi run configure

# Core development cycle
pixi run build              # Build all components
pixi run test               # Run test suite
pixi run lint               # Check code quality
pixi run format             # Auto-format code

# Specialized workflows
pixi run -e gazebo test-gz  # Gazebo physics integration (critical for physics changes)
pixi run benchmark          # Performance regression testing
pixi run docs               # Build documentation
```

## Agent-Critical Patterns

### Architecture Rules

- **Aspect System**: Use runtime extensibility via `dart/common/Aspect.hpp` - NEVER use traditional inheritance for new features
- **Dual Models**: Choose either `World`/`Skeleton` (stable) OR `experimental/ecs` (new) - never mix patterns
- **Unified IO**: Always use `dart/io/` API (`readWorld`, `readSkeleton`) for file loading

### When C++ Changes Require Python Updates

- Any public API change in `dart/` requires updates to `python/dartpy/`
- Template metaprogramming in Aspect system may need binding regeneration
- Run `pixi run test --filter python` after C++ API changes

### Performance Requirements

- Dynamics algorithms must maintain O(n) complexity
- Use Featherstone's Articulated Body Algorithm patterns
- Profile with `pixi run benchmark --filter dynamics` for optimization work

### Testing Requirements

- **All changes**: `pixi run test` must pass
- **Physics changes**: `pixi run -e gazebo test-gz` must pass
- **API changes**: Update relevant documentation in `docs/`
- **Performance changes**: Ensure no benchmark regressions

## Getting Help

- **Architecture decisions**: Use Oracle agent for complex design questions
- **Cross-project research**: Use Librarian agent for robotics implementation patterns
- **Code navigation**: Use Explore agent with LSP tools for template metaprogramming
- **Full context**: See `docs/dev_tasks/ai-agent-optimization.md` for comprehensive AI integration plan
