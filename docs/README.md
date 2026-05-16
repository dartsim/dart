# DART Documentation

This directory contains documentation for DART users, developers, and contributors.

## For New Contributors

**Start here:** [onboarding/README.md](onboarding/README.md) - Complete developer onboarding guide with architecture, components, and workflows.

## Documentation Structure

### Developer And Agent Docs (Markdown)

Located in `onboarding/`, `ai/`, and `plans/` - **architecture, workflow, and
planning guides for developers and agents:**

- **[Onboarding Guide](onboarding/README.md)** - Complete internal architecture overview with diagrams
- [AI Agent Entrypoint](ai/README.md) - AI-native principles, north star, workflow map, verification gates, sessions, and component ownership
- [Living Plans](plans/README.md) - Planning rules, operating dashboard, and strategic roadmap
- [Core Architecture](onboarding/architecture.md) - Deep dive into simulation engine internals
- [Aspect System](onboarding/aspect-system.md) - Aspect/State/Properties design and implementation
- [Dynamics System](onboarding/dynamics.md) - Articulated body system and kinematics
- [Constraint Solver](onboarding/constraints.md) - Constraint resolution and collision response
- [GUI & Rendering](onboarding/gui-rendering.md) - Filament renderer
  architecture, workflow, and verification
- [Python Bindings](onboarding/python-bindings.md) - nanobind bindings architecture
- [Build System](onboarding/build-system.md) - CMake internals and dependency analysis
- [Building from Source](onboarding/building.md) - Step-by-step build instructions
- [Code Style Guide](onboarding/code-style.md) - Code conventions for C++, Python, and CMake
- [Model Loading (IO)](onboarding/io-parsing.md) - Unified model loading API (`dart::io`)

_Format: Markdown (GitHub/LLM-friendly) for internal codebase understanding_

### User Documentation (ReadTheDocs)

Located in `readthedocs/` - **Published documentation for end users:**

- **[User Guides](readthedocs/dart/user_guide/)** - Installation, tutorials, migration guides
- **[API Reference](https://dart.readthedocs.io/)** - Published API documentation

_Format: ReStructuredText (RST) for Sphinx/ReadTheDocs publishing_

## Quick Links by Task

- **New to DART?** → [onboarding/README.md](onboarding/README.md)
- **Building from source?** → [onboarding/building.md](onboarding/building.md)
- **Contributing code?** → [onboarding/contributing.md](onboarding/contributing.md) + [onboarding/code-style.md](onboarding/code-style.md)
- **Understanding architecture?** → [onboarding/architecture.md](onboarding/architecture.md)
- **Revising project plans?** → [plans/README.md](plans/README.md) + [plans/dashboard.md](plans/dashboard.md) + [plans/north-star-roadmap.md](plans/north-star-roadmap.md)
- **Using AI agent workflows?** → [ai/README.md](ai/README.md) + [ai/principles.md](ai/principles.md) + [ai/north-star.md](ai/north-star.md) + [onboarding/ai-tools.md](onboarding/ai-tools.md)
- **Using DART API?** → [dart.readthedocs.io](https://dart.readthedocs.io/)
- **Looking for examples?** → [examples/](../examples/)
