# DART Documentation

This directory contains documentation for DART users, developers, and contributors.

## For New Contributors

**Start here:** [onboarding/README.md](onboarding/README.md) - Complete developer onboarding guide with architecture, components, and workflows.

## Documentation Structure

### Developer Onboarding (Markdown)

Located in `onboarding/` - **Architecture guides for developers and LLMs:**

- **[Onboarding Guide](onboarding/README.md)** - Complete internal architecture overview with diagrams
- [Core Architecture](onboarding/architecture.md) - Deep dive into simulation engine internals
- [Dynamics System](onboarding/dynamics.md) - Articulated body system and kinematics
- [Constraint Solver](onboarding/constraints.md) - Constraint resolution and collision response
- [GUI & Rendering](onboarding/gui-rendering.md) - OpenSceneGraph integration details
- [Python Bindings](onboarding/python-bindings.md) - pybind11 bindings architecture
- [Build System](onboarding/build-system.md) - CMake internals and dependency analysis
- [Building from Source](onboarding/building.md) - Step-by-step build instructions
- [Code Style Guide](onboarding/code-style.md) - Code conventions for C++, Python, and CMake

*Format: Markdown (GitHub/LLM-friendly) for internal codebase understanding*

### User Documentation (ReadTheDocs)

Located in `readthedocs/` - **Published documentation for end users:**

- **[User Guides](readthedocs/dart/user_guide/)** - Installation, tutorials, migration guides
- **[API Reference](https://dart.readthedocs.io/)** - Published API documentation

*Format: ReStructuredText (RST) for Sphinx/ReadTheDocs publishing*

## Quick Links by Task

- **New to DART?** → [onboarding/README.md](onboarding/README.md)
- **Building from source?** → [onboarding/building.md](onboarding/building.md)
- **Contributing code?** → [onboarding/contributing.md](onboarding/contributing.md) + [onboarding/code-style.md](onboarding/code-style.md)
- **Understanding architecture?** → [onboarding/architecture.md](onboarding/architecture.md)
- **Using DART API?** → [dart.readthedocs.io](https://dart.readthedocs.io/)
- **Looking for examples?** → [examples/](../examples/) and [tutorials/](../tutorials/)
