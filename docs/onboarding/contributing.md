# Contributing to DART - Comprehensive Guide

This document provides detailed guidelines for contributing to DART, including workflow, testing, code style, and review processes.

> **Quick Start**: For a brief overview, see the [root CONTRIBUTING.md](../../CONTRIBUTING.md).

## Table of Contents

- [Getting Started](#getting-started)
- [Contribution Workflow](#contribution-workflow)
- [Testing Requirements](#testing-requirements)
- [Code Review Process](#code-review-process)
- [Release Process](#release-process)
- [Contributors](#contributors)

## Getting Started

### Prerequisites

Before contributing, make sure you have:

1. **Development environment set up**: See [building.md](building.md)
2. **Understanding of DART architecture**: Read [README.md](README.md)
3. **Familiarity with code style**: Check [code-style.md](code-style.md)

### Finding Something to Work On

- **Browse Issues**: Check [GitHub Issues](https://github.com/dartsim/dart/issues) for open bugs or feature requests
- **Good First Issues**: Look for issues labeled `good first issue` for newcomers
- **Feature Requests**: Check [GitHub Discussions](https://github.com/dartsim/dart/discussions) for feature ideas
- **Documentation**: Improving docs is always appreciated!

## Contribution Workflow

### 1. Fork and Clone

```bash
# Fork the repository on GitHub, then clone your fork
git clone https://github.com/YOUR_USERNAME/dart.git
cd dart

# Add upstream remote
git remote add upstream https://github.com/dartsim/dart.git
```

### 2. Create a Feature Branch

```bash
# Update your main branch
git checkout main
git pull upstream main

# Create a feature branch
git checkout -b feature/my-awesome-feature
```

### 3. Make Your Changes

- Write code following the [code style guide](code-style.md)
- Keep legacy files in `dart/` and `python/dartpy/` using PascalCase names, but use snake_case for anything new in `dart8/` and its related test/example directories
- Add tests for new functionality
- Update documentation if needed
- Commit with clear, descriptive messages

### 4. Build and Test

```bash
# Using pixi (recommended)
pixi run build
pixi run test

# Or manually
cmake --build build/
cmake --build build/ --target test
```

### 5. Format Your Code

```bash
# Using pixi
pixi run lint

# Or manually
cmake --build build/$PIXI_ENVIRONMENT_NAME/cpp/Release --target format
```

### 6. Push and Create Pull Request

```bash
# Push to your fork
git push origin feature/my-awesome-feature

# Create a pull request on GitHub
```

## Testing Requirements

All contributions must include appropriate tests:

### Unit Tests

- Located in `tests/unit/`
- Use Google Test framework
- Run with: `pixi run test` or `cmake --build build/ --target test`

### Integration Tests

- Located in `tests/integration/`
- Test interactions between components
- Include in PR description

### Python Tests

If modifying Python bindings:

```bash
# Run Python tests
pixi run test-py

# Or manually
python -m pytest python/tests/
```

### Coverage

Check test coverage:

```bash
# Generate coverage report
pixi run coverage-view

# Or manually
cmake --build build/ --target coverage_view
```

## Code Review Process

### Submitting a Pull Request

Your PR description should include:

1. **Summary**: What does this PR do?
2. **Motivation**: Why is this change needed?
3. **Changes**: List of key changes
4. **Testing**: How was this tested?
5. **Breaking Changes**: Any API changes?
6. **Related Issues**: Link to relevant issues

### Review Checklist

Reviewers will check:

- [ ] Code follows [style guide](code-style.md)
- [ ] Tests are included and pass
- [ ] Documentation is updated
- [ ] No unnecessary changes (keep PRs focused)
- [ ] Commit messages are clear
- [ ] No merge conflicts

### Addressing Feedback

- Be responsive to reviewer comments
- Make requested changes in new commits (don't force-push)
- Mark conversations as resolved when addressed
- Ask questions if feedback is unclear

### Merging

- PRs are typically merged by maintainers
- Squash merging is used for clean history
- Your contribution will be acknowledged!

## Release Process

DART follows semantic versioning (MAJOR.MINOR.PATCH):

- **MAJOR**: Breaking API changes
- **MINOR**: New features (backward compatible)
- **PATCH**: Bug fixes (backward compatible)

Releases are managed by project maintainers. Contributors don't need to worry about versioning.

## Contributors

DART is developed by a diverse community of researchers and engineers from around the world.

### Core Team

| Name                                                   | Contributions                                                                               |
| ------------------------------------------------------ | ------------------------------------------------------------------------------------------- |
| [C. Karen Liu](https://github.com/karenliu)            | Project creator, multibody dynamics, constraint resolution, tutorials                       |
| [Mike Stilman](https://github.com/mstilman)            | Project creator                                                                             |
| [Siddhartha S. Srinivasa](https://github.com/siddhss5) | Project advisor                                                                             |
| [Jeongseok Lee](https://github.com/jslee02)            | Project director, multibody dynamics, constraint resolution, collision detection, tutorials |
| [Michael X. Grey](https://github.com/mxgrey)           | Project director, extensive API improvements, inverse kinematics, gui, tutorials       |
| [Tobias Kunz](https://github.com/tobiaskunz)           | Former project director, motion planner                                                     |

### Major Contributors

| Name                                                                     | Contributions                                                                           |
| ------------------------------------------------------------------------ | --------------------------------------------------------------------------------------- |
| [Sumit Jain](http://www.cc.gatech.edu/graphics/projects/Sumit/homepage/) | Multibody dynamics                                                                      |
| [Yuting Ye](https://github.com/yutingye)                                 | Multibody dynamics, GUI                                                                 |
| [Michael Koval](https://github.com/mkoval)                               | URI, resource retriever, bug fixes                                                      |
| [Ana C. Huamán Quispe](https://github.com/ana-GT)                        | URDF parser                                                                             |
| [Chen Tang](https://github.com/chentang)                                 | Collision detection                                                                     |
| [Konstantinos Chatzilygeroudis](https://github.com/costashatz)           | Mimic joint, OSG shadows, shape deep copy, build and bug fixes                          |
| [Sehoon Ha](https://github.com/sehoonha)                                 | Early DART data structure design, [pydart](https://github.com/sehoonha/pydart)          |
| [Addisu Taddese](https://github.com/azeey)                               | ODE collision detector, slip effect, velocity/position integration, constraint grouping |
| [Christoph Hinze](https://github.com/chhinze)                            | Python bindings                                                                         |
| [Silvio Traversaro](https://github.com/traversaro)                       | Build fixes on Windows/MSVC, vcpkg packaging                                            |

### Community Contributors

Many others have contributed bug fixes, documentation, and improvements:

- [Matthew Dutton](https://github.com/mdutton3) - Build and bug fixes
- [Eric Huang](https://github.com/ehuang3) - Build and bug fixes
- [Pushkar Kolhe](https://github.com/pushkar) - Early DART build system design
- [Saul Reynolds-Haertle](https://github.com/saulrh) - Examples, bug fixes
- [Arash Rouhani](https://github.com/Tarrasch) - Build fixes
- [Kristin Siu](https://github.com/kasiu) - Integrators, bug fixes
- [Steven Peters](https://github.com/scpeters) - Build improvements and fixes
- [Can Erdogan](https://github.com/cerdogan) - Planning, examples
- [Jie Tan](https://github.com/jietan) - LCP solver, renderer
- [Yunfei Bai](https://github.com/YunfeiBai) - Build and bug fixes
- [Donny Ward](https://github.com/donnyward) - Build fix
- [Andrew Price](https://github.com/a-price) - Build fix
- [Eric Tobis](https://github.com/tobis) - Build fix
- [Jonathan Martin](https://github.com/nybblr) - Build fix
- [Jia Ye Li](https://github.com/JiaYeLi) - Fix typo of tutorials
- [Benjamin Chrétien](https://github.com/bchretien) - Bug fix
- [Olzhas Adiyatov](https://github.com/olzhas) - Bug fix
- [José Luis Rivero](https://github.com/j-rivero) - Build, especially for Debian
- [Jonathan Scholz](https://github.com/jscholz) - Build fix
- [John Turgeson](https://github.com/JohnTurgeson) - Mesh model
- [Jennifer Buehler](https://github.com/JenniferBuehler) - Heightmap, bug fix
- [Dong Xu](https://github.com/hxbloom) - Motion blur renderer
- [Donghyun Kim](https://github.com/dhkim0821) - Atlas texture images
- [Aditya Vamsikrishna](https://github.com/aditya-vk) - Bug fix
- [pchorak](https://github.com/pchorak) - Bug fixes
- [acxz](https://github.com/acxz) - Doxygen warning fix
- [Erwin Coumans](https://github.com/erwincoumans) - Build fix on Windows/MSVC
- [Martin Pecka](https://github.com/peci1) - Contact surface generalization

You can find the complete contribution history in the [GitHub contributors page](https://github.com/dartsim/dart/graphs/contributors).

### Institutional Support

DART has been supported by various institutions:

- Humanoid Lab, Georgia Tech Research Corporation
- Personal Robotics Lab, Carnegie Mellon University
- Graphics Lab, Georgia Tech Research Corporation
- Personal Robotics Lab, University of Washington
- Open Source Robotics Foundation
- The Movement Lab, Stanford University

## Getting Help

If you need help with contributing:

- **Documentation**: Start with [docs/onboarding/](../onboarding/)
- **Issues**: Ask questions on [GitHub Issues](https://github.com/dartsim/dart/issues)
- **Discussions**: Join [GitHub Discussions](https://github.com/dartsim/dart/discussions)
- **Community**: Connect with other contributors

## License

By contributing to DART, you agree that your contributions will be licensed under the BSD 2-Clause License. See the [LICENSE](../../LICENSE) file for details.

---

**Thank you for contributing to DART!** Your efforts help advance robotics research and development worldwide.
