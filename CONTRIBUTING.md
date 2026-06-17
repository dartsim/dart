# Contributing to DART

Thank you for your interest in contributing to DART! We welcome contributions of all kinds: bug fixes, new features, documentation improvements, and more.

## Quick Start

1. **New to DART?** Read the [Developer Onboarding Guide](docs/onboarding/README.md)
2. **Ready to contribute?** See [Contribution Workflow](docs/onboarding/contributing.md)
3. **Building from source?** Follow the [Build Guide](docs/onboarding/building.md)
4. **Code style?** Check the [Style Guide](docs/onboarding/code-style.md)

## How to Contribute

The best way to contribute is by [opening a GitHub pull request](https://help.github.com/articles/about-pull-requests/). Make sure your code follows DART conventions (see [style guide](docs/onboarding/code-style.md)).

**Don't worry about perfection!** Feel free to post work-in-progress versions to get feedback and start the discussion.

## Copyright Headers

All source files should include a copyright header. We use the year of first publication (2011) following [FSFE REUSE best practices](https://reuse.software/faq/#years-copyright):

```cpp
/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   ...
 */
```

**Important**: Do NOT update the year when modifying files. The year represents the first publication date of the project, not the last modification. This follows industry best practice (used by Google, curl, Gazebo, etc.) and eliminates unnecessary maintenance.

## Before Submitting a PR

- [ ] Code follows the [DART style guide](docs/onboarding/code-style.md)
- [ ] Build succeeds: `pixi run build`
- [ ] Tests pass: `pixi run test` or a more focused `pixi run ...` test task
- [ ] Code is formatted: `pixi run lint`
- [ ] Documentation is updated if needed
- [ ] Bug fixes that apply to the release line have an active-DART-6-LTS PR
      first, then a `main` PR
- [ ] PR uses `.github/PULL_REQUEST_TEMPLATE.md`
- [ ] Target milestone is set (`DART 7.0` for `main`, branch-matching DART 6.x
      patch milestone for the active DART 6 LTS branch)
- [ ] `CHANGELOG.md` is updated according to
      [the changelog guide](docs/onboarding/changelog.md), or the PR explains
      why no changelog entry is needed
- [ ] Any used `docs/dev_tasks/<task>/` folder is removed in this PR after
      durable notes move into onboarding docs

Advanced manual CMake details live in the [build guide](docs/onboarding/building.md),
but the public contribution checklist uses `pixi run ...` tasks.

## Getting Help

- **Documentation**: [docs/onboarding/](docs/onboarding/)
- **Issues**: [GitHub Issues](https://github.com/dartsim/dart/issues)
- **Discussions**: [GitHub Discussions](https://github.com/dartsim/dart/discussions)

## Contributors

DART is developed by a diverse community of researchers and engineers. See the [full contributor list](docs/onboarding/contributing.md#contributors) for acknowledgments.

## License

By contributing to DART, you agree that your contributions will be licensed under the BSD 2-Clause License.
