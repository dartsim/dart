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

### Bug Fix Workflow (Two PRs Required)

**Bug fixes must be applied to both the active DART 6 LTS branch AND `main`**
to ensure fixes are available in both DART 6 and DART 7. Use the highest
maintained `release-6.*` branch advertised by the upstream remote; this checkout
currently sees `release-6.20`.

1. **Fix on release branch first**:

   ```bash
   git fetch upstream 'refs/heads/release-6*:refs/remotes/upstream/release-6*'
   DART6_LTS_BRANCH=$(git branch -r --list 'upstream/release-6.*' | sed 's|.*/||' | sort -V | tail -1)
   git checkout "$DART6_LTS_BRANCH"
   git pull upstream "$DART6_LTS_BRANCH"
   git checkout -b "fix/issue-XXXX-description-${DART6_LTS_BRANCH#release-}"
   # Make your fix, commit, and push
   # Create PR targeting $DART6_LTS_BRANCH with title: "Fix: description (DART 6 LTS)"
   ```

2. **Cherry-pick to main** (or reapply if conflicts):
   ```bash
   git checkout main
   git pull upstream main
   git checkout -b fix/issue-XXXX-description-main
   git cherry-pick <commit-hash>  # Or manually reapply if conflicts
   # Push and create PR targeting main with title: "Fix: description (DART 7)"
   ```

**PR Title Convention**: Use version numbers or release-line labels ("DART 6
LTS", "DART 7") rather than raw branch names for clarity.

### 3. Make Your Changes

- Write code following the [code style guide](code-style.md)
- Keep legacy files in `dart/` and `python/dartpy/` using PascalCase names, but use snake_case for DART 7 simulation work in `dart/simulation/` and its related test/example directories (including `tests/unit/simulation/` and `tests/benchmark/simulation/`)
- Add tests for new functionality
- Update documentation if needed
- Update `CHANGELOG.md` when the change is notable under
  [changelog.md](changelog.md), or record why no entry is needed in the PR
- If you use `docs/dev_tasks/<TASK>/` for tracking, keep it updated during work and remove the folder once the task is complete (after adding a brief note to the most relevant `docs/onboarding/*.md`)
- Commit with clear, descriptive messages

### 4. Build and Test

```bash
pixi run build
pixi run test
```

Use the more focused `pixi run ...` test tasks documented in
`docs/onboarding/testing.md` when a full test run is not needed. Manual CMake
commands are covered in `docs/onboarding/building.md` for advanced build-system
debugging, but contributor workflow steps should use Pixi tasks.

### 5. Format Your Code

```bash
pixi run lint
```

Rule of thumb: run `pixi run lint` before committing so auto-fixes are captured.

#### Git hooks

Run this once per clone to install the cross-tool commit guard:

```bash
pixi run install-hooks
```

It installs a `pre-commit` Git hook that runs the bounded staged-file
`pixi run check-agent-hook` structural gate and blocks the commit if it fails.
If you already have a `pre-commit` hook it is preserved as
`pre-commit.local` and chained. The hook works in linked worktrees too.
Emergency escape hatch: `DART_SKIP_HOOKS=1 git commit ...`.

Claude Code and Codex sessions also use the tracked commit-command guard before
an agent-issued `git commit`, even if the Git hook is not installed. These fast
guards do not replace `pixi run lint`; run the full formatter before every
commit as required above.

### 6. Push and Create Pull Request

```bash
# Push to your fork
git push origin feature/my-awesome-feature

# Create a pull request on GitHub
```

Use the PR template in `.github/PULL_REQUEST_TEMPLATE.md` and set the milestone for the target branch:

- `main`: `DART 7.0` (or the next major milestone)
- Active DART 6 LTS branch, currently `release-6.20`: branch-matching DART 6.x
  release milestone

### Repository Metadata Maintenance

When cleaning up GitHub issue or PR labels, treat repository metadata as a
shared source of truth:

- Prefer built-in GitHub issue types over parallel `type:*` labels, and update
  issue templates before deleting any label they still apply.
- Use issue fields for maintainer-only scheduling data such as priority or
  effort, but check field visibility before replacing public labels. Removing a
  public label can make that signal private if the field is organization-only.
- Keep automation labels that workflows or bots still apply, such as lockfile
  update, dependency, or GitHub Actions labels, unless the automation is updated
  in the same change.
- Before deleting a label, verify open issue/PR usage and remember that deleting
  a label also removes it from closed historical issues and PRs.
- Label, milestone, branch, and PR metadata mutations on GitHub require
  explicit maintainer/user approval.

## Testing Requirements

All contributions must include appropriate tests:

### Unit Tests

- Located in `tests/unit/`
- Use Google Test framework
- Run with: `pixi run test`, `pixi run test-unit`, or a more focused
  `pixi run ...` test task

### Integration Tests

- Located in `tests/integration/`
- Test interactions between components
- Include in PR description

### Python Tests

If modifying Python bindings:

```bash
pixi run test-py
```

### Coverage

Check test coverage:

```bash
pixi run coverage-view
```

## Code Review Process

### Submitting a Pull Request

Use `.github/PULL_REQUEST_TEMPLATE.md`. This section owns PR-writing guidance
for contributors and the PR workflows. Write for a reviewer unfamiliar with
the implementation; recent PRs supply context, not a style authority.

- **Summary:** Start with 1–3 short bullets, normally under 50 words total.
  Put the most important outcome first. Include only the problem context needed
  to understand it; leave file inventories, investigation history, and routine
  compatibility assurances out. This is an editorial target, not a hard limit.
- **Supporting detail:** Add Key Changes, Motivation / Problem, or Before / After
  only when they help the reviewer understand or assess the change. Order by
  impact, group by behavior, and give each bullet one main point. Explain a
  mechanism only when it clarifies an outcome, tradeoff, or risk. Avoid repeating
  the Summary or describing each changed file. Use comparisons when they add
  information; performance claims need a baseline, workload, metric, and limits.
- **Risks and migration:** Put material risks or Breaking Changes immediately
  after Summary, with the affected users and required action. Qualify claims
  where they appear; a shorter description must not imply broader support or
  stronger validation than the evidence establishes.
- **Testing:** Group exact commands, targets, or test names with their results.
  Keep failures, pending checks, skipped checks and their reasons visible.
  Report relevant CI observations or measurements; distinguish measured results
  from expectations. Link lengthy supporting logs or provenance, or put them
  in an expandable details block. Omit routine investigation and review diaries;
  retain required review evidence in a compact, accessible record.
- **Visual verification:** For 3D structure or behavior changes, use
  `dart-verify-sim` and [the simulation-verification guide](agent-sim-verification.md).
  Keep claim-relevant media and before/after comparisons, captions, and visible
  observations directly visible in a Visual verification section after Testing.
  Use the same camera, dimensions, and renderer for before/after captures.
  Preserve baseline identities, text correctness evidence, assessment/verdict,
  claim boundaries, limitations, and reproduction commands. Follow the guide for unavailable evidence and
  GitHub-hosted publication; never commit transient media. Brevity does not
  reduce the required evidence or impose a length limit on this section.
- **Related links and checklist:** Include relevant issues, backports, and
  follow-ups; omit empty sections and "None" boilerplate. Keep the template's
  checklist collapsed at the bottom, set the branch milestone, and mark
  non-applicable items "N/A" with a short reason.

Summary and Testing are the default narrative sections. Add only the supporting
sections the change needs; optional headings do not make applicable evidence,
migration guidance, or backport requirements optional. On updates, rewrite the
title and body around the final diff and current evidence instead of appending
the history of fixes.

#### Compact Example

For a hypothetical CI change, keep the outcome in Summary and the important
exception beside it. Replace the illustrative test results with actual evidence:

```markdown
## Summary

- Restore compiler caching in supported CI jobs.
- Skip unnecessary builds for documentation-only changes.

## Limitations

- Windows MSBuild still builds without compiler caching.

## Testing

- `pixi run test-ai-infra`: passed, including path-filter regression cases.
- Hosted cache-hit measurements: pending; no build-time improvement measured yet.
```

The full template checklist still applies. A simulation change also needs its
assessed visual evidence; the short example does not replace that requirement.

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
- If the PR needs the latest target branch, use explicit maintainer/user
  approval to merge that branch into the PR branch instead of rebasing the
  published PR history
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
| [Michael X. Grey](https://github.com/mxgrey)           | Project director, extensive API improvements, inverse kinematics, gui, tutorials            |
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

## PR Readiness Checklist

Before submitting your pull request, verify:

- [ ] Code follows [style guide](code-style.md) (`pixi run lint` passes)
- [ ] Tests are included for new functionality
- [ ] All tests pass (`pixi run test` shows "100% tests passed")
- [ ] Documentation is updated if needed
- [ ] Commit messages are clear and descriptive
- [ ] No merge conflicts with main branch
- [ ] PR description leads with a concise summary and reports testing outcomes
- [ ] PR description uses `.github/PULL_REQUEST_TEMPLATE.md`
- [ ] Milestone is set for the target branch (`DART 7.0` for `main`,
      branch-matching DART 6.x release milestone for the active DART 6 LTS branch)
- [ ] `CHANGELOG.md` is updated according to [changelog.md](changelog.md), or
      the PR records why no entry is needed
- [ ] Bug fixes that apply to the release line have an active-DART-6-LTS PR
      first, then a `main` PR
- [ ] Any `docs/dev_tasks/<task>/` folder used for tracking is removed after
      durable notes move to the owner selected by `docs/README.md`

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
