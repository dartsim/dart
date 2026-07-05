# DART Documentation

This directory contains documentation for DART users, developers, and contributors.

## For New Contributors

**Start here:** [onboarding/README.md](onboarding/README.md) - Developer
knowledge base and onboarding guide for architecture, components, and
workflows.

## Documentation Structure

| Directory                               | Owns                                                                             |
| --------------------------------------- | -------------------------------------------------------------------------------- |
| [`ai/`](ai/README.md)                   | AI-native principles, terminology, north star, workflow map, sessions, and gates |
| [`assets/`](assets/)                    | Source-controlled assets used by repository docs outside RTD static              |
| [`background/`](background/README.md)   | Theory and research foundations derived from preserved source PDFs               |
| [`design/`](design/README.md)           | Durable technical design proposals and decision rationale                        |
| [`dev_tasks/`](dev_tasks/README.md)     | Active multi-session implementation task tracking                                |
| [`doxygen/`](doxygen/)                  | Doxygen inputs consumed by the published C++ API build                           |
| [`onboarding/`](onboarding/README.md)   | Developer knowledge base, onboarding, architecture, and workflows                |
| [`plans/`](plans/README.md)             | Living roadmap, priority, open gaps, gates, and plan lifecycle                   |
| [`python_api/`](python_api/index.rst)   | Reusable dartpy API module pages for Sphinx autodoc                              |
| [`readthedocs/`](readthedocs/README.md) | Published user documentation site source                                         |

The Markdown developer and agent docs are optimized for GitHub and LLM-friendly
inspection. The published site under `readthedocs/` is Sphinx/RST plus MyST
Markdown and owns end-user navigation.

## Quick Links by Task

- **New to DART development?** → [onboarding/README.md](onboarding/README.md)
- **Building from source?** → [onboarding/building.md](onboarding/building.md)
- **Contributing code?** → [onboarding/contributing.md](onboarding/contributing.md) + [onboarding/code-style.md](onboarding/code-style.md)
- **Writing release notes?** → [onboarding/changelog.md](onboarding/changelog.md) + [onboarding/release-management.md](onboarding/release-management.md)
- **Understanding architecture?** → [onboarding/architecture.md](onboarding/architecture.md)
- **Writing durable design rationale?** → [design/README.md](design/README.md) + [onboarding/api-boundaries.md](onboarding/api-boundaries.md)
- **Understanding theory?** → [background/README.md](background/README.md)
- **Published docs/API site?** → [readthedocs/README.md](readthedocs/README.md) + [onboarding/api-documentation.md](onboarding/api-documentation.md)
- **Revising project plans?** → [plans/README.md](plans/README.md) + [plans/dashboard.md](plans/dashboard.md) + [plans/north-star-roadmap.md](plans/north-star-roadmap.md)
- **Using AI agent workflows?** → [ai/README.md](ai/README.md) + [ai/principles.md](ai/principles.md) + [ai/north-star.md](ai/north-star.md) + [onboarding/ai-tools.md](onboarding/ai-tools.md)
- **Letting AI pick next work?** → [ai/README.md#choosing-the-next-task](ai/README.md#choosing-the-next-task) + [ai/workflows.md](ai/workflows.md)
- **Using DART API?** → [dart.readthedocs.io](https://dart.readthedocs.io/)
- **Looking for examples?** → [examples/](../examples/) and [tutorials/](../tutorials/)
