# AI Agent Skills Expansion — Dev Task

## Current Status

- [x] Phase 1: Minimal skill set (COMPLETED)
- [ ] Phase 2: Evaluate and import additional skills (FUTURE)

## Goal

Expand DART's AI agent skill system with useful external skills from Anthropic marketplace and other sources, while keeping the core skill set minimal and focused.

## Current Skill Inventory

### DART-Specific Skills (Implemented)

| Skill             | Purpose                               | Source |
| ----------------- | ------------------------------------- | ------ |
| `dart-build`      | Build system, CMake, pixi             | Custom |
| `dart-test`       | Testing patterns, CI validation       | Custom |
| `dart-contribute` | PR workflow, dual-branch bugfixes     | Custom |
| `dart-ci`         | CI/CD troubleshooting, GitHub Actions | Custom |
| `dart-io`         | Model loading (URDF/SDF/MJCF/SKEL)    | Custom |
| `dart-python`     | Python bindings (dartpy)              | Custom |

### Utility Skills (Not Installed - Available on Demand)

| Skill           | Purpose                       | How to Install                             |
| --------------- | ----------------------------- | ------------------------------------------ |
| `skill-creator` | Guide for creating new skills | `npx openskills install anthropics/skills` |

### Global Skills (User's Environment)

| Skill            | Purpose                      |
| ---------------- | ---------------------------- |
| `git-master`     | Atomic commits, rebasing     |
| `ultrawork`      | Parallel agent orchestration |
| `frontend-ui-ux` | UI/UX design                 |

## Future Candidates (Phase 2)

### High Priority — Likely Useful

| Skill                    | Why                                | Effort                                     |
| ------------------------ | ---------------------------------- | ------------------------------------------ |
| `open-source-maintainer` | Issue triage, PR review automation | Need to create or find source              |
| `mcp-builder`            | If DART exposes MCP tools          | `npx openskills install anthropics/skills` |

### Medium Priority — Situational

| Skill             | Why                                  |
| ----------------- | ------------------------------------ |
| `doc-coauthoring` | For major documentation efforts      |
| `webapp-testing`  | If DART adds web-based visualization |

### Low Priority — Not Relevant

| Skill                                | Why Skip                                    |
| ------------------------------------ | ------------------------------------------- |
| `pdf`, `docx`, `pptx`, `xlsx`        | DART is C++/Python, not document processing |
| `algorithmic-art`, `canvas-design`   | Not relevant to physics engine              |
| `brand-guidelines`, `internal-comms` | Anthropic-specific                          |
| `slack-gif-creator`                  | Not relevant                                |

## Custom Skills to Create (Future)

| Skill               | Purpose                                | Source Docs                       |
| ------------------- | -------------------------------------- | --------------------------------- |
| `dart-dynamics`     | Articulated bodies, joints, kinematics | `docs/background/dynamics/`       |
| `dart-collision`    | Collision backends (FCL, Bullet, ODE)  | Architecture docs                 |
| `dart-architecture` | Core architecture deep dive            | `docs/onboarding/architecture.md` |

## Key Decisions

- **Minimal start**: Import only essential skills to avoid bloat
- **skill-creator kept**: Enables creating more DART-specific skills
- **Custom over marketplace**: DART-specific skills provide more value than generic ones
- **OpenSkills CLI**: Use `npx openskills` for skill management

## How to Expand Skills

```bash
# Import from Anthropic marketplace
npx openskills install anthropics/skills
# Select specific skills interactively

# Create custom skill
mkdir -p .claude/skills/<skill-name>
# Create SKILL.md with frontmatter

# Sync to AGENTS.md
npx openskills sync

# List current skills
npx openskills list
```

## Immediate Next Steps (When Resuming)

1. Evaluate if `open-source-maintainer` skill exists or needs creation
2. Consider `mcp-builder` if DART adds MCP server functionality
3. Create `dart-dynamics` skill when deep physics work is needed
