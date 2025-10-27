# Development Tasks

Documentation for development tasks in DART.

## Structure

- **active/** - Currently active projects with task trackers

## Documentation Principles

**Task docs should**:
- ✅ Track **current status** and **next steps**
- ✅ Document **key decisions** and **why** (not just what)
- ✅ Point to **code as source of truth**
- ❌ Avoid hardcoded lists (file lists, dependency versions) that become outdated
- ❌ Avoid full history - focus on current state

**When task is completed**:
1. Move long-term content to `docs/onboarding/` (design decisions, architecture, workflows)
2. Remove the entire task folder from `docs/dev_tasks/`
3. No archiving - keep onboarding docs up-to-date instead

**Before submitting PRs:**
1. Run `pixi run test-all` - comprehensive test suite
2. Fix any failures before pushing
3. **Important**: If GitHub CI fails but `test-all` passed locally, update `test-all` to catch that failure

**Before committing:**
- Run `validate_changes` to check for errors
- Update task status in tracker
- No author names or ownership attribution

## Related

- [Main docs](../README.md)
- [Onboarding](../onboarding/README.md)
