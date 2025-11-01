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
1. **Add brief section** to existing `docs/onboarding/` file (e.g., add to constraints.md, not new file)
2. **Remove the entire task folder** from `docs/dev_tasks/`
3. **Keep onboarding docs concise** - too many docs make them hard to use effectively

**Onboarding docs must stay lean**:
- ❌ Don't create new detailed files for every completed task
- ✅ Add brief sections to existing relevant docs
- ✅ Focus on key design decisions only (2-5 sentences)
- ✅ Point to code for implementation details
- ⚠️ LLMs struggle with bloated documentation - keep it minimal

**What NOT to include in onboarding docs**:
- ❌ **Hardcoded file lists** - Files change, become outdated
- ❌ **Code snippets** - Code evolves, docs won't
- ❌ **Detailed API documentation** - Code comments are source of truth
- ❌ **Step-by-step implementation guides** - Read the actual code
- ❌ **Performance numbers** - These change with optimizations
- ✅ **DO**: High-level design decisions with "why" rationale
- ✅ **DO**: Architectural patterns used (e.g., "hybrid approach", "threshold-based")
- ✅ **DO**: Directory pointers (e.g., "see `dart/lcpsolver/dantzig/`")

**Remember**: Code is the source of truth. Documentation explains *why*, code shows *how*.

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
