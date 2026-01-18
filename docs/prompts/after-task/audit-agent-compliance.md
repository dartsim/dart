# DART: Audit Agent Compliance

<!--
CRITICAL: FOR AGENTS READING THIS FILE
This is a PROMPT TEMPLATE, not an active task.
Do NOT execute unless a human pastes this into a new session.
-->

## When to Use

Use this template when an AI agent failed to follow documented rules/guidelines that already exist. This provides a structured way to:

1. Document what went wrong
2. Analyze why the agent missed the guidance
3. Propose doc improvements to prevent recurrence

## Prompt

```text
# DART: Agent Compliance Audit

An AI agent failed to follow existing documented guidelines. Analyze and fix.

## Incident Report

**What rule was violated?**
<DESCRIBE_THE_RULE>

**Where is it documented?**
<FILE_PATH_AND_SECTION>

**What did the agent do instead?**
<ACTUAL_BEHAVIOR>

**What should have happened?**
<EXPECTED_BEHAVIOR>

## Analysis Tasks

1. **Locate the existing documentation**
   - Find the exact file and line where the rule is documented
   - Confirm the rule is clear and unambiguous

2. **Diagnose visibility issues**
   Check these common problems:
   - [ ] Rule buried in prose (not scannable)
   - [ ] Rule in wrong file (agent wouldn't load it for this task type)
   - [ ] Rule lacks emphasis (no bold, no checklist, no "MANDATORY")
   - [ ] Rule conflicts with or duplicates other guidance
   - [ ] Rule is in a section agents typically skip

3. **Propose fixes** (pick applicable)
   - **Relocate**: Move rule to more prominent location
   - **Reformat**: Add checklist, bold, or "MANDATORY" marker
   - **Consolidate**: Merge with related rules for single source of truth
   - **Cross-reference**: Add pointer from task-specific docs to the rule
   - **Elevate**: Promote to root AGENTS.md if critical enough

4. **Implement the fix**
   - Make minimal changes to improve visibility
   - Prefer restructuring over adding new content
   - Update any indexes if files change

## Quality Checklist

Before completing:
- [ ] The rule is now in a location agents WILL read for this task type
- [ ] The rule uses scannable formatting (checklist, bold, headers)
- [ ] No duplicate/conflicting guidance exists elsewhere
- [ ] Related docs cross-reference each other if needed

## Output

1. Root cause (why agent missed the rule)
2. Changes made (files and nature of change)
3. Confidence level that this prevents recurrence (High/Medium/Low + reasoning)
```
