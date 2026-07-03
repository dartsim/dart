# Changelog Guide

`CHANGELOG.md` is DART's human-readable release record. It should explain what
changes for users, downstream projects, package maintainers, and contributors.
It is not a git log, design document, active plan, or exhaustive implementation
diary.

This guide adapts the useful parts of
[Keep a Changelog](https://keepachangelog.com/) and
[Common Changelog](https://common-changelog.org/) to DART's larger release
shape: write for humans, group by reader impact, link each notable change to
evidence, and remove noise before release.

## When To Update

Update `CHANGELOG.md` for:

- breaking changes, removals, deprecations, and migration guidance;
- public C++ API, dartpy, CMake package, build, dependency, platform, model
  loading, GUI, example, tutorial, or command changes visible outside an
  implementation file;
- bug fixes that affect released users, downstream compatibility, simulation
  correctness, packaging, CI gates, or documented workflows;
- major internal architecture work when it changes a public contract, release
  gate, performance claim, benchmark surface, or contributor workflow;
- docs and AI-infra changes that alter how contributors or agents choose, run,
  verify, package, or review work.

Do not add entries for typo-only edits, pure formatting, generated-file churn,
small refactors with no user-visible behavior, or lockfile/dependency refreshes
whose impact is already covered by a broader packaging entry. If a PR skips the
changelog, record the reason in the PR checklist or description.

## Where Entries Go

- `main` targets DART 7, so entries normally go under
  `## DART 7` -> `### [DART 7.0.0 (TBD)]`.
- DART 6 LTS fixes go under the next release section on the active DART 6
  maintenance branch. Prefer the highest maintained `release-6.*` branch
  advertised by the remote or release-management docs; this checkout currently
  sees `release-6.20`.
- Bug fixes that need both DART 6 LTS and DART 7 should get branch-appropriate
  entries in both PRs.
- Release packaging replaces `TBD` with the release date and final milestone
  link, then prunes stale notes or moves them to durable docs before tagging.

For DART 7, use the existing release section shape:

1. Release Highlights
2. Upgrade and Compatibility Notes
3. Requirements and Supported Platforms
4. Breaking Changes
5. Simulation and Solvers
6. Collision and Geometry
7. IO and Parsing
8. Python Bindings
9. GUI, Examples, and Tutorials
10. Gazebo Integration
11. Core Libraries and APIs
12. Build, Packaging, and Developer Tooling
13. Tests, Benchmarks, and Quality Gates

Do not create a one-off category for a single PR unless the release truly needs
that category. Prefer the nearest existing domain section.

## Entry Style

- Write one bullet per reader-visible outcome. Merge closely related PRs into
  one entry when they ship one coherent capability. The DART 7 section is not
  one bullet per PR.
- Start with the outcome, not the implementation chore. Avoid "misc",
  "cleanup", "update", and PR-title-only wording.
- Keep the primary entry short. One to three wrapped lines is the target. If
  the details need more space, put them in an owner doc, plan, migration note,
  or PR description and link that document.
- Keep each DART 7 section scannable for humans. Most sections should have a
  handful of bullets; a section that starts reading like an implementation
  checklist needs consolidation.
- Include the best evidence link at the end, normally the PR:
  `([#1234](https://github.com/dartsim/dart/pull/1234))`.
- Add issue links only when they explain the user-facing problem better than
  the PR. External issues should use an owner/repo-qualified label.
- For `Breaking Changes`, `Removed`, and `Deprecated` entries, include the
  replacement API, support lane, or migration note in the same bullet or link
  to the owning migration doc.
- Quantified performance claims need benchmark or packet evidence. If the
  evidence is not part of the PR, state the unquantified outcome instead.
- Avoid author credits in changelog entries. Git history and PR metadata own
  attribution.

Good DART 7 entry shape:

```markdown
- Added opt-in DART 7 body deactivation for rigid bodies and semi-implicit
  multibodies, with C++/dartpy sleep-state accessors, replay/serialization
  coverage, and a Python visual verifier.
  ([#2939](https://github.com/dartsim/dart/pull/2939))
```

Poor entry shape:

```markdown
- Update world.cpp and tests.
```

## AI And PR Workflow

Agents and contributors should make the changelog decision before PR creation,
then finalize the entry once the PR number exists.

AI workflows should invoke the reusable `dart-changelog` routine when a change
may need release notes. Direct user calls are uncommon; the routine is mainly a
shared decision/writing path for PR, docs, issue-fix, resume, and release
workflows.

1. Decide whether an entry is required using this guide.
2. Record the reusable `Changelog decision` note from `dart-changelog`,
   including the inspected base/diff evidence, target release section, entry
   text or exact no-entry reason, and any PR-link follow-up.
3. If no entry is required, record the evidence-backed reason in the PR
   description or checklist.
4. If an entry is required before the PR number exists, draft the entry without
   a PR link or leave it for the PR-number follow-up.
5. After the PR exists, add the PR link in the same entry. Keep this follow-up
   local until explicit maintainer/user approval allows another push.
6. For AI-generated draft entries, curate aggressively: summarize the outcome,
   remove implementation noise, merge related bullets, and link to the owner
   doc for details.

## Release Audit

Before a DART 7 release or release-hardening pass:

- scan recent merged PRs against the DART 7 section and add missing notable
  reader-facing outcomes;
- confirm every breaking/removal/deprecation entry names a migration path,
  support branch, or owner doc;
- confirm the Release Highlights and Upgrade and Compatibility Notes still
  match `docs/onboarding/release-roadmap.md`,
  `docs/design/dart7_clean_break_strategy.md`, and the architecture docs;
- consolidate any long implementation ledger into a concise changelog summary
  backed by PRs or owner docs;
- run `pixi run lint` so Markdown, generated AI adapters, and changelog
  formatting are checked before commit.
