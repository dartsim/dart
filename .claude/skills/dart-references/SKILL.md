---
name: dart-references
description: "DART References: manage the experimental simulation research catalog (papers, textbooks, engines) with status, priority, and verdict"
---

# DART Research References

Load this skill when adding, updating, auditing, or citing research references
(papers, textbooks, model-format standards, or comparative engines) for the DART
experimental simulation world — its public API and its algorithms.

## Catalog Location

`docs/readthedocs/papers.md` is the single source of truth and the published
website page. It is a companion to the experimental API design docs:

- `docs/design/simulation_cpp_api.md`
- `docs/design/simulation_python_api.md`

## Entry Schema

Each entry has an `id`, a full citation, and these properties:

| Property       | Values                                                                                                     |
| -------------- | ---------------------------------------------------------------------------------------------------------- |
| **Type**       | `textbook`, `paper`, `standard`, `engine`                                                                  |
| **Topic**      | e.g. `dynamics`, `kinematics`, `contact`, `integration`, `collision`, `terminology`, `model-format`, `api` |
| **Status**     | `referenced`, `planned`, `in-progress`, `implemented`, `deferred`, `rejected`                              |
| **Priority**   | `high`, `medium`, `low`, `—`                                                                               |
| **Verdict**    | `adopt`, `baseline`, `reference`, `evaluate`, `reject`                                                     |
| **Where used** | link to the design doc, code, or test that uses (or will use) it                                           |

`Status` is written from the experimental world's perspective. A method shipping
in _classic_ DART but not yet in the experimental world is `planned`, with the
classic location noted in `Notes`.

## Workflow: add or update an entry

1. Ground the entry in real evidence — a design doc, code path, test, or an
   explicit project decision. Do not add aspirational references with no link.
2. Write an accurate citation (authors, title, year, venue/publisher). Prefer an
   official URL; do not fabricate DOIs.
3. Add a row to the matching summary table **and** a detail subsection with the
   full properties and a one-line rationale.
4. Set `Status` from the experimental world's perspective; note classic-DART
   status separately in `Notes`.
5. Set `Verdict` (`adopt` to build on, `baseline` to compare against,
   `reference` to cite, `evaluate` while undecided, `reject` with a reason).
6. Fill `Where used` with the design-doc/code/test link.
7. When the relationship changes (e.g. a `planned` algorithm becomes
   `implemented`, or an `evaluate` is decided), update `Status`/`Verdict` and the
   `Where used` link in the same change.

## Rules

- Prefer robotics/dynamics literature (textbooks, papers) over engine-specific
  names when a reference informs a terminology decision; record the grounding in
  the catalog's "Terminology Grounding" section.
- Keep the catalog scoped to the experimental world for now; the schema is
  general and can extend to the rest of DART later.
- Keep `engine` entries as baselines/comparisons, never as dependencies.
- Use `docs/ai/verification.md` to select the docs-only or AI docs/adapters
  gate set, then run `pixi run lint` after edits.

## Full Documentation

For the catalog and its rationale: `docs/readthedocs/papers.md`

For the API it supports: `docs/design/simulation_cpp_api.md`,
`docs/design/simulation_python_api.md`

For DART's research focus: `docs/ai/north-star.md`
