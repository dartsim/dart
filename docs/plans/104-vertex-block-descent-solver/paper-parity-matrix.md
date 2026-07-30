# PLAN-104 VBD / AVBD Paper-Parity Matrix

This document is the durable, fail-closed completion contract for the VBD and
AVBD paper implementations on DART 7 `main`. The machine-readable row
inventories are:

- [`vbd-paper-coverage-contract.json`](vbd-paper-coverage-contract.json)
- [`avbd-paper-coverage-contract.json`](avbd-paper-coverage-contract.json)

Run `pixi run check-plan104-paper-parity` to validate their authoritative source
pins, canonical row order, evidence paths, backend requirements, predicates,
video timecodes, and aggregate completion status. The older
[`avbd-demo-corpus.md`](avbd-demo-corpus.md) remains useful as a detailed
inventory of narrow DART surfaces, but its historical `Complete` labels mean
only that a local asset or packet met that document's earlier row contract.
They do not establish AVBD solver identity or paper parity.

## Current Fail-Closed Snapshot

Snapshot date: 2026-07-30. No canonical row is complete under the contract
below, so neither solver family has a paper-parity claim.

| Family / source surface |    Rows | Partial | Blocked | Missing | Complete |
| ----------------------- | ------: | ------: | ------: | ------: | -------: |
| VBD method              |      17 |      15 |       0 |       2 |        0 |
| VBD limitations         |       4 |       4 |       0 |       0 |        0 |
| VBD paper figures       |      24 |      13 |       0 |      11 |        0 |
| VBD paper tables        |       1 |       1 |       0 |       0 |        0 |
| VBD official video      |      19 |       6 |       0 |      13 |        0 |
| VBD Gaia dynamics       |      14 |       0 |       0 |      14 |        0 |
| VBD Gaia cloth          |       5 |       0 |       0 |       5 |        0 |
| VBD TinyVBD             |       1 |       1 |       0 |       0 |        0 |
| VBD project page        |       3 |       0 |       0 |       3 |        0 |
| **VBD total**           |  **88** |  **40** |   **0** |  **48** |    **0** |
| AVBD method             |      20 |      16 |       0 |       4 |        0 |
| AVBD paper figures      |      15 |       9 |       0 |       6 |        0 |
| AVBD paper tables       |       2 |       1 |       0 |       1 |        0 |
| AVBD official video     |      14 |       7 |       0 |       7 |        0 |
| AVBD demo2d             |      19 |      19 |       0 |       0 |        0 |
| AVBD demo3d             |      14 |      14 |       0 |       0 |        0 |
| AVBD project page       |       4 |       2 |       0 |       2 |        0 |
| **AVBD total**          |  **88** |  **68** |   **0** |  **20** |    **0** |
| **Grand total**         | **176** | **108** |   **0** |  **68** |    **0** |

`Partial` means only that at least one relevant DART artifact exists and that
the row still records an explicit blocker. It is negative evidence against a
complete claim, not permission to generalize from the artifact.

## Authoritative Sources

The contracts pin the exact source identity. Changing a pin or silently
dropping a row fails the checker.

### VBD

- Paper: _Vertex Block Descent_, official PDF, 20,000,116 bytes,
  SHA-256
  `4ba65c4e49e8e8740aca491c2bf466b6eeb6f8824a152c12d1f7800828959132`.
- Official project video: YouTube `2HCgKfKy3W8`, 331 seconds, audited format 18,
  SHA-256
  `fe5e669c2260a507735cd77e10926a3969fd78fee8c983b328274c25ac5682af`.
- Official project page: HTML SHA-256
  `2c98c3158d94f2d3d6193381e7f266507f7a722c1ae3c6777786d98ccd84c16f`;
  the distinct teaser, teapot GIF, and armadillo GIF media hashes and frame
  contracts are pinned in the JSON inventory.
- Gaia: revision `c229692045465a76233f9fba9197fb22bbfb3694`.
- TinyVBD: revision `dcd011a5d945172e247ecced90a6c2c4b4313520`.

The VBD contract enumerates all 24 paper figures, Table 1, all 19 contiguous
official-video segments, all 14 Gaia dynamics configurations, all 5 Gaia cloth
configurations, the TinyVBD tilted-strand default, and the official project
page's distinct 23.367-second teaser and two stability GIFs.

### AVBD

- Paper: _Augmented Vertex Block Descent_, official PDF, 13,803,273 bytes,
  SHA-256
  `7957d116b9130cfb0aa5a48ab7cd0d74a64ad79f75c99acd291bdece2be3f2d6`.
- Official project video: YouTube `bwJgifqvd5M`, 279 seconds, audited format 18,
  SHA-256
  `2f685b9214c8afd1b35bee54c76c80aaa2d69ac5497cfdef5d229be3292e420d`.
- Official project page: HTML SHA-256
  `602525cc0a21b32b71d742ea38be27f68d6672b7c6968397a2ac44c21a5d3aac`;
  its distinct teaser and chain-mail comparison hashes and frame contracts are
  pinned in the JSON inventory. The self-contained 2D and 3D browser
  deployments are pinned there at SHA-256
  `89bd971d6bd0f3b6b79ae1cd3d7a6215d15852dd5f06bd65d7f338a00171e13e`
  and
  `64c10c6d63474b0d9a7e14e7778390baca964ab4f95d76aa5c3adef12ba0cf3d`.
- `avbd-demo2d`: revision
  `74699a11f8586d3ac34474c92b1ef8feb5f587de`.
- `avbd-demo3d`: revision
  `7701bd427d55ca5d03ea1fdf331912ded9169f4b`.

The AVBD contract enumerates all 15 paper figures, both paper tables, all 14
contiguous official-video segments, all 19 2D source scenes, all 14 3D source
scenes, and four official project-page surfaces: its distinct teaser, both
deployed browser demos, and the chain-mail comparison video. The project page
also links a Two Minute Papers video (`TzIKbjuSy2A`); it reuses the authors'
media and is recorded as a noncanonical explainer rather than a second
independent parity surface. The associated Real-Time Live extended abstract is
pinned too; its equations, two figures, and timing annotations are a subset of
the main paper and project teaser, so it introduces no duplicate row.

## Completion Rule

A canonical row may change to `complete` only when all of the following are
true and recorded in the row:

1. **Source-matched scope.** The DART scene, inputs, parameters, events,
   declared duration, and accuracy target match the pinned paper, repository,
   project-page, or video surface. A smaller continuation scene stays
   `partial`.
2. **Artifact validity.** The evidence schema validates; source and DART build
   identities, hashes, ordering, dimensions, frame counts, replay inputs, and
   media binding are explicit.
3. **Solver-contract validity.** The intended VBD or AVBD implementation
   actually handled every required row and step. Any fallback, invalid state,
   sequential-impulse substitution, undisclosed iteration/contact cap, or
   unsupported-row skip fails the predicate.
4. **Physical outcome.** A scenario-specific quantitative oracle passes for
   the full declared duration. Finite output alone is not a correctness oracle.
5. **CPU and CUDA closure.** Both required backend results are explicitly true.
   Backend-neutral setup may be shared, but evidence from one backend cannot
   stand in for the other.
6. **Visual closure.** Required images and videos decode, are bound to the
   validated current-build run, and receive a recorded manual semantic
   inspection. A successful renderer exit or file existence is insufficient.
7. **Comparable performance.** Performance claims match scene, numerical
   precision, backend, hardware, compiler, timer boundary, warmup, aggregation,
   and achieved accuracy. Unmatched results remain descriptive smoke data.
8. **Current-build closure.** Evidence records the exact DART commit and
   build/configuration identity. Any change to solver, collision, pipeline,
   reference-scene, benchmark, or inherited base code invalidates affected
   closure and requires rerunning and resealing it.
9. **Fail-closed blockers.** Missing tools, unavailable hardware, invalid
   artifacts, crashes, fallbacks, caps, and failed or ambiguous visual verdicts
   remain first-class blockers. They are never converted into a positive claim.
10. **Review closure.** After the last implementation or evidence fix, two
    clean review passes must find no correctness, regression, contract, or
    evidence-boundary defect.

The checker additionally requires every row's exact group predicates and
`cpu`/`cuda` backend results to be boolean `true`, its blocker list to be empty,
and its evidence paths to exist before accepting `complete`. Overall VBD or AVBD
status becomes `complete` only when every canonical row for that family is
complete; full PLAN-104 paper parity requires all 176 rows.

## Evidence Workflow

Use the same lifecycle for each bounded implementation packet:

1. Select one or more explicit contract rows and preserve their source
   locators, pins, and blockers.
2. Implement the smallest reusable solver, pipeline, scene, or evidence
   capability that closes those rows without weakening DART's long-term
   architecture.
3. Add focused equation/kernel tests, scenario-level physical oracles, and
   mutation tests that prove the oracle fails when the intended mechanism is
   disabled.
4. Capture solver identity, row/iteration/contact counters, fallback state,
   build identity, and backend identity in a machine-readable packet.
5. Run source-matched CPU and CUDA correctness, then controlled performance
   comparisons at matched achieved accuracy.
6. Follow
   [`../../onboarding/agent-sim-verification.md`](../../onboarding/agent-sim-verification.md)
   for text-first scene diagnostics, current-build renders, media decoding, and
   recorded manual inspection.
7. Update only the rows directly proven by the packet. Keep narrower,
   contradictory, or failed evidence in the ledger with its blocker.
8. Rerun `pixi run check-plan104-paper-parity`, the focused tests, the relevant
   build/runtime gates, and `pixi run lint`; then perform the two clean review
   passes.

This workflow deliberately separates implementation existence, solver
identity, physical correctness, visual semantics, and performance
comparability. No one predicate substitutes for another.
