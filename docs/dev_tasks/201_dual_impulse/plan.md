# Issue 201 - Split Impulse Plan

## Goal

Implement split impulse contact resolution to separate velocity constraints from
position correction, add regression coverage, and provide a GUI example that
demonstrates the stability benefit.

## Plan

1. [x] Confirm the issue reproduces on current origin/main using a minimal
       contact scenario and capture repro steps.
2. [x] Add solver support for split impulse position correction and adjust
       contact constraints to separate velocity vs position correction.
3. [x] Integrate position corrections into the classic rigid solver without
       affecting velocities.
4. [x] Add regression tests in tests/ and update or add a GUI example.
5. [x] Run pixi lint/test/test-all/test-gz with 16-core parallelism and
       iterate until green.
