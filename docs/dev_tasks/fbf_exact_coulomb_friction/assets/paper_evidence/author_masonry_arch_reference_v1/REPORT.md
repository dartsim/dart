# Current-author masonry-arch reference v1

Status: **valid current-source scientific negative**.

This sealed bundle records the completed pinned author-source run at
`b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0`. The observed command used 500 frames and released the three
cubes at frame 400 (solver substep 1600). The runner's source
default is 400 frames with `drop_frame=400`, which never releases the cubes;
therefore this is a newly declared current-source diagnostic, not a historical
or paper invocation.

## Solver result

- A deterministic projection represents all 2000 solver substeps
  and is lossless with respect to every declared claim-bearing natural and
  configured residual field. The 382,753,953-byte raw history is bound by
  SHA-256 but intentionally omitted; the projection is about 8.1 MB instead of
  a 49 MB raw archive.
- Author convergence flags: 157 converged, 1,843 nonconverged.
- Of the 157 true flags, 40 are zero-outer-iteration accepts from the initial
  natural-residual shortcut and 117 are outer solves accepted by the configured
  nonnegative `coulomb_rel < 1e-6` gate.
- Before release: 142 converged and 1,458 nonconverged substeps.
- After release: 15 converged and 385 nonconverged substeps.
- Only 47 `final_residual` values are at or below `1e-6`. That field is the
  natural residual; it is not the configured `coulomb_rel` convergence gate.
- First nonconverged substep: 53. Maximum natural residual:
  4.1130565788445415 at substep 226.
- Release substep 1600 has 100 contacts and natural residual
  0.017456069692858667. The final substep has 108 contacts and natural residual
  0.5161195175386001; both carry negative convergence flags.

The first post-release contact-count increase is inferred at substep 1944
(100 to 102), and the peak is 109 at substep 1947. The projection does not
record contact-pair identities, so this is not definitive cube-arch contact
evidence.

## Claim boundary

Exit code 0 and intact artifacts validate sealing of this current-source
diagnostic. They do **not** validate all-substep solver success, DART or
cross-solver dynamics parity, paper timing, repeatability, or a historical
paper invocation.
