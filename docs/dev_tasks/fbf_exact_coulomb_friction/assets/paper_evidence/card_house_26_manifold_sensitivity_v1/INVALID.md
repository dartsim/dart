# Invalid comparison artifact

This governance note is intentionally outside the frozen v1 artifact index:
the index binds the nine raw protocol outputs, while this later overlay records
why those bytes must remain fail-closed instead of being promoted.

This v1 bundle is retained as fail-closed diagnostic history. It must not be
used as the authoritative Compact-versus-FourPointPlanar comparison.

Both children emitted 600 rows. `Compact` returned zero, while
`FourPointPlanar` returned one with zero aggregate exact failures because the
trace executable's final convergence gate found its last-group residual above
`1e-6`. The frozen v1 runner admitted return one only for a terminal aggregate
exact-failure prefix, so it correctly set
`comparison_artifact_integrity_valid=false`.

No solver or scene parameter was changed. A new protocol version is required
before rerunning the same fixed commands with the newly observed full-duration
convergence-gate exit class represented explicitly.
