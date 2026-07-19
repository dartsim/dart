# Invalidated arch-25 trajectory

The former `arch25_alldynamic_seating_collapse.csv` is not evidence and was
removed on 2026-07-11.

- Former file SHA-256:
  `a975c88cd71b07a19417c423048f1a4ace04049ec02f26168e42643fa753d0c3`
- Row 270 was concatenated with a different row 300 run (`success` was cut to
  `succe300`).
- A detached fragment appeared later in the file.
- The remaining rows alternated between mutually incompatible trajectories,
  including one with hundreds of fallbacks and another returning to a stable
  state.

Because no complete invocation record can disambiguate or reconstruct the two
interleaved processes, individual rows must not be salvaged. A replacement
requires one fresh sequential run with command metadata and an intact schema.
