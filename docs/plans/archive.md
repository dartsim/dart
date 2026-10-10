# DART 6 Plan Archive

Completed DART 6.20 plan entries move here after their durable output has moved
to `docs/onboarding/`, `docs/design/`, `docs/background/`, `docs/readthedocs/`,
code, tests, examples, or release docs.

Keep entries short. Git history owns the old working plan text.

### PLAN-621: DART 6.20.0 performance closeout

The maintainer accepted the 2026-10-09 Gazebo Jetty before/after evidence on
[#3056](https://github.com/dartsim/dart/issues/3056#issuecomment-6087247739)
as the 6.20.0 lane closeout. The measured revision was `2e048b1d5f19`:
three simulated seconds took a median 30.96 s versus 583.65 s in DART 6.19.4
(18.9x faster), with no sinking bodies and all 3,003 asleep at the end.
This is three repeats on one host over a three-second window, not a claim
about dense piles, sustained motion, sensors, rendering, or wake events.
This retires its task folder independently of issue closure; it does not claim
full real-time Gazebo performance or completion of broader workload evidence.
Continuing work targets 6.21 in the dashboard. Compatibility guidance lives in
[architecture](../onboarding/architecture.md#performance-compatibility),
[collision-backend design](../design/dart6_collision_backends.md),
[release management](../onboarding/release-management.md#abi-window), and
[performance methodology](../onboarding/profiling.md). Unratified experimental
SIMD, packaged-ISA, and manifold-policy proposals are not release contracts;
any future proposal needs fresh evidence and maintainer review.
