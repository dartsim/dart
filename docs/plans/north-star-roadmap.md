# DART 6 North-Star Roadmap

DART 6.20 is the compatibility support lane. Its AI-native roadmap is not to
become DART 7 on a release branch; it is to make maintenance work discoverable,
bounded, verifiable, and safe for downstream users.

## Sequencing Principles

1. Preserve compatibility first. Public headers, installed package components,
   ABI-sensitive class layouts, and Gazebo/gz-physics behavior are
   release-branch constraints.
2. Use DART 7 as reference evidence, not as release-branch proof. Re-run the
   DART 6 gates that cover the affected surface.
3. Prefer one branch and one verification story per packet or PR.
4. Keep behavior-preserving performance or dependency work separate from
   behavior-changing simulation policy.
5. Move durable decisions out of `docs/dev_tasks/` before retiring a task
   folder.

## Planning Surfaces

- `docs/ai/north-star.md` owns the mission and release-branch operating
  posture.
- `docs/plans/dashboard.md` owns current priority and operating state.
- `docs/dev_tasks/` owns active multi-session implementation handoff.
- `docs/design/` owns durable technical decisions.
- `docs/onboarding/` owns landed maintainer and contributor guidance.
- `docs/background/` owns reusable theory and reference foundations.
- `docs/readthedocs/` owns published user guidance.

## Current Roadmap Bias

Near-term work should favor:

- dependency-footprint reduction that preserves package and downstream
  compatibility;
- native collision parity and evidence before any default flip;
- contact-performance work only when new profile evidence or an explicit
  maintainer-approved behavior envelope makes the release-branch target clear;
- deformable-body stability and CPU performance evidence;
- release-branch CI, Gazebo, and documentation gates that catch the real
  compatibility risks.
