# Release Branch North Star

For DART 6.20, the north star is a stable compatibility branch with a smaller
dependency footprint, preserved downstream Gazebo/gz-physics behavior, and
clear maintenance workflow support.

Near-term AI-assisted work should prioritize:

- one-dependency or one-vendored-tree cleanup PRs;
- compatibility evidence for package components and installed headers;
- release-branch CI and Gazebo gates;
- clean handoffs through `docs/dev_tasks/`.

Do not use DART 7 clean-break assumptions as proof that a DART 6.20 removal is
safe. Use DART 7 only as reference evidence and then prove the release-branch
compatibility surface directly.
