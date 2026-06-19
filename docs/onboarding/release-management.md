# Release Management

DART 6.20 work targets `release-6.20` and the branch-matching DART 6.x
milestone.

Release-branch PRs should:

- preserve DART 6 compatibility unless explicitly approved otherwise;
- document package and dependency changes clearly;
- run Gazebo/gz-physics gates when downstream behavior can be affected;
- keep changelog and version metadata changes separate from unrelated cleanup
  when possible.
