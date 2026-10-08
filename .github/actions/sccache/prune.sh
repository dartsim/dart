#!/usr/bin/env bash
# Delete the compiler-cache snapshots that the snapshot a job just saved
# supersedes, so saving on every run stays within the repository's 10 GB
# Actions cache. The build jobs run this themselves: a separate job would wait
# for a runner of its own, and on a cancelled run it would also hold the PR's
# concurrency group, delaying the next push's run.
#
# Usage: prune.sh <prefix> <main|pr>, where <prefix> is
# sccache-v2-<os>-<part>-<compiler version>. Needs GH_TOKEN with actions:write
# and GH_REPO. Failures print warnings only; the next run prunes again.
set -uo pipefail
prefix=$1
snapshot=$2

delete() {
  while read -r id key; do
    echo "Deleting $key"
    gh cache delete "$id" || echo "::warning::Could not delete compiler cache $key"
  done
}

# In this ref, keep this part's newest snapshot. Keys end in
# <run_id>-<run_attempt>, and run IDs follow push order, so an older run that
# finishes last cannot evict a newer commit's snapshot.
gh cache list --ref "$GITHUB_REF" --key "$prefix-$snapshot-" --limit 100 --json id,key \
  --jq 'sort_by(.key | split("-")[-2:] | map(tonumber)) | .[:-1][] | "\(.id) \(.key)"' |
  delete || echo "::warning::Could not list compiler caches to prune"

# Main pushes also delete this part's PR snapshots idle for a day, such as
# those of closed PRs.
if [ "$snapshot" = main ]; then
  gh cache list --key "${prefix%-*}-" --limit 1000 --json id,key,ref,lastAccessedAt \
    --jq '.[] | select((.ref | startswith("refs/pull/")) and .lastAccessedAt < (now - 86400 | todate)) | "\(.id) \(.key)"' |
    delete || echo "::warning::Could not list idle PR compiler caches"
fi
exit 0
