#!/bin/bash

org="$1"
repo="$2"

echo "Deleting workflow runs for $org/$repo"

workflows_temp=$(mktemp) # Creates a temporary file to store workflow data.

gh api repos/$org/$repo/actions/workflows | jq -r '.workflows[] | [.id, .path] | @tsv' >$workflows_temp # Lookup workflow
cat "$workflows_temp"

workflows_names=$(awk '{print $2}' $workflows_temp | grep -v "main")

if [ -z "$workflows_names" ]; then

  echo "All workflows are either successful or failed. Nothing to remove"

else

  echo "Removing all the workflows that are not successful or failed"

  for workflow_name in $workflows_names; do

    workflow_filename=$(basename "$workflow_name")
    echo "Deleting |$workflow_filename|, please wait..."

    gh run list --limit 500 --workflow $workflow_filename --json databaseId |
      jq -r '.[] | .databaseId' |
      xargs -I{} gh run delete {} # Delete all workflow runs for workflow name
  done
fi

rm -rf $workflows_temp

echo "Done."
