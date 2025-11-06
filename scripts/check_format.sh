#!/bin/bash

if [ "$#" -lt 2 ]; then
  echo "Usage: ./check_format.sh <formatter-command> [<file> ...]" >&2
  exit 1
fi

formatter_command=$1
shift 1 # Remove formatter command from arguments

trap 'status=$?; if [ $status -eq 0 ]; then echo "check_format.sh exit status: 0" >&2; else echo "check_format.sh exit status: $status" >&2; fi' EXIT

files_with_issues=()

for file in "$@"; do
  output="$("$formatter_command" -style=file -output-replacements-xml "$file")"
  if grep -q "<replacement " <<<"$output"; then
    files_with_issues+=("$file")
  fi
done

num_changes=${#files_with_issues[@]}

if [ "$num_changes" -eq 0 ]; then
  echo "Every file seems to comply with our code convention."
  exit 0
else
  echo "Found $num_changes files that need changes."
  for file_with_issue in "${files_with_issues[@]}"; do
    echo "Review needed: $file_with_issue"
  done
  exit 1
fi
