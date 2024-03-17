#!/bin/bash

if [ "$#" -lt 2 ]; then
  echo "Usage: ./check_format.sh <formatter-command> [<file> ...]" >&2
  exit 1
fi

formatter_command=$1
shift 1 # Remove formatter command from arguments

# Prepare the file list as a null-delimited string for xargs
printf "%s\0" "${@}" | xargs -0 -P$(nproc) -I{} bash -c '
file="$1"
formatter_command="$2"
output=$($formatter_command -style=file -output-replacements-xml "$file")
if grep -q "<replacement " <<<"$output"; then
  echo "$file"
fi
' _ {} $formatter_command | {
  # Read the results of xargs processing
  files_with_issues=()
  while IFS= read -r file_with_issue; do
    files_with_issues+=("$file_with_issue")
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
}
