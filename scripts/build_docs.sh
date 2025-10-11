#!/usr/bin/env bash

set -ex

script_path="$(
  cd "$(dirname "$0")"
  pwd -P
)"

WORK_DIR="${GITHUB_WORKSPACE}/dart_docs"
DART_CLONE_DIR="${WORK_DIR}/dart"
DART_DOCS_OUTPUT_DIR="${GITHUB_WORKSPACE}/gh-pages"
DART_DOCS_BUILD_DIR="${WORK_DIR}/build/"

help() {
  cat <<EOF
  Usage: ${0##*/} <routine> [<options>...]

  Example: ${0##*/} build  # Build with the default settings

  Routines:
    build      Generate API documentation to ${DART_DOCS_OUTPUT_DIR}
    clean      Clean up the generated documentation
EOF
}

if [ "$#" -lt 1 ]; then
  help
  exit
else
  routine=$1
  shift
fi

while [ $# -gt 0 ]; do
  case "$1" in
  *)
    bash_command=$@
    break
    ;;
  esac
  shift
done

clean() {
  echo "======================================================================="
  echo " [Clean] "
  echo "======================================================================="

  rm -rf ${DART_DOCS_BUILD_DIR}
  rm -rf ${DART_DOCS_OUTPUT_DIR}
  rm -rf ${DART_CLONE_DIR}
  rm -rf ${WORK_DIR}
}

build() {
  echo "======================================================================="
  echo " [Build] "
  echo "======================================================================="

  mkdir -p ${WORK_DIR}

  # For branch builds, the CI could clone that branch with a fixed depth of
  # commits. This means that the clone knows nothing about other Git branches or
  # tags. We fix this by deleting and re-cloning the full repository.
  git clone "https://github.com/dartsim/dart.git" ${DART_CLONE_DIR}
  
  # Checkout the branch if BRANCH_NAME is set, otherwise stay on default branch
  if [ -n "${BRANCH_NAME}" ]; then
    git -C ${DART_CLONE_DIR} checkout ${BRANCH_NAME} || echo "Branch ${BRANCH_NAME} not found in upstream, using default branch"
  fi

  # Organize into "docs" directory
  mkdir -p ${DART_DOCS_OUTPUT_DIR}

  # Initialize list of API versions
  cat <<EOF >${DART_DOCS_OUTPUT_DIR}/index.md
# DART API Documentation
EOF

  # Build docs of multiple versions in "build" directory
  mkdir -p ${DART_DOCS_BUILD_DIR}
  cd ${DART_DOCS_BUILD_DIR}

  # Add entries
  while read version; do
    if [[ ${version} == DART* ]]; then
      printf "\n### ${version}\n\n" >>${DART_DOCS_OUTPUT_DIR}/index.md
      continue
    fi

    # Add entry to list of API versions
    printf "* [${version}](https://dartsim.github.io/dart/${version}/)\n" >>${DART_DOCS_OUTPUT_DIR}/index.md

    # Build documentation
    git -C ${DART_CLONE_DIR} checkout ${version}
    rm -rf *
    cmake ${DART_CLONE_DIR}
    make docs

    # Check if docs/doxygen/html is not empty
    if [ "$(ls -A docs/doxygen/html 2>/dev/null)" ]; then
      echo "Moving docs/doxygen/html to ${DART_DOCS_OUTPUT_DIR}/${version}..."
      mv docs/doxygen/html "${DART_DOCS_OUTPUT_DIR}/${version}"
    # Else, check if doxygen/html is not empty
    elif [ "$(ls -A doxygen/html 2>/dev/null)" ]; then
      echo "Moving doxygen/html ${DART_DOCS_OUTPUT_DIR}/${version}..."
      mv doxygen/html "${DART_DOCS_OUTPUT_DIR}/${version}"
    else
      echo "Both docs/doxygen/html and doxygen/html are empty. No files moved."
    fi

  done <${DART_CLONE_DIR}/scripts/docs_versions.txt

}

# Run the selected routine
"$routine"
