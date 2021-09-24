#!/usr/bin/env bash

set -ex

WORK_DIR="${HOME}/dart_docs"
mkdir -p ${WORK_DIR}

# For branch builds, the CI could clone that branch with a fixed depth of
# commits. This means that the clone knows nothing about other Git branches or
# tags. We fix this by deleting and re-cloning the full repository.
DART_CLONE_DIR="${WORK_DIR}/dart"
git clone "https://github.com/dartsim/dart.git" ${DART_CLONE_DIR}
git -C ${DART_CLONE_DIR} checkout ${BRANCH_NAME}

# Organize into "docs" directory
DART_DOCS_OUTPUT_DIR="${BUILD_DIR}/gh-pages"
mkdir -p ${DART_DOCS_OUTPUT_DIR}

# Initialize list of API versions
cat <<EOF > ${DART_DOCS_OUTPUT_DIR}/index.md
# DART API Documentation
EOF

# Build docs of multiple versions in "build" directory
DART_DOCS_BUILD_DIR="${WORK_DIR}/build/"
mkdir -p ${DART_DOCS_BUILD_DIR}
cd ${DART_DOCS_BUILD_DIR}

# Add entries
while read version; do
  if [[ ${version} == DART* ]]; then
    printf "\n### ${version}\n\n" >> ${DART_DOCS_OUTPUT_DIR}/index.md
    continue
  fi

  # Add entry to list of API versions
  printf "* [${version}](https://dartsim.github.io/dart/${version}/)\n" >> ${DART_DOCS_OUTPUT_DIR}/index.md

  # Build documentation
  git -C ${DART_CLONE_DIR} checkout ${version}
  rm -rf *
  cmake ${DART_CLONE_DIR}
  make docs

  mv doxygen/html ${DART_DOCS_OUTPUT_DIR}/${version}
done < ${DART_CLONE_DIR}/.ci/docs_versions.txt
