#!/usr/bin/env bash

set -ex

DART_CLONE_DIR="${TRAVIS_BUILD_DIR}"

# For branch builds, Travis only clones that branch with a fixed depth of 50
# commits. This means that the clone knows nothing about other Git branches or
# tags. We fix this by deleting and re-cloning the full repository.
#rm -rf "${DART_CLONE_DIR}"
cd "${DART_CLONE_DIR}"
rm -rf *
git clone "https://github.com/${TRAVIS_REPO_SLUG}.git" ${DART_CLONE_DIR}

# TODO: For test. Should be removed before merging
git -C ${DART_CLONE_DIR} checkout auto_docs

WORK_DIR="${HOME}/dart_docs"
mkdir -p ${WORK_DIR}
cd ${WORK_DIR}

DART_DOCS_BUILD_DIR="${WORK_DIR}/build/"
mkdir -p ${DART_DOCS_BUILD_DIR}
cd ${DART_DOCS_BUILD_DIR}

# Organize into "${WORK_DIR}/docs" directory
DART_DOCS_OUTPUT_DIR="${WORK_DIR}/docs/"
mkdir -p ${DART_DOCS_OUTPUT_DIR}

# Initialize list of API versions
cat <<EOF > ${DART_DOCS_OUTPUT_DIR}/README.md
# DART API Documentation
EOF

while read version; do
  if [[ ${version} == DART* ]]; then
    echo "### ${version}" >> ${DART_DOCS_OUTPUT_DIR}/README.md
    continue
  fi

  # Add entry to list of API versions
  echo "* [${version}](https://dartsim.github.io/dart/${version}/)" >> ${DART_DOCS_OUTPUT_DIR}/README.md

  # Build documentation
  git -C ${DART_CLONE_DIR} checkout ${version}
  rm -rf *
  cmake ${DART_CLONE_DIR}
  make docs

  mv doxygen/html ${DART_DOCS_OUTPUT_DIR}/${version}
done < ${DART_CLONE_DIR}/.ci/docs_versions.txt
