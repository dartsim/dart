#!/usr/bin/env bash

set -ex

WORK_DIR="${HOME}/dart_docs"
DART_DIR="${TRAVIS_BUILD_DIR}"

mkdir -p ${WORK_DIR}
cd ${WORK_DIR}

# For branch builds, Travis only clones that branch with a fixed depth of 50
# commits. This means that the clone knows nothing about other Git branches or
# tags. We fix this by deleting and re-cloning the full repository.
rm -rf "${DART_DIR}"
git clone "https://github.com/${TRAVIS_REPO_SLUG}.git" ${DART_DIR}
git checkout auto_docs # TODO: For test. Should be removed

# Organize into "${HOME}/dart_docs" directory
DART_DOCS_DIR="${HOME}/dart_docs"
mkdir -p ${DART_DOCS_DIR}

# Initialize list of API versions
cat <<EOF > ${DART_DOCS_DIR}/README.md
# DART API Documentation
EOF

mkdir build_docs
cd build_docs

while read version; do
  if [[ ${version} == DART* ]]; then
    echo "### ${version}" >> ${DART_DOCS_DIR}/README.md
  fi

  # Add entry to list of API versions
  echo "* [${version}](https://dartsim.github.io/dart/${version}/)" >> ${DART_DOCS_DIR}/README.md

  # Build documentation
  git -C ${DART_DIR} checkout ${version}
  rm -rf *
  cmake ${DART_DIR}
  make docs
  mv doxygen/html ${DART_DOCS_DIR}/${version}
done < ${DART_DIR}/.ci/docs_versions.txt
