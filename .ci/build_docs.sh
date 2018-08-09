#!/usr/bin/env bash

set -ex

DART_DIR="${HOME}/dart_docs"
mkdir -p "${DART_DIR}"
cd "${DART_DIR}"

# For branch builds, Travis only clones that branch with a fixed depth of 50
# commits. This means that the clone knows nothing about other Git branches or
# tags. We fix this by deleting and re-cloning the full repository.
rm -rf ${DART_DIR}
git clone "https://github.com/${TRAVIS_REPO_SLUG}.git" ${DART_DIR}

# Organize into "gh-pages" directory
mkdir -p ${TRAVIS_BUILD_DIR}/gh-pages

# Initialize list of API versions
cat <<EOF > ${TRAVIS_BUILD_DIR}/gh-pages/README.md
# DART API Documentation
EOF

mkdir build_docs
cd build_docs

while read version; do
  # Add entry to list of API versions
  echo "* [${version}](https://personalrobotics.github.io/aikido/${version}/)" >> ${TRAVIS_BUILD_DIR}/gh-pages/README.md

  # Build documentation
  git -C ${DART_DIR} checkout ${version}
  rm -rf *
  cmake ${DART_DIR}
  make docs
  mv doxygen/html ${TRAVIS_BUILD_DIR}/gh-pages/${version}
done < ${TRAVIS_BUILD_DIR}/.ci/docs_versions.txt
