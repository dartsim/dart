FROM ubuntu:questing

ARG DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

# Build tools
RUN apt-get update -qq && apt-get install -y --no-install-recommends \
    build-essential \
    clang \
    clang-format-14 \
    cmake \
    curl \
    doxygen \
    git \
    lcov \
    lsb-release \
    ninja-build \
    pkg-config \
    software-properties-common \
    valgrind

# DART required dependencies
RUN apt-get update -qq && apt-get install -y --no-install-recommends \
    libassimp-dev \
    libeigen3-dev \
    libfcl-dev \
    libfmt-dev

# DART optional dependencies
RUN apt-get update -qq && apt-get install -y --no-install-recommends \
    libtinyxml2-dev \
    liburdfdom-dev \
    liburdfdom-headers-dev \
    libopenscenegraph-dev \
    liboctomap-dev \
    libode-dev \
    libimgui-dev \
    libspdlog-dev

# Python binding dependencies
RUN apt-get update -qq && apt-get install -y --no-install-recommends \
    libpython3-dev \
    python3 \
    python3-dev \
    python3-numpy \
    python3-pip \
    python3-pytest \
    python3-setuptools && \
    PIP_BREAK_SYSTEM_PACKAGES=1 pip3 install --no-cache-dir --ignore-installed nanobind pytest -U && \
    rm -rf /var/lib/apt/lists/*
