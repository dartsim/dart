ARG TARGETPLATFORM=linux/amd64
FROM --platform=$TARGETPLATFORM ubuntu:bionic

ARG DEBIAN_FRONTEND=noninteractive

# Use /bin/bash to use pushd/popd
SHELL ["/bin/bash", "-c"]

# Update apt-get
RUN apt-get update -qq

# ==============================================================================
# Build tools
# ==============================================================================
RUN apt-get install -y --no-install-recommends \
    build-essential \
    clang \
    clang-format-6.0 \
    clang-format-9 \
    cmake \
    curl \
    doxygen \
    git \
    lcov \
    lsb-release \
    pkg-config \
    software-properties-common \
    valgrind

# ==============================================================================
# DART required dependencies
# ==============================================================================
RUN apt-get install -y --no-install-recommends \
    libassimp-dev \
    libboost-filesystem-dev \
    libboost-system-dev \
    libccd-dev \
    libeigen3-dev \
    libfcl-dev \
    libfmt-dev

# DART required dependencies for building API documentation of DART < 6.10
RUN apt-get install -y --no-install-recommends \
    libboost-regex-dev

# ==============================================================================
# DART optional dependencies
# ==============================================================================

RUN apt-get install -y --no-install-recommends \
    coinor-libipopt-dev \
    freeglut3-dev \
    libxi-dev \
    libxmu-dev \
    libbullet-dev \
    liblz4-dev \
    libflann-dev \
    libtinyxml2-dev \
    liburdfdom-dev \
    liburdfdom-headers-dev \
    libopenscenegraph-dev \
    libnlopt-dev \
    liboctomap-dev \
    libode-dev

# pagmo2
RUN apt-get install -y --no-install-recommends \
    coinor-libipopt-dev \
    libboost-serialization-dev \
    libeigen3-dev \
    libtbb-dev
RUN git clone https://github.com/esa/pagmo2.git -b 'v2.17.0' --single-branch --depth 1 \
    && mkdir pagmo2/build \
    && pushd pagmo2/build \
    && cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DPAGMO_WITH_EIGEN3=ON \
    -DPAGMO_WITH_NLOPT=OFF \
    -DPAGMO_WITH_IPOPT=ON \
    -DPAGMO_BUILD_TESTS=OFF \
    -DPAGMO_BUILD_BENCHMARKS=OFF \
    -DPAGMO_BUILD_TUTORIALS=OFF \
    && make -j$(nproc) \
    && make install \
    && popd \
    && rm -rf pagmo2

# ==============================================================================
# Python binding dependencies
# ==============================================================================

RUN apt-get install -y --no-install-recommends \
    python3-dev \
    python3-numpy \
    python3-pip \
    python3-setuptools

RUN git clone https://github.com/pybind/pybind11 -b 'v2.6.2' --single-branch --depth 1 \
    && mkdir -p pybind11/build \
    && pushd pybind11/build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release -DPYBIND11_TEST=OFF \
    && make -j$(nproc) \
    && make install \
    && popd \
    && rm -rf pybind11

RUN pip3 install pytest -U

# ==============================================================================
# Clean up
# ==============================================================================

RUN rm -rf /var/lib/apt/lists/*
