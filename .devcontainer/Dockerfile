FROM ubuntu:20.04

RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get install -q -y --no-install-recommends build-essential cmake pkg-config git && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get install -q -y --no-install-recommends libeigen3-dev libassimp-dev libccd-dev libfcl-dev libboost-regex-dev libboost-system-dev && \
    && rm -rf /var/lib/apt/lists/*
    
RUN apt-get update && \
    apt-get install -q -y --no-install-recommends libopenscenegraph-dev \
    && rm -rf /var/lib/apt/lists/*
