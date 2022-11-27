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
    rm -rf /var/lib/apt/lists/*
    
RUN apt-get update && \
    apt-get install -q -y --no-install-recommends libopenscenegraph-dev && \
    rm -rf /var/lib/apt/lists/*
    
RUN apt-get update && \
    apt-get install -q -y --no-install-recommends libnlopt-cxx-dev && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get install -q -y --no-install-recommends coinor-libipopt-dev && \
    rm -rf /var/lib/apt/lists/*
    
RUN apt-get update && \
    apt-get install -q -y --no-install-recommends libbullet-dev && \
    rm -rf /var/lib/apt/lists/*
    
RUN apt-get update && \
    apt-get install -q -y --no-install-recommends libode-dev && \
    rm -rf /var/lib/apt/lists/*
 
RUN apt-get update && \
    apt-get install -q -y --no-install-recommends liboctomap-dev && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get install -q -y --no-install-recommends libtinyxml2-dev liburdfdom-dev && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get install -q -y --no-install-recommends libxi-dev libxmu-dev freeglut3-dev && \
    rm -rf /var/lib/apt/lists/*
    
RUN apt-get update && \
    apt-get install -q -y --no-install-recommends libopenscenegraph-dev && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get install -q -y --no-install-recommends doxygen && \
    rm -rf /var/lib/apt/lists/*

################################################################################################################################
#
# The following blocks are the dependencies that are not listed in the installation instructions.
#
################################################################################################################################

RUN apt-get update && \
    apt-get install -q -y --no-install-recommends libfmt-dev && \
    rm -rf /var/lib/apt/lists/*
    
RUN apt-get update && \
    apt-get install -q -y --no-install-recommends curl wget && \
    rm -rf /var/lib/apt/lists/*
