FROM ubuntu:questing

ARG DEBIAN_FRONTEND=noninteractive
ARG PIXI_VERSION=0.59.0

SHELL ["/bin/bash", "-c"]

# Base utilities; pixi manages the toolchain and dependencies (see docs/onboarding/building.md)
RUN apt-get update -qq && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    git \
    tini && \
    rm -rf /var/lib/apt/lists/*

# Install pixi globally so the devcontainer uses the managed workflow
ENV PIXI_HOME="/opt/pixi" PATH="/opt/pixi/bin:${PATH}"
RUN curl -fsSL https://pixi.sh/install.sh | PIXI_VERSION=${PIXI_VERSION} PIXI_HOME=${PIXI_HOME} bash

COPY .devcontainer/pixi-auto-shell.sh /etc/profile.d/pixi-auto-shell.sh

WORKDIR /workspaces/dart

ENTRYPOINT ["/usr/bin/tini", "--"]
CMD ["bash", "-lc", "pixi shell || exec bash"]
