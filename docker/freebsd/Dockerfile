FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ca-certificates \
        cloud-image-utils \
        curl \
        openssh-client \
        python3 \
        qemu-system-x86 \
        qemu-utils \
        xz-utils \
    && rm -rf /var/lib/apt/lists/*

COPY entrypoint.py /usr/local/bin/freebsd_vm_entrypoint.py

ENTRYPOINT ["python3", "/usr/local/bin/freebsd_vm_entrypoint.py"]
