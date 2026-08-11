FROM nvidia/cuda:12.2.0-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,video,utility

# "--no-install-recommends" avoids downloading unnecessary extra packages, while "rm -rf /var/lib/apt/lists/*" clears the "apt" cache after installation
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    iproute2 \
    tcpdump \
    ethtool \
    pciutils \
    wget \
    meson \
    ninja-build \
    pkg-config \
    libnuma-dev \
    python3-pyelftools \
    ca-certificates \
    ffmpeg \
    && rm -rf /var/lib/apt/lists/*

# "DPDK" ( "Stable" version "22.11 LTS" )
RUN wget http://fast.dpdk.org/rel/dpdk-22.11.4.tar.xz && \
    tar -xf dpdk-22.11.4.tar.xz && \
    cd dpdk-stable-22.11.4 && \
    meson setup build && \
    cd build && \
    ninja && \
    ninja install && \
    ldconfig && \
    cd ../.. && rm -rf dpdk-22.11.4*

WORKDIR /app

CMD ["/bin/bash"]