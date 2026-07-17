FROM kathara/base

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    python3-pip \
    python3-opencv \
    libgl1 \
    libglib2.0-0 \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libgomp1 \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir \
    --break-system-packages \
    open3d \
    "numpy<2"

WORKDIR /