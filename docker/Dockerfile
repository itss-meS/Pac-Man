# Multi-stage Build
FROM ubuntu:22.04 AS builder
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git python3 python3-pip python3-venv \
    libeigen3-dev python3-pybind11-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY . .
RUN cd cpp_core && cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j$(nproc)
RUN pip install --no-cache-dir -e ./python

# Runtime Stage
FROM nvidia/opengl:1.2-glvnd-runtime-ubuntu22.04 AS runtime
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 python3-pip libgl1-mesa-glx libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /ws/cpp_core/build/pacbot_core*.so /usr/local/lib/python3.10/dist-packages/pacbot/core/
COPY --from=builder /ws/python /opt/pacbot/python
COPY --from=builder /ws/ros2_ws /opt/pacbot/ros2_ws

WORKDIR /opt/pacbot
ENV PYTHONPATH=/opt/pacbot/python:$PYTHONPATH

ENTRYPOINT ["/bin/bash"]
