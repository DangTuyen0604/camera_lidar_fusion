FROM ros:jazzy-ros-base-noble AS workspace-base

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions \
      python3-pip \
      python3-venv \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY . /workspace

RUN source /opt/ros/jazzy/setup.bash \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y \
    && python3 -m venv --system-site-packages /opt/fusion-venv \
    && /opt/fusion-venv/bin/pip install --no-cache-dir -r docker/requirements-runtime.txt \
    && colcon build --symlink-install

ENV PATH="/opt/fusion-venv/bin:${PATH}"
COPY docker/entrypoint.sh /ros_entrypoint_fusion.sh
ENTRYPOINT ["/ros_entrypoint_fusion.sh"]
CMD ["ros2", "launch", "fusion_bringup", "projection_demo.launch.py", "use_rviz:=false"]

FROM workspace-base AS runtime-cpu

ENV ONNXRUNTIME_PROVIDER=CPUExecutionProvider
CMD ["ros2", "launch", "navigation_bringup", "warehouse_full_demo.launch.py", "gui:=false"]

FROM workspace-base AS training

RUN /opt/fusion-venv/bin/pip install --no-cache-dir -r docker/requirements-training.txt
CMD ["bash"]
