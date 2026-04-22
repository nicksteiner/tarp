# TARP bridge container. Runs rosbridge, the replay node, and the static web
# server. Stays on the SLAM container's DDS graph via --network host.
#
# Base image matches lidar_slam so the dustynv L4T layer is already on disk on
# the Jetson — no extra multi-GB pull. FAST-LIO2 / CUDA layers in that image are
# unused here but the cost is zero.

FROM dustynv/ros:humble-ros-base-l4t-r35.4.1

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-rosbridge-suite \
        ros-humble-topic-tools \
        python3-numpy \
    && rm -rf /var/lib/apt/lists/*

# Non-root user so bind-mounted edits from the host don't land as root-owned.
# UID/GID overridable at build time to match the host user.
ARG UID=1000
ARG GID=1000
RUN groupadd -g ${GID} tarp 2>/dev/null || true \
 && useradd  -u ${UID} -g ${GID} -m -s /bin/bash tarp 2>/dev/null || true

WORKDIR /tarp
USER tarp

# Sourced by the entrypoint. Not ENV because ROS_DISTRO lives in /opt/ros/humble/setup.bash.
COPY --chown=tarp:tarp docker/entrypoint.sh /usr/local/bin/tarp-entrypoint
ENTRYPOINT ["/usr/local/bin/tarp-entrypoint"]
CMD ["bash"]
