# TARP bridge container. Runs rosbridge + the replay node + the static web
# server. Shares the SLAM container's DDS graph via network_mode: host.
#
# Base image matches lidar_slam so the big dustynv L4T layer is already on disk
# on the Jetson — the TARP image is effectively a delta on top.
#
# The dustynv base has Humble built from source in /opt/ros/humble/install and
# Ubuntu 20.04 (focal) as the OS. The OSRF apt repo does NOT ship Humble debs
# for focal (Humble only targets Jammy), so we build rosbridge_suite from source
# into an overlay workspace at /ws/install and stack it on the base.
#
# topic_tools is intentionally NOT built here. For v0 bring-up the replay node
# publishes directly on /tarp/points + /tarp/odom. When real hardware lands and
# we need `topic_tools relay` to remap sensor topics, add it to the source
# workspace below (clone ros2/topic_tools, append to --packages-up-to).

FROM dustynv/ros:humble-ros-base-l4t-r35.4.1

ARG DEBIAN_FRONTEND=noninteractive

# ---------------------------------------------------------------------------
# ROS2 apt signing key rotation (2025-05). The dustynv base predates it, so
# `apt-get update` rejects packages.ros.org until we drop the current key in.
# We don't actually install any ros-humble-* debs (they don't exist for focal
# anyway), but having a clean apt state makes the Python runtime-dep install
# below work without warnings.
# ---------------------------------------------------------------------------
RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        | gpg --no-tty --batch --dearmor \
        > /usr/share/keyrings/ros-archive-keyring.gpg

# ---------------------------------------------------------------------------
# Python runtime deps for rosbridge_server + rosbridge_library.
#   - bson, ujson, tornado: rosbridge_server transport + JSON
#   - pil: rosbridge_library image conversion
#   - numpy: replay node + (eventually) adequacy prototypes that get mounted in
#   - cbor2: rosbridge_server binary transport. Not in focal apt (jammy-only),
#     so install via pip as root into system site-packages. rosdep on focal
#     resolves cbor2 to `#pip cbor2` for the same reason.
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        git \
        python3-bson \
        python3-ujson \
        python3-pil \
        python3-tornado \
        python3-numpy \
        python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir cbor2

# ---------------------------------------------------------------------------
# Build rosbridge_suite from source into an overlay workspace. The humble
# branch (v2.0.6) uses python3-bson for the bson encoder (installed above) and
# doesn't need the rosidl_python source checkout that newer branches do.
# --merge-install flattens the overlay into a single share/lib/include triple.
# BUILD_TESTING=OFF drops ament_cmake_mypy and control_msgs test-deps that
# aren't available on focal; we're not running tests in this image.
#
# Branch is pinned deliberately — avoid the rolling `ros2` branch upstream.
# ---------------------------------------------------------------------------
ARG ROSBRIDGE_REF=humble

RUN mkdir -p /ws/src && cd /ws/src \
 && git clone --depth 1 --branch ${ROSBRIDGE_REF} https://github.com/RobotWebTools/rosbridge_suite.git

RUN bash -c "source /opt/ros/humble/install/setup.bash \
 && cd /ws \
 && colcon build \
      --packages-up-to rosbridge_suite \
      --merge-install \
      --cmake-args -DBUILD_TESTING=OFF \
 && rm -rf build log"

# ---------------------------------------------------------------------------
# Non-root user so bind-mounted edits from the host don't come back root-owned.
# UID/GID overridable at build time to match the host user:
#   docker compose build --build-arg UID=$(id -u) --build-arg GID=$(id -g)
# (compose.yml already wires these through.)
# ---------------------------------------------------------------------------
ARG UID=1000
ARG GID=1000
RUN groupadd -g ${GID} tarp 2>/dev/null || true \
 && useradd  -u ${UID} -g ${GID} -m -s /bin/bash tarp 2>/dev/null || true

COPY --chown=root:root docker/entrypoint.sh /usr/local/bin/tarp-entrypoint
RUN chmod +x /usr/local/bin/tarp-entrypoint

WORKDIR /tarp
USER tarp

ENTRYPOINT ["/usr/local/bin/tarp-entrypoint"]
CMD ["bash"]
