ARG ROS_DISTRO=rolling
FROM ros:$ROS_DISTRO-ros-base as ci

LABEL maintainer="Evan Palmer"
LABEL maintainer-email="evanp922@gmail.com"

WORKDIR /root/ws_alpha

COPY . src/alpha

# Install the core apt packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    wget \
    curl \
    sudo \
    clang \
    clang-format-14 \
    clang-tidy \
    clang-tools \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

# Install the alpha driver dependencies
RUN apt-get update \
    && rosdep update \
    && DEBIAN_FRONTEND=noninteractive \
    rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} --as-root=apt:false \
    && rm -rf src \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

FROM ci as source

ENV ROS_UNDERLAY /root/ws_alpha/install
WORKDIR $ROS_UNDERLAY/..

COPY . src/alpha

RUN apt-get update \
    && rosdep update \
    && DEBIAN_FRONTEND=noninteractive \
    rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} --as-root=apt:false \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" \
    && colcon build \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    --ament-cmake-args -DCMAKE_BUILD_TYPE=Release \
    --event-handlers desktop_notification- status- \
    # Update /ros_entrypoint.sh to source the new workspace
    && sed -i "s#/opt/ros/\$ROS_DISTRO/setup.bash#$ROS_UNDERLAY/setup.sh#g" /ros_entrypoint.sh
