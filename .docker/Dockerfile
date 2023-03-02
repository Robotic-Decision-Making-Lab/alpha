FROM ros:humble as base

LABEL maintainer="Evan Palmer"
LABEL maintainer-email="evanp922@gmail.com"
LABEL license="MIT"

ENV DEBIAN_FRONTEND=noninteractive

# Install general apt dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    apt-utils \
    software-properties-common \
    git \
    wget \
    gnupg2 \
    lsb-release \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

# Install required ROS dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rviz2 \
    ros-dev-tools \
    ros-humble-moveit \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

FROM base as develop

RUN apt-get update && apt-get install -y --no-install-recommends \
    iputils-ping \
    net-tools \
    lsb-release \
    gdb \
    clang-format \
    python3-dev \
    python3-pip \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

# Install debugging/linting Python packages
RUN pip3 install \
    black \
    flake8 \
    isort \
    pydocstyle \
    mypy

# Configure a new non-root user
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/* \
    && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc \
    && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc

ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/$USERNAME/.bashrc
RUN echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${WORKSPACE}/" >> /home/$USERNAME/.bashrc
RUN echo "source /usr/share/gazebo/setup.sh" >> /home/$USERNAME/.bashrc
