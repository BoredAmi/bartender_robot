FROM osrf/ros:humble-desktop-full

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        git \
        mesa-utils \
        python3-colcon-common-extensions \
        python3-pip \
        python3-rosdep \
        ros-humble-gz-ros2-control \
        ros-humble-moveit \
        ros-humble-robotiq-controllers \
        ros-humble-robotiq-description \
        ros-humble-ros2-control \
        ros-humble-ros2-controllers \
        ros-humble-ur-description \
        ros-humble-ur-moveit-config \
        ros-humble-ur-simulation-gz \
    && rm -rf /var/lib/apt/lists/*

# Interactive shells automatically pick up an already-built workspace.
# The ROS image entrypoint sources /opt/ros/humble for every command.
RUN printf '%s\n' \
    'if [ -f /workspace/ros2_ws/install/setup.bash ]; then' \
    '  source /workspace/ros2_ws/install/setup.bash' \
    'fi' \
    > /etc/profile.d/bartender-workspace.sh \
    && printf '%s\n' \
    '' \
    '# Source the mounted bartender workspace when it has been built.' \
    '[ ! -f /workspace/ros2_ws/install/setup.bash ] || source /workspace/ros2_ws/install/setup.bash' \
    >> /etc/bash.bashrc

WORKDIR /workspace

CMD ["bash"]
