# ROS 2 Humble + Gazebo Fortress + ros2_control + MoveIt 2 dev image
FROM osrf/ros:humble-desktop

ARG USERNAME=dev
ARG UID=1000
ARG GID=1000
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-ros-gz \
      ros-humble-gz-ros2-control \
      ros-humble-ros2-control \
      ros-humble-ros2-controllers \
      ros-humble-moveit \
      ros-humble-xacro \
      ros-humble-joint-state-publisher-gui \
      ros-humble-cv-bridge \
      ros-humble-vision-msgs \
      ros-humble-tf2-tools \
      python3-opencv \
      python3-colcon-common-extensions \
      mesa-utils \
      sudo \
    && rm -rf /var/lib/apt/lists/*

# Non-root user matching the host UID/GID so files in the mounted workspace stay yours
RUN groupadd --gid ${GID} ${USERNAME} \
 && useradd --uid ${UID} --gid ${GID} -m -s /bin/bash ${USERNAME} \
 && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME}

USER ${USERNAME}
RUN rosdep update \
 && echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc \
 && echo '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> ~/.bashrc \
 && echo 'alias cb="cd /ws && colcon build --symlink-install && source install/setup.bash"' >> ~/.bashrc

WORKDIR /ws
CMD ["sleep", "infinity"]
