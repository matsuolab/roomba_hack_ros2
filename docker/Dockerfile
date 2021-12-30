ARG nvidia_cudagl_version=11.1.1-devel-ubuntu18.04
ARG ros_distro=eloquent

FROM nvidia/cudagl:${nvidia_cudagl_version}

ENV DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

RUN apt-get update -y && \
    apt-get install -y --no-install-recommends \
        wget \
        curl \
        git \
        locales \
        gnupg \
        software-properties-common \
        ca-certificates \
        lsb-release && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US.UTF-8 && \
    update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8
ENV LANG=en_US.UTF-8

# ROS2
ARG ros_distro
ENV ROS_DISTRO=${ros_distro}
### Setup source.list
RUN add-apt-repository universe
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt-get update -y && \
    apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-desktop && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*
###  Dependencies for building packages
RUN apt-get update -y && \
    apt-get install -y --no-install-recommends \
        python3-pip \
        python3-rosdep \
        python3-rosinstall \
        python3-rosinstall-generator \
        python3-wstool \
        python3-catkin-tools \
        libpython3-dev \
        python3-argcomplete \
        python3-colcon-common-extensions \
        build-essential && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*
# RUN rosdep init && \
#     rosdep update

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-joint-state-publisher \
        ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-joy \
        ros-${ROS_DISTRO}-joy-teleop \
        ros-${ROS_DISTRO}-diagnostic-updater && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# external ROS setting
WORKDIR /root
RUN mkdir -p external_ros2_ws/src && \
    cd external_ros2_ws/src && \
    git clone https://github.com/autonomylab/create_robot.git -b foxy && \
    git clone https://github.com/AutonomyLab/libcreate.git && \
    cd /root/external_ros2_ws && \
    bash -c "source /opt/ros/eloquent/setup.bash; rosdep init; rosdep update" && \
    apt-get update && \
    bash -c "source /opt/ros/eloquent/setup.bash; rosdep install --from-paths src -i -r -y" && \
    bash -c "source /opt/ros/eloquent/setup.bash; colcon build --symlink-install"

RUN cd /root/external_ros2_ws/src && \
    git clone https://github.com/youtalk/rplidar_ros -b eloquent && \
    cd /root/external_ros2_ws && \
    bash -c "source /opt/ros/eloquent/setup.bash; colcon build --symlink-install"

WORKDIR /root/

COPY ./docker/scripts/. /root/scripts/
RUN chmod -R +x /root/scripts/*

RUN echo "ROOMBA_HOSTNAME=\"TMP_HOSTNAME\"" >> /root/.bashrc
RUN echo "source /root/scripts/initialize-bash-shell.sh" > /root/.bashrc

ENTRYPOINT []

CMD ["/bin/bash"]