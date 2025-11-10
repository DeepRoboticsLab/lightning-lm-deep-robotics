FROM ros:humble-perception

LABEL maintainer="pengfei.guo2001@gmail.com"
LABEL description="A docker for lightning-lm"

# 环境变量
ENV ROS_WS=/root/slam_ws 

RUN printf "deb https://mirrors.ustc.edu.cn/ubuntu/ jammy main restricted universe multiverse\n\
deb-src https://mirrors.ustc.edu.cn/ubuntu/ jammy main restricted universe multiverse\n\
deb https://mirrors.ustc.edu.cn/ubuntu/ jammy-updates main restricted universe multiverse\n\
deb-src https://mirrors.ustc.edu.cn/ubuntu/ jammy-updates main restricted universe multiverse\n\
deb https://mirrors.ustc.edu.cn/ubuntu/ jammy-backports main restricted universe multiverse\n\
deb-src https://mirrors.ustc.edu.cn/ubuntu/ jammy-backports main restricted universe multiverse\n\
deb https://mirrors.ustc.edu.cn/ubuntu/ jammy-security main restricted universe multiverse\n\
deb-src https://mirrors.ustc.edu.cn/ubuntu/ jammy-security main restricted universe multiverse\n\
deb http://archive.canonical.com/ubuntu/ jammy partner\n\
deb-src http://archive.canonical.com/ubuntu/ jammy partner\n" > /etc/apt/sources.list && \
    \
    # 现在使用新源更新
    apt-get update && \
    rm -rf /var/lib/apt/lists/*

RUN apt update && apt install -y \
    cmake \
    make \
    g++ \
    gcc \
    libssl-dev \
    libboost-all-dev \
    libboost-dev \
    libeigen3-dev \
    libpcl-dev \
    libgoogle-glog-dev \
    libopencv-dev \
    libyaml-cpp-dev \
    libgflags-dev \
    pcl-tools \
    liboctomap-dev \
    libassimp-dev \
    liburdfdom-dev \
    libepoxy-dev \
    doxygen \
    python3-pip \
    python3-wheel \
    python3-setuptools \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-octomap \
    ros-${ROS_DISTRO}-pinocchio \
    zip \
    && rm -rf /var/lib/apt/lists/*

# 安装hpp-fcl库
WORKDIR /tmp
RUN git clone --recurse-submodules https://github.com/leggedrobotics/hpp-fcl.git && \
    cd hpp-fcl && \
    mkdir build && cd build && \
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DBUILD_WITH_COLLISION_SUPPORT=ON \
        -DBUILD_PYTHON_INTERFACE=OFF && \
    make && \
    make install && \
    cd / && \
    rm -rf /tmp/hpp-fcl

# 安装Pangolin库
WORKDIR /tmp

RUN git clone --recurse-submodules https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && \
    mkdir build && cd build && \
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DBUILD_WITH_COLLISION_SUPPORT=ON \
        -DBUILD_PYTHON_INTERFACE=OFF && \
    make && \
    make install && \
    cd / && \
    rm -rf /tmp/Pangolin

# 创建 ROS 工作空间
WORKDIR ${ROS_WS}

# 一键式构建命令
RUN /bin/bash -c "mkdir -p src && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    git clone https://github.com/gaoxiang12/lightning-lm.git src/lightning-lm&& \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    # 将source命令写入bashrc（持久化配置）
    echo 'source /${ROS_WS}/install/setup.bash' >> ~/.bashrc && \
    echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> ~/.bashrc"
