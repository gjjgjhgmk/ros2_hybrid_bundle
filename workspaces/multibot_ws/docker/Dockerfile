# UR Move 多阶段构建 Dockerfile
# 第一阶段：基础环境
FROM ros:jazzy-ros-base-noble AS base

# 设置环境变量
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# 配置阿里源
RUN sed -i 's/archive.ubuntu.com/mirrors.aliyun.com/g' /etc/apt/sources.list.d/ubuntu.sources && \
    sed -i 's/security.ubuntu.com/mirrors.aliyun.com/g' /etc/apt/sources.list.d/ubuntu.sources

# 先安装必要工具并添加ROS2阿里源GPG密钥
RUN apt-get update && apt-get install -y curl gnupg && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.aliyun.com/ros2/ubuntu noble main" > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && apt-get install -y --no-install-recommends \
    # 安装基础编译工具
    build-essential \
    cmake \
    wget \
    pkg-config \
    python3-colcon-common-extensions \
    python3-rosdep \
    python-is-python3 \
    python3-vcstool \
    # 安装 MoveIt 核心包
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-moveit-ros-planning-interface \
    ros-${ROS_DISTRO}-moveit-configs-utils \
    ros-${ROS_DISTRO}-trajectory-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-control-msgs \
    ros-${ROS_DISTRO}-eigen3-cmake-module \
    ros-${ROS_DISTRO}-moveit-core \
    ros-${ROS_DISTRO}-moveit-planners \
    ros-${ROS_DISTRO}-moveit-ros-planning \
    ros-${ROS_DISTRO}-moveit-ros-move-group \
    ros-${ROS_DISTRO}-moveit-ros-visualization \
    ros-${ROS_DISTRO}-moveit-ros-warehouse \
    ros-${ROS_DISTRO}-moveit-simple-controller-manager \
    ros-${ROS_DISTRO}-moveit-kinematics \
    ros-${ROS_DISTRO}-pilz-industrial-motion-planner \
    ros-${ROS_DISTRO}-py-binding-tools \
    ros-${ROS_DISTRO}-trac-ik-kinematics-plugin \
    ros-${ROS_DISTRO}-warehouse-ros-sqlite \
    # 安装 ros2_control 相关
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-joint-state-broadcaster \
    ros-${ROS_DISTRO}-joint-trajectory-controller \
    ros-${ROS_DISTRO}-position-controllers \
    ros-${ROS_DISTRO}-velocity-controllers \
    ros-${ROS_DISTRO}-robot-state-publisher \
    # 安装 UR 机器人相关
    ros-${ROS_DISTRO}-ur \
    ros-${ROS_DISTRO}-ur-robot-driver \
    ros-${ROS_DISTRO}-ur-description \
    ros-${ROS_DISTRO}-ur-controllers \
    ros-${ROS_DISTRO}-ur-moveit-config \
    ros-${ROS_DISTRO}-ur-msgs \
    ros-${ROS_DISTRO}-ur-client-library \
    # 安装可视化工具
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rviz-common \
    ros-${ROS_DISTRO}-rviz-default-plugins \
    ros-${ROS_DISTRO}-rviz-visual-tools \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-xacro \
    # 安装开发工具
    pybind11-dev \
    nlohmann-json3-dev \
    libyaml-cpp-dev \
    libzmq3-dev \
    cppzmq-dev \
    htop \
    tree \
    curl \
    net-tools \
    iputils-ping \
    python3-pip \
    python3-dev \
    python3-numpy \
    python3-yaml \
    socat && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# 配置pip阿里源并安装Python包
RUN pip3 config set global.index-url https://mirrors.aliyun.com/pypi/simple/ && \
    pip3 install \
    pyzmq \
    pybind11 \
    nlohmann_json \
    numpy \
    pyyaml \
    py-trees>=2.2.0 \
    pydot>=1.4.2 \
    matplotlib>=3.7.0 \
    tqdm>=4.65.0 \
    --break-system-packages

# 创建工作空间目录（但不复制源码）
WORKDIR /workspace

# 设置ROS环境
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc && \
    echo "export AMENT_PREFIX_PATH=/opt/ros/${ROS_DISTRO}" >> ~/.bashrc && \
    echo "alias cb='cd /workspace && colcon build --symlink-install'" >> ~/.bashrc && \
    echo "alias sb='source /workspace/install/setup.bash 2>/dev/null || true'" >> ~/.bashrc && \
    echo "alias ws='cd /workspace'" >> ~/.bashrc

# 第二阶段：开发环境（基于基础环境）
FROM base AS dev

# 设置环境变量
ENV ROS_DISTRO=jazzy

# 用户相关构建参数（可通过 --build-arg 覆盖）
ARG USER_NAME=yw
ARG USER_UID=1000
ARG USER_GID=1000

ENV USER_NAME=${USER_NAME}
ENV USER_UID=${USER_UID}
ENV USER_GID=${USER_GID}

USER root
# 创建非root用户（如果组/用户已存在则重命名或使用现有的）
RUN EXISTING_GROUP=$(getent group ${USER_GID} 2>/dev/null | cut -d: -f1) && \
    if [ -z "${EXISTING_GROUP}" ]; then \
    groupadd -r ${USER_NAME} -g ${USER_GID}; \
    elif [ "${EXISTING_GROUP}" != "${USER_NAME}" ]; then \
    groupmod -n ${USER_NAME} ${EXISTING_GROUP}; \
    fi && \
    EXISTING_USER=$(getent passwd ${USER_UID} 2>/dev/null | cut -d: -f1) && \
    if [ -z "${EXISTING_USER}" ]; then \
    useradd -r -u ${USER_UID} -g ${USER_NAME} -m -s /bin/bash ${USER_NAME}; \
    elif [ "${EXISTING_USER}" != "${USER_NAME}" ]; then \
    usermod -l ${USER_NAME} -d /home/${USER_NAME} -m ${EXISTING_USER}; \
    fi && \
    if [ ! -d "/home/${USER_NAME}" ]; then \
    mkdir -p /home/${USER_NAME} && \
    chown ${USER_NAME}:${USER_NAME} /home/${USER_NAME}; \
    fi && \
    mkdir -p /home/${USER_NAME}/workspace/src && \
    chown -R ${USER_NAME}:${USER_NAME} /home/${USER_NAME}

# 创建工作空间
WORKDIR /workspace

# 复制项目源码
RUN rm -rf build install log src && mkdir -p /workspace/src
COPY . /workspace/src/

# 复制并设置entrypoint脚本
COPY docker/docker-entrypoint.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/docker-entrypoint.sh

# 初始化rosdep并安装依赖
# RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
#     rosdep update && \
#     rosdep install --ignore-src --from-paths /workspace/src -y || true"

RUN chown -R ${USER_NAME}:${USER_NAME} /workspace

USER ${USER_NAME}
# 编译工作空间
RUN bash -c "cd /workspace && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rm -rf build install log && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 8"

# 更新ROS环境（添加工作空间安装路径）
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \ 
    echo "source /workspace/install/setup.bash" >> ~/.bashrc && \
    echo "export AMENT_PREFIX_PATH=/opt/ros/${ROS_DISTRO}:/workspace/install" >> ~/.bashrc && \
    echo "alias sb='source /workspace/install/setup.bash'" >> ~/.bashrc

# 设置工作目录
WORKDIR /workspace

# 设置entrypoint和默认命令
ENTRYPOINT ["/usr/local/bin/docker-entrypoint.sh"]
CMD ["/bin/bash"]
