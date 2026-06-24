# 使用 ROS Noetic 作为底座
FROM osrf/ros:noetic-desktop

# 安装基础编译工具（保持原始 apt 源，不动 ROS 源）
RUN apt-get update && apt-get install -y \
    python3-pip \
    git \
    build-essential \
    nano \
    && rm -rf /var/lib/apt/lists/*

# pip 用清华源加速
RUN pip3 install --upgrade pip -i https://pypi.tuna.tsinghua.edu.cn/simple
RUN pip3 install -i https://pypi.tuna.tsinghua.edu.cn/simple \
    catkin-tools \
    numpy pandas scipy matplotlib optuna \
    torch torchvision \
    rospkg bagpy

# 设置默认工作目录
WORKDIR /workspace

# 自动 source ROS 环境变量
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
