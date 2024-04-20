# Use an official ROS environment as a parent image
FROM osrf/ros:humble-desktop

# Set the default shell to bash
SHELL ["/bin/bash", "-c"]

# Set the working directory
WORKDIR /workspace

# Set a non-interactive shell to avoid stuck prompts during build
ARG DEBIAN_FRONTEND=noninteractive

# Install essential and additional C++ and Python packages
RUN apt-get update && apt-get install -y \
    build-essential \
    bash \
    cmake \
    git \
    vim \
    wget \
    curl \
    lsb-release \
    sudo \
    gnupg2 \
    net-tools \
    x11-apps \
    qtbase5-dev \
    qtchooser \
    qt5-qmake \
    qtbase5-dev-tools \
    libxcb-xinerama0 \
    libx11-xcb1 \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-render-util0 \
    libxcb-xkb1 \
    libxkbcommon-x11-0 \
    libxcb-randr0 \
    libxcb-render0 \
    libxcb-shape0 \
    libxcb-sync1 \
    libxcb-xfixes0 \
    libxcb-xinput0 \
    libxcb1 \
    libxrender1 \
    libxi6 \
    python3-pip \
    clang-format \
    gdb \
    libpcl-dev \
    libopencv-dev \
    python3-numpy \
    python3-matplotlib \
    libtinyxml2-dev \
    nlohmann-json3-dev \
    libyaml-cpp-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Install cmake-format via pip
RUN pip3 install cmake-format

# Add a user for the development environment
ARG USERNAME=devuser
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Setup ROS2 environment in the bashrc for interactive bash shells
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc

# Setup GUI forwarding with the correct DISPLAY setting
RUN echo "export DISPLAY=:1" >> /home/$USERNAME/.bashrc

# Expose ports (if needed)
# EXPOSE 6000

# Command to run on container start
CMD ["/bin/bash"]
