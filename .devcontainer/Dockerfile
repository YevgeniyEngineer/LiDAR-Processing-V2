# Use an official ROS environment as a parent image
FROM osrf/ros:humble-desktop

# Set the default shell to bash
SHELL ["/bin/bash", "-c"]

# Set the working directory
WORKDIR /workspace

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

# Add a user for the development environment
ARG USERNAME=devuser
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Setup ROS2 environment in the bashrc for interactive bash shells
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc

USER $USERNAME

# Setup GUI forwarding
RUN echo "export DISPLAY=host.docker.internal:0.0" >> ~/.bashrc

# Expose ports (if needed)
# EXPOSE 6000

# Command to run on container start
CMD [ "/bin/bash" ]
