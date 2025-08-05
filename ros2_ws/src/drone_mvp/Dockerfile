# KAERTEI 2025 FAIO - Optimized Ubuntu Docker Image
FROM ubuntu:22.04

# Prevent interactive prompts during build
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV PYTHONUNBUFFERED=1

# Set locale
RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Install system dependencies
RUN apt-get update && apt-get install -y \
    curl \
    wget \
    git \
    python3 \
    python3-pip \
    python3-dev \
    python3-venv \
    build-essential \
    cmake \
    lsb-release \
    gnupg2 \
    software-properties-common \
    pkg-config \
    libeigen3-dev \
    libgeographic-dev \
    libboost-all-dev \
    libserial-dev \
    libyaml-cpp-dev \
    libopencv-dev \
    python3-opencv \
    python3-numpy \
    python3-scipy \
    python3-matplotlib \
    udev \
    usbutils \
    v4l-utils \
    can-utils \
    i2c-tools \
    python3-serial \
    python3-usb \
    && rm -rf /var/lib/apt/lists/*

# Add ROS 2 repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

# Install ROS 2 Humble
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Install GeographicLib datasets
RUN /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

# Install Python dependencies (lightweight for Docker)
RUN pip3 install --no-cache-dir \
    pymavlink>=2.4.37 \
    numpy \
    opencv-python-headless \
    ultralytics \
    pyserial \
    pyyaml \
    transforms3d

# Create workspace
WORKDIR /workspace/ros2_ws
RUN mkdir -p src build install log

# Copy source code properly
COPY drone_mvp/ src/drone_mvp/drone_mvp/
COPY config/ src/drone_mvp/config/
COPY launch/ src/drone_mvp/launch/
COPY msg/ src/drone_mvp/msg/
COPY srv/ src/drone_mvp/srv/
COPY package.xml src/drone_mvp/
COPY CMakeLists.txt src/drone_mvp/
COPY requirements.txt src/drone_mvp/
COPY *.sh ./
COPY test_docker.py ./

# Make scripts executable
RUN find . -name "*.sh" -exec chmod +x {} \;
RUN find . -name "*.py" -exec chmod +x {} \;

# Build workspace properly in Docker
# Skip build due to CMake target conflicts - use direct Python execution
# RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
#     cd /workspace/ros2_ws && \
#     rm -rf build install log && \
#     colcon build --packages-select drone_mvp --cmake-clean-first"

# Create environment setup for direct Python execution
RUN echo '#!/bin/bash' > /workspace/ros2_ws/setup_env.sh && \
    echo 'source /opt/ros/humble/setup.bash' >> /workspace/ros2_ws/setup_env.sh && \
    echo 'export ROS_DOMAIN_ID=0' >> /workspace/ros2_ws/setup_env.sh && \
    echo 'export PYTHONPATH=/workspace/ros2_ws/src/drone_mvp:/workspace/ros2_ws/src/drone_mvp/drone_mvp:$PYTHONPATH' >> /workspace/ros2_ws/setup_env.sh && \
    echo 'echo "🚁 KAERTEI 2025 Docker environment ready!"' >> /workspace/ros2_ws/setup_env.sh && \
    echo 'echo "📁 Direct Python execution mode"' >> /workspace/ros2_ws/setup_env.sh && \
    echo 'echo "🎯 Ready to run: ./run_direct.sh debug"' >> /workspace/ros2_ws/setup_env.sh && \
    chmod +x /workspace/ros2_ws/setup_env.sh

# Add to bashrc
RUN echo 'source /workspace/ros2_ws/setup_env.sh' >> ~/.bashrc

# Set default command
CMD ["/bin/bash", "-c", "cd /workspace/ros2_ws && source setup_env.sh && bash"]
