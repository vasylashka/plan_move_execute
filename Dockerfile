# Use the ROS2 Humble base image
FROM osrf/ros:humble-desktop

# Install basic dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-rosdep2 \
    build-essential \
    clang \
    cmake \
    git \
    libssl-dev \
    libbullet-dev \
    libbullet-doc \
    freeglut3-dev \
    apt-transport-https \
    software-properties-common \
    sudo \
    udev \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Add the Nvidia package repositories
RUN distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
    && curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add - \
    && curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

# Install Nvidia Container Toolkit
RUN apt-get update && apt-get install -y nvidia-container-toolkit

# Install PyBullet
RUN pip3 install numpy==1.26.4 scipy==1.14.1 pybullet==3.2.6

# Install Bullet Physics with additional options
RUN git clone https://github.com/bulletphysics/bullet3.git \
    && cd bullet3 \
    && cmake -DBUILD_SHARED_LIBS=ON -DBUILD_EXTRAS=ON -DBUILD_PYBULLET=OFF -DBUILD_BULLET3=ON -DINSTALL_LIBS=ON -DBUILD_BULLET_ROBOTICS_EXTRA=ON -DBUILD_BULLET_ROBOTICS_GUI_EXTRA=ON -DBUILD_BULLET2_DEMOS=ON . \
    && make \
    && make install

# Install RealSense packages
# Add Intel RealSense public key
RUN sudo mkdir -p /etc/apt/keyrings && \
    curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | \
    sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

# Add Intel RealSense repository
RUN echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" | \
    sudo tee /etc/apt/sources.list.d/librealsense.list

# Update repositories and install RealSense libraries
RUN apt-get update && apt-get install -y \
    librealsense2-utils \
    librealsense2-dev \
    librealsense2-dbg \
 && rm -rf /var/lib/apt/lists/*

# Set the working directory
ENV DEV_ENV=/home/ros2_ws
WORKDIR $DEV_ENV

# Default command
CMD ["/bin/bash"]
