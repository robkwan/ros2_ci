# File: Dockerfile.fastbot-ros2-gazebo

# Base image with GUI and Gazebo support
FROM osrf/ros:humble-desktop
# Change the default shell to Bash
SHELL [ "/bin/bash" , "-c" ]

# Install Gazebo 11 and other dependencies
RUN apt-get update && apt-get install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-joint-state-publisher \
  ros-humble-robot-state-publisher \
  ros-humble-robot-localization \
  ros-humble-xacro \
  ros-humble-tf2-ros \
  ros-humble-tf2-tools \
  ros-humble-rmw-cyclonedds-cpp \
  python3-colcon-common-extensions \
  libx11-dev \
  libxtst-dev \
  libxrender-dev \
  libxext-dev \
  libxkbcommon-dev \
  libwayland-dev \
  libegl1-mesa-dev \
  libgl1-mesa-glx \
  libglib2.0-dev \
  libqt5gui5 \
  libqt5x11extras5-dev \
  qtwayland5 \
  # Additional packages that might help with Qt/X11 or debugging
  x11-utils \
  xterm \
  # Ensure all necessary OpenGL libraries are present for Gazebo
  mesa-utils \
  ros-humble-controller-manager \
  ros-humble-gazebo-ros2-control-demos \
  ros-humble-rviz2 \
  ros-humble-gazebo-ros \
  ros-humble-gazebo-plugins \
  ros-humble-teleop-twist-keyboard \
  python3-rosdep \
  python3-rosinstall \
  python3-rosinstall-generator \
  && rm -rf /var/lib/apt/lists/*

# ... after installing packages ...
ENV QT_XCB_GLUE_ALWAYS_IN_USE=1
# ... after installing packages ...
ENV QT_X11_NO_MITSHM=1
ENV GAZEBO_MODEL_PATH=/.gazebo/models:/ros2_ws/src/fastbot_description/onshape/assets:/ros2_ws/src/fastbot_description/models/meshes:/ros2_ws/src/fastbot_description/models/urdf
ENV DISPLAY=:1
ENV GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI}

# Create workspace and download simulation repository

RUN source /opt/ros/humble/setup.bash \
 && mkdir -p /ros2_ws/src \
 && cd /ros2_ws/src

# Create necessary directories

RUN mkdir -p /ros2_ws/src/fastbot_gazebo
RUN mkdir -p /.gazebo/models/fastbot_description
RUN mkdir -p /.gazebo/models/meshes/meshes
RUN mkdir -p /.gazebo/models/urdf

# Copy your simulation package
COPY fastbot/fastbot_gazebo/ /ros2_ws/src/fastbot_gazebo
COPY fastbot/fastbot_description/ /ros2_ws/src/fastbot_description

COPY fastbot/fastbot_gazebo/models /.gazebo/models
COPY fastbot/fastbot_description/ /.gazebo/models/fastbot_description
COPY fastbot/fastbot_description/models/meshes /.gazebo/models/meshes/meshes
COPY fastbot/fastbot_description/models/urdf /.gazebo/models/urdf

COPY fastbot_waypoints /ros2_ws/src/fastbot_waypoints

COPY entrypoint.sh /ros2_ws/entrypoint.sh

RUN chmod +x /ros2_ws/entrypoint.sh

# Build the Colcon workspace and ensure it's sourced
RUN source /opt/ros/humble/setup.bash \
 && cd /ros2_ws \
 && colcon build --symlink-install

RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Set up a workspace directory
WORKDIR /ros2_ws/

# Set environment variables
ENV ROS_DOMAIN_ID=1
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Launch your Gazebo simulation
#CMD ["bash"]
