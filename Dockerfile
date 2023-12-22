FROM ghcr.io/luxonis/rae-base:2023.12.19

ARG BUILD_TYPE="RelWithDebInfo"
ARG SIM=0
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get -y install --no-install-recommends \
    software-properties-common \
    libusb-1.0-0-dev \
    python3-colcon-common-extensions \
    python3-rosdep \
    build-essential \
    gpiod \
    libasound2-dev \
    ros-humble-cv-bridge \ 
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-rmw-cyclonedds-cpp \
    gstreamer1.0-plugins-bad \
    alsa-utils \
    mpg123 \
    libmpg123-dev \
    ros-humble-rtabmap-slam \ 
    unzip \
    ffmpeg \
    ros-humble-image-proc \
    git \
    htop

ENV WS=/ws
RUN mkdir -p $WS/src
COPY ./ .$WS/src/rae-ros

RUN cp -R .$WS/src/rae-ros/assets/. /usr/share
RUN rm -rf .$WS/src/rae-ros/assets
RUN rm -rf .$WS/src/rae-ros/rae_gazebo

RUN cd .$WS/src && git clone https://github.com/Serafadam/ira_laser_tools.git && git clone https://github.com/Serafadam/depth_nav_tools.git
RUN cd .$WS/src && git clone https://github.com/BrettRD/ros-gst-bridge && \
    cd ros-gst-bridge && \
    git checkout 23980326ce8c0fefc0d5d590c2bfc9d308f35a73  # Pin latest master version at the time.
RUN cd ${WS}/src && git clone --branch dai_ros_py https://github.com/luxonis/depthai-ros.git
RUN cd .$WS/ && rosdep install --from-paths src --ignore-src  -y --skip-keys depthai --skip-keys depthai_bridge --skip-keys depthai_ros_driver --skip-keys audio_msgs --skip-keys laserscan_kinect --skip-keys ira_laser_tools
RUN cd .$WS/ && . /opt/ros/${ROS_DISTRO}/setup.sh && . /sai_ros/spectacularai_ros2/install/setup.sh && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=${BUILD_TYPE}
RUN echo "if [ -f ${WS}/install/setup.bash ]; then source ${WS}/install/setup.bash; fi" >> $HOME/.bashrc
RUN echo "if [ -f ${WS}/install/setup.zsh ]; then source ${WS}/install/setup.zsh; fi" >> $HOME/.zshrc
RUN chmod +x /ws/src/rae-ros/entrypoint.sh

ENTRYPOINT [ "/ws/src/rae/entrypoint.sh" ]

RUN rm -rf /usr/share/doc

CMD ["bash"]
