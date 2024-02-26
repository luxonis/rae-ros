FROM ghcr.io/luxonis/rae-base:2024.01.18-humble

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
    htop \
    libsndfile1-dev \
    libsndfile1

RUN pip3 install openai

ENV WS=/ws
RUN mkdir -p $WS/src
COPY ./ .$WS/src/rae-ros

RUN rm -rf .$WS/src/rae-ros/assets
RUN rm -rf .$WS/src/rae-ros/rae_gazebo

RUN cd .$WS/ && rosdep install --from-paths src --ignore-src  -y --skip-keys depthai --skip-keys depthai_bridge --skip-keys depthai_ros_driver --skip-keys audio_msgs --skip-keys laserscan_kinect --skip-keys ira_laser_tools
RUN cd .$WS/ && . /opt/ros/${ROS_DISTRO}/setup.sh && . /sai_ros/spectacularai_ros2/install/setup.sh && . /${UNDERLAY_WS}/install/setup.sh && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=${BUILD_TYPE}
RUN echo "if [ -f ${WS}/install/setup.bash ]; then source ${WS}/install/setup.bash; fi" >> $HOME/.bashrc
RUN echo "if [ -f ${WS}/install/setup.zsh ]; then source ${WS}/install/setup.zsh; fi" >> $HOME/.zshrc
RUN chmod +x /ws/src/rae-ros/entrypoint.sh

ENTRYPOINT [ "/ws/src/rae-ros/entrypoint.sh" ]

RUN rm -rf /usr/share/doc

CMD ["bash"]
