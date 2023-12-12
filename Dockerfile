ARG ROS_DISTRO=humble

FROM alpine/git:latest AS rae-ros-downloader
RUN git clone --branch robot_py_library https://github.com/luxonis/rae-ros

FROM alpine/git:latest AS ros-gst-bridge-downloader
RUN git clone https://github.com/BrettRD/ros-gst-bridge && \
    cd ros-gst-bridge && \
    git checkout 23980326ce8c0fefc0d5d590c2bfc9d308f35a73  # Pin latest master version at the time.

FROM ros:${ROS_DISTRO}-ros-core

ARG CORE_NUM=10
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
    python3-pip \
    libasound2-dev \
    ros-humble-depthai-ros-msgs \
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

COPY ./entrypoint.sh .$WS/src/rae/entrypoint.sh
RUN mkdir -p $WS/src/rae

COPY --from=rae-ros-downloader /git/rae-ros/rae_hw $WS/src/rae/rae_hw
COPY --from=rae-ros-downloader /git/rae-ros/rae_description $WS/src/rae/rae_description
COPY --from=rae-ros-downloader /git/rae-ros/rae_msgs $WS/src/rae/rae_msgs
COPY --from=rae-ros-downloader /git/rae-ros/robot_py $WS/src/rae/robot_py


RUN rosdep init

COPY --from=ros-gst-bridge-downloader /git/ros-gst-bridge $WS/src/ros-gst-bridge

RUN apt update && rosdep update

RUN pip3 install mypy
RUN cd /tmp \
   && git clone --recursive https://github.com/luxonis/depthai-python.git --branch rvc3_develop \
   && cd /tmp/depthai-python && cmake -Hdepthai-core -Bdepthai-core/build -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=/usr/local \
   && cmake --build depthai-core/build --target install --parallel 8 \
   && cd /tmp/depthai-python && python3 -m pip install . \
   && cd /tmp \
   && rm -rf depthai-python

RUN cd .$WS/src && git clone https://github.com/Serafadam/ira_laser_tools.git && git clone https://github.com/Serafadam/depth_nav_tools.git
RUN apt install curl
RUN --mount=type=secret,id=SPECTACULAR_AI_TOKEN rm -rf sai_ros \
      && git clone --single-branch --branch python-bindings https://github.com/Serafadam/sai_ros.git  sai_ros \
      && cd sai_ros \
      && apt-get -y install unzip --no-install-recommends \
      && ROS_DISTRO=$ROS_DISTRO DEPTHAI_WS=$UNDERLAY_WS GITHUB_RAE_PAT_TOKEN=$(cat /run/secrets/SPECTACULAR_AI_TOKEN) . ./scripts/download_and_build_static.sh \
      && apt-get -y remove unzip \
      && echo "if [ -f $(pwd)/spectacularai_ros2/install/setup.bash ]; then source $(pwd)/spectacularai_ros2/install/setup.bash; fi" >> $HOME/.bashrc \
      && echo "if [ -f $(pwd)/spectacularai_ros2/install/setup.zsh ]; then source $(pwd)/spectacularai_ros2/install/setup.zsh; fi" >> $HOME/.zshrc; 
RUN cd ${WS}/src && git clone --branch dai_ros_py https://github.com/luxonis/depthai-ros.git
RUN cd .$WS/ && rosdep install --from-paths src --ignore-src -y --skip-keys depthai
RUN cd .$WS/ && . /opt/ros/${ROS_DISTRO}/setup.sh && . /sai_ros/spectacularai_ros2/install/setup.sh && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=${BUILD_TYPE}
RUN echo "if [ -f ${WS}/install/setup.bash ]; then source ${WS}/install/setup.bash; fi" >> $HOME/.bashrc
RUN echo "if [ -f ${WS}/install/setup.zsh ]; then source ${WS}/install/setup.zsh; fi" >> $HOME/.zshrc
RUN chmod +x /ws/src/rae/entrypoint.sh

ENTRYPOINT [ "/ws/src/rae/entrypoint.sh" ]

RUN rm -rf /usr/share/doc

CMD ["bash"]
