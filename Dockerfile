ARG ROS_DISTRO=humble

FROM alpine/git:latest AS rae-ros-downloader
RUN git clone --branch humble https://github.com/luxonis/rae-ros

FROM alpine/git:latest AS ros-gst-bridge-downloader
RUN git clone https://github.com/BrettRD/ros-gst-bridge && \
    cd ros-gst-bridge && \
    git checkout 23980326ce8c0fefc0d5d590c2bfc9d308f35a73  # Pin latest master version at the time

FROM ros:${ROS_DISTRO}-ros-core

ARG CORE_NUM=10
ARG BUILD_TYPE="RelWithDebInfo"

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
    gstreamer1.0-plugins-bad


ENV WS=/ws

COPY ./entrypoint.sh .$WS/src/rae/entrypoint.sh
RUN mkdir -p $WS/src/rae

COPY --from=rae-ros-downloader /git/rae-ros/rae_hw $WS/src/rae/rae_hw
COPY --from=rae-ros-downloader /git/rae-ros/rae_description $WS/src/rae/rae_description
COPY --from=rae-ros-downloader /git/rae-ros/rae_msgs $WS/src/rae/rae_msgs

RUN rosdep init

COPY --from=ros-gst-bridge-downloader /git/ros-gst-bridge $WS/src/ros-gst-bridge

RUN apt update && rosdep update

RUN cd .$WS/ && rosdep install --from-paths src --ignore-src -y --skip-keys depthai
RUN cd .$WS/ && . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=${BUILD_TYPE}

RUN echo "if [ -f ${WS}/install/setup.bash ]; then source ${WS}/install/setup.bash; fi" >> $HOME/.bashrc

COPY ./requirements.txt .$WS/src/requirements.txt
RUN python3 -m pip install --extra-index-url https://artifacts.luxonis.com/artifactory/luxonis-python-snapshot-local/ -r .$WS/src/requirements.txt

RUN chmod +x /ws/src/rae/entrypoint.sh

ENTRYPOINT [ "/ws/src/rae/entrypoint.sh" ]

RUN rm -rf /usr/share/doc

CMD ["bash"]