ARG ROS_DISTRO=humble
FROM luxonis/depthai-ros-rae AS builder
ARG SIM=0
ARG CORE_NUM=1
ARG BUILD_TYPE="RelWithDebInfo"

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
   && apt-get -y install --no-install-recommends software-properties-common git libusb-1.0-0-dev wget zsh python3-colcon-common-extensions python3-rosdep build-essential neovim tmux htop net-tools iputils-ping gpiod gstreamer1.0-plugins-bad gstreamer1.0-alsa libasound2-dev busybox

ENV WS=/ws
RUN mkdir -p $WS/src

RUN cd ./$WS/src && git clone https://github.com/BrettRD/ros-gst-bridge.git
RUN cd ./$WS/src && git clone https://github.com/Serafadam/ira_laser_tools.git

COPY ./ .$WS/src/rae-ros

RUN cd  .$WS/ && apt update && rosdep update && rosdep install --from-paths src --ignore-src  -y --skip-keys depthai --skip-keys depthai_bridge

RUN cd .$WS/ && . /opt/ros/${ROS_DISTRO}/setup.sh && . $DEPTHAI_WS/install/setup.sh && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=${BUILD_TYPE}

RUN echo "if [ -f ${WS}/install/setup.zsh ]; then source ${WS}/install/setup.zsh; fi" >> $HOME/.zshrc
RUN echo 'eval "$(register-python-argcomplete3 ros2)"' >> $HOME/.zshrc
RUN echo 'eval "$(register-python-argcomplete3 colcon)"' >> $HOME/.zshrc
RUN echo "if [ -f ${WS}/install/setup.bash ]; then source ${WS}/install/setup.bash; fi" >> $HOME/.bashrc
ENTRYPOINT [ "/ws/src/rae-ros/entrypoint.sh" ]
RUN rm -rf /usr/share/doc

CMD ["zsh"]
