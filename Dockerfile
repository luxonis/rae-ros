ARG ROS_DISTRO=humble
FROM luxonis/depthai-ros-rae AS builder
ARG SIM=0
ARG CORE_NUM=1
ARG BUILD_TYPE="RelWithDebInfo"

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
   && apt-get -y install --no-install-recommends software-properties-common git libusb-1.0-0-dev wget zsh python3-colcon-common-extensions python3-rosdep build-essential neovim tmux htop net-tools iputils-ping gpiod


ENV WS=/ws
RUN mkdir -p $WS/src

RUN apt update && rosdep update

COPY ./ .$WS/src/rae
RUN cd .$WS/ && rosdep install --from-paths src --ignore-src  -y --skip-keys depthai --skip-keys depthai_bridge
RUN cd .$WS/ && . $DEPTHAI_WS/install/setup.sh && . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --symlink-install

RUN cd .$WS/ && . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=${BUILD_TYPE}
RUN echo "if [ -f ${WS}/install/setup.zsh ]; then source ${WS}/install/setup.zsh; fi" >> $HOME/.zshrc
RUN echo 'eval "$(register-python-argcomplete3 ros2)"' >> $HOME/.zshrc
RUN echo 'eval "$(register-python-argcomplete3 colcon)"' >> $HOME/.zshrc
RUN echo "if [ -f ${WS}/install/setup.bash ]; then source ${WS}/install/setup.bash; fi" >> $HOME/.bashrc
ENTRYPOINT [ "/ws/src/rae/entrypoint.sh" ]
RUN rm -rf /usr/share/doc
CMD ["zsh"]