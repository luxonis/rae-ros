ARG ROS_DISTRO=humble
FROM luxonis/depthai-ros-rae AS builder
ARG SIM=0
ARG CORE_NUM=1
ARG BUILD_TYPE="RelWithDebInfo"
ARG INCLUDE_SPECTACULARAI_ROS=YES
ARG SPECTACULARAI_ROS_VERSION=v0.0.1

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
   && apt-get -y install --no-install-recommends software-properties-common git libusb-1.0-0-dev wget zsh python3-colcon-common-extensions python3-rosdep build-essential neovim tmux htop net-tools iputils-ping gpiod gstreamer1.0-plugins-bad gstreamer1.0-alsa libasound2-dev busybox

ENV WS=/ws
RUN mkdir -p $WS/src
COPY ./ .$WS/src/rae-ros

RUN rm -rf .$WS/src/rae-ros/rae_gazebo

RUN cd  .$WS/ && apt update && rosdep update && rosdep install --from-paths src --ignore-src  -y --skip-keys depthai --skip-keys depthai_bridge --skip-keys depthai_ros_driver --skip-keys audio_msgs --skip-keys laserscan_kinect --skip-keys ira_laser_tools

RUN cd .$WS/ && . /opt/ros/${ROS_DISTRO}/setup.sh && . $UNDERLAY_WS/install/setup.sh && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=${BUILD_TYPE}

RUN echo "if [ -f ${WS}/install/setup.zsh ]; then source ${WS}/install/setup.zsh; fi" >> $HOME/.zshrc
RUN echo 'eval "$(register-python-argcomplete3 ros2)"' >> $HOME/.zshrc
RUN echo 'eval "$(register-python-argcomplete3 colcon)"' >> $HOME/.zshrc
RUN echo "if [ -f ${WS}/install/setup.bash ]; then source ${WS}/install/setup.bash; fi" >> $HOME/.bashrc
RUN --mount=type=secret,id=SPECTACULAR_AI_TOKEN if [ "$INCLUDE_SPECTACULARAI_ROS" = "YES" ]; then \
      rm -rf sai_ros \
      && ls -la /run/secrets \
      && git clone --single-branch --branch ${SPECTACULARAI_ROS_VERSION} https://github.com/SpectacularAI/ros.git sai_ros \
      && cd sai_ros \
      && apt-get -y install unzip --no-install-recommends \
      && ROS_DISTRO=$ROS_DISTRO DEPTHAI_WS=$UNDERLAY_WS GITHUB_RAE_PAT_TOKEN=$(cat /run/secrets/SPECTACULAR_AI_TOKEN) . ./scripts/download_and_build_static.sh \
      && apt-get -y remove unzip \
      && echo "if [ -f $(pwd)/spectacularai_ros2/install/setup.bash ]; then source $(pwd)/spectacularai_ros2/install/setup.bash; fi" >> $HOME/.bashrc; \
  fi

ENTRYPOINT [ "/ws/src/rae-ros/entrypoint.sh" ]
RUN rm -rf /usr/share/doc

CMD ["zsh"]
