ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-core AS builder
ARG SIM=0
ARG CORE_NUM=1
ARG BUILD_TYPE="RelWithDebInfo"

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
   && apt-get -y install --no-install-recommends software-properties-common git libusb-1.0-0-dev wget zsh python3-colcon-common-extensions python3-rosdep build-essential neovim tmux htop net-tools iputils-ping gpiod

RUN rosdep init

ENV DEBIAN_FRONTEND=dialog
RUN sh -c "$(wget https://raw.github.com/ohmyzsh/ohmyzsh/master/tools/install.sh -O -)"

RUN if [ "$SIM" = "0" ] ; then cd /tmp \
   && git clone --recursive https://github.com/luxonis/depthai-core.git --branch rvc3_develop \
   && cmake -Hdepthai-core -Bdepthai-core/build -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=/usr/local \
   && cmake --build depthai-core/build --target install --parallel ${CORE_NUM} \
   && cd /tmp \
   && rm -r depthai-core ; fi

ENV WS=/ws
RUN mkdir -p $WS/src
RUN if [ "$SIM" = "0" ] ; then cd .$WS/src && git clone --branch rae_pipeline_humble https://github.com/luxonis/depthai-ros.git ; fi
RUN if [ "$SIM" = "0" ] ; then cd .$WS/src/depthai-ros && rm -rf depthai_ros_driver depthai-ros depthai_examples depthai_descriptions depthai_filters depthai_ros_msgs ; fi

RUN apt update && rosdep update

COPY ./ .$WS/src/rae
RUN cd .$WS/ && rosdep install --from-paths src --ignore-src  -y --skip-keys depthai
RUN if [ "$SIM" = "0" ] ; then cd .$WS/ && . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --symlink-install ; fi

RUN cd .$WS/ && . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=${BUILD_TYPE}
RUN echo "if [ -f ${WS}/install/setup.zsh ]; then source ${WS}/install/setup.zsh; fi" >> $HOME/.zshrc
RUN echo 'eval "$(register-python-argcomplete3 ros2)"' >> $HOME/.zshrc
RUN echo 'eval "$(register-python-argcomplete3 colcon)"' >> $HOME/.zshrc
RUN echo "if [ -f ${WS}/install/setup.bash ]; then source ${WS}/install/setup.bash; fi" >> $HOME/.bashrc
ENTRYPOINT [ "/ws/src/rae/entrypoint.sh" ]
RUN rm -rf /usr/share/doc
CMD ["zsh"]