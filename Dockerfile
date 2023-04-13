ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-core AS builder
ARG USE_RVIZ
ARG SIM=0
ARG CORE_NUM=1
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
   && apt-get -y install --no-install-recommends software-properties-common git libusb-1.0-0-dev wget zsh python3-colcon-common-extensions python3-rosdep build-essential

ENV DEBIAN_FRONTEND=dialog
RUN sh -c "$(wget https://raw.github.com/ohmyzsh/ohmyzsh/master/tools/install.sh -O -)"

RUN  if [ "$SIM" = "1" ] ; then wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
 && apt-get update \
 && apt-get install -y ignition-fortress ; fi
RUN cd /tmp \
   && git clone --recursive https://github.com/luxonis/depthai-core.git --branch rvc3_develop \
   && cmake -Hdepthai-core -Bdepthai-core/build -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=/usr/local \
   && cmake --build depthai-core/build --target install --parallel ${CORE_NUM} \
   && cd /tmp \
   && rm -r depthai-core

ENV WS=/ws
RUN mkdir -p $WS/src
RUN if [ "$SIM" = "0" ] ; then cd .$WS/src && git clone --branch rae_pipeline_humble https://github.com/luxonis/depthai-ros.git ; fi
RUN apt update && rosdep init && rosdep update
RUN if [ "$SIM" = "0" ] ; then cd .$WS/ && rosdep install --from-paths src --ignore-src  -y --skip-keys depthai ; fi
RUN if [ "$SIM" = "0" ] ; then cd .$WS/ && . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --symlink-install ; fi

FROM builder AS final
COPY ./ .$WS/src/rae
RUN if [ "$SIM" = "0" ] ; then rm -rf .$WS/src/rae/rae_gazebo ; fi
RUN cd .$WS/ && rosdep install --from-paths src --ignore-src  -y --skip-keys depthai
RUN apt install -y gpiod

RUN cd .$WS/ && . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --symlink-install
RUN if [ "$USE_RVIZ" = "1" ] ; then echo "RVIZ ENABLED" && sudo apt install -y ros-${ROS_DISTRO}-rviz2 ros-${ROS_DISTRO}-rviz-imu-plugin ; else echo "RVIZ NOT ENABLED"; fi
RUN echo "if [ -f ${WS}/install/setup.zsh ]; then source ${WS}/install/setup.zsh; fi" >> $HOME/.zshrc
RUN echo 'eval "$(register-python-argcomplete3 ros2)"' >> $HOME/.zshrc
RUN echo 'eval "$(register-python-argcomplete3 colcon)"' >> $HOME/.zshrc
RUN echo "if [ -f ${WS}/install/setup.bash ]; then source ${WS}/install/setup.bash; fi" >> $HOME/.bashrc
ENTRYPOINT [ "/ws/src/rae/entrypoint.sh" ]
RUN rm -rf /usr/share/doc
RUN apt install -y neovim tmux htop
CMD ["zsh"]