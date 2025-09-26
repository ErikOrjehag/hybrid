FROM osrf/ros:humble-desktop-full


ENV ROS_DISTRO=humble
ENV AMENT_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV COLCON_PREFIX_PATH=/opt/ros/${ROS_DISTRO}
ENV LD_LIBRARY_PATH=/opt/ros/${ROS_DISTRO}/lib
ENV PYTHONPATH=/opt/ros/${ROS_DISTRO}/lib/python3.8/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2
ENV DEPENDENCIES_WS=/home/ros/dependencies_ws
ENV WS=/home/ros/ws
ENV USER_GID=1000
ENV USER_UID=1000
ENV USERNAME=ros
ENV DEBIAN_FRONTEND=noninteractive

RUN groupadd -g ${USER_GID} ${USERNAME}
RUN useradd -s /bin/bash -m -c "Ubuntu ROS User" ${USERNAME} -p "" -u ${USER_UID} -g ${USER_GID} -G sudo -G dialout -G video 
RUN echo "${USERNAME} ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers && \
    chmod 0440 /etc/sudoers && \
    chmod g+w /etc/passwd

RUN mkdir -p /home/${USERNAME}
RUN chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}

USER ${USERNAME}

WORKDIR /home/${USERNAME}

RUN bash -c "$(curl -fsSL https://raw.githubusercontent.com/ohmybash/oh-my-bash/master/tools/install.sh)"
RUN sed -i 's/font/pure/g' ${HOME}/.bashrc
RUN echo "export RCUTILS_COLORIZED_OUTPUT=1" >> ${HOME}/.bashrc
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ${HOME}/.bashrc

RUN mkdir -p ${WS}/src/hybrid

COPY --chown=${USERNAME}:${USERNAME} . /home/${USERNAME}/ws/src/hybrid

WORKDIR /home/${USERNAME}/ws

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    sudo apt update && \
    sudo apt install -y libsfml-dev libboost-all-dev libeigen3-dev ros-${ROS_DISTRO}-ompl nlohmann-json3-dev && \
    sudo apt install -y ros-humble-cyclonedds ros-humble-rmw-cyclonedds-cpp && \
    colcon build --symlink-install --merge-install && \
    sudo apt autoremove -y && \
    sudo rm -rf /var/lib/apt/lists/*

RUN echo "source ${WS}/install/setup.bash" >> ${HOME}/.bashrc


