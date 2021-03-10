FROM osrf/ros:noetic-desktop-full

ARG clean_ws

RUN apt-get update && \
    apt-get install -y nano python3-catkin-tools python3-pip ros-noetic-octomap-msgs \
    ros-noetic-octomap-ros protobuf-compiler libgoogle-glog-dev liblapacke-dev libtool libtool-bin git && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install osrf-pycommon && \
    rm -r ~/.cache/pip

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /mav_ipp/devel/setup.bash" >> ~/.bashrc

WORKDIR "/mav_ipp"