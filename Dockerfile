FROM osrf/ros:noetic-desktop-full

ARG clean_ws

RUN apt-get update && \
    apt-get install -y nano python3-catkin-tools python3-pip ros-noetic-octomap-msgs python3-tk caffe-cpu \
    ros-noetic-octomap-ros protobuf-compiler libgoogle-glog-dev liblapacke-dev libtool libtool-bin git && \
    rm -rf /var/lib/apt/lists/*

COPY requirements.txt .
RUN pip3 install -r requirements.txt && \
    rm -r ~/.cache/pip

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /mav_ipp/devel/setup.bash" >> ~/.bashrc

WORKDIR "/mav_ipp"