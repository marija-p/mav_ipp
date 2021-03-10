FROM osrf/ros:noetic-desktop-full

RUN apt-get update && apt-get install -y nano python3-catkin-tools python3-pip ros-noetic-octomap-msgs ros-noetic-octomap-ros \
                                         protobuf-compiler libgoogle-glog-dev liblapacke-dev libtool libtool-bin
RUN pip3 install osrf-pycommon

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /mav_ipp/devel/setup.bash" >> ~/.bashrc