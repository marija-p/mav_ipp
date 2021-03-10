FROM osrf/ros:noetic-desktop-full

ARG clean_ws

RUN apt-get update && apt-get install -y nano python3-catkin-tools python3-pip ros-noetic-octomap-msgs ros-noetic-octomap-ros \
                                         protobuf-compiler libgoogle-glog-dev liblapacke-dev libtool libtool-bin git
RUN pip3 install osrf-pycommon

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /mav_ipp/devel/setup.bash" >> ~/.bashrc

COPY . /mav_ipp
WORKDIR /mav_ipp

RUN if [ -z "$clean_ws" ]; then deployment/remove-workspace.sh; fi

RUN deployment/install-third-party-dependencies.sh
RUN deployment/init-workspace.sh

RUN bash -c "source /opt/ros/noetic/setup.bash && catkin build"