# MAV - Informative Path Planning

Informative path planning simulation with RIT-18 dataset.

## Installation & Deployment

### Docker-Compose Setup
The installation is based on Docker for easy transferability between different hardware setups.
Furthermore, for smooth deployment, a docker-compose setup is established.
Please make sure to install [Docker](https://docs.docker.com/get-docker/) and [docker-compose](https://docs.docker.com/compose/install/).

For a fully automatic installation of third party dependencies, ROS workspace initialization, and compilation of the framework, run:
```commandline
docker-compose build
./compile.sh
```

To recompile the framework from a clean workspace, use:
```commandline
./clean.sh
./compile.sh
```

Create a file named `.env` and set the env-variables expected by the docker container:

```commandline
REPO_DIR=/path/to/your/repo/
```

To run the framework/container, use:
```commandline
./run.sh <LAUNCH_FILENAME>
```
where `<LAUNCH_FILENAME>` is an optional argument and by default set to `noetic_mav_ipp_sim_segnet.launch`.

To stop the container, use:

```commandline
./stop.sh
```

To connect to the running pipeline/container with a bash, use:
```commandline
./connect.sh
```

### Setup without Docker-Compose

If you would like to install and run the framework directly in your usual host machine's ROS environment, execute the following steps (requires Ubuntu 20.04 and ROS noetic):
```commandline
sudo apt-get update
sudo apt-get install -y nano python3-catkin-tools python3-pip ros-noetic-octomap-msgs python3-tk caffe-cpu ros-noetic-octomap-ros protobuf-compiler libgoogle-glog-dev liblapacke-dev libtool libtool-bin
pip3 install -r requirements.txt
./deployment/install-third-party-dependencies.sh
./deployment/init-workspace.sh
source /opt/ros/noetic/setup.bash
catkin build
source devel/setup.bash
roslaunch mav_ipp_sim/launch/<LAUNCH_FILENAME>
```
