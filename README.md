# MAV - Informative Path Planning

Informative path planning simulation with RIT-18 dataset.

## Installation & Deployment

### Docker-Compose Setup
The installation is based on Docker for easy transferability between different hardware setups.
Furthermore, for smooth deployment, a docker-compose setup is established.
Please make sure to install [Docker](https://docs.docker.com/get-docker/) and [docker-compose](https://docs.docker.com/compose/install/).

For a fully automatic installation of third party dependencies, ROS workspace initialization, and compilation of the framework, run:
```bash
docker-compose build
```

To recompile the framework from a clean workspace, use:
```bash
docker-compose build --build-arg clean_ws
```

To run the framework/container, use:
```bash
./run_mav_ipp.sh <LAUNCH_FILENAME>
```
where `<LAUNCH_FILENAME>` is an optional argument and by default set to `mav_ipp_sim.launch`.

To stop the container, use:

```bash
./stop_mav_ipp.sh
```

### Setup without Docker-Compose

If you would like to install and run the framework directly in your usual host machine's ROS environment, execute the following steps:
```bash
./deployment/install-third-party-dependencies.sh
./deployment/init-workspace.sh
catkin build
source devel/setup.bash
roslaunch mav_ipp_sim/launch/<LAUNCH_FILENAME>
```

