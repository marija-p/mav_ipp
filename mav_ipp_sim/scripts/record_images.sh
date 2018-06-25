#!/usr/bin/env bash

BAG_NAME="$(date +"%F-%H-%M-%S").bag"
rosbag record --output-name=$BAG_NAME \
/firefly/ground_truth/odometry \
/firefly/camera/camera_sim/image_raw
