LAUNCH_FILENAME=${1:-mav_ipp_sim_segnet.launch}
echo "Start with launch file ${LAUNCH_FILENAME}"
docker-compose down
docker-compose up -d
CONTAINER_ID=$(docker inspect --format='{{ .Config.Hostname }}' mav_ipp_mav_ipp_sim_1)
xhost +local:${CONTAINER_ID}
docker exec -it mav_ipp_mav_ipp_sim_1 bash -c "source /mav_ipp/devel/setup.bash && roslaunch mav_ipp_sim/launch/${LAUNCH_FILENAME}"
