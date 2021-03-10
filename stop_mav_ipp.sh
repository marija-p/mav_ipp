CONTAINER_ID=$(docker inspect --format='{{ .Config.Hostname }}' mav_ipp_mav_ipp_sim_1)
xhost -local:${CONTAINER_ID}
docker-compose down
