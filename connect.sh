if [ ! "$(docker ps -a | grep mav_ipp_mav_ipp_sim_1)" ]; then
	echo "Pipeline not running"
else
	docker exec -it mav_ipp_mav_ipp_sim_1 bash
fi
