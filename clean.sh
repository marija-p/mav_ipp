SRC_DIR="src/"
if [ -d "${SRC_DIR}" ]; then
	echo "Remove ${SRC_DIR}"
	rm -rf ${SRC_DIR}
else
	echo "${SRC_DIR} does not exist"
fi

echo "Execute catkin clean..."
docker stop mav_ipp_sim_compiler
docker rm mav_ipp_sim_compiler
docker run --name mav_ipp_sim_compiler -v $(pwd)/:/mav_ipp mav_ipp_mav_ipp_sim bash -c "source /opt/ros/noetic/setup.bash && catkin clean --yes"
