sh ./deployment/install-third-party-dependencies.sh
sh ./deployment/init-workspace.sh

docker stop mav_ipp_sim_compiler
docker rm mav_ipp_sim_compiler
docker run --name mav_ipp_sim_compiler -v $(pwd)/:/mav_ipp mav_ipp_mav_ipp_sim bash -c "source /opt/ros/noetic/setup.bash && catkin build"
