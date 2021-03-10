SRC_DIR="src/"
if [ ! -d "$SRC_DIR" ]; then
	echo "Create workspace directory ${SRC_DIR}..."
	mkdir $SRC_DIR
fi

echo "Copy ROS packages to ${SRC_DIR}..."
PACKAGE_DIRS="mav_ipp_sim catkin_simple rotors_simulator mav_comm glog_catkin eigen_checks mav_control_rw eigen_catkin"

for package_dir in $PACKAGE_DIRS; do
    if [ ! -d "${package_dir}" ]; then
        echo "Warning: Cannot find package ${package_dir}!"
    fi
    if [ ! -d "${SRC_DIR}/${package_dir}" ]; then
        echo "Copy ${package_dir} into ${SRC_DIR}..."
        cp -r ${package_dir} ${SRC_DIR}/${package_dir}
    fi
done