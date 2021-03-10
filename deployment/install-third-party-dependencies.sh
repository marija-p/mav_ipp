ETH_ASL_PACKAGE_DIRS="rotors_simulator mav_comm glog_catkin eigen_checks mav_control_rw eigen_catkin"
for package_dir in $ETH_ASL_PACKAGE_DIRS; do
    if [ ! -d "${package_dir}" ]; then
        echo "Clone ${package_dir}..."
        git clone https://github.com/ethz-asl/${package_dir}.git
    else
        echo "${package_dir} already exists"
    fi
done

if [ ! -d "catkin_simple/" ]; then
    echo "Clone catkin_simple..."
    git clone https://github.com/catkin/catkin_simple.git
else
    echo "catkin_simple already exists"
fi