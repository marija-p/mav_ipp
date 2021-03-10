SRC_DIR="src/"
if [ -d "$SRC_DIR"]; then
	echo "Remove ${SRC_DIR}"
	rm -rf ${SRC_DIR}
else
	echo "${SRC_DIR} does not exist"
fi

echo "Execute catkin clean..."
catkin clean --yes