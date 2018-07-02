[pose_pub, pose_msg] = rospublisher('/firefly/command/pose', 'geometry_msgs/PoseStamped');
pose_msg.Pose.Position.X = 0;
pose_msg.Pose.Position.Y = 0;
pose_msg.Pose.Position.Z = 100;
send(pose_pub, pose_msg)
disp('Sent pose message')