# Franka Pose Control

This is an example to move the end-effector of Franka Research3 arm to the desired  pose.

## launch 

launch file to open RViz and load robot model

## src/franka_targetPose_publisher.cpp

This a node to receive the pose message from topic "sigma1/pose". Then publish this message to Franka motion node with the topic: "target_pose". 

## src/franka_pose.cpp

This a node to receive the message from topic "target_pose". The the motion interface node will plan a path to the desired pose.
