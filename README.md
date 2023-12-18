# Franka_Sigma_ws

This is an example to control FR3 arm by sigma.7 haptic device. 

## Steps:

### 1. Build workspace

```
colcon build
```

### 2. Turn on sigma 7

```
// super user
sudo su
. install/setup.bash
// run main code
ros2 run sigma7 sigma_main
```

You should see the message published on the terminal

### 3. Launch RViz

```
ros2 launch franka_pose_control rviz.launch.py
```

You should see the FR3 model on RViz2. You can uncheck MotionPlanning option to unshow the interaction marker. 

### 4. Run pose publisher

```
ros2 run franka_pose_control franka_targetPose_publisher
```

### 5. Run planner

```
ros2 run franka_pose_control franka_pose
```

You should see the FR3 moves to the predefined positon as the initilization pose (0.28, 0, 0.6). Now you can move your sigma 7 device. The FR3 should mimic your motion. You can change the rate and frequence to decrease the delay. 

### Note:

You can make a launch file to make the process easier.
