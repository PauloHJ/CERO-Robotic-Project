# CERO commands 

## Mode selection(remote PC)

Commands to select the operation mode

```
rosrun topic_tools mux_select mux_cmdvel cmd_vel_joy
rosrun topic_tools mux_select mux_cmdvel cmd_vel_auto
rosrun topic_tools mux_select mux_cmdvel cmd_vel_avoid
```

## Rviz(remote PC)

Hector Slam Rviz

```
roslaunch hector_slam_launch tutorial.launch
```

Lidar visalization RViz

```
roslaunch rplidar_ros view_rplidar.launch
```

## Lauanch files

Launch file CERO Robot(onboard Raspberry-pi)

```
roslaunch cero_robot ceroRobot.launch
```

Launch file teleoperation with controller(remote PC)

```
roslaunch cero_robot ceroProy.launch
```

Commands to start and stop job that automatically runs on boot on the rasp(onboard Raspberry-pi)

```
--Begin job that runs on boot ceroRobot.launch--
sudo systemctl start my_robot_ros ros.service


--Finish the job that runs on boot ceroRobot.launch--
sudo systemctl stop my_robot_ros.service
```

# Basic operation

### On the Onboard Raspberry

Turn on the Raspberry of the robot, it will automatically run the ceroRobot.launch on boot. This launch file launches all necesary nodes for operation.

### On the Remote PC

Run `roslaunch cero_robot ceroProy.launch` to begin the node that handles the xbox controller for teleoperation

The default mode is teleoperated mode with joy controller, to change modes use:

```
rosrun topic_tools mux_select mux_cmdvel cmd_vel_joy
rosrun topic_tools mux_select mux_cmdvel cmd_vel_auto
rosrun topic_tools mux_select mux_cmdvel cmd_vel_avoid
```

To display hector slam use

```
roslaunch hector_slam_launch tutorial.launch
```

