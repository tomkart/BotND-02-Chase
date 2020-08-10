# BotND-02-Chase

Project: Go Chase It!

**To run**

```
cd ~/BotND-02-Chase
catkin_make
source devel/setup.bash
roslaunch my_robot world.launch 
```
In rviz, load config file "my_robot.rviz" in "BotND-02-Chase/src/my_robot/viz"

Open another terminal

```
cd ~/BotND-02-Chase
source devel/setup.bash
roslaunch ball_chaser ball_chaser.launch
```
Move the white ball in Gazebo where the robot can see, and the robot will start chasing the white ball
