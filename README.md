<p align="center">
  <h2 align="center">MONITORING ROBOT FOR RADIOLOGICAL FACILITIES</h2>
  <p align="center">Davy Rojas Yana - Universidad Católica Boliviana</p>
  </p>
  
  <p align="center">
<img src="https://user-images.githubusercontent.com/11477020/237125504-5a48e328-3fd0-48c8-bfad-4f8f253e253f.PNG">
</p>

### Description
This repository stores the files of the monitoring robot that is a Turtlebot-3. The Turtlebot-3 uses its Lidar sensor to map their environment, then it uses the generated map to navigate autonomously.
Using SLAM.
<p align="center">
<img src="https://user-images.githubusercontent.com/11477020/237125512-700f7525-8f68-4f9d-b02e-a0365d483967.png" width="600" height="600">
<img src="https://user-images.githubusercontent.com/11477020/237125750-5a905833-c7a8-49d1-9089-5b9b884e4e07.PNG">
  </p>
  
### Setup

First Install:
- `Ros Noetic:` http://wiki.ros.org/noetic/Installation

Then Clone this repo:
```
git clone https://github.com/Davydero/monitoring-robot
```
Launch the world
```
roslaunch auto_nav my_world.launch
```
Seting our personalized world:
```
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/lab2.yaml
```
Run the pyhton script for initialize the pose of the robot respect to map:
```
rosrun auto_nav init_pose.py
```
Python script for the navigation algoritm:
```
rosrun auto_nav goal_pose_lab.py
```
Python script for UI:
```
rosrun auto_nav gamma.py
```
