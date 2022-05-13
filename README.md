# exp_ass3_moveit_4949035


# How to build the package?
-   exp_assignment3 and erl2 should be installed
-   if some moveit package are missing when you will run the code then you should: sudo apt update     and    sudo apt install ros-noetic-moveit    
-   You should put all the aruco models inside exp_assignment3/models into .gazebo/models folder
-   catkin_make


# How to run the code?
- First tab: roslaunch exp_ass3_moveit_4949035 my_launch_file_moveit.launch
- Second tab: rosrun exp_ass3_moveit_4949035 my_opencv.py
- Third tab: rosrun exp_ass3_moveit_4949035 robot_control.py


# What my_launch_file_moveit.launch is used for?
It will start all the simulation and the final_assignment node.


# What my_opencv.py is used for?
1) It keep tracks of camera_image data to understand if a new aruco has been found.
2) It saves all the aruco found inside a particular list
3) It checks if there is an hint that has been completed and check if it is the winner one
4) It uses the rosparam server to comunicate the victory


# What robot_control.py is used for?
During the all execution of the program if the system get a victory comunication then the program will be completed
1) It will look around using the robot arm to check the aruco in the room
2) It will make the robot go to the next room
3) It will continue to execute point 1) and 2) until all the rooms have been completed
4) It will try to fix some arucos
5) It will repeat all the process until all the aruco needed have been found.

# Action
Move base creates automatically an action server where you can comunicate your goal. This action is used by robot control to set a goal
  
# Param
"/victory": this rosparam is used to comunicate if the game has ended. It will be set by my_opencv and read from robot_control


# Subscriber
"/odom": this topic is used from the robot_control node to check if the robot is arrived in the correct position
"/camera/color/image_raw": this topic is used from the my_opencv node to read the camera image

# Clients
"/oracle_hint": this topic is used for a service client comunication. In particular from the "my_opencv" node i can get the hints after finding a new aruco id
"/oracle_solution": this topic is used for a service client comunication. In particular from the "my_opencv" node i can get check which one is the winner id.

# State diagram
![state_diagram](https://user-images.githubusercontent.com/78590047/167359843-8cf1058e-fe37-4062-99a4-dd46e17313c7.png)
As shown in the diagram the robot will continue to look around and change the room to find new arucos. It will continue to repeat this until it has found all the arucos.

# Software architecture
![software_architecture](https://user-images.githubusercontent.com/78590047/167359808-2939db8f-28d4-4348-a802-129d8917c8d5.png)
Robot control node will force the robot to move to a new room or to look around. In the meanwhile my_opencv will continue to read image from the sensImg topic, in this way they are pretty much separeted. Once the my_opencv node find the winner solution it will comunicate the victory in the "/victory" topic.



# Video example

You can find a video example in the Video Example folder, you will find it compressed.