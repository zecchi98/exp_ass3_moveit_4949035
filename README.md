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
It will start the all simulation and the final_assignment node.


# What my_opencv.py is used for?
1) It keep tracks of camera_image data to understand if a new aruco has been found.
2) It saves all the aruco inside a particular list
3) It checks if an hint has been completed and check if it is the winner one
4) It uses the rosparam server to comunicate the victory


# What robot_control.py is used for?
During the all execution of the program if the system get a victory comunication then the program will be completed
1) It will look around using the robot arm to check the aruco in the room
2) It will make the robot go the next room
3) It will continue to execute point 1) and 2) until all the rooms have been completed
4) It will try to fix some arucos
5) It will repeat the all process until all the aruco needed have been found.
