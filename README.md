# exp_ass3_moveit_4949035


# How to build the package?
- The package need to be inside a package with armor already installed and ready to be used.
- Clone this package into the src folder of the ros_ws
- catkin_make the workspace
- Go inside this package and execute the script "update_library.sh" by ./update_library.sh

# Is the owl file in the right position?
The package has been mainly built to work in the docker container workspace. This package need to be installed in the ros_ws workspace. Otherwise it will not work.
If you are running this package outside of that folder you will need to change manually the code. In particular, go to "my_library" folder and to the library.ph file.
Then go to the Armor_Communication class and to the load_file method.
Find the line: "req.args= ['/root/ros_ws/src/exp_robotics_ass1_4949035/cluedo_ontology.owl', 'http://www.emarolab.it/cluedo-ontology', 'true', 'PELLET', 'true']"
Modify the first argument of this square brackets, by inserting your path to the cluedo file.
After saving this modification you will have to buil the library.
In the main folder execute the script "update_library.sh" by ./update_library.sh

# How to run the code?
- First tab: roscore
- Second tab: rosrun armor execute it.emarolab.armor.ARMORMainService
- Third tab: roslaunch exp_robotics_ass1_4949035 launcher.launch


# In which language is the project written?
Python

#Missing points:
global ros variable to comunicate after victory to close all windows
