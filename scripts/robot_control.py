#!/usr/bin/env python
#fold all: ctrl + k + 0
#unfold all: ctrl + k + j
import copy
import math
import sys
import time
from logging import setLoggerClass
from math import cos, pi, sin
from os import access
from re import X

import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
import numpy as np
import rospy
from moveit_commander import *
from moveit_commander.conversions import pose_to_list
from moveit_commander.move_group import MoveGroupCommander
from moveit_commander.robot import RobotCommander
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from exp_ass3_moveit_4949035.srv import *


  
flagMiddlePanelCreated=False
bool_exit=False
class Transformation_class():
  def __init__(self):
        null=0
  def rad_to_grad(self,angle):
    return angle*180/3.1415
  def grad_to_rad(self,angle):
    return angle*3.1415/180
  def rot2eul(self,R):
    beta = -np.arcsin(R[2,0])
    alpha = np.arctan2(R[2,1]/np.cos(beta),R[2,2]/np.cos(beta))
    gamma = np.arctan2(R[1,0]/np.cos(beta),R[0,0]/np.cos(beta))
    return np.array((alpha, beta, gamma))
  def rpy_from_quat (self,quaternion):
    orientation_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    return [roll,pitch,yaw]
  def Rotation_from_quat(self,quaternion):
    
    euler_angles=self.rpy_from_quat(quaternion)
    return self.eul2rot(euler_angles)
  def eul2rot(self,theta) :

    R = np.array([[np.cos(theta[1])*np.cos(theta[2]),       np.sin(theta[0])*np.sin(theta[1])*np.cos(theta[2]) - np.sin(theta[2])*np.cos(theta[0]),      np.sin(theta[1])*np.cos(theta[0])*np.cos(theta[2]) + np.sin(theta[0])*np.sin(theta[2])],
                  [np.sin(theta[2])*np.cos(theta[1]),       np.sin(theta[0])*np.sin(theta[1])*np.sin(theta[2]) + np.cos(theta[0])*np.cos(theta[2]),      np.sin(theta[1])*np.sin(theta[2])*np.cos(theta[0]) - np.sin(theta[0])*np.cos(theta[2])],
                  [-np.sin(theta[1]),                        np.sin(theta[0])*np.cos(theta[1]),                                                           np.cos(theta[0])*np.cos(theta[1])]])

    return R
  def Rotation_matrix_of_Affine_matrix(self,aff_matrix):
    R=np.zeros([3,3])
    for r in range(3):
      for c in range(3):
        R[r,c]=aff_matrix[r,c]
    return R
  def Translation_vector_of_Affine_matrix(self,AffineMat):
    vet=np.zeros(3)
    for r in range(3):
      vet[r]=AffineMat[r,3]
    return vet
  def create_affine_matrix_from_rotation_matrix_and_translation_vector(self,R,transl):
    #input: una matrice di rotazione e un vettore riga di traslazione"[0,1,3]"
    #return the result affine matrix
    AffineMat=np.zeros([4,4])
    
    #copio matrice di rotazione
    for r in range(3):
      for c in range(3):
        AffineMat[r,c]=R[r,c]

    #copio vettore traslazione
    AffineMat[0,3]=transl[0]
    AffineMat[1,3]=transl[1]
    AffineMat[2,3]=transl[2]

    #setto ultima riga standard
    AffineMat[3,0]=0
    AffineMat[3,1]=0
    AffineMat[3,2]=0
    AffineMat[3,3]=1

    return AffineMat
  def from_euler_to_quaternion(self,euler_vet):
    #Input: vettore degli angoli di eulero
    #Output: Pose with the correct orientation in quaternion
    roll=euler_vet[0]
    pitch=euler_vet[1]
    yaw=euler_vet[2]
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    pose=Pose()
    pose.orientation.x=qx
    pose.orientation.y=qy
    pose.orientation.z=qz
    pose.orientation.w=qw
    
    return pose 
  def from_rotation_to_quaternion(self,R):
    #Input: Rotation matrix
    #Output: Pose with the correct orientation in quaternion
    euler_vet=self.rot2eul(R)
    pose_oriented=self.from_euler_to_quaternion(euler_vet)
    return pose_oriented
  def from_vet_to_posePosition(self,vet):
    #Input: vettore contentente la posizione di un frame
    #Output: Pose con la corretta position
    pose=Pose()
    pose.position.x=vet[0]
    pose.position.y=vet[1]
    pose.position.z=vet[2]
    return pose
  def from_pose_to_matrix(self,pose_):
    R=self.Rotation_from_quat(pose_.orientation)
    vet=[pose_.position.x,pose_.position.y,pose_.position.z]
    AffineMat=self.create_affine_matrix_from_rotation_matrix_and_translation_vector(R,vet)
    return AffineMat
  def from_matrix_to_pose(self,AffineMat):
    #Input: Affine matrix
    #Output: Pose
    pose=Pose()
    
    R=self.Rotation_matrix_of_Affine_matrix(AffineMat)
    translation_vet=self.Translation_vector_of_Affine_matrix(AffineMat)
    
    #pose with the correct position
    pose_position=self.from_vet_to_posePosition(translation_vet)

    #pose with the correct orientation
    pose_orientation=self.from_rotation_to_quaternion(R)

    pose.orientation=pose_orientation.orientation
    pose.position=pose_position.position
    return pose
  def inverse_matrix(self,AffineMat):
    return np.linalg.inv(AffineMat)
class Collision_Box():
    def __init__(self):
        self.box_pose = PoseStamped()
        self.box_size = np.zeros(3)
        self.box_name = String()  
class Affine_valid():
    def __init__(self):
        self.is_valid = False
        self.Affine_matrix = np.zeros((4,4))
class Pose_valid():
    def __init__(self):
        self.is_valid = False
        self.pose = Pose()
class Move_group_class(object):
  """Move_group_class"""
  def __init__(self):
    super(Move_group_class, self).__init__()

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    #rospy.init_node('state_machine', anonymous=True)

    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    
    robot = RobotCommander()
    
    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the
    ## surrounding world:
    scene = PlanningSceneInterface()
    
    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints).  In this tutorial the group is the primary
    ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
    ## If you are using a different robot, change this value to the name of your robot
    ## arm planning group.
    ## This interface can be used to plan and execute motions:
    group_name = "arm"
    
    move_group =MoveGroupCommander(group_name)
    
    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## END_SUB_TUTORIAL

    ## BEGIN_SUB_TUTORIAL basic_info
    ##
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    
    print ("============ Planning frame: %s" + planning_frame)

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print ("============ End effector link: %s" + eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    #print "============ Available Planning Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    #print "============ Printing robot state"
    #print robot.get_current_state()
    #print ""
    ## END_SUB_TUTORIAL
    
    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
  def all_close(self,goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    all_equal = True
    if type(goal) is list:
      for index in range(len(goal)):
        if abs(actual[index] - goal[index]) > tolerance:
          return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
      return self.all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
      return self.all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True
  def ruota_giunto(self,id_giunto,angle):
    joints=self.get_joints_values()
    joints[id_giunto]=joints[id_giunto]+angle
    
    self.go_to_joint_state(joints)
  def go_to_joint_state(self,joints_vet):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_to_joint_state
    ##
    ## Planning to a Joint Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^^
    ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
    ## thing we want to do is move it to a slightly better configuration.
    # We can get the joint values from the group and adjust some of the values:
    

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    move_group.go(joints_vet, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()

    ## END_SUB_TUTORIAL

    # For testing:
    current_joints = move_group.get_current_joint_values()
    return self.all_close(joints_vet, current_joints, 0.01)
  def go_to_pose_goal(self,pose_goal):
    move_group = self.move_group
    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    # ## end-effector:
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.w = 1.0
    # pose_goal.position.x = 0.4
    # pose_goal.position.y = 0.1
    # pose_goal.position.z = 0.4
    
    
    move_group.set_pose_target(pose_goal)
    
    ## Now, we call the planner to compute the plan and execute it.
    plan = move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    move_group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.move_group.get_current_pose().pose
    return self.all_close(pose_goal, current_pose, 0.01)
  def go_to_pose_cartesian(self,pose_goal):
  
    move_group = self.move_group

    waypoints = []

    waypoints.append(copy.deepcopy(pose_goal))

    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    self.execute_plan(plan)
    self.display_trajectory(plan)

    return plan, fraction

    ## END_SUB_TUTORIAL
  def plan_cartesian_path(self, scale=1):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_cartesian_path
    ##
    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through. If executing  interactively in a
    ## Python shell, set scale = 1.0.
    ##
    waypoints = []

    wpose = move_group.get_current_pose().pose
    wpose.position.z -= scale * 0.1  # First move up (z)
    wpose.position.y += scale * 0.2  # and sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= scale * 0.1  # Third move sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

    ## END_SUB_TUTORIAL
  def display_trajectory(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    ## BEGIN_SUB_TUTORIAL display_trajectory
    ##
    ## Displaying a Trajectory
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
    ## group.plan() method does this automatically so this is not that useful
    ## here (it just displays the same trajectory again):
    ##
    ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
    ## We populate the trajectory_start with our current robot state to copy over
    ## any AttachedCollisionObjects and add our plan to the trajectory.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory);

    ## END_SUB_TUTORIAL
  def execute_plan(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL execute_plan
    ##
    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    move_group.execute(plan, wait=True)

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
    ## END_SUB_TUTORIAL
  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL wait_for_scene_update
    ##
    ## Ensuring Collision Updates Are Receieved
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## If the Python node dies before publishing a collision object update message, the message
    ## could get lost and the box will not appear. To ensure that the updates are
    ## made, we wait until we see the changes reflected in the
    ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
    ## For the purpose of this tutorial, we call this function after adding,
    ## removing, attaching or detaching an object in the planning scene. We then wait
    ## until the updates have been made or ``timeout`` seconds have passed
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False
    ## END_SUB_TUTORIAL
  def add_box(self,collision_box, timeout=0):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    #box_name = self.box_name
    #print('adding box')
    #print(box_name)
    #print(box_size)
    #print(box_pose)
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL add_box
    ##
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene at the location of the left finger:
    #box_pose = geometry_msgs.msg.PoseStamped()
    #box_pose.header.frame_id = "base_link"
    #box_pose.pose.orientation.w = 1.0
    #box_pose.pose.position.x = -0.1 # slightly above the end effector
    #box_name = "box"
    size=(collision_box.box_size[0],collision_box.box_size[1],collision_box.box_size[2])
    scene.add_box(collision_box.box_name, collision_box.box_pose, size)
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)
  def attach_box(self,box_pose,box_name,box_size, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    robot = self.robot
    scene = self.scene
    eef_link = self.eef_link
    group_names = self.group_names

    ## BEGIN_SUB_TUTORIAL attach_object
    ##
    ## Attaching Objects to the Robot
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
    ## robot be able to touch them without the planning scene reporting the contact as a
    ## collision. By adding link names to the ``touch_links`` array, we are telling the
    ## planning scene to ignore collisions between those links and the box. For the Panda
    ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
    ## you should change this value to the name of your end effector group name.
    grasping_group = 'manipulator'
    #touch_links = robot.get_link_names()
    touch_links=['wrist_3_link', 'ee_link', 'tool0', 'camera_ur_mount', 'camera_link1', 'camera_link', 'camera_camera_lens', 'camera_camera', 'camera_camera_gazebo', 'robotiq_arg2f_base_link', 'left_outer_knuckle', 'left_outer_finger', 'left_inner_finger', 'left_finger_tip_temp', 'left_finger_tip', 'left_inner_finger2', 'left_inner_knuckle', 'left_inner_knuckle2', 'plate1', 'dys_middle', 'right_inner_knuckle', 'right_inner_knuckle2', 'right_outer_knuckle', 'right_outer_finger', 'right_inner_finger', 'right_finger_tip_temp', 'right_finger_tip', 'right_inner_finger2']

    #print(touch_links)
    scene.attach_box(eef_link, box_name, touch_links=touch_links)
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)
  def detach_box(self,box_pose,box_name,box_size, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    scene = self.scene
    eef_link = self.eef_link

    scene.remove_attached_object(eef_link, name=box_name)
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)
  def remove_box(self,box_name,timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    #box_name = self.box_name
    scene = self.scene
    ## BEGIN_SUB_TUTORIAL remove_object
    ##
    ## Removing Objects from the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## We can remove the box from the world.
    scene.remove_world_object(box_name)
    ## **Note:** The object must be detached before we can remove it from the world
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)
  def get_joints_values(self):
    joints_grad=movegroup_library.move_group.get_current_joint_values()
    # joints_rad=np.zeros(6)
    # for i in range(6):
    #   joints_rad[i]=transformation_library.grad_to_rad(joints_grad[i])
    return joints_grad
  def FermaRobot(self):
    print("Il robot sta per essere fermato")
    self.move_group.stop()
    self.move_group.clear_pose_targets()
  def Stampa_Info_Robot(self):
    print("\n\nRobot Status")
    pose=self.move_group.get_current_pose().pose
    print("Pose:")
    print(pose)

    joints=self.get_joints_values()
    print("\nJoints:")
    print(joints)
    
    rpy=transformation_library.rpy_from_quat(pose.orientation)
    print("\nRpy:")
    print(rpy)
    print("\n\nActive joints:")
    print(self.move_group.get_active_joints())
    aruco_library.print_situation_aruco()
class Comunication_class(object):
  def __init__(self):
    super(Comunication_class, self).__init__()
    rospy.wait_for_service('cv_server')

    self.bridge_service_information = rospy.ServiceProxy('cv_server', cv_server)
  def call_bridge_service(self,first_information,second_information):
    print("Deprecated")
    return self.read_data_from_cv()
    # try:
    #       req=aruco_serviceRequest()
    #       req.modality=first_information
    #       req.second_information=second_information

    #       msg = self.bridge_service_information(req)

    #       resp=aruco_serviceResponse()
    #       resp.x=msg.x
    #       resp.y=msg.y
    #       resp.z=msg.z
    #       resp.all_aruco_found=msg.all_aruco_found
    #       resp.aruco_found=msg.aruco_found
    #       resp.dim=msg.dim
    #       resp.finger_joint=msg.finger_joint
    #       resp.id_aruco=msg.id_aruco
    #       resp.moreTargets=msg.moreTargets
    #       resp.vector=msg.vector

    #       #self.print_bridge_response(resp)
          
    #       return resp
    # except rospy.ServiceException as e:
      #print("Service call failed: %s"%e)
  def call_cv_service(self,first_information,second_information):
    
    try:
          req=cv_serverRequest()
          req.message=first_information
          req.second_information=second_information

          msg = self.bridge_service_information(req)

    except rospy.ServiceException as e:
      print("Service call failed: %s"%e)
  def print_bridge_response(self,response):
    
    print("Aruco trovato:"+str(response.aruco_found))
    print("ID Aruco:"+str(response.id_aruco))
  def ask_matrix_camera_aruco(self):
    ValidMatrix=Affine_valid()
    msg=self.read_data_from_cv()
    if not msg.aruco_found:
      ValidMatrix.is_valid=False
      return ValidMatrix
    ValidMatrix.is_valid=True
    translation_vet=[msg.x,msg.y,msg.z]
    R=np.matrix([[msg.vector[0],msg.vector[1],msg.vector[2]],[msg.vector[3],msg.vector[4],msg.vector[5]],[msg.vector[6],msg.vector[7],msg.vector[8]]])
    
    AffineMat=transformation_library.create_affine_matrix_from_rotation_matrix_and_translation_vector(R,translation_vet)
    ValidMatrix.Affine_matrix=AffineMat
    return ValidMatrix
  def read_data_from_cv(self):
    try:
      msg=rospy.wait_for_message("/aruco_bridge_opencv",cv_to_bridge)
      resp=cv_to_bridge()
      resp.aruco_found=msg.aruco_found
      resp.pose_camera_aruco=msg.pose_camera_aruco
      return resp
    except:
      print("error reading from cv")
  def matrix_from_cv_of_specified_aruco(self,id_aruco):
    data=self.read_data_from_cv()
    ValidMatrix=Affine_valid()
    if(data.aruco_found[int(id_aruco)]):
      ValidMatrix.is_valid=True
      ValidMatrix.Affine_matrix=transformation_library.from_pose_to_matrix(data.pose_camera_aruco[int(id_aruco)])
    else:
      ValidMatrix.is_valid=False
    return ValidMatrix
    
      

class Movimenti_base_class(object):
  def __init__(self):
      super(Movimenti_base_class, self).__init__()
  def go_to_initial_position(self):
    joint_vet=movegroup_library.move_group.get_active_joints()
    joint_vet[0]=transformation_library.grad_to_rad(0)
    joint_vet[1]=transformation_library.grad_to_rad(-120)
    joint_vet[2]=transformation_library.grad_to_rad(100)
    joint_vet[3]=transformation_library.grad_to_rad(20)
    joint_vet[4]=transformation_library.grad_to_rad(90)
    joint_vet[5]=transformation_library.grad_to_rad(90)
    movegroup_library.go_to_joint_state(joint_vet)
class Aruco_class():
    def __init__(self):
      self.Button_1=Pose_valid()
      self.Button_2=Pose_valid()
      self.Button_3=Pose_valid()
      self.Button_4=Pose_valid()
      self.Button_5=Pose_valid()
      self.Button_6=Pose_valid()
      self.Button_7=Pose_valid()
      self.Button_8=Pose_valid()
      self.Button_9=Pose_valid()
      self.IMU_Module=Pose_valid()
      self.IMU_Destination=Pose_valid()
      self.Inspection_Window=Pose_valid()
      self.Inspection_Window_Cover=Pose_valid()
      self.Inspection_Window_Cover_Storage=Pose_valid()
      self.number_of_aruco=14
    def elaborate_number_of_aruco_found(self):
      cont=0
      for id in range(1,self.number_of_aruco+1):
        id_str=str(id)
        Aruco_valid=self.from_id_to_pose_valid_of_the_aruco(id_str)
        if(Aruco_valid.is_valid):
          cont=cont+1
      return cont
      """
      if(self.Button_1.is_valid):
        cont=cont+1
      if(self.Button_2.is_valid):
        cont=cont+1
      if(self.Button_3.is_valid):
        cont=cont+1
      if(self.Button_4.is_valid):
        cont=cont+1
      if(self.Button_5.is_valid):
        cont=cont+1
      if(self.Button_6.is_valid):
        cont=cont+1
      if(self.Button_7.is_valid):
        cont=cont+1
      if(self.Button_8.is_valid):
        cont=cont+1
      if(self.Button_9.is_valid):
        cont=cont+1
      if(self.IMU_Module.is_valid):
        cont=cont+1
      if(self.IMU_Destination.is_valid):
        cont=cont+1
      if(self.Inspection_Window.is_valid):
        cont=cont+1
      if(self.Inspection_Window_Cover.is_valid):
        cont=cont+1
      if(self.Inspection_Window_Cover_Storage.is_valid):
        cont=cont+1
      """
    def print_situation_aruco(self):
      tot=self.elaborate_number_of_aruco_found()
      print("\nTotal aruco found:"+str(tot))
    def from_id_to_pose_valid_of_the_aruco(self,id):
      #INPUT: id dell aruco in stringa(per esempio, il Button1 ha id="1")
      #Output: Pose_valid classe, dove puoi controllare se l aruco e stato trovato e qual'e il suo valore
      
      if(id=="1"):
        return self.Button_1
      if(id=="2"):
        return self.Button_2
      if(id=="3"):
        return self.Button_3
      if(id=="4"):
        return self.Button_4
      if(id=="5"):
        return self.Button_5
      if(id=="6"):
        return self.Button_6
      if(id=="7"):
        return self.Button_7
      if(id=="8"):
        return self.Button_8
      if(id=="9"):
        return self.Button_9
      if(id=="10"):
        return self.IMU_Module
      if(id=="11"):
        return self.IMU_Destination
      if(id=="12"):
        return self.Inspection_Window
      if(id=="13"):
        return self.Inspection_Window_Cover
      if(id=="14"):
        return self.Inspection_Window_Cover_Storage
      print("NO aruco corresponds to your request")
      return Pose_valid()
    def set_new_aruco(self,id,pose_aruco):
      

      if(id=="1"):
        self.Button_1.is_valid=True
        self.Button_1.pose=pose_aruco
      if(id=="2"):
        self.Button_2.is_valid=True
        self.Button_2.pose=pose_aruco
      if(id=="3"):
        self.Button_3.is_valid=True
        self.Button_3.pose=pose_aruco
      if(id=="4"):
        self.Button_4.is_valid=True
        self.Button_4.pose=pose_aruco
      if(id=="5"):
        self.Button_5.is_valid=True
        self.Button_5.pose=pose_aruco
      if(id=="6"):
        self.Button_6.is_valid=True
        self.Button_6.pose=pose_aruco
      if(id=="7"):
        self.Button_7.is_valid=True
        self.Button_7.pose=pose_aruco
      if(id=="8"):
        self.Button_8.is_valid=True
        self.Button_8.pose=pose_aruco
      if(id=="9"):
        self.Button_9.is_valid=True
        self.Button_9.pose=pose_aruco
      if(id=="10"):
        self.IMU_Module.is_valid=True
        self.IMU_Module.pose=pose_aruco
      if(id=="11"):
        self.IMU_Destination.is_valid=True
        self.IMU_Destination.pose=pose_aruco
      if(id=="12"):
        self.Inspection_Window.is_valid=True
        self.Inspection_Window.pose=pose_aruco
      if(id=="13"):
        self.Inspection_Window_Cover.is_valid=True
        self.Inspection_Window_Cover.pose=pose_aruco
      if(id=="14"):
        self.Inspection_Window_Cover_Storage.is_valid=True
        self.Inspection_Window_Cover_Storage.pose=pose_aruco
    def cambia_id_aruco_selezionato(self,ID):
      res=comunication_object.call_bridge_service("md_next_aruco",ID) 
      for i in range(100):
        time.sleep(0.1)
        res=comunication_object.call_bridge_service("","")
        if str(res.id_aruco)==ID:
          return True
          
      return False
    def is_aruco_visible_from_camera(self,ID):
      #Restituisce se l'aruco e' attualmente visibile dalla camera
      res=comunication_object.call_bridge_service("md_next_aruco",ID)
      if str(res.id_aruco)==ID:
        return res.aruco_found
      else:
        self.cambia_id_aruco_selezionato(ID)
        res=comunication_object.call_bridge_service("md_next_aruco",ID)
        return res.aruco_found
    def elaborate_affine_matrix_of_visible_aruco(self,ID):
      target=Pose_valid()
      T_camera_gaz_aruco_valid=comunication_object.matrix_from_cv_of_specified_aruco(ID)
      if not T_camera_gaz_aruco_valid.is_valid :
        target.is_valid=False
        #print("Non ho trovato nessun aruco")
        return target

      target.is_valid=True
      T_camera_gaz_aruco=T_camera_gaz_aruco_valid.Affine_matrix

      actual_pose=movegroup_library.move_group.get_current_pose().pose
      T_0_tool=transformation_library.from_pose_to_matrix(actual_pose)
      T_0_aruco=T_0_tool*T_tool_camera_gazebo*T_camera_gaz_aruco

      target.pose=transformation_library.from_matrix_to_pose(T_0_aruco)  


      return target
    def salva_aruco_visibili_OOOLD(self):
      nul=0
      #   self.print_situation_aruco()

      #   res=comunication_object.call_bridge_service("","")
      #   for id in range(1,len(res.aruco_found)):

      #     id_str=str(id)
      #     if res.aruco_found[id]:#Controllo che l'aruco sia stato trovato
            
      #       collision_to_be_done=False
      #       if not self.from_id_to_pose_valid_of_the_aruco(id_str).is_valid: #controllo se ho mai trovato questo aruco, in caso negativo aggiungo la collisione che ne deriva
      #         collision_to_be_done=True
  
      #       aruco_valid=self.elaborate_affine_matrix_of_visible_aruco(id_str)#mi calcolo l'aruco
      #       if aruco_valid.is_valid:#se ho trovato l'aruco
      #         self.set_new_aruco(id_str,aruco_valid.pose)
      #         print("Salvo aruco:"+id_str)

      #         if collision_to_be_done:
      #           self.add_collision_from_aruco(id_str)#aggiungo la collisione corrispondente
    def save_visible_arucos(self):
      self.print_situation_aruco()
      for id in range(1,16):
        id_str=str(id)
        mat_valid=comunication_object.matrix_from_cv_of_specified_aruco(id)
        if mat_valid.is_valid:
          collision_to_be_done=False
          if not self.from_id_to_pose_valid_of_the_aruco(id_str).is_valid: #controllo se ho mai trovato questo aruco, in caso negativo aggiungo la collisione che ne deriva
            collision_to_be_done=True
 
          aruco_valid=self.elaborate_affine_matrix_of_visible_aruco(id_str)#mi calcolo l'aruco
          if aruco_valid.is_valid:#se ho trovato l'aruco
            self.set_new_aruco(id_str,aruco_valid.pose)
            print("Salvo aruco:"+id_str)

            if collision_to_be_done:
              self.add_collision_from_aruco(id_str)#aggiungo la collisione corrispondente
      print("Finish saving arucos")
    def add_collision_from_aruco(self,ID):
      global flagMiddlePanelCreated
      if (ID=="2" or ID=="5" or ID=="8") and not(flagMiddlePanelCreated):
        collision_box=Collision_Box()

        if not aruco_library.from_id_to_pose_valid_of_the_aruco(ID).is_valid:
          return False

        flagMiddlePanelCreated=True
        collision_box.box_name="middle_panel"

        collision_box.box_size[0]=0.001
        collision_box.box_size[1]=0.3
        collision_box.box_size[2]=3*0.5
        collision_box.box_pose.header.frame_id="base_link"

        pose_orientation=transformation_library.from_euler_to_quaternion([0,0,0])
        collision_box.box_pose.pose.orientation=pose_orientation.orientation
        
        collision_box.box_pose.pose.position.x=aruco_library.from_id_to_pose_valid_of_the_aruco(ID).pose.position.x+collision_box.box_size[0]/2
        collision_box.box_pose.pose.position.y=aruco_library.from_id_to_pose_valid_of_the_aruco(ID).pose.position.y
        collision_box.box_pose.pose.position.z=aruco_library.from_id_to_pose_valid_of_the_aruco(ID).pose.position.z
        """
        collision_boxes[box_name].name=box_name;
        collision_boxes[box_name].pose=box_pose;
        collision_boxes[box_name].size[0]=box_size[0];
        collision_boxes[box_name].size[1]=box_size[1];
        collision_boxes[box_name].size[2]=box_size[2];
        """
        movegroup_library.add_box(collision_box)
      if (ID>="1" and ID<="9"):
        coll_box=Collision_Box()
        pose_aruco_valid=aruco_library.from_id_to_pose_valid_of_the_aruco(ID)

        if not pose_aruco_valid.is_valid:    
          return False

        pose_0_aruco=pose_aruco_valid.pose
        Rot=transformation_library.eul2rot([0,0,0])
        T_aruco_finalpose=transformation_library.create_affine_matrix_from_rotation_matrix_and_translation_vector(Rot,[0,-0.055,0])
        T_0_aruco=transformation_library.from_pose_to_matrix(pose_0_aruco)
        T_0_finalpose=T_0_aruco.dot(T_aruco_finalpose)


        pose_final=transformation_library.from_matrix_to_pose(T_0_finalpose)

        coll_box.box_name="button_"+ID

        coll_box.box_size[0]=0.023
        coll_box.box_size[1]=0.04
        coll_box.box_size[2]=0.04

        coll_box.box_pose.header.frame_id="base_link"

        pose_orient=transformation_library.from_euler_to_quaternion([0,0,0])
        coll_box.box_pose.pose.orientation=pose_orient.orientation
        
        
        coll_box.box_pose.pose.position.x=pose_final.position.x-coll_box.box_size[0]/2
        coll_box.box_pose.pose.position.y=pose_final.position.y
        coll_box.box_pose.pose.position.z=pose_final.position.z
        
        """
        collision_boxes[box_name].name=box_name;
        collision_boxes[box_name].pose=box_pose;
        collision_boxes[box_name].size[0]=box_size[0];
        collision_boxes[box_name].size[1]=box_size[1];
        collision_boxes[box_name].size[2]=box_size[2];
        """
        movegroup_library.add_box(coll_box)
      

#Funzioni da finire
def move_gripper(gripper_goal):
  print("Per ora non esiste")

def handle_move_arm_service(req):
  print("Server request received")
  nul=0
  response=comunicationResponse()
  print(req)

    
  
  vect1=movegroup_library.get_joints_values()

  vect2=movegroup_library.get_joints_values()


  vect1[1]=transformation_library.grad_to_rad(-25)
  vect1[2]=transformation_library.grad_to_rad(-17)
  vect1[3]=transformation_library.grad_to_rad(32)

  
  vect2[1]=transformation_library.grad_to_rad(-17)
  vect2[2]=transformation_library.grad_to_rad(-90)
  vect2[3]=transformation_library.grad_to_rad(32)

  if (req.msg1=="wp1"):
    vect1[0]=transformation_library.grad_to_rad(77)
    
    vect2[0]=transformation_library.grad_to_rad(77)

  if (req.msg1=="wp2"):
    vect1[0]=transformation_library.grad_to_rad(107)
    vect2[0]=transformation_library.grad_to_rad(107)
    
  if (req.msg1=="wp3"):
    vect1[0]=transformation_library.grad_to_rad(84)
    vect2[0]=transformation_library.grad_to_rad(84)
  if (req.msg1=="wp4"):

    vect1[0]=transformation_library.grad_to_rad(93)
    vect2[0]=transformation_library.grad_to_rad(93)

  movegroup_library.move_group.set_start_state_to_current_state
  movegroup_library.go_to_joint_state(vect1)


  movegroup_library.move_group.set_start_state_to_current_state
  movegroup_library.go_to_joint_state(vect2)

  response.response="ok"
  response.success=True
  response.data=[]
  return response

#std functions
def callback_user_interface(msg):
  global bool_message_from_user,msg_from_user
  msg_from_user=UserInterfaceRequest()
  msg_from_user.modality=msg.modality
  msg_from_user.second_information=msg.second_information
  msg_from_user.target_pose=msg.target_pose
  msg_from_user.target_joints=msg.target_joints
  bool_message_from_user=True
  return UserInterfaceResponse()
def handle_user_request():
  global bool_message_from_user
  bool_message_from_user=False
  print("Msg received:"+msg_from_user.modality)
  if msg_from_user.modality=="automazione_go_pos_iniziale":
    movimenti_base_library.go_to_initial_position()
  if msg_from_user.modality=="controlla_gripper":
    move_gripper(msg_from_user.second_information)
  if msg_from_user.modality=="joystick":
    handle_joystick_input(msg_from_user.second_information)
  if msg_from_user.modality=="Stampa_pose_robot":
    movegroup_library.Stampa_Info_Robot()
  if msg_from_user.modality=="stop_trajectory":
    movegroup_library.FermaRobot()
  if msg_from_user.modality=="salva_aruco":
    aruco_library.save_visible_arucos()    
  if msg_from_user.modality=="turn_on_off_camera":
    comunication_object.call_cv_service("turn_on_off_camera","")
def handle_joystick_input(input):
  global joystick_verso_rotazione,joystick_translation_step,joystick_angle_step
  #q w -> asse x
  #a s -> asse y
  #z x -> asse z
  #o p -> rotazione asse x
  #k l -> rotazione asse y
  #n m -> rotazione asse z

  #if bool_pose_move==True allora si dovra effettuare un movimento go_to_pose
  #if bool_joint_move==True allora si dovra effettuare un movimento go_to_joint_state
  
  bool_pose_move=False
  bool_joint_move=False
  
  actual_pose=movegroup_library.move_group.get_current_pose().pose
  target_pose=actual_pose
  actual_rpy_vet=transformation_library.rpy_from_quat(actual_pose.orientation)
  target_rpy_vet=actual_rpy_vet

  if(input=="q"):
    target_pose.position.x=actual_pose.position.x+joystick_translation_step
    bool_pose_move=True
  if(input=="w"):
    bool_pose_move=True
    target_pose.position.x=actual_pose.position.x-joystick_translation_step
  if(input=="a"):
    bool_pose_move=True
    target_pose.position.y=actual_pose.position.y+joystick_translation_step
  if(input=="s"):
    bool_pose_move=True
    target_pose.position.y=actual_pose.position.y-joystick_translation_step
  if(input=="z"):
    bool_pose_move=True
    target_pose.position.z=actual_pose.position.z+joystick_translation_step
  if(input=="x"):
    bool_pose_move=True
    target_pose.position.z=actual_pose.position.z-joystick_translation_step
  if(input=="o"):
    bool_pose_move=True
    target_rpy_vet[0]=actual_rpy_vet[0]+joystick_angle_step
  if(input=="p"):
    bool_pose_move=True
    target_rpy_vet[0]=actual_rpy_vet[0]-joystick_angle_step
  if(input=="k"):
    bool_pose_move=True
    target_rpy_vet[1]=actual_rpy_vet[1]+joystick_angle_step
  if(input=="l"):
    bool_pose_move=True
    target_rpy_vet[1]=actual_rpy_vet[1]-joystick_angle_step
  if(input=="n"):
    bool_pose_move=True
    target_rpy_vet[2]=actual_rpy_vet[2]+joystick_angle_step
  if(input=="m"):
    bool_pose_move=True
    target_rpy_vet[2]=actual_rpy_vet[2]-joystick_angle_step

  if(input>="1" and input<="6"):
    #trasforma char in numero tramite ascii
    num=ord(input)-ord("1")+1
    angle_grad_step=transformation_library.rad_to_grad(joystick_angle_step)
    movegroup_library.ruota_giunto(num-1,joystick_angle_step*joystick_verso_rotazione)

  if(input=="0"):
    joystick_verso_rotazione=(-1)*(joystick_verso_rotazione)



  if bool_pose_move:
    quaternion_target=transformation_library.from_euler_to_quaternion(target_rpy_vet)
    target_pose.orientation=quaternion_target.orientation
    
    movegroup_library.go_to_pose_goal(target_pose)



def define_all_initial_functions():
    global movegroup_library,comunication_object,transformation_library,movimenti_base_library,aruco_library
    global joystick_verso_rotazione,joystick_angle_step,joystick_translation_step,bool_message_from_user

    rospy.init_node('state_machine', anonymous=True)
    movegroup_library = Move_group_class()
    transformation_library=Transformation_class()
    bool_exit=False
    prova()

def prova():
    nul=0

def main():
  define_all_initial_functions()
  s=rospy.Service('/move_arm',comunication,handle_move_arm_service)
  print("Service ready")
  try:
    while (not rospy.core.is_shutdown()) and (not bool_exit):
        rospy.rostime.wallsleep(0.5)
        #rospy.spin()
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

  except bool_exit==True:
      return
if __name__ == '__main__':
  main()
