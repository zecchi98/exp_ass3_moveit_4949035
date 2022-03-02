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
from std_srvs.srv import Empty
from rosplan_knowledge_msgs.srv import *
  
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

class my_rosplan_class():
  def __init__(self):
    super(my_rosplan_class, self).__init__()
    print("Waiting for services")
    rospy.wait_for_service("rosplan_knowledge_base/update")
    rospy.wait_for_service("rosplan_knowledge_base/clear")
    rospy.wait_for_service("rosplan_problem_interface/problem_generation_server")
    rospy.wait_for_service("rosplan_planner_interface/planning_server")
    rospy.wait_for_service("rosplan_parsing_interface/parse_plan")
    rospy.wait_for_service("rosplan_plan_dispatcher/cancel_dispatch")
    print("All services ready")
    self.clear_plan=rospy.ServiceProxy('/rosplan_knowledge_base/clear',Empty)
    self.update_the_plan=rospy.ServiceProxy('/rosplan_knowledge_base/update',KnowledgeUpdateService)
    self.generate_problem_client=rospy.ServiceProxy('/rosplan_problem_interface/problem_generation_server',Empty)
    self.planning_server_client=rospy.ServiceProxy('/rosplan_planner_interface/planning_server',Empty)
    self.parse_plan_client=rospy.ServiceProxy('/rosplan_parsing_interface/parse_plan',Empty)
    self.cancel_dispatch_client=rospy.ServiceProxy('/rosplan_plan_dispatcher/cancel_dispatch',Empty)
    self.debug=True
  def clear_plan(self):
    self.clear_plan()
  def add_GOAL_predicate_single_param(self,predicate_name,key,value,bool):
    #key is the type while value is the name of the variable
    #bool is used to set to True or False the goal
    req=KnowledgeUpdateServiceRequest()
    req.knowledge.is_negative=not(bool)
    req.update_type=1
    req.knowledge.knowledge_type=1
    req.knowledge.attribute_name=predicate_name
    x=diagnostic_msgs.msg.KeyValue()
    x.key=key
    x.value=value
    req.knowledge.values.append(x)
    result=self.update_the_plan(req)
    if self.debug==True:
      print(result)
  def add_GOAL_predicate_NO_param(self,predicate_name,bool):
    #key is the type while value is the name of the variable
    #bool is used to set to True or False the goal
    req=KnowledgeUpdateServiceRequest()
    req.knowledge.is_negative=not(bool)
    req.update_type=1
    req.knowledge.knowledge_type=1
    req.knowledge.attribute_name=predicate_name
    result=self.update_the_plan(req)
    if self.debug==True:
      print(result)
  def add_INSTANCE(self,type,name):
    #key is the type while value is the name of the variable
    #bool is used to set to True or False the goal
    req=KnowledgeUpdateServiceRequest()

    req.update_type=0
    req.knowledge.knowledge_type=0
    req.knowledge.instance_name=name
    req.knowledge.instance_type=type
    result=self.update_the_plan(req)
    if self.debug==True:
      print(result)
  def add_FACT_predicate_single_param(self,predicate_name,key,value,bool):
    #key is the type while value is the name of the variable
    #bool is used to set to True or False the goal
    req=KnowledgeUpdateServiceRequest()
    req.knowledge.is_negative=not(bool)
    req.update_type=0
    req.knowledge.knowledge_type=1
    req.knowledge.attribute_name=predicate_name
    x=diagnostic_msgs.msg.KeyValue()
    x.key=key
    x.value=value
    req.knowledge.values.append(x)
    result=self.update_the_plan(req)
    if self.debug==True:
      print(result)
  def add_FACT_predicate_NO_param(self,predicate_name,bool):
    #key is the type while value is the name of the variable
    #bool is used to set to True or False the goal
    req=KnowledgeUpdateServiceRequest()
    req.knowledge.is_negative=not(bool)
    req.update_type=0
    req.knowledge.knowledge_type=1
    req.knowledge.attribute_name=predicate_name
    result=self.update_the_plan(req)
    if self.debug==True:
      print(result)
  def generate_the_problem_and_plan_and_parse(self):
    self.generate_problem_client()
    self.planning_server_client()
    self.parse_plan_client()
  def cancel_dispatch(self):
    self.cancel_dispatch_client()


def handle_replan_service(req):
  print("Server request received")
  
  response=comunicationResponse()
  rosplan_library.cancel_dispatch()
  rosplan_library.clear_plan()

  rosplan_library.add_INSTANCE("waypoint","wp0")
  rosplan_library.add_INSTANCE("waypoint","wp1")
  rosplan_library.add_INSTANCE("waypoint","wp2")
  rosplan_library.add_INSTANCE("waypoint","wp3")
  rosplan_library.add_INSTANCE("waypoint","wp4")

  rosplan_library.add_FACT_predicate_single_param("hint_taken","waypoint","wp0",True)
  #rosplan_library.add_FACT_predicate_NO_param("not_initialized",True)


  if(req.msg1=="waypoint"):
    #Let's set the location
    rosplan_library.add_FACT_predicate_single_param("robot_at","waypoint",req.msg2,True)
  elif (req.msg1=="need_to_test"):
    rosplan_library.add_FACT_predicate_single_param("robot_at","waypoint",req.msg2,True)
    rosplan_library.add_FACT_predicate_single_param("hint_taken","waypoint","wp1",True)
    rosplan_library.add_FACT_predicate_single_param("hint_taken","waypoint","wp2",True)
    rosplan_library.add_FACT_predicate_single_param("hint_taken","waypoint","wp3",True)
    rosplan_library.add_FACT_predicate_single_param("hint_taken","waypoint","wp4",True)
  else:
    rosplan_library.add_FACT_predicate_single_param("robot_at","waypoint","wp0",True)

  #Let's set the goal
  rosplan_library.add_GOAL_predicate_NO_param("HP_tested",True)

  rosplan_library.generate_the_problem_and_plan_and_parse()
  response.success=True

  #va rifatto tutto da problem generation a parse plan
  return response




def define_all_initial_functions():
    global movegroup_library,comunication_object,transformation_library,movimenti_base_library,aruco_library
    global joystick_verso_rotazione,joystick_angle_step,joystick_translation_step,bool_message_from_user
    #global clear_plan,update_the_plan
    global rosplan_library
    rospy.init_node('re_plan_service', anonymous=True)
    rosplan_library=my_rosplan_class()
    #clear_plan=rospy.ServiceProxy('rosplan_knowledge_base/clear',Empty)
    #update_the_plan=rospy.ServiceProxy('/rosplan_knowledge_base/update',KnowledgeUpdateService)
    bool_exit=False
    prova()

def prova():


    nul=0

def main():
  define_all_initial_functions()
  s=rospy.Service('/replan_service',comunication,handle_replan_service)
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


#Cose da aggiungere per il pddl
"""
start_node wp0 ; se aggingo questo predicato, grazie ad azioni specifiche posso gestire il nodo in maniera diversa
action go_to_home che prende in ingresso un waypoint con start_node attivo

"""
