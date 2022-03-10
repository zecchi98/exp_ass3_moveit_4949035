#!/usr/bin/env python

import rospy
import numpy as np

import sys
import os
from sensor_msgs.msg import Image as sensImg
from sensor_msgs.msg import CameraInfo
#frofm sensor_msgs.msg import PointCloud2 as sensPCld


import cv2 as cv
import cv2.aruco as aruco
#from cv2 import aruco as aruco
from cv_bridge import CvBridge

from ur3_control.srv import aruco_service,aruco_serviceResponse
from ur3_control.srv import cv_server,cv_serverResponse, cv_serverRequest
from ur3_control.msg import cv_to_bridge as bridge_msg
from exp_assignment3.srv import Marker
#PUBLISHER DI ARRAY:
#aruco_position_pub = rospy.Publisher('/almax/aruco_target',Float64MultiArray,queue_size=20)
#array = [69.1,0,1,33,1,1,1,0]
#robaccia = Float64MultiArray(data=array)
#aruco_position_pub.publish(robaccia)
#------------------------------------------------

pub = rospy.Publisher('aruco_bridge_opencv', bridge_msg, queue_size=1)
bool_exit=False
total_aruco_found=0
ARUCO_PARAMETERS = aruco.DetectorParameters_create()

aruLibrary={
            '1':aruco.DICT_ARUCO_ORIGINAL
            ,'original':aruco.DICT_ARUCO_ORIGINAL
            ,'51000':aruco.DICT_5X5_1000
            ,'61000':aruco.DICT_6X6_1000
            ,'71000':aruco.DICT_7X7_1000
            ,'assignment':aruco.DICT_6X6_250
            }
#ARUCO_DICT = aruco.Dictionary_get(aruLibrary['original'])

def loadArucoDict(requestedDict):#TODO
    global ARUCO_DICT
    ARUCO_DICT = aruco.Dictionary_get(aruLibrary[requestedDict])

selectedDictionary='original'
loadArucoDict(selectedDictionary)   
 
#----------------------------------------------
    
def loadCameraParam(myCam):
    global cameraMatr
    global cameraDistCoefs
    global cameraFocLen
    
    print('loading camera parameters...')
    cameraInfoMsg=rospy.wait_for_message(myCam+'/camera_info',CameraInfo)
    cameraMatr=np.reshape(cameraInfoMsg.K,[3,3])
    cameraDistCoefs=cameraInfoMsg.D
    cameraFocLen=np.mean([np.ravel(cameraMatr[0])[0],np.ravel(cameraMatr[1])[1]])
    
#------------------------------------------------------------
    
#aruTargetDict={'panelSwitch1':(101,
#                        40)
#                ,'panelSwitch2':(102,
#                        40)
#                ,'panelSwitch3':(103,
#                        40)
#                ,'panelSwitch4':(104,
#                        40)
#                ,'panelSwitch5':(105,
#                        40)
#                ,'panelSwitch6':(106,
#                        40)
#                ,'panelSwitch7':(107,
#                        40)
#                ,'panelSwitch8':(108,
#                        40)
#                }

targetList=[	[1,50],
		[2,50],
		[3,50],
		[4,50],
		[5,50],
		[6,50],
		[7,50],
		[8,50],
		[9,50],
		[10,40],
		[11,50],
		[12,50],
		[13,40],
		[14,50],
		[102,40],
		[104,40],
		[106,40],
		[108,40]]

targetListLen=len(targetList)
targetCounter=0
remaining_targets=targetListLen

global targetMarkId,targetMarkSize
class hypothesis_general():
    ##
    #\class hypothesis_general
    #\brief struct to handle hypothesis made of people,places,weapons and hypothesis_code. All this values are arrays, which not happen in the "hypothesis" class
    def __init__(self):
        ##
        #\brief init value to initialize arrays
        #super(hypothesis_general, self).__init__()
        self.people=[]
        self.places=[]
        self.weapons=[]
        self.hypothesis_code="HP-1"

    def print_data(self):
        ##
        #\brief print data function
        print("\nhypo_code:")
        print(self.hypothesis_code)
        print("people:")
        print(self.people)
        print("places:")
        print(self.places)
        print("weapons:")
        print(self.weapons)
    def add_person(self,person):
      if len(self.people)==0:
        self.people.append(person)
    def add_weapon(self,weapon):
      if len(self.weapons)==0:
        self.weapons.append(weapon)
    def add_place(self,place):
      if len(self.places)==0:
        self.places.append(place)
    def check_complete_and_consistent(self):
      if len(self.weapons)==1 and len(self.people)==1 and len(self.places)==1:
        return True
      else:
        return False
    def set_hypo_code(self,hypo_code):
      self.hypothesis_code=hypo_code





#----------------------------------------
bridge=CvBridge()
ids_found=np.zeros(100000)
def save_hint(response):
  global hypothesis
  response=response.oracle_hint
  #print(response)
  # int32 ID;  string key; string value
  if(response.value=='-1'):
      print("No valid hint")
      return
  hypothesis[response.ID].set_hypo_code(response.ID)
  if response.key=="where":
    hypothesis[response.ID].add_place(response.value)

  if response.key=="who":
    hypothesis[response.ID].add_person(response.value)

  if response.key=="what":
    hypothesis[response.ID].add_weapon(response.value)
  print("hint received")
  hypothesis[response.ID].print_data()
def callbackRaw(raw_img):
    global aruco_success
    global msgVector
    global msgRotMatrix
    global targetCounter
    global findNewTarget
    global remaining_targets
    global ids_found
    global bool_exit
    if bool_exit:
        cv.destroyAllWindows()
        os._exit(os.EX_OK)
    cv_image=bridge.imgmsg_to_cv2(raw_img, desired_encoding='passthrough')
    cv_gray=cv.cvtColor(cv_image,cv.COLOR_RGB2GRAY)
    
    msg=bridge_msg()
    msg.aruco_found=[False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False]
    (targetMarkId,targetMarkSize)=tuple(targetList[targetCounter])
    detCorners, detIds, _ = aruco.detectMarkers(cv_gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)
        
    if detIds is not None and len(detIds) >= 1: # Check if at least one marker has been found
        
        detAruImg = aruco.drawDetectedMarkers(cv_image.copy(), detCorners, borderColor=(0, 255, 0))
        
        aruco_success=False     
        for mId, aruPoints in zip(detIds, detCorners):
            id_trovato=mId[0]
            if id_trovato<11 and id_trovato>40:
                print("errore")
                print(str(id_trovato)+":")
            if(ids_found[id_trovato]==0) and id_trovato<10000 and id_trovato>=11 and id_trovato<=40:
                total_aruco_found=total_aruco_found+1
                response=client_oracle(id_trovato)
                print(str(id_trovato)+":")
                save_hint(response)
                ids_found[id_trovato]=1
                print("total aruco:"+str(total_aruco_found))
    else:
        aruco_success=False
        detAruImg=cv_image.copy()#

    cv.imshow('detected markers',detAruImg)
    msg.id_aruco=targetCounter+1

    if msg.success:
        msg.x=0.001*msgVector[0]
        msg.y=0.001*msgVector[1]
        msg.z=0.001*msgVector[2]
        msg.vector=msgRotMatrix.flatten()
    #print(msg.aruco_found)
    pub.publish(msg)

    key = cv.waitKey(12) & 0xFF# key still unused
#    if key == 27:# 27:esc, ord('q'):q
#       exit_somehow()
        
    
#-----------------------------------------------------------------

msgVector=[0,0,0]#np.zeros([1,3])
msgRotMatrix=[[0,0,0,],[0,0,0],[0,0,0]]#np.zeros([3,3])

        
tglWristLengthRecovery=1
# recovered percentage
recovLenRatio=1

def callback_service(req):
    global aruco_success,msgVector,msgRotMatrix,targetCounter,findNewTarget,remaining_targets,bool_exit

    #print('Arucopy:ServiceRequested')
    if req.message=="exit":
        bool_exit=True
    #if req.next_aruco:
    if req.message=="select_next_aruco":
        targetCounter=int(req.second_information)-1


    return cv_serverResponse(
        success=aruco_success,
        moreTargets=remaining_targets,
        x=0.001*msgVector[0], #+(recovLenRatio*0.08 if tglWristLengthRecovery else 0),#[m]
        y=0.001*msgVector[1],
        z=0.001*msgVector[2],
        vector=np.ravel(msgRotMatrix)#flattened array
        )


    
def listener(myCam,myTop,myType,myCallk):
    global client_oracle,hypothesis
    rospy.init_node('camera_listener', anonymous=True)
    loadCameraParam(myCam)
    print('ready')
    rospy.Subscriber(myCam+myTop,myType,myCallk,queue_size = 1)
    rospy.Publisher('aruco_bridge_opencv', bridge_msg, queue_size=10)
    rospy.Service('cv_server', cv_server, callback_service)
    client_oracle=rospy.ServiceProxy('/oracle_hint',Marker)

    hypothesis=[]
    for i in range(0,6):
      hypothesis.append(hypothesis_general())
    try:
        rospy.spin()
    except KeyboardInterrupt:#
        print('Closing')
    cv.destroyAllWindows()


if __name__ == '__main__':

    myCamera="/camera/color"
    myTopicFull="/image_raw"
    
    print('connecting to:'+myCamera+myTopicFull+'...')
    listener(myCamera,myTopicFull,sensImg,callbackRaw)




