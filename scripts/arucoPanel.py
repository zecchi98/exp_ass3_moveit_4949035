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

from exp_assignment3.srv import Marker

bool_exit=False
total_aruco_found=0
ARUCO_PARAMETERS = aruco.DetectorParameters_create()

aruLibrary={
            '1':aruco.DICT_ARUCO_ORIGINAL
            ,'original':aruco.DICT_ARUCO_ORIGINAL
            ,'51000':aruco.DICT_5X5_1000
            ,'61000':aruco.DICT_6X6_1000
            ,'71000':aruco.DICT_7X7_1000
            }

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
    global ids_found
    global bool_exit,total_aruco_found
    if bool_exit:
        cv.destroyAllWindows()
        os._exit(os.EX_OK)
    cv_image=bridge.imgmsg_to_cv2(raw_img, desired_encoding='passthrough')
    cv_gray=cv.cvtColor(cv_image,cv.COLOR_RGB2GRAY)

    detCorners, detIds, _ = aruco.detectMarkers(cv_gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)
        
    if detIds is not None and len(detIds) >= 1: # Check if at least one marker has been found
        
        detAruImg = aruco.drawDetectedMarkers(cv_image.copy(), detCorners, borderColor=(0, 255, 0))

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
        detAruImg=cv_image.copy()#

    cv.imshow('detected markers',detAruImg)

    key = cv.waitKey(12) & 0xFF


def listener(myCam,myTop,myType,myCallk):
    global client_oracle,hypothesis
    rospy.init_node('camera_listener', anonymous=True)
    loadCameraParam(myCam)
    print('ready')
    rospy.Subscriber(myCam+myTop,myType,myCallk,queue_size = 1)
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




