#!/usr/bin/env python

import rospy
import numpy as np

import sys
import os
from sensor_msgs.msg import Image as sensImg
from sensor_msgs.msg import CameraInfo
from std_srvs.srv import Empty
import cv2 as cv
import cv2.aruco as aruco
from cv_bridge import CvBridge
from exp_assignment3.srv import Marker
from exp_ass3_moveit_4949035.srv import *
from exp_ass3_moveit_4949035.msg import ErlOracle

def loadCameraParam(myCam):
    ##
    #\brief this function will load camera parameters
    #@param myCam Topic of the camera
    #@return No return
    global cameraMatr,cameraDistCoefs,cameraFocLen
    global cameraDistCoefs
    global cameraFocLen
    
    print('loading camera parameters...')
    cameraInfoMsg=rospy.wait_for_message(myCam+'/camera_info',CameraInfo)
    cameraMatr=np.reshape(cameraInfoMsg.K,[3,3])
    cameraDistCoefs=cameraInfoMsg.D
    cameraFocLen=np.mean([np.ravel(cameraMatr[0])[0],np.ravel(cameraMatr[1])[1]])
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
def check_victory(hint):
    ##
    #\brief this function will check if the input hint is complete then it will call the server:"oracle solution" in order to know the solution. And will return if the input hint is the winner one or not.
    #@param hint This is the hypothesis_general variable that will be checked
    #@return No return

    global bool_exit
    #Check if the hint is complete
    is_complete=hint.check_complete_and_consistent()

    #if the hint is complete then it call the server to know the solution
    if is_complete :

        print("ID: "+str(hint.hypothesis_code)+" is complete")

        #initialize the server to know the solution
        client_solution=rospy.ServiceProxy('/oracle_solution',Oracle)

        #call the server to know the solution
        risposta=client_solution()
        winner_id=risposta.ID

        print("----------------the winner id is:" + str(winner_id))

        #if the winner id is equal to the one we are analyzing and we know it is complete then we have won
        if winner_id==hint.hypothesis_code:
            print("YOU WIN")
            bool_exit=True
            rospy.set_param("/victory",True)
        else:
            print("Still not the winner id")
def save_hint(response):
  ##
  #\brief This function will save the hint received in the global hypothesis array
  #@param response It is a response from the server "oracle_hint" containing an hint
  #@return No return

  global hypothesis

  response=response.oracle_hint

  #We do not save any data
  if(response.value=='-1'):
      print("No valid hint")
      return

  hypothesis[response.ID].set_hypo_code(response.ID)

  # We will save the hint in the correct array knowing what type of hint we are reading
  # That's why we check the key

  if response.key=="where":
    hypothesis[response.ID].add_place(response.value)

  if response.key=="who":
    hypothesis[response.ID].add_person(response.value)

  if response.key=="what":
    hypothesis[response.ID].add_weapon(response.value)

  hypothesis[response.ID].print_data()

  #Before going on we check if we have won
  check_victory(hypothesis[response.ID])

def callbackRaw(raw_img):
    ##
    #\brief This function is the callback which is subscribed to the raw channel of the image. It will detect arucos and in case it is a new one it will save it and check for victory.
    #@param raw_img It is a sensor_msgs of type Image which contain the image we will analyze
    #@return No return

    global ids_found,bool_exit,total_aruco_found

    #if this bool is a true than it close everything
    if bool_exit:
        cv.destroyAllWindows()
        os._exit(os.EX_OK)

    #Here the detection will be computed based on the dictionary
    cv_image=bridge.imgmsg_to_cv2(raw_img, desired_encoding='passthrough')
    cv_gray=cv.cvtColor(cv_image,cv.COLOR_RGB2GRAY)
    detCorners, detIds, _ = aruco.detectMarkers(cv_gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)
        
    # Check if at least one marker has been found
    if detIds is not None and len(detIds) >= 1:

        #Draw the borders around the aruco
        detAruImg = aruco.drawDetectedMarkers(cv_image.copy(), detCorners, borderColor=(0, 255, 0))

        #Foreach aruco found it will work on it
        for mId, aruPoints in zip(detIds, detCorners):
            id_trovato=mId[0]

            #It should never found something in this range
            if id_trovato<11 and id_trovato>40:
                print("errore")
                print(str(id_trovato)+":")

            #It will check if the id is inside a correct range and if it has ever been founds
            if id_trovato<10000 and id_trovato>=11 and id_trovato<=40:
                if ids_found[id_trovato]==0:
                  total_aruco_found=total_aruco_found+1

                  #Here we ask for the oracle hint
                  response=client_oracle(id_trovato)
                  print(str(id_trovato)+":")

                  #Here we save it in the global array and we will also check the victory
                  save_hint(response)

                  #Thanks to this line we will not check anymore that id
                  ids_found[id_trovato]=1
                  print("total aruco:"+str(total_aruco_found))
                  if total_aruco_found>20 :
                    print(ids_found)
    else:
        detAruImg=cv_image.copy()#

    cv.imshow('detected markers',detAruImg)
    key = cv.waitKey(12) & 0xFF


def main():
    ##
    #\brief Here we initialize some variables and thanks to spin we wait for callbackRaw
    global client_oracle,hypothesis,ARUCO_DICT,ARUCO_PARAMETERS,bridge,ids_found,total_aruco_found,bool_exit

    bool_exit=False
    total_aruco_found=0
    bridge=CvBridge()

    #From here we will keep track of the ids we have already analyzed
    ids_found=np.zeros(60)

    #Camera topics
    raw_topic="/camera/color/image_raw"
    myCamera="/camera/color"

    #rospy initialization
    rospy.init_node('camera_listener', anonymous=True)
    #Subscriber initialization (image callback)
    rospy.Subscriber(raw_topic,sensImg,callbackRaw,queue_size = 1)
    #Client initialization (from here we will know which is the hint for a particular aruco)
    client_oracle=rospy.ServiceProxy('/oracle_hint',Marker)
    #Initialize victory rosparam
    rospy.set_param("/victory",False)
    ARUCO_PARAMETERS = aruco.DetectorParameters_create()
    ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)


    #print('connecting to:'+myCamera+'...')
    #loadCameraParam(myCamera)
    #print('ready')

    # We initialize an array of 6 values
    hypothesis=[]
    for i in range(0,6):
      hypothesis.append(hypothesis_general())
    #Thanks to spin we will wait callbacks
    try:
        rospy.spin()
    except KeyboardInterrupt:#
        print('Closing')
    cv.destroyAllWindows()


if __name__ == '__main__':
    main()




