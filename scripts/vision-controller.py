#! /usr/bin/env python3

import sys
import rospy
import cv2
from sensor_msgs.msg import Image as ImgMsg
from cv_bridge import CvBridge, CvBridgeError
from kinova_msgs.msg import JointVelocity
# from .velocity_controller import publishVelCmd
from transformers import YolosImageProcessor, YolosForObjectDetection
from PIL import Image
import torch
from sensor_msgs.msg import JointState
import numpy as np
import subprocess
import time

STATE = 'DETECTING'

def detect_white_box(img):
    gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret,binary_image = cv2.threshold(gray_image,170,255,1)
    contours, _ = cv2.findContours(binary_image,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    good_contours = []
    for contour in contours:
        x,y,w,h = cv2.boundingRect(contour)

        if w>70 and h>70 and w<620 and h<470:
            binary_image = cv2.rectangle(binary_image,(x,y),(x+w,y+h),(0,255,0),2) ## this is our object
            good_contours.append(contour)
    return binary_image, good_contours


def getFeedbackCallback(data,args): 
  #generic but joint_state/effort is not published by kinova_driver
  joint_cmd = args[0]
  error_type = args[1]
  max_error = args[2]
  count = args[3]
  for i in range(0,len(joint_cmd)):
    if error_type == 'velocity':
      error = abs(joint_cmd[i] - data.velocity[i]*180/3.1415)
    if error_type == 'torque':
      error = abs(joint_cmd[i] - data.effort[i])     
    if count[0]>50:     
      max_error[i] = max(error,max_error[i])
    count[0] = count[0] +1 

def publishVelCmd(jointCmds, duration_sec, prefix):
  
  #subscriber to get feedback    
  topic_name = '/' + prefix + 'driver/out/joint_state'
  max_error = [0,0,0,0,0,0,0]
  counter = [0]
  sub = rospy.Subscriber(topic_name, JointState, getFeedbackCallback, (jointCmds,'velocity',max_error,counter))

  topic_name = '/' + prefix + 'driver/in/joint_velocity'
  pub = rospy.Publisher(topic_name, JointVelocity, queue_size=1)
  jointCmd = JointVelocity()
  jointCmd.joint1 = jointCmds[0]
  jointCmd.joint2 = jointCmds[1]
  jointCmd.joint3 = jointCmds[2]
  jointCmd.joint4 = jointCmds[3]
  jointCmd.joint5 = jointCmds[4]
  jointCmd.joint6 = jointCmds[5]
  jointCmd.joint7 = jointCmds[6]
  count = 0    
  rate = rospy.Rate(100)
  while (count < 100*duration_sec):
    count = count + 1
    pub.publish(jointCmd)
    rate.sleep()
  sub.unregister()
#   print("max error {} {} {} {} {} {} {}".format(max_error[0], max_error[1], max_error[2], max_error[3], max_error[4], max_error[5], max_error[6]))


class Controller:

    def __init__(self):

        # self.model = YolosForObjectDetection.from_pretrained('hustvl/yolos-tiny')
        # self.image_processor = YolosImageProcessor.from_pretrained("hustvl/yolos-tiny")



        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",ImgMsg,self.callback)
        self.desired_center_x = 320
        self.desired_center_y = 240
        self.desired_w = 400
        self.Kp1 = 0.1
        self.Kp2 = 0.1
        self.Kp3 = 0.01
        self.Kp4 = -0.1
        self.Kp5 = -0.1
        self.Kp6 = 0.1
        self.duration_sec = 0.011  ## duration for each velocity control command




    def callback(self, data):
        global STATE
        
        try:
            rgb_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            rgb_image, contours = detect_white_box(rgb_image)
            ## detect object

            if len(contours) == 1:
               STATE = 'DETECTING'
            else:
               STATE = 'GOING_TO_INIT'

            if STATE == 'DETECTING':
                for contour in contours:
                    x, y, w, h = cv2.boundingRect(contour)
                    err_x, err_y, err_w = self.calculate_error(x, y, x+w, y+h)
                    ## error adjust for x axis
                    # cmd = [0, err_x*self.Kp3, 0, 0, err_x*self.Kp5, 0, 0]


                    
                    ## error adjust for y axis
                    # cmd = [-err_y*self.Kp1, 0, 0, 0, 0, -err_y*self.Kp6, 0]

                    ## error adjust for z axis
                    # cmd = [0, err_w*self.Kp2, -err_w*self.Kp3, 0, err_w*self.Kp5, 0, 0]

                    ## joint error control
                    cmd = [-err_y*self.Kp1, err_x*self.Kp3 + err_w*self.Kp2, -err_w*self.Kp3, 0, err_x*self.Kp5 + err_w*self.Kp5, -err_y*self.Kp6, 0]


                    self.last_cmd = cmd
                    publishVelCmd(duration_sec = self.duration_sec,
                                    jointCmds = cmd,
                                    prefix = 'j2s6s300_')
            
            # elif STATE == 'GOING_TO_INIT':
                
            #     subprocess.run('/home/jonas/catkin_ws/src/visual_servo_controller/kinova_arm_control/joint_conrol.sh', shell = True, executable="/bin/bash")
            #     self.image_sub.unregister()
            cv2.imshow("RGB", rgb_image)
            cv2.waitKey(3)


        except CvBridgeError as e:
            print(e)
    
    def calculate_error(self, x1, y1, x2, y2):
        ## calculate co-ordinate of the center 

        actual_center_x = int((x1 + x2)/2)
        actual_center_y = int((y1 + y2)/2)
        # print(f"actual x: {actual_center_x}\tdesired x: {self.desired_center_x}\tactual y: {actual_center_y}\tdesired y: {self.desired_center_y}")
        err_x = actual_center_x - self.desired_center_x
        err_y = actual_center_y - self.desired_center_y
        err_w = int(x2 - x1) - self.desired_w
        print(f"error_x = {err_x}\terr_y = {err_y}\terr_w = {err_w}")

        return err_x, err_y, err_w


def main(args):
    
    rospy.init_node('rgb_feed')
    ic = Controller()
    try:
        rospy.spin()
    
    except:
        print("Shutting Down")
    
    cv2.destroyAllWindows()



if __name__ == '__main__':
    main(sys.argv)