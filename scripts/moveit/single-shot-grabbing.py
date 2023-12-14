#! /usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf
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


class Controller:

    def __init__(self) -> None:

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("test_node_moveit", anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("arm")
        self.display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=10)
        self.bridge = CvBridge()
        self.image_valid = False
        self.state = 'GOING_TO_INITIAL_POS'
        

    def get_image(self, data):


        rgb_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        gray_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
        ret,binary_image = cv2.threshold(gray_image,100,255,1)
        contours, _ = cv2.findContours(binary_image,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            # print(x, y, w, h)
            # print('looking for object')


            if w>70 and w<200 and h>70 and h < 200 and self.state == 'LOOKING_FOR_OBJECT':
                print('Object found')
                self.x, self.y, self.w, self.h = x, y, w, h
                self.image_valid = True
                rgb_image = cv2.rectangle(rgb_image,(self.x,self.y),(self.x+self.w,self.y+self.h),(0,255,0),2) ## this is our object
                print(x, y, w, h)
                self.state = 'GO_TO_OBJECT'

            if self.state == 'GO_TO_OBJECT':
                print('giong to object')
                middle_x = self.x + self.w/2
                middle_y = self.y + self.h/2

                x_target = middle_x/600.0
                y_target = middle_y/1200.0
                print(x_target, y_target)

                # x_target = 0.3
                # y_target = 0.3

                controller.execute_plan(group=controller.group,
                                        x = -x_target,
                                        y = y_target,
                                        z = 0.0,
                                        roll = -3.1,
                                        pitch = 0,
                                        yaw=-1.5)
                
                self.state = 'LIFT'

            if self.state == 'LIFT':
                self.go_to_final_position()

        cv2.imshow("RGB", rgb_image)
        cv2.waitKey(3)

    def execute_plan(self, group, x, y, z, roll, pitch, yaw):

        pose_target = geometry_msgs.msg.Pose()
        # pose_target.orientation.w = 1.0
        pose_target.position.x = x
        pose_target.position.y = y
        pose_target.position.z = z
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        pose_target.orientation.x = quaternion[0]
        pose_target.orientation.y = quaternion[1]
        pose_target.orientation.z = quaternion[2]
        pose_target.orientation.w = quaternion[3]
        group.set_pose_target(pose_target)
        # plan1 = group.plan()
        group.go(wait = True)

    def go_to_initial_position(self):
        x = -0.3
        y = 0.2
        z = 0.3
        roll = -3.1
        pitch = 0
        yaw = -1.5
        self.execute_plan(group=self.group,
                        x=x,
                        y=y,
                        z=z,
                        roll=roll,
                        pitch=pitch,
                        yaw=yaw)
        self.state = 'LOOKING_FOR_OBJECT'
        self.img_sub = rospy.Subscriber("/camera/rgb/image_raw",ImgMsg,self.get_image)


    def go_to_initial_position(self):
        x = -0.3
        y = 0.2
        z = 0.3
        roll = -3.1
        pitch = 0
        yaw = -1.5
        self.execute_plan(group=self.group,
                        x=x,
                        y=y,
                        z=z,
                        roll=roll,
                        pitch=pitch,
                        yaw=yaw)
        self.state = 'LOOKING_FOR_OBJECT'
        self.img_sub = rospy.Subscriber("/camera/rgb/image_raw",ImgMsg,self.get_image)

    def main(self):
        self.go_to_initial_position()


controller = Controller()
controller.main()
rospy.spin()
