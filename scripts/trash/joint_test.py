#! /usr/bin/env python3

import sys
import rospy
import cv2
from sensor_msgs.msg import Image as ImgMsg
from cv_bridge import CvBridge, CvBridgeError
from kinova_msgs.msg import JointVelocity

from transformers import YolosImageProcessor, YolosForObjectDetection
from PIL import Image
import torch

class ImageConverter:

    def __init__(self):
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",ImgMsg,self.callback)
        self.control_pub = rospy.Publisher("/j2n6s300_driver/in/joint_velocity", JointVelocity, queue_size=10)
        self.desired_center_x = 320
        self.desired_center_y = 240
        self.Kp1 = 0.5
        self.Kp2 = 0.2
        self.counter = 0




    def callback(self, data):
        try:

            
            
            cmd = JointVelocity()
            cmd.joint1 = self.Kp1 * 30
            cmd.joint2 = 0
            # cmd.joint3 = self.Kp2 * 30
            cmd.joint4 = 0
            cmd.joint5 = 0
            cmd.joint6 = 0
            print(cmd)
            self.control_pub.publish(cmd)




    


        except CvBridgeError as e:
            print(e)

        

    
    def calculate_error(self, x1, y1, x2, y2):
        ## calculate co-ordinate of the center 

        actual_center_x = int((x1 + x2)/2)
        actual_center_y = int((y1 + y2)/2)
        print(f"actual x: {actual_center_x}\tdesired x: {self.desired_center_x}\tactual y: {actual_center_y}\tdesired y: {self.desired_center_y}")
        err_x = actual_center_x - self.desired_center_x
        err_y = actual_center_y - self.desired_center_y

        return err_x, err_y


def main(args):
    ic = ImageConverter()
    rospy.init_node('test_node')

    try:
        rospy.spin()
    
    except KeyboardInterrupt:
        print("Shutting Down")
    
    # cv2.destroyAllWindows()



if __name__ == '__main__':
    main(sys.argv)