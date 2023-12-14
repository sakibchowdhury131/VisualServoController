#! /usr/bin/env python3

import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from kinova_msgs.msg import JointVelocity



class ImageConverter:

    def __init__(self):



        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)
        self.control_pub = rospy.Publisher("/j2s6s300_driver/in/joint_velocity", JointVelocity, queue_size=10)
        self.desired_center_x = 320
        self.desired_center_y = 240
        self.Kp1 = 8.0
        self.Kp2 = 8.0
        self.counter = 0




    def callback(self, data):
        try:
            
            rgb_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            gray_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
            ret,binary_image = cv2.threshold(gray_image,150,255,1)
            contours, _ = cv2.findContours(binary_image,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                x,y,w,h = cv2.boundingRect(contour)

                if w>70 and h>70:
                    rgb_image = cv2.rectangle(rgb_image,(x,y),(x+w,y+h),(0,255,0),2) ## this is our object

                    #calculate_error 

                    err_x , err_y = self.calculate_error(x, y, w, h)
                    # print(f"error X: {err_x},\terror Y: {err_y}")

                    ## generate msg
                    cmd = JointVelocity()
                    cmd.joint1 = self.Kp1 * err_x
                    cmd.joint2 = 0
                    cmd.joint3 = self.Kp2 * err_y
                    cmd.joint4 = 0
                    cmd.joint5 = 0
                    cmd.joint6 = 0
                    self.control_pub.publish(cmd)


    


        except CvBridgeError as e:
            print(e)

        cv2.imshow("RGB", rgb_image)
        cv2.imshow("BINARY", binary_image)
        cv2.waitKey(3)

    
    def calculate_error(self, x, y, w, h):
        ## calculate co-ordinate of the center 

        actual_center_x = int(x + w/2)
        actual_center_y = int(y + h/2)
        print(f"actual x: {actual_center_x}\tdesired x: {self.desired_center_x}\tactual y: {actual_center_y}\tdesired y: {self.desired_center_y}")
        err_x = actual_center_x - self.desired_center_x
        err_y = actual_center_y - self.desired_center_y

        return err_x, err_y


def main(args):
    ic = ImageConverter()
    rospy.init_node('rgb_feed')

    try:
        rospy.spin()
    
    except KeyboardInterrupt:
        print("Shutting Down")
    
    cv2.destroyAllWindows()



if __name__ == '__main__':
    main(sys.argv)