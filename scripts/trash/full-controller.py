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

callback_active = True


def detect_brown_box(img):
    # Read the image

    # Convert the image to HSV color space
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Define the lower and upper bounds for brown color in HSV
    lower_brown = np.array([10, 50, 20])
    upper_brown = np.array([30, 255, 255])

    # Threshold the image to extract the brown color
    mask = cv2.inRange(hsv, lower_brown, upper_brown)

    # Find contours in the thresholded image
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Filter out contours representing rectangles
    for contour in contours:
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        if len(approx) == 4:
            x, y, w, h = cv2.boundingRect(contour)
            
            # Draw rectangles around detected brown-colored boxes
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)

    return img


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
  print("max error {} {} {} {} {} {} {}".format(max_error[0], max_error[1], max_error[2], max_error[3], max_error[4], max_error[5], max_error[6]))


class ImageConverter:

    def __init__(self):

        self.model = YolosForObjectDetection.from_pretrained('hustvl/yolos-tiny')
        self.image_processor = YolosImageProcessor.from_pretrained("hustvl/yolos-tiny")



        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",ImgMsg,self.callback)
        self.desired_center_x = 320
        self.desired_center_y = 240
        self.Kp1 = 0.5
        self.Kp2 = 0.0
        self.counter = 0




    def callback(self, data):

        global callback_active

        if not callback_active:
            return
        
        try:
            callback_active = False
            rgb_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            image = Image.fromarray(rgb_image)
            inputs = self.image_processor(images=image, return_tensors="pt")
            outputs = self.model(**inputs)


            # print results
            target_sizes = torch.tensor([image.size[::-1]])
            results = self.image_processor.post_process_object_detection(outputs, threshold=0.9, target_sizes=target_sizes)[0]
            for score, label, box in zip(results["scores"], results["labels"], results["boxes"]):
                box = [round(i, 2) for i in box.tolist()]

                object_class = self.model.config.id2label[label.item()]
                if object_class == 'person':
                    x_start = int(box[0])
                    y_start = int(box[1])
                    x_end = int(box[2])
                    y_end = int(box[3])
                    rgb_image = cv2.rectangle(rgb_image,(x_start, y_start),(x_end, y_end),(0,255,0),2) ## this is our object
                    err_x , err_y = self.calculate_error(x_start, y_start, x_end, y_end)
                    cmd = [err_x*self.Kp1, 0, err_y*self.Kp2, 0, 0, 0, 0]
                    publishVelCmd(duration_sec = 0.05,
                                    jointCmds = cmd,
                                    prefix = 'j2n6s300_')
            cv2.imshow("RGB", rgb_image)
            cv2.waitKey(3)


        except CvBridgeError as e:
            print(e)
        
        callback_active = True
    
    def calculate_error(self, x1, y1, x2, y2):
        ## calculate co-ordinate of the center 

        actual_center_x = int((x1 + x2)/2)
        actual_center_y = int((y1 + y2)/2)
        print(f"actual x: {actual_center_x}\tdesired x: {self.desired_center_x}\tactual y: {actual_center_y}\tdesired y: {self.desired_center_y}")
        err_x = actual_center_x - self.desired_center_x
        err_y = actual_center_y - self.desired_center_y

        return err_x, err_y


def main(args):
    
    rospy.init_node('rgb_feed')
    ic = ImageConverter()
    try:
        rospy.spin()
    
    except KeyboardInterrupt:
        print("Shutting Down")
    
    cv2.destroyAllWindows()



if __name__ == '__main__':
    main(sys.argv)