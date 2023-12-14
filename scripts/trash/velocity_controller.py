#! /usr/bin/env python3

import rospy
from kinova_msgs.msg import JointVelocity
from sensor_msgs.msg import JointState
import sys


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


def main(args):
  prefix = 'j2s6s300_'
  jointCmd = [-20, 0, 0, 0, 0, 0, 0]
  duration_sec = 4
  publishVelCmd(duration_sec=duration_sec, jointCmds=jointCmd, prefix=prefix)




if __name__ == '__main__':
    rospy.init_node('test_velocity')
    main(sys.argv)