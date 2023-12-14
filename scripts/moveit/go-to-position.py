#! /usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("test_node_moveit", anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("arm")
display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=10)

pose_target = geometry_msgs.msg.Pose()
# pose_target.orientation.w = 1.0
pose_target.position.x = -0.4
pose_target.position.y = 0.4
pose_target.position.z = 0.4

roll = 3.2
pitch = 0
yaw = 0


quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
pose_target.orientation.x = quaternion[0]
pose_target.orientation.y = quaternion[1]
pose_target.orientation.z = quaternion[2]
pose_target.orientation.w = quaternion[3]


group.set_pose_target(pose_target)

plan1 = group.plan()
group.go(wait = True)
# rospy.sleep(5)
moveit_commander.roscpp_shutdown()