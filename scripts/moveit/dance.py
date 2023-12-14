#! /usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf

def execute_plan(group, x, y, z, roll, pitch, yaw):

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


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("test_node_moveit", anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("arm")
display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=10)
x = -0.4
y = 0.4
z = 0.4
roll = -1.2
pitch = 0
yaw = 0

x1 = -0.4
y1 = 0.4
z1 = 0.2
roll1 = -1.2
pitch1 = 0
yaw1 = 0

while True:
    execute_plan(group=group,
                x=x,
                y=y,
                z=z,
                roll=roll,
                pitch=pitch,
                yaw=yaw)

    execute_plan(group=group,
                x=x1,
                y=y1,
                z=z1,
                roll=roll1,
                pitch=pitch1,
                yaw=yaw1)
moveit_commander.roscpp_shutdown()