pos_x=0.1
pos_y=0
pos_z=0

angle_x=0
angle_y=0
angle_z=0

source ~/catkin_ws/devel/setup.bash
rosrun kinova_demo pose_action_client.py -v j2n6s300 mdeg -- $pos_x $pos_y $pos_z $angle_x $angle_y $angle_z