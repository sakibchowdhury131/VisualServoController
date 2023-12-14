source ~/catkin_ws/devel/setup.bash


joint_1=200
joint_2=150
joint_3=220
joint_4=-10
joint_5=70
joint_6=25
rosrun kinova_demo joints_action_client.py -v j2s6s300 degree -- $joint_1 $joint_2 $joint_3 $joint_4 $joint_5 $joint_6
