#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander 
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_python_node',anonymous=True)

robot = moveit_commander.RobotCommander()  
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("arm")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path' , moveit_msgs.msg.DisplayTrajectory,queue_size=20)


pose_target = geometry_msgs.msg.Pose()
pose_target.position.x =-2.08365654579#0.68984736329 #-0.10607573932
pose_target.position.y = 0.18187809109#-0.211705715086#-0.473475319872
pose_target.position.z =  1.12746589794#1.88154680768#0.206447634761

pose_target.orientation.x = -0.69165858197#-0.100929549323 #-0.66162996081
pose_target.orientation.y =  0.00943545331471#-0.699867889445#0.591403556332
pose_target.orientation.z = 0.722052421803#0.684826303896#-0.14763775647
pose_target.orientation.w = 0.0126363909815#0.176099678847#0.436681487335
#print(pose_target)
#group.set_goal_orientation_tolerance (0.09)
#group.set_goal_position_tolerance (0.01)
group.set_goal_tolerance(0.01)
#group.set_planning_attempts(20)
group.set_planning_time(20)
group.set_pose_target(pose_target)

plan2 = group.plan()

rospy.sleep(5)
group.go(wait=True)

rospy.sleep(5)
print(group.get_current_pose())
moveit_commander.roscpp_shutdown()
