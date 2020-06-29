#! /usr/bin/env python

import sys
import copy
import rospy
import math
import time
import tf
import moveit_commander 
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib

from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, JointConstraint, Constraints

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_python_node',anonymous=True)

robot = moveit_commander.RobotCommander()  
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("arm")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path' , moveit_msgs.msg.DisplayTrajectory,queue_size=20)
listener = tf.TransformListener()
flag = True
scale = 1
waypoints = []

while len(waypoints)<50:
    wpose = group.get_current_pose().pose
    if wpose not in waypoints:
        print(len(waypoints))
        waypoints.append(copy.deepcopy(wpose))



(plan, fraction) = group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold

print(plan)
