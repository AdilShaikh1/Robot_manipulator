#! /usr/bin/env python

import sys
import copy
import rospy
import math
import tf
import time
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
group1 = moveit_commander.MoveGroupCommander("gripper")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path' , moveit_msgs.msg.DisplayTrajectory,queue_size=20)
listener = tf.TransformListener()
start_t = time.time()
tol = 0.008


group.set_named_target("goal")
plan1 = group.go(wait=True)
rospy.sleep(2)
#rospy.sleep(2)
obj = "object_27"
#flag = True
while not rospy.is_shutdown():
    bol = listener.frameExists(obj)
    if bol is True:
        trans,rot = listener.lookupTransform("world",obj,rospy.Time())
        x,y,z = trans
        curr_x = group.get_current_pose().pose.position.x
        curr_y = group.get_current_pose().pose.position.y
        err_x = abs(x-curr_x)
        err_y = abs(y-curr_y)
        print(err_x)
        if err_x >= 0.100 or err_y>=0.100:
            print("in the loop")
            pose_target = geometry_msgs.msg.Pose()
            pose_target.position.x = round(x,2) +0.050
            pose_target.position.y = round(y,2) 
            pose_target.position.z = (round(z,2) + 0.3)  
            pose_target.orientation.x = -0.69165858197
            pose_target.orientation.y = 0.00943545331471
            pose_target.orientation.z = 0.722052421803
            pose_target.orientation.w =  0.0126363909815
            group.set_goal_tolerance(tol)
            group.set_pose_target(pose_target)
            group.set_planning_time(20)
            plan2 = group.go(wait=True)
        else:
            print("too close")
            
    elif bol is False: 
        print("error no such transform")

rospy.sleep(3)
moveit_commander.roscpp_shutdown()