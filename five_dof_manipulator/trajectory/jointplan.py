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


group.set_named_target("pose1")
plan1 = group.go(wait=True)
rospy.sleep(2)
#rospy.sleep(2)
obj = "object_27"
flag = True
while not rospy.is_shutdown() and flag is True:
    bol = listener.frameExists(obj)
    if bol is True and flag is True:
        trans,rot = listener.lookupTransform("world",obj,rospy.Time())
        x,y,z = trans
       # print(x)
        flag = False
    elif bol is False and flag is True: 
        print("error no such transform")

print(trans)

'''
x= -2
y= -0.2
z=1.07
'''
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

pose_target.orientation.x = -0.5#-0.69165858197
pose_target.orientation.y = 0.5#0.00943545331471
pose_target.orientation.z = 0.5#0.722052421803
pose_target.orientation.w =  0.5#0.0126363909815
group.set_goal_tolerance(tol)
group.set_pose_target(pose_target)
group.set_planning_time(20)
plan3 = group.go(wait=True)

group1.set_named_target("open")
plan3 = group1.go(wait=True)

group_variable_values = group.get_current_joint_values()
group_variable_values[4] = 1.5708
group.set_joint_value_target(group_variable_values)
#group1.set_named_target("close")
plan4 = group.go(wait=True)

pose_target.position.z = round(z,1) + 0.02
group.set_pose_target(pose_target)
plan5 = group.go(wait=True)

group_variable_values = group1.get_current_joint_values()
group_variable_values[0] = 0.05
group_variable_values[1] = 0.05
group1.set_joint_value_target(group_variable_values)
plan6 = group1.go(wait=True)

pose_target.position.z = round(z,1) + 0.3
group.set_pose_target(pose_target)
plan7 = group.go(wait=True)
'''
#shush hv patience 
#group.set_path_constraints(upright_constraints)

group.set_goal_tolerance(tol)
group.set_pose_target(pose_target)

group.set_planning_time(20)
plan2 = group.go(wait=True)
rospy.sleep(1)


group1.set_named_target("open")
plan3 = group1.go()

group.set_path_constraints(None)
rospy.sleep(1)
pose_target.position.z = round(z,1)
group.set_pose_target(pose_target)
plan4 = group.go()
rospy.sleep(1)

if (group.get_current_pose().pose.position.x != pose_target.position.x) or (group.get_current_pose().pose.position.y != pose_target.position.y) or (group.get_current_pose().pose.position.z != pose_target.position.z):
    k = 1
    err_x = group.get_current_pose().pose.position.x - pose_target.position.x
    err_y = group.get_current_pose().pose.position.y - pose_target.position.y
    err_z = group.get_current_pose().pose.position.z - pose_target.position.z
    print(err_x)
    pose_target.position.x = round(pose_target.position.x + k*err_x,1) 
    pose_target.position.y = round(pose_target.position.y + k*err_y,1)
    pose_target.position.z = round(pose_target.position.z + k*err_z,1)
    pose_target.orientation.x = -0.7
    pose_target.orientation.y = 0
    pose_target.orientation.z =  0.7
    pose_target.orientation.w =  0
    #print(pose_target) 
    group.set_pose_target(pose_target)
    plan4 = group.go(wait=True)
    rospy.sleep(1)



group_variable_values = group1.get_current_joint_values()
group_variable_values[0] = 0.045
group_variable_values[1] = 0.045
group1.set_joint_value_target(group_variable_values)
#group1.set_named_target("close")
plan5 = group1.go()

rospy.sleep(2)

pose_target.position.z = (round(z,1) + 0.3)
group.set_pose_target(pose_target)
plan6 = group.go()

rospy.sleep(2)
group.set_path_constraints(None)

pose_target = geometry_msgs.msg.Pose()
pose_target.position.x = 2
pose_target.position.y = 0 
pose_target.position.z = 1.07 
pose_target.orientation.x = -0.7
pose_target.orientation.y = 0
pose_target.orientation.z =  0.7
pose_target.orientation.w =  0
group.set_pose_target(pose_target)
plan7 = group.go()
rospy.sleep(1)

group1.set_named_target("open")
plan8 = group1.go()

rospy.sleep(1)

group.set_named_target("idle")
plan9 = group.go()

'''
rospy.sleep(3)
moveit_commander.roscpp_shutdown()
        
