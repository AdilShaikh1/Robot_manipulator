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

class MoveArm():

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_python_node',anonymous=True)
        self.robot = moveit_commander.RobotCommander()  
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("arm")
        self.group1 = moveit_commander.MoveGroupCommander("gripper")
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path' , moveit_msgs.msg.DisplayTrajectory,queue_size=20)
        self.listener = tf.TransformListener()
        self.start_t = time.time()

    def gettf(self,obj):
        flag = True
        while not rospy.is_shutdown() and flag is True:
            bol = self.listener.frameExists(obj)
            if bol is True and flag is True:
                self.trans,self.rot = self.listener.lookupTransform("world",obj,rospy.Time())
                your_euler = tf.transformations.euler_from_quaternion(self.rot)
                r,p,y = your_euler
                self.rot = tf.transformations.quaternion_from_euler(r,p,y)
                flag = False
                break
            elif bol is False and flag is True and time.time() is (self.start_t + 3): 
                print("error no such transform")   
        return self.trans, self.rot

    def plan_arm(self, x, y, z, tol):
        pose_target = geometry_msgs.msg.Pose()
        pose_target.position.x = round(x,1) 
        pose_target.position.y = round(y,1) 
        pose_target.position.z = round(z,1)  
        pose_target.orientation.x = 0
        pose_target.orientation.y = 0
        pose_target.orientation.z =  1
        pose_target.orientation.w =  0
        self.group.set_goal_tolerance(tol)
        self.group.set_pose_target(pose_target)
        self.group.set_planning_time(20)
        self.plan1 = self.group.go()
        return self.plan1
    
    def plan_gripper(self,val):
        group_variable_values = self.group1.get_current_joint_values()
        group_variable_values[0] = val
        group_variable_values[1] = val
        self.group1.set_joint_value_target(group_variable_values)
        plan2 = self.group1.go()

    def Constraint(self,param):
        if param == True:
            upright_constraints = Constraints()
            joint_constraint = JointConstraint()
            upright_constraints.name = "upright"
            joint_constraint.position = -1.60
            joint_constraint.tolerance_above = 1.4
            joint_constraint.tolerance_below = 1.61
            joint_constraint.weight = 1
            joint_constraint.joint_name = "link2_link3_joint"
            upright_constraints.joint_constraints.append(joint_constraint)
            self.group.set_path_constraints(upright_constraints)
        elif param == False:
            self.group.set_path_constraints(None)
        else:
            print("wrong input")

        def err_(self,grp,desired_pose):
            error = grp.get_current_pose().pose.position - desired_pose
            return error



    def name_grp(self,grp,pos):
        grp.set_named_target(pos)
        plan3 = grp.go()
        

    def move_(self, plan): 
        self.group.execute(plan)
        
    def traj(self,plan):
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)

    def current_pose(self):
        return self.group.get_current_pose().pose
    
    def rpy(self,quat):
        return tf.transformations.euler_from_quaternion(quat)

    def euler(self,r,p,y):
        return tf.transformations.quaternion_from_euler(r,p,y)

    

 

moveit = MoveArm()
moveit.name_grp(moveit.group,"pose1")
trans,rot = moveit.gettf("object_27")
x,y,z = trans #(-2,0,1.08)
print(x)

moveit.plan_arm(x,y,z + 0.25,0.03)
rospy.sleep(1)

moveit.name_grp(moveit.group1,"open")
rospy.sleep(1)

moveit.plan_arm(x+0.015,y, z + 0.01 ,0.03)
rospy.sleep(1)

moveit.plan_gripper(0.048)
rospy.sleep(2)
moveit.plan_arm(x,y,z+0.2,0.03)
rospy.sleep(1)
moveit.plan_arm(2,0,1.3,0.03)
rospy.sleep(1)
moveit.plan_arm(2,0,1.12,0.03)
rospy.sleep(1)
moveit.name_grp(moveit.group1,"open")
rospy.sleep(1)
moveit.plan_arm(2,0,1.3,0.03)
moveit.name_grp(moveit.group,"idle")

moveit.name_grp(moveit.group1,"close")

rospy.sleep(3)
moveit_commander.roscpp_shutdown()
