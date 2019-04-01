#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


class MoveCommanderUr10 (object): 
    """ Move commander created by CLP with the template of MoveGroupPythonInterfaceTutorial"""

    def __init__(self): 

        super(MoveCommanderSr10, self).__init__() # Is this necessary?

        #SETUP
        moveit_commander.roscpp_initialize (sys.argv) # Initialize the moveit commander
        
        self.robot_comm = moveit_commander.RobotCommander() #Initiate a RobotCommander object
        self.scene_interface = moveit_commander.PlanningSceneInterface() #Initiate a Planning scene Interface object

        group_name = "right_arm_and_manipulator" #Initiate a Move Group Commander object 
        self.group = moveit_commander.MoveGroupCommander(group_name)

        #Publisher to publish trajectories for RViz to visualize: 
        self.display_trajectory_publisher = rospy.Publisher("/move_group", moveit_msgs.msg.DisplayTrajectory, queue_size=20)

        self.planning_frame = group.get_planning_frame()
        self.eef_link= group.get_end_effector_link()
        self.group_names = robot.get_group_names()

   
    def plan_to_pose_target (self, position_x, position_y, position_z, orient_x, orient_y, orient_z, orient_w):

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position_x = position_x
        pose_goal.position_y = position_y
        pose_goal.position_z = position_z
        pose_goal.orientation_x = orient_x
        pose_goal.orientation_y = orient_y
        pose_goal.orientation_z = orient_z
        pose_goal.orientation_w = orient_w
        
        group.set_pose_target(pose_goal)

        plan = group.plan()


        return plan

    def execute_plan (self, plan):
        
        group.execute(plan, wait=True)


    # def go_to_joint_target (self, )


if __name__ == '__main__':
    
    rospy.init_node("move_commander_ur10", anonymous= True) # Define the node

    ur10_commander= MoveCommanderUr10()
    
    plan1= ur10_commander.plan_to_pose_target(2,2,2,2,2,2,2)

    raw_input("if Plan is ok. press enter to execute")
    
    ur10_commander.execute_plan(plan1)

    carotest = "please delete" 


