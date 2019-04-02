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

        super(MoveCommanderUr10, self).__init__() # Is this necessary?

        #SETUP
        moveit_commander.roscpp_initialize (sys.argv) # Initialize the moveit commander
        
        self.robot_comm = moveit_commander.RobotCommander() #Initiate a RobotCommander object
        self.scene_interface = moveit_commander.PlanningSceneInterface() #Initiate a Planning scene Interface object

        group_name = "right_arm_and_manipulator" #Initiate a Move Group Commander object 
        self.group = moveit_commander.MoveGroupCommander(group_name)

        #Publisher to publish trajectories for RViz to visualize: 
        self.display_trajectory_publisher = rospy.Publisher("/move_group", moveit_msgs.msg.DisplayTrajectory, queue_size=20)

        self.planning_frame = self.group.get_planning_frame()
        self.eef_link= self.group.get_end_effector_link()
        self.group_names = self.robot_comm.get_group_names()

   
    def plan_to_pose_target (self, position_x, position_y, position_z, orient_x, orient_y, orient_z, orient_w):

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = position_x
        pose_goal.position.y = position_y
        pose_goal.position.z = position_z
        pose_goal.orientation.x = orient_x
        pose_goal.orientation.y = orient_y
        pose_goal.orientation.z = orient_z
        pose_goal.orientation.w = orient_w
        
        self.group.set_pose_target(pose_goal)

        plan = self.group.plan()


        return plan

    def execute_plan (self, plan):
        
        self.group.execute(plan, wait=True)


    # def go_to_joint_target (self, )


if __name__ == '__main__':
    
    rospy.init_node("move_commander_ur10", anonymous= True) # Define the node

    ur10_commander= MoveCommanderUr10()
    
    #Random pose
    plan1= ur10_commander.plan_to_pose_target(0.89,0.36,0.5,0,0.707,0.707 ,0)
     
    if len(plan1.joint_trajectory.points)>0: 
        
        raw_input("if Plan is ok. press enter to execute")
    else: 
        rospy.logerr ( "Fail to make a plan")

    ur10_commander.execute_plan(plan1)
   
         
    # Pose to vertical axis (i)
    plan_vertical= ur10_commander.plan_to_pose_target(0.04016, 0.36714, 1.52268,0.00021975, 0.70679, 0.70742, -0.00028481)
     
    if len(plan_vertical.joint_trajectory.points)>0: 
        
        raw_input("if Plan is ok. press enter to execute")
  
    else: 
        rospy.logerr ( "Fail to make a plan")

    ur10_commander.execute_plan(plan_vertical)

     # Pose to horizontal axis (1)
    plan_horizm1= ur10_commander.plan_to_pose_target(-1.2768, 0.36727, 0.26319,-0.013261,0.70689,0.70708 ,0.012899)
     
    if len(plan_horizm1.joint_trajectory.points)>0: 
        
        raw_input("if Plan is ok. press enter to execute")
  
    else: 
        rospy.logerr ( "Fail to make a plan")

    ur10_commander.execute_plan(plan_horizm1)

    # Pose to horizontal axis (1)
    plan_horiz1= ur10_commander.plan_to_pose_target(1.2768, 0.36727, 0.26319,-0.0092237,0.707,0.7071 ,0.0084781)
     
    if len(plan_horiz1.joint_trajectory.points)>0: 
        
        raw_input("if Plan is ok. press enter to execute")
  
    else: 
        rospy.logerr ( "Fail to make a plan")

    ur10_commander.execute_plan(plan_horiz1)